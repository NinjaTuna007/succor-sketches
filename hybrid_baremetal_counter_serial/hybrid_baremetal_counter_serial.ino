// hybrid_counter_serial.ino
//
// Teensy 4.1 (i.MX RT1062)
// HYBRID (TX/RX) via compile-time switch, **NO micro-ROS**.
// RX mode:
//   - PPS rising edge ISR snapshots a 1 MHz timebase (GPT2_CNT => microseconds).
//   - MODEM_FLAG rising edge is **hardware input-captured** into GPT2_ICR1.
//   - loop() polls GPT2_SR.IF1, reads capture time, clears flag, computes delta,
//     and prints delta over USB Serial.
//
// TX mode:
//   - PPS rising edge ISR sets a flag.
//   - loop() sends a modem command on PPS and prints what was sent.
//
// Timing principle:
//   - Only interrupt used for timing is PPS (minimal ISR).
//   - MODEM_FLAG capture is hardware-latched, read in loop (no ISR).
//   - Serial printing latency does NOT affect measured timestamps.
//
// --------------------- MODE SELECT ---------------------
// 1 = transmitter mode (PPS triggers modem broadcast)
// 0 = receiver mode (FLAG capture => print delta_us)
#define MODE_TRANSMITTER 0
// -------------------------------------------------------
//
// ------------------- PIN ASSUMPTIONS -------------------
// PPS can be any interrupt-capable pin (here: 40).
// MODEM_FLAG for GPT2 hardware capture MUST be on a pad that supports GPT2 CAPIN1.
// A known-good mapping on Teensy 4.1 is pin 15 (GPIO_AD_B1_03 => ALT8 = GPT2 Capture1).
//
// If your MODEM_FLAG is physically on pin 41:
//   - You CAN interrupt on it as GPIO, but that is NOT input capture.
//   - For true input-capture timing, rewire MODEM_FLAG to pin 15 (recommended).
// -------------------------------------------------------

#include <Arduino.h>

// ---------------- Pins ----------------
static constexpr uint8_t PIN_PPS          = 40;
static constexpr uint8_t PIN_FLAG_CAPTURE = 15;  // GPT2 CAPIN1 known-good mapping
static constexpr uint8_t PIN_DIAG         = 42;  // optional scope pin: HIGH at PPS, LOW at FLAG

// ---------------- Modem UART ----------------
#define MODEM_SERIAL Serial1
static constexpr uint32_t MODEM_BAUD = 9600;

// ---------------- Serial output ----------------
static constexpr uint32_t USB_BAUD = 115200; // ignored on Teensy USB Serial, but fine to set

// ---------------- TX command ----------------
#if MODE_TRANSMITTER
static const char TX_CMD[] = "$P007";
#endif

// ---------------- Timing state ----------------
// GPT2_CNT ticks at 1 MHz => 1 tick = 1 us
static volatile uint32_t last_pps_us_isr = 0;
static volatile bool     pps_pending     = false;

#if MODE_TRANSMITTER
static volatile bool     pps_tx_pending  = false;
#endif

// ---------------- GPT2 Capture Init (1 MHz, CAPIN1 on pin 15) ----------------
// This sets up GPT2 as a free-running 32-bit timer at 1 MHz and enables capture on CAPIN1.
// We do NOT enable GPT2 interrupts; loop() polls GPT2_SR.IF1.
static void gpt2_capture_init_pin15_1mhz()
{
  // Enable GPT2 clocks
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON) | CCM_CCGR0_GPT2_SERIAL(CCM_CCGR_ON);

  // Route CAPIN1 input select + set pad mux to GPT2 capture
  // Teensy 4.1 pin 15 == GPIO_AD_B1_03
  IOMUXC_GPT2_IPP_IND_CAPIN1_SELECT_INPUT = 1;   // remap GPT2 capture 1
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_03 = 8;       // ALT8 = GPT2 Capture1
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_03 = 0x13000; // pulldown + hysteresis

  // Stop/reset GPT2
  GPT2_CR = 0;
  GPT2_PR = 0;
  GPT2_SR = 0x3F;  // clear status flags
  GPT2_IR = 0;     // no interrupts

  // Prescale 24 MHz -> 1 MHz: PR = 23 => divide by 24
  GPT2_PR = 23;

  // Start: CLKSRC(1)=24MHz, free-run, capture on input1
  GPT2_CR = GPT_CR_EN | GPT_CR_CLKSRC(1) | GPT_CR_FRR | GPT_CR_IM1(1);
}

// ---------------- PPS ISR ----------------
// Minimal ISR: snapshot timer + set flags (NO printing, NO comms)
void isr_pps_rising()
{
  last_pps_us_isr = GPT2_CNT;
  pps_pending = true;
  digitalWriteFast(PIN_DIAG, HIGH);

#if MODE_TRANSMITTER
  pps_tx_pending = true;
#endif
}

// ---------------- UART RX line printing (optional) ----------------
static void pump_modem_rx_to_serial()
{
  while (MODEM_SERIAL.available()) {
    int c = MODEM_SERIAL.read();
    if (c >= 0) Serial.write((char)c);
  }
}

#if MODE_TRANSMITTER
static void modem_send_cmd(const char *cmd)
{
  const size_t len = strlen(cmd);
  if (len == 0) return;

  // Optional: restart UART each PPS (matches some "phase-jitter reduction" experiments)
  // Comment out if you don't want this behavior.
  MODEM_SERIAL.end();
  MODEM_SERIAL.begin(MODEM_BAUD);

  MODEM_SERIAL.write((const uint8_t*)cmd, len);
  MODEM_SERIAL.flush();
}
#endif

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(PIN_DIAG, OUTPUT);
  digitalWriteFast(PIN_DIAG, LOW);

  pinMode(PIN_PPS, INPUT);
  pinMode(PIN_FLAG_CAPTURE, INPUT);

  Serial.begin(USB_BAUD);
  // Teensy USB Serial: optional wait for host
  // while (!Serial && millis() < 1500) {}

  MODEM_SERIAL.begin(MODEM_BAUD);

  gpt2_capture_init_pin15_1mhz();
  attachInterrupt(digitalPinToInterrupt(PIN_PPS), isr_pps_rising, RISING);

#if MODE_TRANSMITTER
  Serial.println("MODE: TRANSMITTER");
#else
  Serial.println("MODE: RECEIVER (HW capture on pin 15, PPS ISR only)");
  Serial.println("Output format: pps_us,flag_us,delta_us");
#endif
}

void loop()
{
  // Optional: mirror modem RX to USB serial for debugging
  pump_modem_rx_to_serial();

  // PPS event (timestamp already captured in ISR)
  if (pps_pending) {
    noInterrupts();
    uint32_t pps_us = last_pps_us_isr;
    pps_pending = false;
    interrupts();

#if MODE_TRANSMITTER
    // In TX mode, we can print PPS time too if helpful
    Serial.print("PPS @ ");
    Serial.print(pps_us);
    Serial.println(" us");
#else
    // RX mode: you may want PPS markers too (comment out if noisy)
    // Serial.print("PPS @ "); Serial.print(pps_us); Serial.println(" us");
    (void)pps_us;
#endif

    digitalToggleFast(LED_BUILTIN);
  }

#if MODE_TRANSMITTER
  // TX: send command on PPS (in loop, not ISR)
  if (pps_tx_pending) {
    noInterrupts();
    pps_tx_pending = false;
    uint32_t pps_us = last_pps_us_isr;
    interrupts();

    modem_send_cmd(TX_CMD);
    Serial.print("TX @ ");
    Serial.print(pps_us);
    Serial.print(" us : ");
    Serial.println(TX_CMD);
  }

#else
  // RX: poll GPT2 capture flag (IF1) => MODEM_FLAG rising edge captured in HW
  if (GPT2_SR & GPT_SR_IF1) {
    const uint32_t flag_us = GPT2_ICR1;  // latched at FLAG rising edge
    GPT2_SR = GPT_SR_IF1;                // clear IF1
    digitalWriteFast(PIN_DIAG, LOW);

    // Snapshot last PPS time (coherent read)
    uint32_t pps_us;
    noInterrupts();
    pps_us = last_pps_us_isr;
    interrupts();

    // Wrap-safe delta (unsigned subtraction), represent as signed for readability
    const uint32_t du_u32 = (uint32_t)(flag_us - pps_us);
    const int32_t  delta_us = (int32_t)du_u32;

    // CSV output: pps_us,flag_us,delta_us
    Serial.print(pps_us);
    Serial.print(',');
    Serial.print(flag_us);
    Serial.print(',');
    Serial.println(delta_us);
  }
#endif
}
