// PPS/FLAG + modem helper (Teensy 4.1) - NO ROS
//
// Modes:
//   MODE_TRANSMITTER = 1: PPS triggers modem command send (Serial1)
//   MODE_TRANSMITTER = 0: FLAG rising logs delta_t = (FLAG - last PPS)
//
// Logging (USB Serial):
//   PPS,<t_us>
//   TX,<t_us>,<cmd>
//   RX,<t_us>,<line>
//   DELTA,<t_us>,<delta_us>   (RX only; t_us is FLAG edge time)
//
// Timing rules:
//   - ISRs do minimal work only (micros() + flags).
//   - No Serial printing in ISRs.
//   - Logger is non-blocking: drops logs if USB can't keep up (keeps timing accurate).
//
// Modem UART: Serial1 @ 9600 baud

#include <Arduino.h>

// --------------------- MODE SELECT ---------------------
// 1 = transmitter mode (PPS triggers modem broadcast)
// 0 = receiver mode (FLAG rising logs delta_t)
#define MODE_TRANSMITTER 0
// -------------------------------------------------------

static constexpr uint8_t  PIN_PPS  = 40;
static constexpr uint8_t  PIN_FLAG = 41;

#define MODEM_SERIAL Serial1
static constexpr uint32_t MODEM_BAUD = 9600;

static constexpr size_t   LINE_MAX = 256;  // max RX line length (excluding CRLF)

// ---------------- Non-blocking logger ----------------
// Ring buffer of fixed-size log lines; drops if full.
// Keeps your timing accurate even if USB host is slow.
static constexpr uint16_t LOG_Q_DEPTH = 64;
static constexpr uint16_t LOG_LINE_MAX = 320; // enough for "RX,<ts>,<256 bytes>"

static char log_q[LOG_Q_DEPTH][LOG_LINE_MAX];
static volatile uint16_t log_q_head = 0; // write index
static volatile uint16_t log_q_tail = 0; // read index

static inline uint16_t q_next(uint16_t v) { return (uint16_t)((v + 1) % LOG_Q_DEPTH); }

// Push a complete c-string line into queue (non-blocking). Drops if queue full.
static void log_enqueue_line(const char *line)
{
  noInterrupts();
  uint16_t head = log_q_head;
  uint16_t next = q_next(head);
  if (next == log_q_tail) {
    // queue full -> drop (timing > logging)
    interrupts();
    return;
  }
  // Copy (bounded)
  size_t n = strnlen(line, LOG_LINE_MAX - 1);
  memcpy(log_q[head], line, n);
  log_q[head][n] = '\0';
  log_q_head = next;
  interrupts();
}

// Drain queue to USB Serial without blocking hard.
// Only writes when Serial has space; otherwise returns quickly.
static void log_drain_usb()
{
  // Small budget per loop to avoid spending too long here.
  // You can increase if you want more aggressive draining.
  uint8_t budget = 8;

  while (budget--) {
    noInterrupts();
    if (log_q_tail == log_q_head) {
      interrupts();
      return; // empty
    }
    const char *line = log_q[log_q_tail];
    // Make a local copy of tail index (safe enough; line stable until we advance tail).
    interrupts();

    size_t len = strnlen(line, LOG_LINE_MAX - 1);

    // Need len + 1 for '\n'
    if (Serial.availableForWrite() < (int)(len + 1)) {
      return; // not enough room right now
    }

    Serial.write((const uint8_t*)line, len);
    Serial.write('\n');

    noInterrupts();
    log_q_tail = q_next(log_q_tail);
    interrupts();
  }
}

// --------------- ISR-shared state ----------------
static volatile uint32_t last_pps_us = 0;
static volatile bool     pps_led_toggle_pending = false;

#if MODE_TRANSMITTER
static volatile bool     pps_tx_pending = false;
#else
static volatile int32_t  delta_us_isr = 0;
static volatile uint32_t flag_time_us_isr = 0;
static volatile bool     delta_pending = false;
#endif

// --------------- modem RX line assembly ------------
static char   modem_line[LINE_MAX];
static size_t modem_len = 0;
static bool   saw_cr = false;

// --------------- ISRs ------------------------------
void isr_pps_rising()
{
  last_pps_us = micros();
  pps_led_toggle_pending = true;

#if MODE_TRANSMITTER
  pps_tx_pending = true;
#endif
}

#if !MODE_TRANSMITTER
void isr_flag_rising()
{
  const uint32_t now = micros();
  const uint32_t pps = last_pps_us;
  const uint32_t du  = (uint32_t)(now - pps);  // wrap-safe
  delta_us_isr = (int32_t)du;
  flag_time_us_isr = now;
  delta_pending = true;
}
#endif

// --------------- UART RX pump ----------------------
// Publishes ONLY full lines (CRLF terminated, tolerant lone '\n').
// When line completes, we enqueue: "RX,<t_us>,<line>"
static void pump_modem_rx_full_lines()
{
  while (MODEM_SERIAL.available()) {
    char c = (char)MODEM_SERIAL.read();

    // If previous char was CR, ignore an immediate LF
    if (saw_cr) {
      saw_cr = false;
      if (c == '\n') continue;
      // else treat normally (belongs to next line)
    }

    if (c == '\r') {
      if (modem_len > 0) {
        modem_line[modem_len] = '\0';
        char out[LOG_LINE_MAX];
        uint32_t t = micros();
        // RX,<t_us>,<line>
        snprintf(out, sizeof(out), "RX,%lu,%s", (unsigned long)t, modem_line);
        log_enqueue_line(out);
        modem_len = 0;
      }
      saw_cr = true;
      continue;
    }

    if (c == '\n') {
      if (modem_len > 0) {
        modem_line[modem_len] = '\0';
        char out[LOG_LINE_MAX];
        uint32_t t = micros();
        snprintf(out, sizeof(out), "RX,%lu,%s", (unsigned long)t, modem_line);
        log_enqueue_line(out);
        modem_len = 0;
      }
      continue;
    }

    // Accumulate, clamp (drop overflow bytes until EOL)
    if (modem_len < (LINE_MAX - 1)) {
      modem_line[modem_len++] = c;
    }
  }
}

#if MODE_TRANSMITTER
// TX mode command (as-is; no CRLF unless you include it).
static const char tx_cmd[] = "$P007";

// Restart Serial1 + single write() + flush() for consistent offset, like your original.
static void modem_send_cmd(const char *cmd)
{
  const size_t len = strlen(cmd);
  if (len == 0 || len >= 128) return;

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

  pinMode(PIN_PPS, INPUT);
  pinMode(PIN_FLAG, INPUT);

  MODEM_SERIAL.begin(MODEM_BAUD);

  // USB serial: keep it fast.
  Serial.begin(2000000);

  attachInterrupt(digitalPinToInterrupt(PIN_PPS), isr_pps_rising, RISING);
#if !MODE_TRANSMITTER
  attachInterrupt(digitalPinToInterrupt(PIN_FLAG), isr_flag_rising, RISING);
#endif

#if MODE_TRANSMITTER
  log_enqueue_line("BOOT,transmitter_ready");
#else
  log_enqueue_line("BOOT,receiver_ready");
#endif
}

void loop()
{
  // Drain logs early/often
  log_drain_usb();

  // PPS LED toggle
  if (pps_led_toggle_pending) {
    noInterrupts();
    pps_led_toggle_pending = false;
    uint32_t t = last_pps_us; // PPS timestamp
    interrupts();

    digitalToggleFast(LED_BUILTIN);

    char out[64];
    snprintf(out, sizeof(out), "PPS,%lu", (unsigned long)t);
    log_enqueue_line(out);
  }

#if MODE_TRANSMITTER
  if (pps_tx_pending) {
    noInterrupts();
    pps_tx_pending = false;
    uint32_t t = last_pps_us;
    interrupts();

    modem_send_cmd(tx_cmd);

    char out[LOG_LINE_MAX];
    snprintf(out, sizeof(out), "TX,%lu,%s", (unsigned long)t, tx_cmd);
    log_enqueue_line(out);
  }
#else
  // FLAG rising -> log delta once per edge
  if (delta_pending) {
    noInterrupts();
    int32_t du = delta_us_isr;
    uint32_t t = flag_time_us_isr; // when FLAG ISR ran
    delta_pending = false;
    interrupts();

    char out[96];
    snprintf(out, sizeof(out), "DELTA,%lu,%ld", (unsigned long)t, (long)du);
    log_enqueue_line(out);
  }
#endif

  // Modem RX parsing
  pump_modem_rx_full_lines();

  // Drain again
  log_drain_usb();

  // No delay() -> best timing. If you *must* yield USB, you can do:
  // yield();
}
