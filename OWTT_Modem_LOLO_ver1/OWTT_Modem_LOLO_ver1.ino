// OWTT_Modem_LoLo_ver1.ino
//
// Teensy 4.1 bridge between Host (LoLo or Frigates) and Succorfish modem.
//
// UARTS:
//   Serial1 = Succorfish modem
//   Host / LoLo can be selected at compile time:
//     HOST_OVER_USB = 1 -> Serial  = Host over Teensy Micro USB
//     HOST_OVER_USB = 0 -> Serial2 = Host over UART
//
// Main behavior:
//   - By default the Teensy boots in WIRE mode.
//   - Host commands normally pass directly to the modem.
//   - Modem responses/messages are always forwarded to the host first.
//   - Special Teensy config commands start with $Y and are not forwarded.
//   - Special GPS update commands start with $G and are only intercepted in
//     transmitter mode.
//
// Runtime config command:
//   $Y<own_id><mode><args>
//
// Examples:
//   $Y007W
//      Set modem address to 007, then enter WIRE mode.
//      Host <-> Teensy <-> Modem, no extra behavior.
//
//   $Y101R
//      Set modem address to 101, then enter RECEIVER mode.
//      Modem messages are forwarded to host first.
//      If a received modem message contains GPS coordinates, Teensy sends:
//          #I<delta_us>
//      after the raw modem message.
//
//   $Y042T0001s
//      Set modem address to 042, then enter TRANSMITTER mode.
//      listen_id = 000 means first leader.
//      Broadcast latest stored GPS every 1 PPS/holdover epoch.
//
//   $Y042T0074s
//      Set modem address to 042, then enter TRANSMITTER mode.
//      Wait until a modem message/frame from source 007 is received.
//      Then broadcast latest stored GPS after 4 PPS/holdover epochs.
//
// Robust config behavior:
//   - On valid $Y config, Teensy sends $A<own_id> to the modem.
//   - New config is applied only after modem confirms with #A<own_id>.
//   - If modem replies E or times out, old config remains active.
//   - While config is pending, host commands are rejected with BUSY.
//
// GPS update command, transmitter mode only:
//   $G<lat>,<lon>
//      Updates internal latest GPS variable only.
//      Not forwarded directly to modem.
//
// Automatic transmitter broadcast:
//   $Bnn<lat>,<lon>
//      Sent to modem on scheduled PPS/holdover epoch.
//      nn is the byte length of "<lat>,<lon>".
//
// Receiver timing:
//   - External 10 MHz OCXO clock into pin 14.
//   - PPS into pin 40 for synchronization of vehicles.
//   - RxS flag into pin 15 for acoustic packet/header detection.
//   - GPT2 runs at 1 MHz, so 1 tick = 1 us.
//   - If PPS disappears, GPT2 compare keeps virtual 1-second epochs
//     using the OCXO holdover clock.
//
// Receiver output order:
//   1. Forward raw modem line to host, e.g.
//      #Bxxxnndd...   
//      #B00722<lat>,<lon>
//   2. If that line contains GPS coordinates, send timing info:
//        #I<delta_us>
//
// Notes:
//   - delta_us = RxS_capture_time_us - current_epoch_time_us.
//   - current_epoch_time_us comes from real PPS when available,
//     otherwise from OCXO holdover epochs.
//   - For reactive round-robin, follower delay must be large enough for:
//       propagation_time + acoustic_packet_duration + guard
//     otherwise the follower may learn too late that its trigger leader spoke.
// $G11.2328,12.12385

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

// =======================================================
// ===================== USER CONFIG =====================
// =======================================================

//   1 = Host / LoLo talks through Teensy Micro USB
//   0 = Host / LoLo talks through UART Serial2
//
// If HOST_OVER_USB = 1:
//   USB Serial must stay protocol-clean, so USB debug must be disabled
//
// If HOST_OVER_USB = 0:
//   Serial2 is the host port, and USB Serial can be used for debug.

#define HOST_OVER_USB 1

#define MODEM_SERIAL Serial1

#if HOST_OVER_USB
  #define HOST_SERIAL Serial
  #define ENABLE_USB_DEBUG 0
  #define USB_MIRROR_HOST_OUTPUT 0
#else
  #define HOST_SERIAL Serial2
  #define ENABLE_USB_DEBUG 1
  #define USB_MIRROR_HOST_OUTPUT 1
#endif

static constexpr uint32_t MODEM_BAUD = 9600;
static constexpr uint32_t HOST_BAUD  = 115200;

// Teensy 4.1 timing pins
static constexpr uint8_t PIN_OCXO = 14;
static constexpr uint8_t PIN_RXS_CAPTURE   = 15;
static constexpr uint8_t PIN_PPS_CAPTURE   = 40;

// External 10 MHz clock / (9 + 1) = 1 MHz GPT2 tick.
static constexpr uint32_t GPT2_PRESCALER_VALUE = 9;

static constexpr size_t LINE_MAX = 256;
static constexpr size_t GPS_MAX  = 64;

// OCXO Holdover constants
static constexpr uint32_t EPOCH_US       = 1000000UL;
static constexpr uint32_t PPS_TIMEOUT_US = 1200000UL;

// Config timeout constant
static constexpr uint32_t CONFIG_ADDR_TIMEOUT_MS = 1000;

// =======================================================
// ===================== MODES ===========================
// =======================================================

enum BridgeMode : uint8_t {
  MODE_WIRE = 0,
  MODE_RECEIVER = 1,
  MODE_TRANSMITTER = 2
};

struct BridgeConfig {
  BridgeMode mode;

  // This modem's own address, configured through $A<own_id>.
  char own_id[4];

  // For transmitter round robin:
  // "000" = first transmitter, no predecessor.
  // Otherwise, wait for #B<listen_id>
  char listen_id[4];

  // Broadcast period/delay in PPS ticks.
  uint32_t period_pps;

  bool tx_scheduled;
  uint32_t next_tx_epoch_count;
};

static BridgeConfig cfg = {
  MODE_WIRE,
  "000",  // own_id
  "000",  // listen_id
  1, // period_pps
  false, // tx_scheduled
  0 // next_tx_epoch_count
};

struct PendingConfig {
  bool active;

  BridgeMode mode;
  char own_id[4];
  char listen_id[4];
  uint32_t period_pps;

  uint32_t deadline_ms;
  char original_cmd[96];
};

static PendingConfig pending_cfg = {
  false, // active
  MODE_WIRE, // mode
  "000", // own_id
  "000", // listen_id
  1, // period_pps
  0, // deadline_ms
  {0} // original_cmd
};

// Latest GPS payload stored on transmitter.
// Payload is stored as raw "<lat>,<lon>", without "$G".
static char latest_gps[GPS_MAX + 1] = {0};
static size_t latest_gps_len = 0;
static bool latest_gps_valid = false;

static constexpr uint32_t GPS_VALID_EPOCHS = 5;

static uint32_t latest_gps_epoch_count = 0;

// Automatic TX due flag.
// Set by PPS timing logic, executed later after host commands are serviced.
static bool tx_auto_due = false;

// =======================================================
// ===================== LINE READER =====================
// =======================================================

struct LineReader {
  char buf[LINE_MAX];
  size_t len;
  bool saw_cr;
};

static LineReader host_reader  = {{0}, 0, false};
static LineReader modem_reader = {{0}, 0, false};

static bool read_line(Stream &s, LineReader &r, char *out, size_t out_sz)
{
  while (s.available() > 0) {
    int b = s.read();
    if (b < 0) {
      return false;
    }

    char c = (char)b;

    if (r.saw_cr) {
      r.saw_cr = false;
      if (c == '\n') {
        continue;
      }
    }

    if (c == '\r') {
      r.buf[r.len] = '\0';
      strncpy(out, r.buf, out_sz - 1);
      out[out_sz - 1] = '\0';
      r.len = 0;
      r.saw_cr = true;
      return true;
    }

    if (c == '\n') {
      r.buf[r.len] = '\0';
      strncpy(out, r.buf, out_sz - 1);
      out[out_sz - 1] = '\0';
      r.len = 0;
      return true;
    }

    if (r.len < LINE_MAX - 1) {
      r.buf[r.len++] = c;
    } else {
      r.len = 0;
      r.saw_cr = false;
#if ENABLE_USB_DEBUG
      Serial.println("WARN: line overflow, dropped");
#endif
    }
  }

  return false;
}

static void host_send_line(const char *line)
{
  // send to LoLo
  HOST_SERIAL.print(line);
  HOST_SERIAL.print("\r\n");

#if ENABLE_USB_DEBUG && USB_MIRROR_HOST_OUTPUT
  // Marks on the programming side what LoLo sees in case we don't have access to that
  Serial.print("HOST SEES: ");
  Serial.println(line);
#endif
}

static void modem_send_command_no_terminator(const char *cmd)
{
  size_t n = strlen(cmd);
  if (n == 0) {
    return;
  }

  // Succorfish modem commands require no CR/LF terminator
  MODEM_SERIAL.write((const uint8_t *)cmd, n);
  MODEM_SERIAL.flush();

#if ENABLE_USB_DEBUG
  Serial.print("Sent to Succor: ");
  Serial.println(cmd);
#endif
}

static void modem_set_address(const char id3[4])
{
  char cmd[8];
  snprintf(cmd, sizeof(cmd), "$A%s", id3);
  modem_send_command_no_terminator(cmd);
}

// =======================================================
// ===================== SMALL HELPERS ===================
// =======================================================

static bool is_digit_char(char c)
{
  return c >= '0' && c <= '9';
}

static bool starts_with(const char *s, const char *prefix)
{
  return strncmp(s, prefix, strlen(prefix)) == 0;
}

static void trim_spaces(char *s)
{
  if (s == NULL) {
    return;
  }

  while (*s == ' ' || *s == '\t') {
    memmove(s, s + 1, strlen(s));
  }

  size_t n = strlen(s);
  while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t')) {
    s[n - 1] = '\0';
    n--;
  }
}


static bool parse_two_digit_len(const char *p, size_t *out_len)
{
  if (!is_digit_char(p[0]) || !is_digit_char(p[1])) {
    return false;
  }

  *out_len = (size_t)((p[0] - '0') * 10 + (p[1] - '0'));
  return true;
}


static bool valid_3_digit_id(const char *p)
{
  return is_digit_char(p[0]) && is_digit_char(p[1]) && is_digit_char(p[2]);
}

static bool parse_id3_u8(const char *p, char out_id[4])
{
  if (!valid_3_digit_id(p)) {
    return false;
  }

  uint16_t value =
    (uint16_t)(p[0] - '0') * 100 +
    (uint16_t)(p[1] - '0') * 10 +
    (uint16_t)(p[2] - '0');

  if (value > 255) {
    return false;
  }

  out_id[0] = p[0];
  out_id[1] = p[1];
  out_id[2] = p[2];
  out_id[3] = '\0';
  return true;
}

static bool same_id3(const char *a, const char *b)
{
  return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

static bool is_first_transmitter()
{
  return same_id3(cfg.listen_id, "000");
}

static bool looks_like_gps_coords(const char *s)
{
  if (s == NULL || s[0] == '\0') {
    return false;
  }

  // Expected:
  //   <lat>,<lon>
  // Example:
  //   59.1234567,18.1234567
  //
  // $G11.2328,12.12385

  char *end1 = NULL;
  char *end2 = NULL;

  double lat = strtod(s, &end1);

  if (end1 == s || end1 == NULL || *end1 != ',') {
    return false;
  }

  const char *lon_start = end1 + 1;
  double lon = strtod(lon_start, &end2);

  if (end2 == lon_start || end2 == NULL) {
    return false;
  }

  // Require exact end after longitude.
  if (*end2 != '\0') {
    return false;
  }

  if (lat < -90.0 || lat > 90.0) {
    return false;
  }

  if (lon < -180.0 || lon > 180.0) {
    return false;
  }

  return true;
}

static void host_send_config_error(const char *cmd, const char *reason)
{
  char msg[96];

  if (cmd == NULL) {
    cmd = "";
  }

  if (reason == NULL) {
    reason = "UNKNOWN";
  }

  snprintf(msg, sizeof(msg), "#E,Y,%s,%s", reason, cmd);
  host_send_line(msg);
}

static void copy_id3(char dst[4], const char src[4])
{
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = '\0';
}

static char mode_to_char(BridgeMode mode)
{
  if (mode == MODE_WIRE) {
    return 'W';
  }

  if (mode == MODE_RECEIVER) {
    return 'R';
  }

  if (mode == MODE_TRANSMITTER) {
    return 'T';
  }

  return '?';
}

static void host_send_config_ok()
{
  char msg[96];

  if (cfg.mode == MODE_TRANSMITTER) {
    snprintf(
      msg,
      sizeof(msg),
      "#Y,OK,%s,T,%s,%lu",
      cfg.own_id,
      cfg.listen_id,
      (unsigned long)cfg.period_pps
    );
  } else {
    snprintf(
      msg,
      sizeof(msg),
      "#Y,OK,%s,%c",
      cfg.own_id,
      mode_to_char(cfg.mode)
    );
  }

  host_send_line(msg);
}

// =======================================================
// ===================== TIMING ==========================
// =======================================================

// PPS rising edge, record the time and process it when have time
static volatile bool pps_pending = false;
// Store timestamp when rising edge hit
static volatile uint32_t pps_stamp = 0;

// Succor flagged a received message, record that time and then process it after passing messages to HOST
static volatile bool recv_flag_pending = false;
// Store timestamp when Succor flag was risen
static volatile uint32_t recv_flag_stamp = 0;

// For HOLDOVER (no PPS)
// Counter reached compare value (1 epoch has passed)
static volatile bool cmp_pending = false;
// Store timestamp and process it when free
static volatile uint32_t cmp_stamp = 0;

// Used for OCXO holdover.
// Compare interrupt fires every EPOCH_US = 1 second.
// The transmitter period is handled separately by counting epochs:
//   next_tx_epoch_count = epoch_count + cfg.period_pps
static volatile uint32_t gpt2_next_compare = 0;

enum TimingMode : uint8_t {
  TIMING_WAIT_PPS   = 0,
  TIMING_PPS_LOCKED = 1,
  TIMING_HOLDOVER   = 2
};

static TimingMode timing_mode = TIMING_WAIT_PPS;

// It increments on real PPS and on holdover virtual PPS. Keeps tracks of how many transmissions passed -> good for orchestrating multiple transmitters
static uint32_t epoch_count = 0;

static uint32_t current_epoch_us = 0;
static uint32_t last_real_pps_us = 0; // Time of last synchronisation
static bool pps_valid = false;

static bool pending_delta_valid = false; // Calculates deltas only if received GPS coords from a transmitter
static int32_t pending_delta_us = 0;
static uint32_t pending_delta_ms = 0;

extern "C" void GPT2_IRQHandler();

static inline uint32_t OCXO_counter_us()
{
  return GPT2_CNT;
}

static inline void gpt2_set_compare_us(uint32_t t_us)
{
  noInterrupts();
  gpt2_next_compare = t_us;
  GPT2_OCR1 = t_us;
  interrupts();
}

static inline void gpt2_enable_compare_irq()
{
  noInterrupts();
  GPT2_IR |= GPT_IR_OF1IE;
  interrupts();
}

static inline void gpt2_disable_compare_irq()
{
  noInterrupts();
  GPT2_IR &= ~GPT_IR_OF1IE;
  interrupts();
}

static void gpt2_extclk_capture_init_1mhz()
{
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON) |
               CCM_CCGR0_GPT2_SERIAL(CCM_CCGR_ON);

  GPT2_CR = 0;
  GPT2_PR = 0;
  GPT2_SR = 0x3F;
  GPT2_IR = 0;

  // External clock on pin 14:
  // GPIO_AD_B1_02 -> GPT2_CLK
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;
  IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT = 1;

  // Receive Flag capture on pin 15:
  // GPIO_AD_B1_03 -> GPT2_CAPTURE1
  IOMUXC_GPT2_IPP_IND_CAPIN1_SELECT_INPUT = 1;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_03 = 8;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_03 = 0x13000;

  // PPS capture on pin 40:
  // GPIO_AD_B1_04 -> GPT2_CAPTURE2
  IOMUXC_GPT2_IPP_IND_CAPIN2_SELECT_INPUT = 1;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_04 = 8;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_04 = 0x13000;

  GPT2_PR = GPT_PR_PRESCALER(GPT2_PRESCALER_VALUE);

  // CLKSRC(3): external clock
  // FRR: free-running
  // IM1(1): capture 1 rising edge, Receive Flag
  // IM2(1): capture 2 rising edge, PPS
  GPT2_CR = GPT_CR_CLKSRC(3) |
            GPT_CR_FRR |
            GPT_CR_IM1(1) |
            GPT_CR_IM2(1);

  GPT2_IR = GPT_IR_IF1IE | GPT_IR_IF2IE;

  NVIC_CLEAR_PENDING(IRQ_GPT2);
  attachInterruptVector(IRQ_GPT2, GPT2_IRQHandler);
  NVIC_ENABLE_IRQ(IRQ_GPT2);

  GPT2_CR |= GPT_CR_EN;
}

extern "C" void GPT2_IRQHandler()
{
  uint32_t sr = GPT2_SR;

  // Receive Flag capture: acoustic packet/header detected.
  if (sr & GPT_SR_IF1) {
    recv_flag_stamp = GPT2_ICR1;
    recv_flag_pending = true;
    GPT2_SR = GPT_SR_IF1;
  }

  // PPS capture
  if (sr & GPT_SR_IF2) {
    pps_stamp = GPT2_ICR2;
    pps_pending = true;
    GPT2_SR = GPT_SR_IF2;
  }

  // Holdover virtual PPS compare.
  if (sr & GPT_SR_OF1) {
    cmp_stamp = GPT2_OCR1;
    cmp_pending = true;
    GPT2_SR = GPT_SR_OF1;

    gpt2_next_compare += EPOCH_US;
    GPT2_OCR1 = gpt2_next_compare;
  }

  asm volatile("dsb");
}

static void schedule_first_transmitter_after_config()
{
  cfg.tx_scheduled = false;

  if (cfg.mode != MODE_TRANSMITTER) {
    return;
  }

  if (!is_first_transmitter()) {
    return;
  }

  // First leader starts at the next PPS, then repeats every period_pps.
  cfg.next_tx_epoch_count = epoch_count + 1;
  cfg.tx_scheduled = true;
}

static void begin_pending_config(
  BridgeMode new_mode,
  const char new_own_id[4],
  const char new_listen_id[4],
  uint32_t new_period_pps,
  const char *original_cmd)
{
  pending_cfg.active = true;
  pending_cfg.mode = new_mode;

  copy_id3(pending_cfg.own_id, new_own_id);
  copy_id3(pending_cfg.listen_id, new_listen_id);

  pending_cfg.period_pps = new_period_pps;
  pending_cfg.deadline_ms = millis() + CONFIG_ADDR_TIMEOUT_MS;

  if (original_cmd == NULL) {
    original_cmd = "";
  }

  strncpy(pending_cfg.original_cmd, original_cmd, sizeof(pending_cfg.original_cmd) - 1);
  pending_cfg.original_cmd[sizeof(pending_cfg.original_cmd) - 1] = '\0';

  // Freeze automatic transmission while the modem address is being confirmed.
  tx_auto_due = false;

  modem_set_address(pending_cfg.own_id);
}

static void apply_pending_config()
{
  cfg.mode = pending_cfg.mode;

  copy_id3(cfg.own_id, pending_cfg.own_id);
  copy_id3(cfg.listen_id, pending_cfg.listen_id);

  cfg.period_pps = pending_cfg.period_pps;

  cfg.tx_scheduled = false;
  tx_auto_due = false;
  pending_delta_valid = false;

  pending_cfg.active = false;

  schedule_first_transmitter_after_config();

#if ENABLE_USB_DEBUG
  Serial.print("CONFIG APPLIED: own_id=");
  Serial.print(cfg.own_id);
  Serial.print(", mode=");
  Serial.print(mode_to_char(cfg.mode));
  Serial.print(", listen_id=");
  Serial.print(cfg.listen_id);
  Serial.print(", period_pps=");
  Serial.println(cfg.period_pps);
#endif

  host_send_config_ok();
}

static void abort_pending_config(const char *reason)
{
  if (reason == NULL) {
    reason = "UNKNOWN";
  }

  host_send_config_error(pending_cfg.original_cmd, reason);
  pending_cfg.active = false;
  tx_auto_due = false;
}

static void handle_epoch_event(uint32_t t_epoch, bool real_pps)
{
  current_epoch_us = t_epoch;
  pps_valid = true;
  epoch_count++;

  if (latest_gps_valid &&
      (uint32_t)(epoch_count - latest_gps_epoch_count) > GPS_VALID_EPOCHS) {
    latest_gps_valid = false;
    latest_gps_len = 0;
  }

  if (real_pps) {
    last_real_pps_us = t_epoch;
    timing_mode = TIMING_PPS_LOCKED;

    gpt2_set_compare_us(t_epoch + EPOCH_US);
    gpt2_disable_compare_irq();
  }

  if (cfg.mode == MODE_TRANSMITTER && cfg.tx_scheduled) {
    if ((int32_t)(epoch_count - cfg.next_tx_epoch_count) >= 0) {
      tx_auto_due = true;

      if (is_first_transmitter()) {
        cfg.next_tx_epoch_count = epoch_count + cfg.period_pps;
        cfg.tx_scheduled = true;
      } else {
        cfg.tx_scheduled = false;
      }
    }
  }
}

static uint32_t epoch_ref_for_capture(uint32_t t_capture)
{
  uint32_t ref = current_epoch_us;

  // If the captured flag timestamp is before the current epoch edge,
  // then the flag actually belongs to the previous epoch.
  //
  // This can happen when both PPS and receive flag are pending, and the
  // main loop processes PPS first.
  if ((int32_t)(t_capture - ref) < 0) {
    ref -= EPOCH_US;
  }

  return ref;
}

static void handle_rxs_event(uint32_t t_rxs)
{
  // Only receiver mode produces #I timing outputs.
  if (cfg.mode != MODE_RECEIVER) {
    return;
  }

  if (!pps_valid) {
    pending_delta_valid = false;
    return;
  }

  uint32_t ref_epoch_us = epoch_ref_for_capture(t_rxs);

  pending_delta_us = (int32_t)((uint32_t)(t_rxs - ref_epoch_us));
  pending_delta_ms = millis();
  pending_delta_valid = true;

#if ENABLE_USB_DEBUG
  Serial.print("Rx flag captured delta_us=");
  Serial.print(pending_delta_us);
  Serial.print(", ref_epoch_us=");
  Serial.println(ref_epoch_us);
#endif
}

static void maybe_enter_holdover()
{
  if (timing_mode != TIMING_PPS_LOCKED) {
    return;
  }

  uint32_t now = OCXO_counter_us();
  uint32_t since_last_pps = (uint32_t)(now - last_real_pps_us);

  if (since_last_pps <= PPS_TIMEOUT_US) {
    return;
  }

  uint32_t missed_epochs = since_last_pps / EPOCH_US;
  if (missed_epochs == 0) {
    missed_epochs = 1;
  }

  current_epoch_us = last_real_pps_us + missed_epochs * EPOCH_US;

  gpt2_set_compare_us(current_epoch_us + EPOCH_US);
  gpt2_enable_compare_irq();

  timing_mode = TIMING_HOLDOVER;

#if ENABLE_USB_DEBUG
  Serial.println("TIMING: entered OCXO holdover");
#endif
}

static void process_timing_events()
{
  // Real PPS prioritized. If PPS comes back, it re-locks.
  if (pps_pending) {
    noInterrupts();
    uint32_t t = pps_stamp;
    pps_pending = false;
    interrupts();

    handle_epoch_event(t, true);
  }

  // Check if PPS has disappeared -> switch to OCXO holdover
  maybe_enter_holdover();

  // Virtual PPS from GPT2 compare while in holdover.
  if (cmp_pending) {
    noInterrupts();
    uint32_t t = cmp_stamp;
    cmp_pending = false;
    interrupts();

    if (timing_mode == TIMING_HOLDOVER) {
      handle_epoch_event(t, false);
    }
  }

  // Receive flag capture after epoch bookkeeping.
  // The delta calculation chooses the correct epoch based on the captured
  // timestamp, not simply the newest processed PPS.

  if (recv_flag_pending) {
    noInterrupts();
    uint32_t t = recv_flag_stamp;
    recv_flag_pending = false;
    interrupts();

    handle_rxs_event(t);
  }
}

static bool consume_delta(int32_t *out_delta_us)
{
  if (out_delta_us == NULL) {
    return false;
  }

  if (!pending_delta_valid) {
    return false;
  }

  *out_delta_us = pending_delta_us;
  pending_delta_valid = false;
  return true;
}

// =======================================================
// ===================== CONFIG PARSING ==================
// =======================================================

static bool parse_period_pps_strict(const char *p, uint32_t *out_period_pps)
{
  if (p == NULL || out_period_pps == NULL || p[0] == '\0') {
    return false;
  }

  uint32_t value = 0;
  bool saw_digit = false;

  while (is_digit_char(*p)) {
    saw_digit = true;
    value = value * 10 + (uint32_t)(*p - '0');
    p++;
  }

  if (!saw_digit || value == 0 || value > 3600) {
    return false;
  }

  // Accept:
  //   1s
  //   4s
  //   9s
  //   1
  //   4
  //
  // The unit is PPS ticks. With 1 Hz PPS, 1 tick = 1 second.
  if (*p == 's' || *p == 'S') {
    p++;
  }

  if (*p != '\0') {
    return false;
  }

  *out_period_pps = value;
  return true;
}


static bool handle_config_command(char *line)
{
  if (!starts_with(line, "$Y")) {
    return false;
  }

  if (pending_cfg.active) {
    host_send_config_error(line, "BUSY");
    return true;
  }

  // Strict format:
  //
  //   $Y<own_id><mode><args>
  //
  // Examples:
  //   $Y007W
  //   $Y101R
  //   $Y008T0001s
  //   $Y042T0074s
  // if transmitter mode is expected to be T<listen_id><period>, means that this transmitter only transmits 1 period after the modem with <listen_id> has transmitted

  const char *p = line + 2;

  char new_own_id[4];

  if (!parse_id3_u8(p, new_own_id)) {
    host_send_config_error(line, "BAD_OWN_ID");
    return true;
  }

  char mode = p[3];
  const char *args = p + 4;

  if (mode == 'W' || mode == 'w') {
    if (*args != '\0') {
      host_send_config_error(line, "BAD_EXTRA");
      return true;
    }

    begin_pending_config(
      MODE_WIRE,
      new_own_id,
      "000",
      1,
      line
    );

#if ENABLE_USB_DEBUG
    Serial.print("CONFIG PENDING: WIRE, own_id=");
    Serial.println(new_own_id);
#endif

    return true;
  }

  if (mode == 'R' || mode == 'r') {
    if (*args != '\0') {
      host_send_config_error(line, "BAD_ARGS");
      return true;
    }

    begin_pending_config(
      MODE_RECEIVER,
      new_own_id,
      "000",
      1,
      line
    );

#if ENABLE_USB_DEBUG
    Serial.print("CONFIG PENDING: RECEIVER, own_id=");
    Serial.println(new_own_id);
#endif

    return true;
  }

  if (mode == 'T' || mode == 't') {
    char new_listen_id[4];
    uint32_t new_period_pps = 1;

    if (!parse_id3_u8(args, new_listen_id)) {
      host_send_config_error(line, "BAD_LISTEN_ID");
      return true;
    }

    if (!parse_period_pps_strict(args + 3, &new_period_pps)) {
      host_send_config_error(line, "BAD_PERIOD");
      return true;
    }

    begin_pending_config(
      MODE_TRANSMITTER,
      new_own_id,
      new_listen_id,
      new_period_pps,
      line
    );

#if ENABLE_USB_DEBUG
    Serial.print("CONFIG PENDING: TRANSMITTER, own_id=");
    Serial.print(new_own_id);
    Serial.print(", listen_id=");
    Serial.print(new_listen_id);
    Serial.print(", period_pps=");
    Serial.println(new_period_pps);
#endif

    return true;
  }

  host_send_config_error(line, "BAD_MODE");
  return true;
}


// =======================================================
// ===================== GPS TX ==========================
// =======================================================

static bool update_latest_gps_from_host(const char *line)
{
  if (cfg.mode != MODE_TRANSMITTER) {
    return false;
  }

  if (!starts_with(line, "$G")) {
    return false;
  }

  const char *gps = line + 2;

  if (!looks_like_gps_coords(gps)) {
#if ENABLE_USB_DEBUG
    Serial.print("WARN: bad GPS update ignored: ");
    Serial.println(line);   
#endif
    return true;
  }

  size_t n = strlen(gps);
  if (n < 2 || n > GPS_MAX) {
#if ENABLE_USB_DEBUG
    Serial.print("WARN: GPS length invalid: ");
    Serial.println(n);
#endif
    return true;
  }

  memcpy(latest_gps, gps, n);
  latest_gps[n] = '\0';
  latest_gps_len = n;

  latest_gps_epoch_count = epoch_count;
  latest_gps_valid = true;

#if ENABLE_USB_DEBUG
  Serial.print("GPS updated: ");
  Serial.println(latest_gps);
#endif

  return true;
}

static bool send_latest_gps_broadcast()
{
  if (cfg.mode != MODE_TRANSMITTER) {
    return false;
  }

  if (!latest_gps_valid || latest_gps_len < 2 || latest_gps_len > 64) {
#if ENABLE_USB_DEBUG
    Serial.println("WARN: no valid GPS to send to HOST");
#endif
    return false;
  }

  char cmd[96];

  int n = snprintf(
    cmd,
    sizeof(cmd),
    "$B%02u%s",
    (unsigned int)latest_gps_len,
    latest_gps
  );

  if (n < 0 || (size_t)n >= sizeof(cmd)) {
#if ENABLE_USB_DEBUG
    Serial.println("WARN: command overflow, auto-TX skipped");
#endif
    return false;
  }

  modem_send_command_no_terminator(cmd);
  return true;
}

// =======================================================
// ===================== MODEM PARSING ===================
// =======================================================

static bool extract_source_id_from_modem_line(const char *line, char *src_id)
{
  if (line == NULL || src_id == NULL) {
    return false;
  }

  // Broadcast receive:
  //   #B<aaa><yy><data>...
  //
  // Frame beacon:
  //   #F<aaa>...
  if (line[0] == '#' && (line[1] == 'B' || line[1] == 'F')) {
    if (valid_3_digit_id(line + 2)) {
      src_id[0] = line[2];
      src_id[1] = line[3];
      src_id[2] = line[4];
      src_id[3] = '\0';
      return true;
    }
  }

  return false;
}

static bool extract_gps_payload_from_modem_line(const char *line, char *payload, size_t payload_sz)
{
  if (line == NULL || payload == NULL || payload_sz == 0) {
    return false;
  }

  payload[0] = '\0';

  // Broadcast receive format:
  //   #B<aaa><yy><data>[Q..][T..]
  //
  // Index:
  //   0 '#'
  //   1 'B'
  //   2..4 aaa source address
  //   5..6 yy length
  //   7.. payload begins
  if (line[0] == '#' && line[1] == 'B') {
    if (!valid_3_digit_id(line + 2)) {
      return false;
    }

    size_t n = 0;
    if (!parse_two_digit_len(line + 5, &n)) {
      return false;
    }

    if (n >= payload_sz) {
      n = payload_sz - 1;
    }

    size_t line_len = strlen(line);
    if (line_len < 7 + n) {
      return false;
    }

    memcpy(payload, line + 7, n);
    payload[n] = '\0';

    return looks_like_gps_coords(payload);
  }

  // Unicast receive format:
  //   #U<yy><data>[Q..][T..]
  //
  // Index:
  //   0 '#'
  //   1 'U'
  //   2..3 yy length
  //   4.. payload begins
  if (line[0] == '#' && line[1] == 'U') {
    size_t n = 0;
    if (!parse_two_digit_len(line + 2, &n)) {
      return false;
    }

    if (n >= payload_sz) {
      n = payload_sz - 1;
    }

    size_t line_len = strlen(line);
    if (line_len < 4 + n) {
      return false;
    }

    memcpy(payload, line + 4, n);
    payload[n] = '\0';

    return looks_like_gps_coords(payload);
  }

  return false;
}

static void maybe_schedule_round_robin_tx_from_modem_line(const char *line)
{
  if (cfg.mode != MODE_TRANSMITTER) {
    return;
  }

  if (is_first_transmitter()) {
    return;
  }

  char src_id[4];

  if (!extract_source_id_from_modem_line(line, src_id)) {
    return;
  }

  if (!same_id3(src_id, cfg.listen_id)) {
    return;
  }

  // Heard the predecessor leader.
  // Schedule own broadcast after configured PPS delay.
  cfg.next_tx_epoch_count = epoch_count + cfg.period_pps;
  cfg.tx_scheduled = true;

#if ENABLE_USB_DEBUG
  Serial.print("ROUND_ROBIN: heard ");
  Serial.print(src_id);
  Serial.print(", scheduled TX at epoch_count=");
  Serial.println(cfg.next_tx_epoch_count);
#endif
}

static void maybe_emit_receiver_delta_after_modem_line(const char *line)
{
  if (cfg.mode != MODE_RECEIVER) {
    return;
  }

  char payload[GPS_MAX + 1];

  if (!extract_gps_payload_from_modem_line(line, payload, sizeof(payload))) {
    return;
  }

  // At this point the modem line containing the GPS has already been passed
  // to the host. Now emit only timing info.
  int32_t delta_us = 0;
  char info[32];

  if (consume_delta(&delta_us)) {
    snprintf(info, sizeof(info), "#I%ld", (long)delta_us);
  } else {
    snprintf(info, sizeof(info), "#INA");
  }

  host_send_line(info);
}

// =======================================================
// ===================== HOST / MODEM LOGIC ==============
// =======================================================

static void handle_host_line(char *line)
{
  trim_spaces(line);

  #if ENABLE_USB_DEBUG
    Serial.print("HOST_RX: ");
    Serial.println(line);
  #endif

    if (line[0] == '\0') {
      return;
    }

    if (pending_cfg.active) {
      if (starts_with(line, "$Y")) {
        host_send_config_error(line, "BUSY");
      } else {
        host_send_line("#E,Y,BUSY");
      }
      return;
    }

    if (handle_config_command(line)) {
      return;
    }

    if (update_latest_gps_from_host(line)) {
      return;
    }

    modem_send_command_no_terminator(line);
}

static bool handle_pending_config_modem_line(const char *line)
{
  if (!pending_cfg.active) {
    return false;
  }

  if (line == NULL) {
    return false;
  }

  char expected[8];
  snprintf(expected, sizeof(expected), "#A%s", pending_cfg.own_id);

  // Exact match only. Avoid accidentally accepting a status line like:
  // #A007V21996...
  if (strcmp(line, expected) == 0) {
    apply_pending_config();
    return true;
  }

  if (strcmp(line, "E") == 0) {
    abort_pending_config("ADDRESS_REJECTED");
    return true;
  }

  return true;
}

static void check_pending_config_timeout()
{
  if (!pending_cfg.active) {
    return;
  }

  if ((int32_t)(millis() - pending_cfg.deadline_ms) >= 0) {
    abort_pending_config("ADDR_TIMEOUT");
  }
}

static void handle_modem_line(char *line)
{
#if ENABLE_USB_DEBUG
  Serial.print("Received from Succor: ");
  Serial.println(line);
#endif

  // Always pass modem response/message to host first.
  host_send_line(line);

  // If we are waiting for #Axxx/E from the modem, resolve config here
  // and do not interpret this line as GPS or round-robin data.
  if (handle_pending_config_modem_line(line)) {
    return;
  }

  if (cfg.mode == MODE_WIRE) {
    return;
  }

  maybe_emit_receiver_delta_after_modem_line(line);
  maybe_schedule_round_robin_tx_from_modem_line(line);
}

static void execute_auto_tx_if_due()
{
  if (!tx_auto_due) {
    return;
  }

  tx_auto_due = false;

  if (pending_cfg.active) {
    return;
  }

  if (cfg.mode != MODE_TRANSMITTER) {
    return;
  }

  send_latest_gps_broadcast();
}

// =======================================================
// ===================== SETUP / LOOP ====================
// =======================================================

void setup()
{
#if ENABLE_USB_DEBUG
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("Booting Translucent Teensy");
#endif

  MODEM_SERIAL.begin(MODEM_BAUD, SERIAL_8N1);
  #if HOST_OVER_USB
  
  HOST_SERIAL.begin(HOST_BAUD);
  #else
  HOST_SERIAL.begin(HOST_BAUD, SERIAL_8N1);
  #endif

  pinMode(PIN_OCXO, INPUT);
  pinMode(PIN_RXS_CAPTURE, INPUT);
  pinMode(PIN_PPS_CAPTURE, INPUT);

  gpt2_extclk_capture_init_1mhz();

#if ENABLE_USB_DEBUG
  Serial.println("Serial1 modem ready");
  #if HOST_OVER_USB
    Serial.println("USB Serial host ready");
  #else
    Serial.println("Serial2 host ready");
  #endif
  Serial.println("GPT2 timing ready");
  Serial.println("Default mode: WIRE");
#endif
}

void loop()
{
  char line[LINE_MAX];

  // First: latch timing events so delta_t is independent of serial delays.
  process_timing_events();

  check_pending_config_timeout();

  // Host commands get forwarded immediately and do not wait for PPS.
  while (read_line(HOST_SERIAL, host_reader, line, sizeof(line))) {
    handle_host_line(line);
  }

  // Modem messages are transparently forwarded first;
  // receiver then adds #I<delta_t> only for GPS payloads.
  while (read_line(MODEM_SERIAL, modem_reader, line, sizeof(line))) {
    handle_modem_line(line);
  }

  // Automatic transmitter broadcast runs after host commands,
  // so manual host commands have priority over scheduled GPS TX.
  execute_auto_tx_if_due();
}