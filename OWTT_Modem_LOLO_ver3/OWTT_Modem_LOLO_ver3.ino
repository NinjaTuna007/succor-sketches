// OWTT_Modem_LOLO_ver3.ino
//
// Teensy 4.1 bridge between Host (LoLo or Frigates) and Succorfish modem.
//
// ver2 adds a telemetry frame on top of ver1's GPS broadcast:
//   - Host command "$K<payload>" stores opaque telemetry.
//   - It is broadcast as "$BnnTEL:<payload>" on the PPS schedule.
//   - Receivers treat any broadcast whose payload starts with "TEL:" as a
//     telemetry frame and still emit an "#I<delta_us>" timing line for it, so
//     telemetry broadcasts are range-able. See the $K / TEL: notes below.
//
// UARTS / USB:
//   Serial1    = Succorfish modem
//   Serial2    = ZED-X20P UART1
//   Serial     = Host / LoLo over the first Teensy USB CDC port
//   SerialUSB1 = Transparent ZED-X20P bridge over the second USB CDC port
//
// Arduino IDE / Teensyduino:
//   Tools -> USB Type -> Dual Serial
//
// The GNSS bridge is byte-transparent in both directions. Bytes received from
// the X20P are also observed by a passive UBX parser so UBX-NAV-TIMEUTC can be
// used internally without removing or changing any byte sent to the PC.
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
// Telemetry update command, transmitter mode only:
//   $K<payload>
//      Updates internal latest telemetry variable only.
//      "$K" is used (not "$T") because "$T" is an existing Succorfish modem
//      command; the host->Teensy command prefix must not collide with it.
//      <payload> is opaque, host-encoded data (e.g. position + speed +
//      bt_tip status). The Teensy does not interpret it.
//      Not forwarded directly to modem.
//      Stored/broadcast with a leading "TEL:" telemetry marker, i.e.
//      "$K<payload>" goes out as "$BnnTEL:<payload>".
//      When telemetry is valid it takes precedence over GPS for the
//      scheduled broadcast (see below).
//
// Automatic transmitter broadcast:
//   $Bnn<lat>,<lon>           (GPS frame, when no telemetry is set)
//   $BnnTEL:<payload>         (telemetry frame, when telemetry is set)
//      Sent to modem on scheduled PPS/holdover epoch.
//      nn is the byte length of the broadcast payload (the bytes after nn).
//      The telemetry frame is identified by a leading "TEL:" marker in the
//      broadcast payload.
//
// Receiver telemetry handling:
//   A received broadcast whose payload starts with "TEL:" is treated as a
//   telemetry frame. Just like a GPS frame, the raw modem line is forwarded
//   to the host first and then a timing line #I<delta_us> is emitted, so
//   ranges can be derived from telemetry broadcasts. Decoding the telemetry
//   payload is the host's responsibility.
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
//   2. If that line contains GPS coordinates/telemetry, send timing info:
//        #I<delta_us>
//   3. When the RxS capture has an absolute UTC epoch label, also send:
//        #J<rx_utc_unix_s>,<delta_us>
//
// GNSS diagnostics from the first USB/host port:
//   $ZGNSS?
//      One-shot activity report plus the newest observed NMEA sentence.
//   $ZGNSSDEBUG=1
//      Enable one human-readable GNSS summary per second.
//   $ZGNSSDEBUG=0
//      Disable live summaries.
//   $ZGNSSBAUD=<baud>
//      Change only the Teensy Serial2 listening baud, e.g. 38400 or 115200.
//
// GNSS UTC status query from the host:
//   $ZUTC?
//      Returns the maintained epoch UTC and reports PPS_LOCKED/HOLDOVER.
//
// Experiment controls:
//   $ZPAYLOADTTL=0    Keep the latest $G/$K payload indefinitely (default).
//   $ZPAYLOADTTL=N    Expire it after N PPS/holdover epochs.
//   $ZTXWARN=1/0      Enable/disable one-shot TX warnings (default off).
//   $ZOWTTHEADER?     Reprint the #OWTT CSV column header.
//
// Timestamped acoustic payload:
//   T<unix_us:16d>|<seq:4hex>|<P/H/W>|<holdover:8hex>|<GPS-or-TEL-payload>
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

// The first USB CDC interface remains the LoLo/Succorfish protocol port.
// The second USB CDC interface is dedicated to the X20P UART bridge.
#define MODEM_SERIAL    Serial1
#define HOST_SERIAL     Serial
#define GNSS_SERIAL     Serial2
#define GNSS_USB_SERIAL SerialUSB1

// USB host protocol must stay clean. Do not print asynchronous debug text to
// Serial or SerialUSB1.
#define ENABLE_USB_DEBUG 0
#define USB_MIRROR_HOST_OUTPUT 0

static constexpr uint32_t MODEM_BAUD = 9600;
static constexpr uint32_t HOST_BAUD  = 115200;

// Use the actual UART1 baud currently stored in the X20P. Your tested module
// was using 115200. A factory-default X20P normally uses 38400.
static constexpr uint32_t GNSS_INITIAL_BAUD = 38400;

// Normal-operation default: keep Serial2 fixed at GNSS_INITIAL_BAUD.
//
// A terminal program can report 9600 as its USB CDC baud even though USB CDC
// itself has no physical baud rate. If this option is true, opening the GNSS
// USB port with `screen` at 9600 would also switch Serial2 to 9600 and make a
// 115200-baud X20P appear silent. Enable this only for service tools that must
// intentionally change the receiver UART baud, such as some firmware-update
// workflows.
static constexpr bool GNSS_FOLLOW_USB_BAUD = false;

// Enable UBX-NAV-TIMEUTC output on X20P UART1 at one message per navigation
// epoch. The command is sent to RAM only, so it does not permanently rewrite
// the receiver configuration.
static constexpr bool GNSS_AUTO_ENABLE_TIMEUTC = true;

// Keep the legacy #I<delta_us> line and optionally add:
//   #J<rx_utc_unix_s>,<delta_us>
// The #J line labels the receive capture with the absolute UTC second.
static constexpr bool EMIT_RX_UTC_LINE = true;

// Teensy 4.1 timing pins
static constexpr uint8_t PIN_OCXO = 14;
static constexpr uint8_t PIN_RXS_CAPTURE   = 15;
static constexpr uint8_t PIN_PPS_CAPTURE   = 40;

// External 10 MHz clock / (9 + 1) = 1 MHz GPT2 tick.
static constexpr uint32_t GPT2_PRESCALER_VALUE = 9;

static constexpr size_t LINE_MAX = 256;
static constexpr size_t GPS_MAX  = 64;

// GNSS UTC parser / association limits.
static constexpr uint16_t UBX_MAX_PAYLOAD = 1024;
static constexpr uint32_t UTC_BIND_MAX_MESSAGE_LATENCY_US = 900000UL;
static constexpr uint32_t UTC_STATUS_MAX_AGE_MS = 3000UL;

// GNSS diagnostics. Live debug is disabled at boot and can be enabled from
// the first USB/host port with `$ZGNSSDEBUG=1`. It prints one summary per
// second and never injects debug bytes into the transparent GNSS USB port.
static constexpr uint32_t GNSS_ACTIVITY_FRESH_MS = 2000UL;
static constexpr uint32_t GNSS_DEBUG_PERIOD_MS = 1000UL;
static constexpr size_t GNSS_NMEA_MAX = 160;

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

// Payload lifetime in timing epochs. A value of 0 means the last valid GPS or
// telemetry payload remains valid indefinitely. This experiment build boots
// with infinite lifetime so one $G command can drive a long test. Change at
// runtime with $ZPAYLOADTTL=<epochs>.
static uint32_t payload_valid_epochs = 0;

static uint32_t latest_gps_epoch_count = 0;

// Latest telemetry payload stored on transmitter.
// Stored WITH the leading "TEL:" frame marker, e.g. "TEL:<encoded...>".
// The payload after "TEL:" is opaque host-encoded data; the Teensy never
// parses it. When valid, telemetry takes precedence over GPS for the
// scheduled broadcast. payload_valid_epochs controls expiry; zero means the
// stored frame remains valid indefinitely for long experiments.
static char latest_telem[GPS_MAX + 1] = {0};
static size_t latest_telem_len = 0;
static bool latest_telem_valid = false;
static uint32_t latest_telem_epoch_count = 0;

// Experiment transmission metadata and output policy.
static uint16_t tx_sequence = 0;
static bool tx_warning_enabled = false;

enum TxWarningReason : uint8_t {
  TX_WARN_NONE = 0,
  TX_WARN_NO_PAYLOAD,
  TX_WARN_NO_UTC,
  TX_WARN_PAYLOAD_TOO_LONG
};

static TxWarningReason tx_last_warning = TX_WARN_NONE;

// Succorfish broadcast payload limit. Timestamp metadata is included inside
// this limit. GPS coordinates comfortably fit; very large telemetry payloads
// may need to be shortened.
static constexpr size_t MODEM_PAYLOAD_MAX = 64;

// Range is emitted in integer millimetres to keep CSV output deterministic.
// 1500 m/s = 1,500,000 mm/s. Calibrate RANGE_FIXED_DELAY_US after measuring
// command-to-acoustic-emission and receive-flag fixed latency.
static constexpr int64_t SOUND_SPEED_MM_PER_S = 1500000LL;
static constexpr int32_t RANGE_FIXED_DELAY_US = 0;

// Automatic TX due flag.
// Set by PPS timing logic, executed later after host commands are serviced.
static bool tx_auto_due = false;

// =======================================================
// ===================== GNSS UTC STATE ==================
// =======================================================

struct GnssUtcState {
  bool valid;
  uint32_t iTOW_ms;
  uint32_t tAcc_ns;
  int32_t nano_ns;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t valid_flags;

  // Integer UTC second containing the navigation epoch. This is a Unix-style
  // second label used only for epoch disambiguation; sub-second timing still
  // comes from the captured PPS/RxS timestamps.
  int64_t unix_second;

  uint32_t received_ms;
  uint32_t received_counter_us;
};

static GnssUtcState gnss_utc = {
  false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

// Absolute UTC label associated with current_epoch_us. Once initialized from
// a valid UBX-NAV-TIMEUTC message, it advances on real and holdover epochs.
static bool current_epoch_utc_valid = false;
static int64_t current_epoch_utc_second = 0;
static uint32_t current_epoch_utc_sync_ms = 0;

// UTC label captured together with pending_delta_us.
static bool pending_delta_utc_valid = false;
static int64_t pending_delta_utc_second = 0;

// UBX byte parser. It only stores the 20-byte UBX-NAV-TIMEUTC payload and
// ignores the contents of all other UBX messages while still validating their
// framing/checksum.
enum UbxParserState : uint8_t {
  UBX_WAIT_SYNC1 = 0,
  UBX_WAIT_SYNC2,
  UBX_READ_CLASS,
  UBX_READ_ID,
  UBX_READ_LEN1,
  UBX_READ_LEN2,
  UBX_READ_PAYLOAD,
  UBX_READ_CK_A,
  UBX_READ_CK_B
};

struct UbxParser {
  UbxParserState state;
  uint8_t msg_class;
  uint8_t msg_id;
  uint16_t payload_len;
  uint16_t payload_index;
  uint8_t payload[20];
  uint8_t ck_a;
  uint8_t ck_b;
  uint8_t received_ck_a;
};

static UbxParser gnss_ubx = {
  UBX_WAIT_SYNC1, 0, 0, 0, 0, {0}, 0, 0, 0
};

static uint32_t gnss_uart_baud = GNSS_INITIAL_BAUD;
static bool gnss_timeutc_config_sent = false;
static uint32_t gnss_timeutc_config_due_ms = 0;

// Passive GNSS activity/debug counters. These prove that bytes reach Serial2
// even when the stream is binary UBX and therefore unreadable in `screen`.
static uint32_t gnss_rx_byte_count = 0;
static uint32_t gnss_tx_byte_count = 0;
static uint32_t gnss_last_rx_ms = 0;
static uint32_t gnss_ubx_ok_count = 0;
static uint32_t gnss_ubx_bad_checksum_count = 0;
static uint32_t gnss_timeutc_count = 0;
static uint32_t gnss_nmea_line_count = 0;
static uint32_t gnss_last_nmea_ms = 0;
static char gnss_last_nmea[GNSS_NMEA_MAX] = {0};

static bool gnss_live_debug_enabled = false;
static uint32_t gnss_next_debug_ms = 0;

struct GnssNmeaObserver {
  bool active;
  size_t len;
  char line[GNSS_NMEA_MAX];
};

static GnssNmeaObserver gnss_nmea = {false, 0, {0}};

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

// A telemetry frame is any payload that starts with the "TEL:" marker, e.g.
// "TEL:<encoded...>". The bytes after "TEL:" are opaque host-encoded data and
// are not interpreted here. A multi-char marker (vs a bare 'T') makes an
// accidental collision with ordinary broadcast data far less likely. Require
// at least one byte after the marker so an empty "TEL:" is not a valid frame.
static const char TELEMETRY_MARKER[] = "TEL:";
static const size_t TELEMETRY_MARKER_LEN = 4;

static bool looks_like_telemetry(const char *s)
{
  return (s != NULL &&
          strncmp(s, TELEMETRY_MARKER, TELEMETRY_MARKER_LEN) == 0 &&
          s[TELEMETRY_MARKER_LEN] != '\0');
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

static void emit_owtt_csv_header();

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

// Explicit prototype required for Arduino's .ino preprocessor. Without it,
// Arduino may auto-generate this prototype before TimingMode is declared.
static char timing_mode_code(TimingMode mode);

static TimingMode timing_mode = TIMING_WAIT_PPS;

// Number of OCXO-generated virtual seconds since the last real PPS. Reset to
// zero on every real PPS.
static uint32_t holdover_age_s = 0;

// It increments on real PPS and on holdover virtual PPS. Keeps tracks of how many transmissions passed -> good for orchestrating multiple transmitters
static uint32_t epoch_count = 0;

static uint32_t current_epoch_us = 0;
static uint32_t last_real_pps_us = 0; // Time of last synchronisation
static bool pps_valid = false;

static bool pending_delta_valid = false; // Calculates deltas only if received GPS coords from a transmitter
static int32_t pending_delta_us = 0;
static uint32_t pending_delta_ms = 0;
static uint32_t pending_delta_holdover_age_s = 0;
static uint32_t pending_delta_gnss_age_ms = 0;
static char pending_delta_timing_mode = 'W';

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

static char timing_mode_code(TimingMode mode)
{
  switch (mode) {
    case TIMING_PPS_LOCKED:
      return 'P';
    case TIMING_HOLDOVER:
      return 'H';
    case TIMING_WAIT_PPS:
    default:
      return 'W';
  }
}

// Build an absolute UTC timestamp from the UTC-labelled epoch and the OCXO
// counter. This does not use serial-message arrival time for the fractional
// timestamp. The serial UTC message labels the second; GPT2 supplies the
// microseconds inside it.
static bool utc_timestamp_for_counter(
  uint32_t counter_us,
  uint64_t *out_timestamp_us,
  int64_t *out_epoch_second,
  uint32_t *out_fraction_us)
{
  if (out_timestamp_us == NULL ||
      out_epoch_second == NULL ||
      out_fraction_us == NULL ||
      !pps_valid ||
      !current_epoch_utc_valid) {
    return false;
  }

  int64_t second = current_epoch_utc_second;
  uint32_t ref = current_epoch_us;

  // If a capture belongs to the immediately previous epoch, move both the
  // counter reference and UTC label back together.
  if ((int32_t)(counter_us - ref) < 0) {
    ref -= EPOCH_US;
    second--;
  }

  uint32_t elapsed_us = (uint32_t)(counter_us - ref);

  // This also handles the small window where an epoch interrupt occurred but
  // the main loop has not processed it yet.
  second += (int64_t)(elapsed_us / EPOCH_US);
  const uint32_t fraction_us = elapsed_us % EPOCH_US;

  *out_epoch_second = second;
  *out_fraction_us = fraction_us;
  *out_timestamp_us =
    (uint64_t)second * (uint64_t)EPOCH_US + (uint64_t)fraction_us;
  return true;
}

// =======================================================
// ===================== GNSS UBX PARSER =================
// =======================================================

static uint16_t read_u16_le(const uint8_t *p)
{
  return (uint16_t)p[0] |
         ((uint16_t)p[1] << 8);
}

static uint32_t read_u32_le(const uint8_t *p)
{
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static int32_t read_i32_le(const uint8_t *p)
{
  return (int32_t)read_u32_le(p);
}

// Days since 1970-01-01. Based on the proleptic Gregorian calendar.
static int64_t days_from_civil(int32_t year, uint32_t month, uint32_t day)
{
  year -= (month <= 2);
  const int32_t era = (year >= 0 ? year : year - 399) / 400;
  const uint32_t yoe = (uint32_t)(year - era * 400);
  const int32_t month_prime =
    (int32_t)month + (month > 2 ? -3 : 9);
  const uint32_t doy =
    (153U * (uint32_t)month_prime + 2U) / 5U +
    day - 1U;
  const uint32_t doe =
    yoe * 365U + yoe / 4U - yoe / 100U + doy;

  return (int64_t)era * 146097LL + (int64_t)doe - 719468LL;
}

static bool civil_utc_to_unix_second(
  uint16_t year,
  uint8_t month,
  uint8_t day,
  uint8_t hour,
  uint8_t minute,
  uint8_t second,
  int64_t *out)
{
  if (out == NULL) {
    return false;
  }

  if (year < 1999 || year > 2099 ||
      month < 1 || month > 12 ||
      day < 1 || day > 31 ||
      hour > 23 ||
      minute > 59 ||
      second > 60) {
    return false;
  }

  const int64_t days = days_from_civil(year, month, day);

  // second==60 is accepted for a positive UTC leap second. A Unix-style
  // scalar cannot uniquely represent that leap second, but this remains
  // adequate as an epoch label outside the leap insertion itself.
  *out =
    days * 86400LL +
    (int64_t)hour * 3600LL +
    (int64_t)minute * 60LL +
    (int64_t)second;

  return true;
}

static bool gnss_utc_is_fresh()
{
  if (!gnss_utc.valid) {
    return false;
  }

  return (uint32_t)(millis() - gnss_utc.received_ms) <=
         UTC_STATUS_MAX_AGE_MS;
}

static void maybe_bind_gnss_utc_to_current_epoch()
{
  if (!gnss_utc.valid ||
      !pps_valid ||
      timing_mode != TIMING_PPS_LOCKED) {
    return;
  }

  // UBX-NAV-TIMEUTC belongs to a navigation epoch. For direct association
  // with 1PPS, require that navigation epoch to be at the top of a second.
  const uint32_t tow_ms_in_second = gnss_utc.iTOW_ms % 1000UL;
  if (tow_ms_in_second > 20UL && tow_ms_in_second < 980UL) {
    return;
  }

  const int32_t message_after_pps_us =
    (int32_t)(gnss_utc.received_counter_us - current_epoch_us);

  // The TIMEUTC packet should be emitted after the PPS/navigation epoch to
  // which it refers. Reject a stale or unexpectedly delayed packet rather
  // than introducing a one-second error.
  if (message_after_pps_us < 0 ||
      (uint32_t)message_after_pps_us >
        UTC_BIND_MAX_MESSAGE_LATENCY_US) {
    return;
  }

  current_epoch_utc_second = gnss_utc.unix_second;
  current_epoch_utc_valid = true;
  current_epoch_utc_sync_ms = millis();
}

static void handle_ubx_nav_timeutc(const uint8_t payload[20])
{
  const uint8_t valid = payload[19];

  // Require valid time-of-week, valid week number and valid UTC conversion.
  if ((valid & 0x07U) != 0x07U) {
    return;
  }

  GnssUtcState next = {};
  next.iTOW_ms = read_u32_le(payload + 0);
  next.tAcc_ns = read_u32_le(payload + 4);
  next.nano_ns = read_i32_le(payload + 8);
  next.year = read_u16_le(payload + 12);
  next.month = payload[14];
  next.day = payload[15];
  next.hour = payload[16];
  next.minute = payload[17];
  next.second = payload[18];
  next.valid_flags = valid;

  if (!civil_utc_to_unix_second(
        next.year,
        next.month,
        next.day,
        next.hour,
        next.minute,
        next.second,
        &next.unix_second)) {
    return;
  }

  next.received_ms = millis();
  next.received_counter_us = OCXO_counter_us();
  next.valid = true;

  gnss_utc = next;
  gnss_timeutc_count++;
  maybe_bind_gnss_utc_to_current_epoch();
}

static void ubx_checksum_add(uint8_t b)
{
  gnss_ubx.ck_a = (uint8_t)(gnss_ubx.ck_a + b);
  gnss_ubx.ck_b = (uint8_t)(gnss_ubx.ck_b + gnss_ubx.ck_a);
}

static void ubx_parser_reset()
{
  gnss_ubx.state = UBX_WAIT_SYNC1;
  gnss_ubx.msg_class = 0;
  gnss_ubx.msg_id = 0;
  gnss_ubx.payload_len = 0;
  gnss_ubx.payload_index = 0;
  gnss_ubx.ck_a = 0;
  gnss_ubx.ck_b = 0;
  gnss_ubx.received_ck_a = 0;
}

static void gnss_ubx_feed(uint8_t b)
{
  switch (gnss_ubx.state) {
    case UBX_WAIT_SYNC1:
      if (b == 0xB5U) {
        gnss_ubx.state = UBX_WAIT_SYNC2;
      }
      return;

    case UBX_WAIT_SYNC2:
      if (b == 0x62U) {
        gnss_ubx.state = UBX_READ_CLASS;
        gnss_ubx.ck_a = 0;
        gnss_ubx.ck_b = 0;
      } else if (b != 0xB5U) {
        gnss_ubx.state = UBX_WAIT_SYNC1;
      }
      return;

    case UBX_READ_CLASS:
      gnss_ubx.msg_class = b;
      ubx_checksum_add(b);
      gnss_ubx.state = UBX_READ_ID;
      return;

    case UBX_READ_ID:
      gnss_ubx.msg_id = b;
      ubx_checksum_add(b);
      gnss_ubx.state = UBX_READ_LEN1;
      return;

    case UBX_READ_LEN1:
      gnss_ubx.payload_len = b;
      ubx_checksum_add(b);
      gnss_ubx.state = UBX_READ_LEN2;
      return;

    case UBX_READ_LEN2:
      gnss_ubx.payload_len |= ((uint16_t)b << 8);
      ubx_checksum_add(b);
      gnss_ubx.payload_index = 0;

      if (gnss_ubx.payload_len > UBX_MAX_PAYLOAD) {
        ubx_parser_reset();
      } else if (gnss_ubx.payload_len == 0) {
        gnss_ubx.state = UBX_READ_CK_A;
      } else {
        gnss_ubx.state = UBX_READ_PAYLOAD;
      }
      return;

    case UBX_READ_PAYLOAD:
      if (gnss_ubx.payload_index < sizeof(gnss_ubx.payload)) {
        gnss_ubx.payload[gnss_ubx.payload_index] = b;
      }

      ubx_checksum_add(b);
      gnss_ubx.payload_index++;

      if (gnss_ubx.payload_index >= gnss_ubx.payload_len) {
        gnss_ubx.state = UBX_READ_CK_A;
      }
      return;

    case UBX_READ_CK_A:
      gnss_ubx.received_ck_a = b;
      gnss_ubx.state = UBX_READ_CK_B;
      return;

    case UBX_READ_CK_B: {
      const bool checksum_ok =
        gnss_ubx.received_ck_a == gnss_ubx.ck_a &&
        b == gnss_ubx.ck_b;

      if (checksum_ok) {
        gnss_ubx_ok_count++;

        if (gnss_ubx.msg_class == 0x01U &&
            gnss_ubx.msg_id == 0x21U &&
            gnss_ubx.payload_len == 20U) {
          handle_ubx_nav_timeutc(gnss_ubx.payload);
        }
      } else {
        gnss_ubx_bad_checksum_count++;
      }

      ubx_parser_reset();
      return;
    }
  }

  ubx_parser_reset();
}

static void gnss_nmea_feed(uint8_t b)
{
  // NMEA is ASCII and starts with '$'. This observer does not consume or
  // alter the byte; it only remembers the newest complete line for debug.
  if (b == (uint8_t)'$') {
    gnss_nmea.active = true;
    gnss_nmea.len = 0;
    gnss_nmea.line[gnss_nmea.len++] = '$';
    return;
  }

  if (!gnss_nmea.active) {
    return;
  }

  if (b == (uint8_t)'\r' || b == (uint8_t)'\n') {
    if (gnss_nmea.len > 1) {
      gnss_nmea.line[gnss_nmea.len] = '\0';
      strncpy(gnss_last_nmea, gnss_nmea.line, sizeof(gnss_last_nmea) - 1);
      gnss_last_nmea[sizeof(gnss_last_nmea) - 1] = '\0';
      gnss_nmea_line_count++;
      gnss_last_nmea_ms = millis();
    }

    gnss_nmea.active = false;
    gnss_nmea.len = 0;
    return;
  }

  // Abort on binary/non-printable bytes. This avoids mistaking UBX payloads
  // containing a '$' byte for an NMEA sentence.
  if (b < 0x20U || b > 0x7EU) {
    gnss_nmea.active = false;
    gnss_nmea.len = 0;
    return;
  }

  if (gnss_nmea.len < sizeof(gnss_nmea.line) - 1) {
    gnss_nmea.line[gnss_nmea.len++] = (char)b;
  } else {
    gnss_nmea.active = false;
    gnss_nmea.len = 0;
  }
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
  if (cfg.mode == MODE_RECEIVER) {
    emit_owtt_csv_header();
  }
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

  if (current_epoch_utc_valid) {
    current_epoch_utc_second++;
  }

  if (real_pps) {
    holdover_age_s = 0;
  } else if (timing_mode == TIMING_HOLDOVER) {
    holdover_age_s++;
  }

  if (payload_valid_epochs != 0 &&
      latest_gps_valid &&
      (uint32_t)(epoch_count - latest_gps_epoch_count) > payload_valid_epochs) {
    latest_gps_valid = false;
    latest_gps_len = 0;
    tx_last_warning = TX_WARN_NONE;
  }

  if (payload_valid_epochs != 0 &&
      latest_telem_valid &&
      (uint32_t)(epoch_count - latest_telem_epoch_count) > payload_valid_epochs) {
    latest_telem_valid = false;
    latest_telem_len = 0;
    tx_last_warning = TX_WARN_NONE;
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
    pending_delta_utc_valid = false;
    return;
  }

  uint32_t ref_epoch_us = epoch_ref_for_capture(t_rxs);

  pending_delta_us = (int32_t)((uint32_t)(t_rxs - ref_epoch_us));
  pending_delta_ms = millis();
  pending_delta_valid = true;
  pending_delta_holdover_age_s = holdover_age_s;
  pending_delta_timing_mode = timing_mode_code(timing_mode);
  pending_delta_gnss_age_ms = gnss_utc.valid
    ? (uint32_t)(millis() - gnss_utc.received_ms)
    : 0xFFFFFFFFUL;

  pending_delta_utc_valid = current_epoch_utc_valid;
  pending_delta_utc_second = current_epoch_utc_second;

  // epoch_ref_for_capture() can select the previous epoch when PPS and RxS
  // interrupts are processed together in the main loop.
  if (pending_delta_utc_valid && ref_epoch_us != current_epoch_us) {
    pending_delta_utc_second--;
  }

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

  // Account for each virtual epoch that has already elapsed since the last
  // real PPS. The previous implementation moved current_epoch_us forward
  // without incrementing epoch_count, which also made an absolute UTC label
  // lag by one second after entering holdover.
  timing_mode = TIMING_HOLDOVER;

  for (uint32_t i = 1; i <= missed_epochs; i++) {
    handle_epoch_event(last_real_pps_us + i * EPOCH_US, false);
  }

  gpt2_set_compare_us(current_epoch_us + EPOCH_US);
  gpt2_enable_compare_irq();

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

static bool consume_delta(
  int32_t *out_delta_us,
  bool *out_utc_valid,
  int64_t *out_utc_second,
  uint32_t *out_holdover_age_s,
  uint32_t *out_gnss_age_ms,
  char *out_timing_mode,
  uint32_t *out_decode_delay_ms)
{
  if (out_delta_us == NULL ||
      out_utc_valid == NULL ||
      out_utc_second == NULL ||
      out_holdover_age_s == NULL ||
      out_gnss_age_ms == NULL ||
      out_timing_mode == NULL ||
      out_decode_delay_ms == NULL) {
    return false;
  }

  if (!pending_delta_valid) {
    return false;
  }

  *out_delta_us = pending_delta_us;
  *out_utc_valid = pending_delta_utc_valid;
  *out_utc_second = pending_delta_utc_second;
  *out_holdover_age_s = pending_delta_holdover_age_s;
  *out_gnss_age_ms = pending_delta_gnss_age_ms;
  *out_timing_mode = pending_delta_timing_mode;
  *out_decode_delay_ms = (uint32_t)(millis() - pending_delta_ms);

  pending_delta_valid = false;
  pending_delta_utc_valid = false;
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
  tx_last_warning = TX_WARN_NONE;

#if ENABLE_USB_DEBUG
  Serial.print("GPS updated: ");
  Serial.println(latest_gps);
#endif

  return true;
}

static const char *tx_warning_name(TxWarningReason reason)
{
  switch (reason) {
    case TX_WARN_NO_PAYLOAD:
      return "NO_PAYLOAD";
    case TX_WARN_NO_UTC:
      return "NO_UTC";
    case TX_WARN_PAYLOAD_TOO_LONG:
      return "PAYLOAD_TOO_LONG";
    case TX_WARN_NONE:
    default:
      return "NONE";
  }
}

static void maybe_emit_tx_warning(TxWarningReason reason)
{
  if (reason == TX_WARN_NONE) {
    tx_last_warning = TX_WARN_NONE;
    return;
  }

  if (!tx_warning_enabled || tx_last_warning == reason) {
    tx_last_warning = reason;
    return;
  }

  tx_last_warning = reason;
  char msg[64];
  snprintf(msg, sizeof(msg), "#TXWARN,%s", tx_warning_name(reason));
  host_send_line(msg);
}

// Timestamp envelope carried acoustically:
//   T<unix_time_us:16d>|<seq:4hex>|<P/H/W>|<holdover:8hex>|<application payload>
//
// Example:
//   T1784123000123456|003A|P|00000000|59.351034,18.068089
//
// unix_time_us is constructed from the UTC-labelled epoch plus the GPT2 OCXO
// counter. It is sampled immediately before formatting/sending the modem
// command. The remaining fixed command-to-emission latency must be calibrated.
static bool send_timestamped_broadcast(const char *application_payload)
{
  if (application_payload == NULL || application_payload[0] == '\0') {
    return false;
  }

  uint64_t tx_time_us = 0;
  int64_t tx_epoch_second = 0;
  uint32_t tx_fraction_us = 0;

  if (!utc_timestamp_for_counter(
        OCXO_counter_us(),
        &tx_time_us,
        &tx_epoch_second,
        &tx_fraction_us)) {
    maybe_emit_tx_warning(TX_WARN_NO_UTC);
    return false;
  }

  (void)tx_epoch_second;
  (void)tx_fraction_us;

  char payload[MODEM_PAYLOAD_MAX + 1];
  const int payload_len = snprintf(
    payload,
    sizeof(payload),
    "T%016llu|%04X|%c|%08lX|%s",
    (unsigned long long)tx_time_us,
    (unsigned int)tx_sequence,
    timing_mode_code(timing_mode),
    (unsigned long)holdover_age_s,
    application_payload
  );

  if (payload_len < 1 ||
      (size_t)payload_len > MODEM_PAYLOAD_MAX ||
      (size_t)payload_len >= sizeof(payload)) {
    maybe_emit_tx_warning(TX_WARN_PAYLOAD_TOO_LONG);
    return false;
  }

  char cmd[96];
  const int cmd_len = snprintf(
    cmd,
    sizeof(cmd),
    "$B%02u%s",
    (unsigned int)payload_len,
    payload
  );

  if (cmd_len < 0 || (size_t)cmd_len >= sizeof(cmd)) {
    maybe_emit_tx_warning(TX_WARN_PAYLOAD_TOO_LONG);
    return false;
  }

  modem_send_command_no_terminator(cmd);
  tx_sequence++;
  maybe_emit_tx_warning(TX_WARN_NONE);
  return true;
}

static bool send_latest_gps_broadcast()
{
  if (cfg.mode != MODE_TRANSMITTER ||
      !latest_gps_valid ||
      latest_gps_len < 2) {
    return false;
  }

  return send_timestamped_broadcast(latest_gps);
}

static bool update_latest_telemetry_from_host(const char *line)
{
  if (cfg.mode != MODE_TRANSMITTER) {
    return false;
  }

  if (!starts_with(line, "$K")) {
    return false;
  }

  // User payload follows the "$K" prefix. The broadcast payload is the
  // "TEL:" marker plus this user data, so its total length is
  // (un + TELEMETRY_MARKER_LEN).
  const char *user = line + 2;

  size_t un = strlen(user);
  if (un < 1 || (un + TELEMETRY_MARKER_LEN) > GPS_MAX) {
#if ENABLE_USB_DEBUG
    Serial.print("WARN: telemetry length invalid: ");
    Serial.println(un);
#endif
    return true;
  }

  memcpy(latest_telem, TELEMETRY_MARKER, TELEMETRY_MARKER_LEN);
  memcpy(latest_telem + TELEMETRY_MARKER_LEN, user, un);
  latest_telem[TELEMETRY_MARKER_LEN + un] = '\0';
  latest_telem_len = TELEMETRY_MARKER_LEN + un;

  latest_telem_epoch_count = epoch_count;
  latest_telem_valid = true;
  tx_last_warning = TX_WARN_NONE;

#if ENABLE_USB_DEBUG
  Serial.print("Telemetry updated: ");
  Serial.println(latest_telem);
#endif

  return true;
}

static bool send_latest_telemetry_broadcast()
{
  if (cfg.mode != MODE_TRANSMITTER ||
      !latest_telem_valid ||
      latest_telem_len < 2) {
    return false;
  }

  return send_timestamped_broadcast(latest_telem);
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

// Extracts the data payload from a received broadcast/unicast modem line and
// reports whether it is a "timed" payload, i.e. one we should emit #I for.
// A payload is timed when it either looks like raw GPS coordinates
// ("<lat>,<lon>") or is a telemetry frame (starts with the "TEL:" marker).
struct TimedPayloadInfo {
  bool timestamped;
  char source_id[4];
  uint64_t tx_time_us;
  uint16_t sequence;
  char tx_timing_mode;
  uint32_t tx_holdover_age_s;
  char application_payload[GPS_MAX + 1];
};

// Explicit prototypes required for Arduino's .ino preprocessor. The parser
// otherwise places generated prototypes before TimedPayloadInfo is declared.
static bool parse_timestamp_envelope(
  const char *raw_payload,
  TimedPayloadInfo *out);

static bool extract_timed_payload_from_modem_line(
  const char *line,
  TimedPayloadInfo *out);

static bool parse_timestamp_envelope(
  const char *raw_payload,
  TimedPayloadInfo *out)
{
  if (raw_payload == NULL || out == NULL || raw_payload[0] != 'T') {
    return false;
  }

  char *end = NULL;
  const unsigned long long tx_us = strtoull(raw_payload + 1, &end, 10);
  if (end == raw_payload + 1 || end == NULL || *end != '|') {
    return false;
  }

  const char *seq_start = end + 1;
  const unsigned long seq = strtoul(seq_start, &end, 16);
  if (end == seq_start || end == NULL || *end != '|' || seq > 0xFFFFUL) {
    return false;
  }

  const char *mode_start = end + 1;
  if ((mode_start[0] != 'P' && mode_start[0] != 'H' && mode_start[0] != 'W') ||
      mode_start[1] != '|') {
    return false;
  }

  const char *age_start = mode_start + 2;
  const unsigned long tx_holdover_age = strtoul(age_start, &end, 16);
  if (end == age_start || end == NULL || *end != '|') {
    return false;
  }

  const char *application = end + 1;
  if (!looks_like_gps_coords(application) &&
      !looks_like_telemetry(application)) {
    return false;
  }

  const size_t n = strlen(application);
  if (n >= sizeof(out->application_payload)) {
    return false;
  }

  out->timestamped = true;
  out->tx_time_us = (uint64_t)tx_us;
  out->sequence = (uint16_t)seq;
  out->tx_timing_mode = mode_start[0];
  out->tx_holdover_age_s = (uint32_t)tx_holdover_age;
  memcpy(out->application_payload, application, n + 1);
  return true;
}

static bool extract_timed_payload_from_modem_line(
  const char *line,
  TimedPayloadInfo *out)
{
  if (line == NULL || out == NULL) {
    return false;
  }

  memset(out, 0, sizeof(*out));
  strcpy(out->source_id, "000");
  char raw_payload[MODEM_PAYLOAD_MAX + 1] = {0};
  size_t n = 0;
  const char *payload_start = NULL;

  // Broadcast: #B<source:3><length:2><payload>...
  if (line[0] == '#' && line[1] == 'B') {
    if (!valid_3_digit_id(line + 2) ||
        !parse_two_digit_len(line + 5, &n)) {
      return false;
    }

    out->source_id[0] = line[2];
    out->source_id[1] = line[3];
    out->source_id[2] = line[4];
    out->source_id[3] = '\0';
    payload_start = line + 7;
  } else if (line[0] == '#' && line[1] == 'U') {
    // Unicast: #U<length:2><payload>...
    if (!parse_two_digit_len(line + 2, &n)) {
      return false;
    }
    payload_start = line + 4;
  } else {
    return false;
  }

  if (n > MODEM_PAYLOAD_MAX ||
      strlen(line) < (size_t)(payload_start - line) + n) {
    return false;
  }

  memcpy(raw_payload, payload_start, n);
  raw_payload[n] = '\0';

  if (parse_timestamp_envelope(raw_payload, out)) {
    return true;
  }

  // Backward-compatible legacy GPS/TEL payload. It still produces #I/#J but
  // cannot produce an absolute #OWTT row because TX time is absent.
  if (!looks_like_gps_coords(raw_payload) &&
      !looks_like_telemetry(raw_payload)) {
    return false;
  }

  out->timestamped = false;
  const size_t copy_n = strlen(raw_payload);
  if (copy_n >= sizeof(out->application_payload)) {
    return false;
  }
  memcpy(out->application_payload, raw_payload, copy_n + 1);
  return true;
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

static void emit_owtt_csv_header()
{
  host_send_line(
    "#OWTT_HEADER,seq,src,tx_us,rx_us,tof_us,delta_us,range_mm,"
    "rx_mode,rx_holdover_age_s,rx_gnss_age_ms,tx_mode,"
    "tx_holdover_age_s,decode_delay_ms,lat,lon"
  );
}

static bool split_gps_payload(
  const char *payload,
  char *lat, size_t lat_size,
  char *lon, size_t lon_size)
{
  if (payload == NULL || lat == NULL || lon == NULL ||
      lat_size == 0 || lon_size == 0 || !looks_like_gps_coords(payload)) {
    return false;
  }

  const char *comma = strchr(payload, ',');
  if (comma == NULL) {
    return false;
  }

  const size_t lat_len = (size_t)(comma - payload);
  const size_t lon_len = strlen(comma + 1);
  if (lat_len >= lat_size || lon_len >= lon_size) {
    return false;
  }

  memcpy(lat, payload, lat_len);
  lat[lat_len] = '\0';
  memcpy(lon, comma + 1, lon_len + 1);
  return true;
}

static void maybe_emit_receiver_delta_after_modem_line(const char *line)
{
  if (cfg.mode != MODE_RECEIVER) {
    return;
  }

  TimedPayloadInfo payload_info = {};
  if (!extract_timed_payload_from_modem_line(line, &payload_info)) {
    return;
  }

  int32_t delta_us = 0;
  bool rx_utc_valid = false;
  int64_t rx_utc_second = 0;
  uint32_t rx_holdover_age_s = 0;
  uint32_t rx_gnss_age_ms = 0;
  char rx_timing_mode = 'W';
  uint32_t decode_delay_ms = 0;
  char info[32];

  if (!consume_delta(
        &delta_us,
        &rx_utc_valid,
        &rx_utc_second,
        &rx_holdover_age_s,
        &rx_gnss_age_ms,
        &rx_timing_mode,
        &decode_delay_ms)) {
    host_send_line("#INA");
    return;
  }

  snprintf(info, sizeof(info), "#I%ld", (long)delta_us);
  host_send_line(info);

  if (EMIT_RX_UTC_LINE && rx_utc_valid) {
    char utc_info[64];
    snprintf(
      utc_info,
      sizeof(utc_info),
      "#J%lld,%ld",
      (long long)rx_utc_second,
      (long)delta_us
    );
    host_send_line(utc_info);
  }

  if (!payload_info.timestamped || !rx_utc_valid) {
    return;
  }

  const int64_t rx_time_us =
    rx_utc_second * (int64_t)EPOCH_US + (int64_t)delta_us;
  const int64_t raw_tof_us =
    rx_time_us - (int64_t)payload_info.tx_time_us;
  const int64_t calibrated_tof_us =
    raw_tof_us - (int64_t)RANGE_FIXED_DELAY_US;
  const int64_t range_mm =
    (calibrated_tof_us * SOUND_SPEED_MM_PER_S) / 1000000LL;

  char lat[24] = {0};
  char lon[24] = {0};
  split_gps_payload(
    payload_info.application_payload,
    lat, sizeof(lat),
    lon, sizeof(lon)
  );

  char row[320];
  snprintf(
    row,
    sizeof(row),
    "#OWTT,%u,%s,%llu,%lld,%lld,%ld,%lld,%c,%lu,%lu,%c,%lu,%lu,%s,%s",
    (unsigned int)payload_info.sequence,
    payload_info.source_id,
    (unsigned long long)payload_info.tx_time_us,
    (long long)rx_time_us,
    (long long)calibrated_tof_us,
    (long)delta_us,
    (long long)range_mm,
    rx_timing_mode,
    (unsigned long)rx_holdover_age_s,
    (unsigned long)rx_gnss_age_ms,
    payload_info.tx_timing_mode,
    (unsigned long)payload_info.tx_holdover_age_s,
    (unsigned long)decode_delay_ms,
    lat,
    lon
  );
  host_send_line(row);
}

// =======================================================
// ===================== HOST / MODEM LOGIC ==============
// =======================================================

static const char *timing_mode_name()
{
  switch (timing_mode) {
    case TIMING_WAIT_PPS:
      return "WAIT_PPS";
    case TIMING_PPS_LOCKED:
      return "PPS_LOCKED";
    case TIMING_HOLDOVER:
      return "HOLDOVER";
  }

  return "UNKNOWN";
}

static void emit_gnss_status(bool include_last_nmea)
{
  const uint32_t now_ms = millis();
  const bool have_rx = gnss_rx_byte_count != 0;
  const uint32_t rx_age_ms =
    have_rx ? (uint32_t)(now_ms - gnss_last_rx_ms) : 0;
  const bool activity_fresh =
    have_rx && rx_age_ms <= GNSS_ACTIVITY_FRESH_MS;

  char msg[256];
  snprintf(
    msg,
    sizeof(msg),
    "#GNSS,%s,uart_baud=%lu,usb_open=%u,rx_bytes=%lu,tx_bytes=%lu,"
    "rx_age_ms=%lu,ubx_ok=%lu,ubx_bad=%lu,timeutc=%lu,nmea=%lu,"
    "timing=%s,utc_valid=%u,epoch_unix_s=%lld,holdover_age_s=%lu",
    !have_rx ? "NO_DATA" : (activity_fresh ? "ACTIVE" : "STALE"),
    (unsigned long)gnss_uart_baud,
    (bool)GNSS_USB_SERIAL ? 1U : 0U,
    (unsigned long)gnss_rx_byte_count,
    (unsigned long)gnss_tx_byte_count,
    (unsigned long)rx_age_ms,
    (unsigned long)gnss_ubx_ok_count,
    (unsigned long)gnss_ubx_bad_checksum_count,
    (unsigned long)gnss_timeutc_count,
    (unsigned long)gnss_nmea_line_count,
    timing_mode_name(),
    current_epoch_utc_valid ? 1U : 0U,
    (long long)current_epoch_utc_second,
    (unsigned long)holdover_age_s
  );
  host_send_line(msg);

  if (include_last_nmea && gnss_last_nmea[0] != '\0') {
    char nmea_msg[GNSS_NMEA_MAX + 48];
    const uint32_t nmea_age_ms =
      (uint32_t)(now_ms - gnss_last_nmea_ms);
    snprintf(
      nmea_msg,
      sizeof(nmea_msg),
      "#GNSS,NMEA,age_ms=%lu,%s",
      (unsigned long)nmea_age_ms,
      gnss_last_nmea
    );
    host_send_line(nmea_msg);
  }
}

static bool handle_gnss_debug_command(const char *line)
{
  if (line == NULL) {
    return false;
  }

  if (strcmp(line, "$ZGNSS?") == 0) {
    emit_gnss_status(true);
    return true;
  }

  if (strcmp(line, "$ZGNSSDEBUG=1") == 0) {
    gnss_live_debug_enabled = true;
    gnss_next_debug_ms = millis();
    host_send_line("#GNSS,DEBUG,ON");
    return true;
  }

  if (strcmp(line, "$ZGNSSDEBUG=0") == 0) {
    gnss_live_debug_enabled = false;
    host_send_line("#GNSS,DEBUG,OFF");
    return true;
  }

  static const char baud_prefix[] = "$ZGNSSBAUD=";
  if (starts_with(line, baud_prefix)) {
    const char *value_text = line + strlen(baud_prefix);
    char *end = NULL;
    const unsigned long requested = strtoul(value_text, &end, 10);

    if (value_text[0] == '\0' || end == NULL || *end != '\0' ||
        requested < 1200UL || requested > 3000000UL) {
      host_send_line("#GNSS,BAUD,ERROR");
      return true;
    }

    GNSS_SERIAL.flush();
    GNSS_SERIAL.end();
    GNSS_SERIAL.begin((uint32_t)requested, SERIAL_8N1);
    gnss_uart_baud = (uint32_t)requested;
    ubx_parser_reset();

    // Re-send the RAM-only TIMEUTC output configuration at the newly selected
    // baud. This does not change the X20P's own stored UART baud.
    gnss_timeutc_config_sent = false;
    gnss_timeutc_config_due_ms = millis() + 200UL;

    char response[48];
    snprintf(
      response,
      sizeof(response),
      "#GNSS,BAUD,%lu",
      requested
    );
    host_send_line(response);
    return true;
  }

  return false;
}

static void maybe_emit_gnss_live_debug()
{
  if (!gnss_live_debug_enabled) {
    return;
  }

  const uint32_t now_ms = millis();
  if ((int32_t)(now_ms - gnss_next_debug_ms) < 0) {
    return;
  }

  gnss_next_debug_ms = now_ms + GNSS_DEBUG_PERIOD_MS;
  emit_gnss_status(false);
}

static bool handle_gnss_utc_query(const char *line)
{
  if (line == NULL || strcmp(line, "$ZUTC?") != 0) {
    return false;
  }

  if (!gnss_utc.valid && !current_epoch_utc_valid) {
    host_send_line("#UTC,NA,timing=WAIT_PPS");
    return true;
  }

  const uint32_t age_ms = gnss_utc.valid
    ? (uint32_t)(millis() - gnss_utc.received_ms)
    : 0;

  const char *status = "STALE";
  if (current_epoch_utc_valid && timing_mode == TIMING_HOLDOVER) {
    status = "HOLDOVER";
  } else if (gnss_utc_is_fresh()) {
    status = "GNSS";
  }

  char msg[224];
  snprintf(
    msg,
    sizeof(msg),
    "#UTC,%s,timing=%s,epoch_valid=%u,epoch_unix_s=%lld,"
    "last_gnss_unix_s=%lld,last_gnss_age_ms=%lu,tAcc_ns=%lu,nano=%ld,"
    "holdover_age_s=%lu",
    status,
    timing_mode_name(),
    current_epoch_utc_valid ? 1U : 0U,
    (long long)current_epoch_utc_second,
    (long long)(gnss_utc.valid ? gnss_utc.unix_second : 0),
    (unsigned long)age_ms,
    (unsigned long)(gnss_utc.valid ? gnss_utc.tAcc_ns : 0),
    (long)(gnss_utc.valid ? gnss_utc.nano_ns : 0),
    (unsigned long)holdover_age_s
  );

  host_send_line(msg);
  return true;
}

static bool handle_experiment_command(const char *line)
{
  if (line == NULL) {
    return false;
  }

  if (strcmp(line, "$ZPAYLOADTTL?") == 0) {
    char msg[64];
    snprintf(
      msg,
      sizeof(msg),
      "#PAYLOADTTL,%lu",
      (unsigned long)payload_valid_epochs
    );
    host_send_line(msg);
    return true;
  }

  static const char ttl_prefix[] = "$ZPAYLOADTTL=";
  if (starts_with(line, ttl_prefix)) {
    const char *value_text = line + strlen(ttl_prefix);
    char *end = NULL;
    const unsigned long requested = strtoul(value_text, &end, 10);
    if (value_text[0] == '\0' || end == NULL || *end != '\0') {
      host_send_line("#PAYLOADTTL,ERROR");
      return true;
    }

    payload_valid_epochs = (uint32_t)requested;
    latest_gps_epoch_count = epoch_count;
    latest_telem_epoch_count = epoch_count;
    char msg[64];
    snprintf(msg, sizeof(msg), "#PAYLOADTTL,%lu", requested);
    host_send_line(msg);
    return true;
  }

  if (strcmp(line, "$ZTXWARN?") == 0) {
    host_send_line(tx_warning_enabled ? "#TXWARN,ON" : "#TXWARN,OFF");
    return true;
  }

  if (strcmp(line, "$ZTXWARN=1") == 0) {
    tx_warning_enabled = true;
    tx_last_warning = TX_WARN_NONE;
    host_send_line("#TXWARN,ON");
    return true;
  }

  if (strcmp(line, "$ZTXWARN=0") == 0) {
    tx_warning_enabled = false;
    tx_last_warning = TX_WARN_NONE;
    host_send_line("#TXWARN,OFF");
    return true;
  }

  if (strcmp(line, "$ZOWTTHEADER?") == 0) {
    emit_owtt_csv_header();
    return true;
  }

  return false;
}

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

    if (handle_gnss_debug_command(line)) {
      return;
    }

    if (handle_gnss_utc_query(line)) {
      return;
    }

    if (handle_experiment_command(line)) {
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

    if (update_latest_telemetry_from_host(line)) {
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

  // Telemetry takes precedence: if the host has pushed a fresh telemetry
  // frame, broadcast that (it can itself carry position). Otherwise fall back
  // to the legacy GPS-only broadcast.
  if (send_latest_telemetry_broadcast()) {
    return;
  }

  if (send_latest_gps_broadcast()) {
    return;
  }

  if (!latest_telem_valid && !latest_gps_valid) {
    maybe_emit_tx_warning(TX_WARN_NO_PAYLOAD);
  }
}

// =======================================================
// ===================== GNSS USB BRIDGE =================
// =======================================================

static void gnss_send_ubx(
  uint8_t msg_class,
  uint8_t msg_id,
  const uint8_t *payload,
  uint16_t payload_len)
{
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;

  auto add_checksum = [&ck_a, &ck_b](uint8_t b) {
    ck_a = (uint8_t)(ck_a + b);
    ck_b = (uint8_t)(ck_b + ck_a);
  };

  GNSS_SERIAL.write((uint8_t)0xB5);
  GNSS_SERIAL.write((uint8_t)0x62);

  GNSS_SERIAL.write(msg_class);
  add_checksum(msg_class);

  GNSS_SERIAL.write(msg_id);
  add_checksum(msg_id);

  const uint8_t len_l = (uint8_t)(payload_len & 0xFFU);
  const uint8_t len_h = (uint8_t)(payload_len >> 8);

  GNSS_SERIAL.write(len_l);
  add_checksum(len_l);

  GNSS_SERIAL.write(len_h);
  add_checksum(len_h);

  for (uint16_t i = 0; i < payload_len; i++) {
    const uint8_t b = payload[i];
    GNSS_SERIAL.write(b);
    add_checksum(b);
  }

  GNSS_SERIAL.write(ck_a);
  GNSS_SERIAL.write(ck_b);
  GNSS_SERIAL.flush();
}

static void gnss_enable_timeutc_uart1_ram()
{
  // UBX-CFG-VALSET version 0, RAM layer only.
  //
  // Key:
  //   CFG-MSGOUT-UBX_NAV_TIMEUTC_UART1 = 0x2091005C
  // Value:
  //   1 = emit once per navigation epoch
  //
  // Key and value are encoded little-endian.
  static const uint8_t payload[] = {
    0x00,             // version
    0x01,             // layers: RAM
    0x00, 0x00,       // reserved
    0x5C, 0x00, 0x91, 0x20, // key 0x2091005C
    0x01              // U1 value
  };

  gnss_send_ubx(0x06, 0x8A, payload, sizeof(payload));
}

static void maybe_send_gnss_timeutc_config()
{
  if (!GNSS_AUTO_ENABLE_TIMEUTC || gnss_timeutc_config_sent) {
    return;
  }

  if ((int32_t)(millis() - gnss_timeutc_config_due_ms) < 0) {
    return;
  }

  gnss_enable_timeutc_uart1_ram();
  gnss_timeutc_config_sent = true;
}

static void maybe_follow_gnss_usb_baud()
{
  if (!GNSS_FOLLOW_USB_BAUD || !(bool)GNSS_USB_SERIAL) {
    return;
  }

  const uint32_t requested_baud = GNSS_USB_SERIAL.baud();

  if (requested_baud < 1200UL ||
      requested_baud > 3000000UL ||
      requested_baud == gnss_uart_baud) {
    return;
  }

  GNSS_SERIAL.flush();
  GNSS_SERIAL.end();
  GNSS_SERIAL.begin(requested_baud, SERIAL_8N1);

  gnss_uart_baud = requested_baud;
  ubx_parser_reset();
}

static void service_gnss_bridge()
{
  maybe_follow_gnss_usb_baud();

  // PC / u-center -> X20P.
  while (GNSS_USB_SERIAL.available() > 0 &&
         GNSS_SERIAL.availableForWrite() > 0) {
    const int b = GNSS_USB_SERIAL.read();
    if (b >= 0) {
      GNSS_SERIAL.write((uint8_t)b);
      gnss_tx_byte_count++;
    }
  }

  // X20P -> PC / u-center. Every byte is first observed by the passive UBX
  // parser, then forwarded unchanged. If the second USB port is open but its
  // TX buffer is temporarily full, stop reading Serial2 so bytes are not
  // deliberately discarded from the transparent bridge.
  const bool gnss_usb_open = (bool)GNSS_USB_SERIAL;

  while (GNSS_SERIAL.available() > 0) {
    if (gnss_usb_open && GNSS_USB_SERIAL.availableForWrite() <= 0) {
      break;
    }

    const int b = GNSS_SERIAL.read();
    if (b < 0) {
      break;
    }

    gnss_rx_byte_count++;
    gnss_last_rx_ms = millis();
    gnss_nmea_feed((uint8_t)b);
    gnss_ubx_feed((uint8_t)b);

    if (gnss_usb_open) {
      GNSS_USB_SERIAL.write((uint8_t)b);
    }
  }
}

// =======================================================
// ===================== SETUP / LOOP ====================
// =======================================================

void setup()
{
  // First USB CDC port: LoLo / Succorfish protocol.
  HOST_SERIAL.begin(HOST_BAUD);

  // Second USB CDC port: transparent X20P serial bridge.
  GNSS_USB_SERIAL.begin(GNSS_INITIAL_BAUD);

  MODEM_SERIAL.begin(MODEM_BAUD, SERIAL_8N1);
  GNSS_SERIAL.begin(GNSS_INITIAL_BAUD, SERIAL_8N1);
  gnss_uart_baud = GNSS_INITIAL_BAUD;

  pinMode(PIN_OCXO, INPUT);
  pinMode(PIN_RXS_CAPTURE, INPUT);
  pinMode(PIN_PPS_CAPTURE, INPUT);

  gpt2_extclk_capture_init_1mhz();

  // Give the X20P time to boot before enabling UBX-NAV-TIMEUTC.
  gnss_timeutc_config_due_ms = millis() + 1000UL;
}

void loop()
{
  char line[LINE_MAX];

  // Keep the X20P bridge responsive and passively parse UTC messages.
  service_gnss_bridge();
  maybe_send_gnss_timeutc_config();

  // First: latch timing events so delta_t is independent of serial delays.
  process_timing_events();

  check_pending_config_timeout();

  // Host commands get forwarded immediately and do not wait for PPS.
  while (read_line(HOST_SERIAL, host_reader, line, sizeof(line))) {
    handle_host_line(line);
  }

  // Modem messages are transparently forwarded first;
  // receiver then adds #I<delta_t> and, when available, #J<UTC,delta_t>.
  while (read_line(MODEM_SERIAL, modem_reader, line, sizeof(line))) {
    handle_modem_line(line);
  }

  // Service the higher-bandwidth GNSS path again after line processing.
  service_gnss_bridge();

  // Optional human-readable GNSS diagnostics go only to the first USB
  // host port. The transparent GNSS USB port always remains byte-clean.
  maybe_emit_gnss_live_debug();

  // Automatic transmitter broadcast runs after host commands,
  // so manual host commands have priority over scheduled GPS TX.
  execute_auto_tx_if_due();
}