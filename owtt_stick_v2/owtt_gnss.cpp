// GNSS UART bridge and the UTC label on each epoch.

#include "owtt_state.h"
#include "owtt_io.h"
#include "owtt_clock.h"
#include "owtt_gnss.h"
#include "owtt_radio.h"

static uint16_t read_u16_le(const uint8_t *p);
static uint32_t read_u32_le(const uint8_t *p);
static int32_t read_i32_le(const uint8_t *p);
static int64_t days_from_civil(int32_t year, uint32_t month, uint32_t day);
static bool civil_utc_to_unix_second( uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, int64_t *out);
static void maybe_bind_gnss_utc_to_current_epoch();
static void handle_ubx_nav_timeutc(const uint8_t payload[20]);
static void gnss_ubx_feed(uint8_t b);
static void gnss_nmea_feed(uint8_t b);
static void emit_gnss_status(bool include_last_nmea);
static void gnss_send_ubx( uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len);
static void gnss_enable_timeutc_uart1_ram();
static void gnss_set_uart1_baud_ram(uint32_t baud);
static void gnss_set_local_baud(uint32_t baud);
static bool gnss_have_recent_good_frame(uint32_t now_ms);
static void maybe_follow_gnss_usb_baud();


// Extra HardwareSerial (Serial2) buffering for the X20P bridge.
// Default Teensy RX is tiny (~64 B). When USB TX backpressures, service_gnss_bridge()
// stops reading Serial2 so bytes aren't dropped on the USB side — but then Serial2's
// own RX overflows and corrupts UBX frames (checksum failures). Larger RX absorbs
// those stalls; larger TX absorbs RTCM bursts host -> X20P.
// (apply helper is defined later — must not be the first function in this .ino or
// Arduino's auto-prototypes are emitted before BridgeMode/LineReader exist.)
static uint8_t gnss_uart_rx_extra[8192];

static uint8_t gnss_uart_tx_extra[4096];

static uint32_t current_epoch_utc_sync_ms = 0;

static bool gnss_timeutc_config_sent = false;


// Passive GNSS activity/debug counters. These prove that bytes reach Serial2
// even when the stream is binary UBX and therefore unreadable in `screen`.
static uint32_t gnss_rx_byte_count = 0;

static uint32_t gnss_tx_byte_count = 0;

static uint32_t gnss_last_rx_ms = 0;

static uint32_t gnss_timeutc_count = 0;

// Bridge bytes dropped because the USB-side reader stalled (local parsing
// always continues; only forwarding is sacrificed).
static uint32_t gnss_bridge_drop_count = 0;

static uint32_t gnss_nmea_line_count = 0;

static uint32_t gnss_last_nmea_ms = 0;

static char gnss_last_nmea[GNSS_NMEA_MAX] = {0};


static bool gnss_live_debug_enabled = false;

static uint32_t gnss_next_debug_ms = 0;


static GnssNmeaObserver gnss_nmea = {false, 0, {0}};


// PPS↔UTC labelling, from GNSS data only (no host, no heuristics).
//
// The PPS edge train marks tops of UTC seconds; only the *name* of each
// second must come from NAV-TIMEUTC. A message describing nav-epoch instant
// T = unix_second + nano (exact regardless of u-blox civil rounding) reaches
// the parser d seconds later, at after_pps seconds past the newest PPS edge.
// If N is that edge's label:
//
//   N + after_pps = T + d   =>   N = floor(T - after_pps) + 1  for d in (0,1)
//
// Serial latency d is NOT guaranteed under 1 s here (a busy GNSS/USB bridge
// pushes TIMEUTC past the next edge), so no single sample can be trusted.
// But d is never negative — a message cannot precede the instant it
// describes — so latency only ever makes the implied label too SMALL by
// floor(d) whole seconds, never too large. Work in the PPS-tick-locked frame
// m = implied_label - epoch_count (constant for a given true labelling) and
// take the maximum m over recent samples: it equals the truth as long as one
// message per window arrives with sub-second latency.
//
// The tracker below feeds every TIMEUTC sample and:
//   - (re)binds only after UTC_BIND_CONFIRM_SAMPLES samples agree on max m
//     (a lone upward glitch can never bind or rebind),
//   - drops a max unseen for UTC_BIND_MAX_UNSEEN_SAMPLES samples (glitch, or
//     the latency floor shifted) and rebuilds from current samples,
//   - keeps auditing after binding, emitting #UTC,REBIND on any correction
//     instead of silently carrying a ±1 s error in every TOF until reboot.
static bool utc_track_have = false;

static int64_t utc_track_max_m = 0;

static uint8_t utc_track_max_count = 0;

static uint8_t utc_track_max_unseen = 0;


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


bool gnss_utc_is_fresh()
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

  const int32_t after_pps_ticks =
    (int32_t)(gnss_utc.received_counter_us - current_epoch_us);

  if (after_pps_ticks < 0) {
    return;
  }

  // GPT2 counts 10 MHz ticks; the bind math and the 2.5 s sanity cap are
  // in microseconds.
  const int64_t after_pps_us = (int64_t)ticks_to_us((uint32_t)after_pps_ticks);

  if (after_pps_us > (int64_t)UTC_BIND_MAX_AFTER_PPS_US) {
    return;
  }

  // implied = floor(T - after_pps) + 1, in µs. The ±1 µs from nano/1000
  // truncation is irrelevant against the whole-second decision.
  const int64_t nav_epoch_us =
    gnss_utc.unix_second * 1000000LL + (int64_t)(gnss_utc.nano_ns / 1000L);
  const int64_t implied_label =
    (nav_epoch_us - after_pps_us) / 1000000LL + 1LL;
  const int64_t m = implied_label - (int64_t)epoch_count;

  if (!utc_track_have || m > utc_track_max_m) {
    utc_track_have = true;
    utc_track_max_m = m;
    utc_track_max_count = 1;
    utc_track_max_unseen = 0;
  } else if (m == utc_track_max_m) {
    if (utc_track_max_count < 255U) {
      utc_track_max_count++;
    }
    utc_track_max_unseen = 0;
  } else {
    // Sample below the max: latency burst (harmless) — unless the max itself
    // never repeats, in which case it was the outlier.
    if (++utc_track_max_unseen > UTC_BIND_MAX_UNSEEN_SAMPLES) {
      utc_track_max_m = m;
      utc_track_max_count = 1;
      utc_track_max_unseen = 0;
    }
  }

  if (utc_track_max_count < UTC_BIND_CONFIRM_SAMPLES) {
    return;
  }

  const int64_t label = utc_track_max_m + (int64_t)epoch_count;

  if (!current_epoch_utc_valid) {
    current_epoch_utc_second = label;
    current_epoch_utc_valid = true;
    current_epoch_utc_sync_ms = millis();
    char msg[96];
    snprintf(
      msg,
      sizeof(msg),
      "#UTC,BOUND,epoch_unix_s=%lld,after_pps_us=%lld",
      (long long)current_epoch_utc_second,
      (long long)after_pps_us
    );
    host_send_line(msg);
    return;
  }

  if (label != current_epoch_utc_second) {
    const int64_t diff = label - current_epoch_utc_second;
    current_epoch_utc_second = label;
    current_epoch_utc_sync_ms = millis();
    char msg[64];
    snprintf(
      msg,
      sizeof(msg),
      "#UTC,REBIND,diff_s=%lld,epoch_unix_s=%lld",
      (long long)diff,
      (long long)current_epoch_utc_second
    );
    host_send_line(msg);
    return;
  }

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


static void gnss_ubx_feed(uint8_t b)
{
  uint8_t payload[20];
  const OwttUbxFeed ev = owtt_ubx_feed(b, payload);
  if (ev == OWTT_UBX_FRAME || ev == OWTT_UBX_TIMEUTC) {
    gnss_last_good_frame_ms = millis();
    if (ev == OWTT_UBX_TIMEUTC) {
      handle_ubx_nav_timeutc(payload);
    }
  }
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
      gnss_last_good_frame_ms = gnss_last_nmea_ms;
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
    "bridge_drops=%lu,timing=%s,utc_valid=%u,epoch_unix_s=%lld,"
    "holdover_age_s=%lu",
    !have_rx ? "NO_DATA" : (activity_fresh ? "ACTIVE" : "STALE"),
    (unsigned long)gnss_uart_baud,
    (bool)GNSS_USB_SERIAL ? 1U : 0U,
    (unsigned long)gnss_rx_byte_count,
    (unsigned long)gnss_tx_byte_count,
    (unsigned long)rx_age_ms,
    (unsigned long)owtt_ubx_ok_count(),
    (unsigned long)owtt_ubx_bad_count(),
    (unsigned long)gnss_timeutc_count,
    (unsigned long)gnss_nmea_line_count,
    (unsigned long)gnss_bridge_drop_count,
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


bool handle_gnss_debug_command(const char *line)
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
  gnss_uart_apply_extra_buffers();
  gnss_uart_baud = (uint32_t)requested;
    owtt_ubx_reset();
    gnss_last_good_frame_ms = 0;
    gnss_link_epoch_ms = millis();

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


void maybe_emit_gnss_live_debug()
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


bool handle_gnss_utc_query(const char *line)
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


static void gnss_set_uart1_baud_ram(uint32_t baud)
{
  // UBX-CFG-VALSET, RAM layer: CFG-UART1-BAUDRATE = 0x40520001 (U4).
  uint8_t payload[] = {
    0x00,             // version
    0x01,             // layers: RAM
    0x00, 0x00,       // reserved
    0x01, 0x00, 0x52, 0x40, // key 0x40520001
    (uint8_t)(baud & 0xFFU),
    (uint8_t)((baud >> 8) & 0xFFU),
    (uint8_t)((baud >> 16) & 0xFFU),
    (uint8_t)((baud >> 24) & 0xFFU)
  };

  gnss_send_ubx(0x06, 0x8A, payload, sizeof(payload));
}


static void gnss_set_local_baud(uint32_t baud)
{
  GNSS_SERIAL.flush();
  GNSS_SERIAL.end();
  GNSS_SERIAL.begin(baud, SERIAL_8N1);
  gnss_uart_apply_extra_buffers();
  gnss_uart_baud = baud;
  owtt_ubx_reset();
  // Frames decoded at the previous baud do not prove this listen window.
  gnss_last_good_frame_ms = 0;
  gnss_link_epoch_ms = millis();
}


static bool gnss_have_recent_good_frame(uint32_t now_ms)
{
  return gnss_last_good_frame_ms != 0 &&
         (uint32_t)(now_ms - gnss_last_good_frame_ms) <= GNSS_LINK_SILENT_MS;
}


// Boot / recovery configuration of the X20P UART1 link.
//
// Sequence: listen at the current probe baud until a framed UBX/NMEA proves
// the link, *then* command UART1 up to GNSS_TARGET_BAUD (RAM-only), follow
// locally, wait for framed traffic at the new rate, and enable TIMEUTC.
// Blindly jumping to 921600 before any decode caused RELINK spam when the
// X20P was still at 38400/115200 (VALSET never heard).
void maybe_send_gnss_boot_config()
{
  if (!GNSS_AUTO_ENABLE_TIMEUTC || gnss_timeutc_config_sent) {
    return;
  }

  const uint32_t now_ms = millis();
  if ((int32_t)(now_ms - gnss_timeutc_config_due_ms) < 0) {
    return;
  }

  if (!gnss_have_recent_good_frame(now_ms)) {
    return;
  }

  if (gnss_uart_baud != GNSS_TARGET_BAUD) {
    gnss_set_uart1_baud_ram(GNSS_TARGET_BAUD);
    gnss_set_local_baud(GNSS_TARGET_BAUD);
    // Let the receiver switch and its first frames at the new rate arrive
    // before sending the TIMEUTC enable.
    gnss_timeutc_config_due_ms = millis() + 250UL;
    return;
  }

  gnss_enable_timeutc_uart1_ram();
  gnss_timeutc_config_sent = true;
}


void maybe_recover_gnss_link()
{
  const uint32_t now_ms = millis();

  // Give boot/RELINK settling time before declaring the probe baud dead.
  if ((int32_t)(now_ms - gnss_timeutc_config_due_ms) < 0) {
    return;
  }

  // Require a recently *decoded* frame. A wrong baud (or TX stuck low) still
  // produces rx_bytes and would never look "silent". If nothing has decoded
  // yet in this listen window, age from gnss_link_epoch_ms — never from
  // millis()-since-boot (that made RELINK fire every loop after 3 s).
  const uint32_t ref_ms =
    (gnss_last_good_frame_ms != 0) ? gnss_last_good_frame_ms : gnss_link_epoch_ms;
  const uint32_t good_age_ms = (uint32_t)(now_ms - ref_ms);

  if (good_age_ms <= GNSS_LINK_SILENT_MS) {
    return;
  }

  // Walk factory → previously-provisioned → target, then repeat. Boot config
  // re-arms on the baud that finally yields framed traffic and bumps UART1
  // up to GNSS_TARGET_BAUD in RAM.
  uint32_t next_baud;
  if (gnss_uart_baud == GNSS_TARGET_BAUD) {
    next_baud = GNSS_INITIAL_BAUD;
  } else if (gnss_uart_baud == GNSS_INITIAL_BAUD) {
    next_baud = GNSS_FALLBACK_BAUD;
  } else {
    next_baud = GNSS_TARGET_BAUD;
  }

  gnss_set_local_baud(next_baud);
  gnss_last_rx_ms = now_ms;
  gnss_timeutc_config_sent = false;
  gnss_timeutc_config_due_ms = now_ms + 500UL;

  char msg[48];
  snprintf(msg, sizeof(msg), "#GNSS,RELINK,%lu", (unsigned long)next_baud);
  host_send_line(msg);
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
  gnss_uart_apply_extra_buffers();

  gnss_uart_baud = requested_baud;
  owtt_ubx_reset();
  gnss_last_good_frame_ms = 0;
  gnss_link_epoch_ms = millis();
}


void gnss_uart_apply_extra_buffers()
{
  GNSS_SERIAL.addMemoryForRead(gnss_uart_rx_extra, sizeof(gnss_uart_rx_extra));
  GNSS_SERIAL.addMemoryForWrite(gnss_uart_tx_extra, sizeof(gnss_uart_tx_extra));
}


void service_gnss_bridge()
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

  // X20P -> PC / u-center. Every byte is ALWAYS consumed by the local
  // NMEA/UBX parsers — PPS↔UTC labelling must never starve. Forwarding to the
  // USB bridge is best-effort: if the host reader stalls (driver restart,
  // hung process), bridge bytes are dropped and counted instead of blocking
  // the timing core. A stalled reader previously froze UTC parsing entirely,
  // and the delayed TIMEUTC burst afterwards could mislabel epochs by whole
  // seconds (observed as multi-second TOF outliers).
  const bool gnss_usb_open = (bool)GNSS_USB_SERIAL;

  while (GNSS_SERIAL.available() > 0) {
    const int b = GNSS_SERIAL.read();
    if (b < 0) {
      break;
    }

    gnss_rx_byte_count++;
    gnss_last_rx_ms = millis();
    gnss_nmea_feed((uint8_t)b);
    gnss_ubx_feed((uint8_t)b);

    if (gnss_usb_open) {
      if (GNSS_USB_SERIAL.availableForWrite() > 0) {
        GNSS_USB_SERIAL.write((uint8_t)b);
      } else {
        gnss_bridge_drop_count++;
      }
    }
  }
}

