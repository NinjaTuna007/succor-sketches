// Protocol and timing notes for owtt_stick_v2.
// The sketch tab is owtt_stick_v2.ino. This behavior lives in
// owtt_io, owtt_clock, owtt_gnss, and owtt_radio.
// The e-ink panel is owtt_epd. Status also goes out on the host serial lines.
//
// Teensy 4.1 bridge between Host (LoLo or Frigates) and Succorfish modem.
//
// ver2 adds a telemetry frame on top of ver1's GPS broadcast:
//   - Host command "$K<nn><payload>" stores opaque telemetry (length-prefixed
//     so the payload may contain CR/LF/NUL). ASCII "$K<payload>\\r\\n" still
//     works when the two bytes after "$K" are not digits.
//   - The Teensy shunts those bytes on air (timestamp envelope + payload).
//     It does not prefix TEL: — DCCL id (or ASCII lat,lon) is the type.
//   - #I is emitted for any timestamped broadcast once UTC is bound, GPS or
//     opaque. Decoding the application payload is the host's job.
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
//   $K<nn><payload>
//      Updates internal latest telemetry variable only. nn is the two-digit
//      byte length of <payload> (same rule as $B). The payload may contain
//      any byte (DCCL, CRC-8, CR/LF). Not forwarded to the modem.
//      "$K" is used (not "$T") because "$T" is an existing Succorfish modem
//      command. Bytes are stored and broadcast as-is (no TEL: prefix).
//      ASCII "$K<payload>\\r\\n" is still accepted when the two bytes after
//      "$K" are not digits.
//      When telemetry is valid it takes precedence over GPS for the
//      scheduled broadcast (see below).
//
// Automatic transmitter broadcast:
//   $Bnn<envelope><lat>,<lon>     (GPS frame, when no telemetry is set)
//   $Bnn<envelope><payload>       (telemetry / DCCL, when telemetry is set)
//      Sent to modem on scheduled PPS/holdover epoch.
//      nn is the byte length of the broadcast payload (the bytes after nn).
//
// Receiver telemetry handling:
//   Any timestamped #B/#U (GPS ASCII or opaque $K bytes) is forwarded to the
//   host, then #I<tof_us> is emitted when UTC is bound. Decoding DCCL / ASCII
//   is the host's responsibility. #I does not require a TEL: marker.
//
// Receiver timing:
//   - External 10 MHz OCXO clock into pin 14.
//   - PPS into pin 40 for synchronization of vehicles.
//   - RxS flag into pin 15 for acoustic packet/header detection.
//   - GPT2 counts the raw 10 MHz OCXO (1 tick = 0.1 us). The true tick rate is
//     measured against PPS (see OCXO RATE CALIBRATION) because the oscillator's
//     calibration tolerance alone is enough to dominate holdover error.
//   - If PPS disappears, GPT2 compare keeps virtual 1-second epochs
//     using the OCXO holdover clock, advanced at the measured rate.
//
// Receiver output order (after a timed modem line, when UTC is valid):
//   1. Forward the raw modem frame to the host as bytes + CRLF, e.g.
//      #Bxxxnndd...
//      #B00722<lat>,<lon>
//      (payload may contain CR/LF; the host ROS driver length-prefixes #B/#U)
//   2. Optional UTC-labelled capture residual (EMIT_RX_UTC_LINE):
//        #J<rx_utc_unix_s>,<delta_us>
//   3. Absolute one-way travel time (TX stamp -> RX capture), for ROS ranging:
//        #I<tof_us>
//   4. Full CSV context row for bags / post-analysis:
//        #OWTT,<seq>,<src>,<tx_us>,<rx_us>,<tof_us>,<delta_us>,...
//   Independently, one #TEMPMON line every few seconds from loop() (see below).
//   If UTC is missing or TOF is implausible, emit #INA / #E,I,... instead of
//   #I/#OWTT (see emit path after extract_timed_payload_from_modem_frame).
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
//   $ZPAYLOADTTL=1    Expire latest $G/$K after 1 PPS/holdover epoch (default).
//   $ZPAYLOADTTL=0    Keep the latest payload indefinitely (bench/debug only).
//   $ZPAYLOADTTL=N    Expire it after N PPS/holdover epochs.
//   $ZTXWARN=1/0      Enable/disable one-shot TX warnings (default off).
//   $ZOWTTHEADER?     Reprint the #OWTT CSV column header.
//   $ZTEMP?           Latest on-die TEMPMON reading as #TEMPMON.
//   $ZZEROSTART       Bench only. This instant is epoch zero on the OCXO
//                     counter, unix second 0, holdover ('H'). Pin-40 edges
//                     are ignored until $ZZEROSTART=OFF, so two boards can
//                     share the timebase even if one of them has PPS.
//                     $ZZEROSTART=<unix_s> uses that second.
//                     $ZZEROSTART? reports.
//
// OCXO holdover experiment:
//   $ZIGNOREPPSAFTER=0    Never ignore PPS (default; all normal sticks).
//                         Clears any armed deadline; next real PPS re-locks.
//   $ZIGNOREPPSAFTER=N    After N seconds from *this command* (not Teensy boot),
//                         stop accepting real PPS captures. last_real_pps_us
//                         then goes stale and timing drifts into OCXO holdover
//                         (PPS_TIMEOUT_TICKS later), as if the PPS wire were cut.
//                         Payloads are stamped 'H' and $ZUTC? reports HOLDOVER.
//                         Re-send =0 then =N to restart the countdown (e.g. ROS
//                         driver restart mid-run).
//   $ZIGNOREPPSAFTER?     Report the configured delay (seconds, 0 = disabled).
//
//   Either form of $ZIGNOREPPSAFTER= also refreshes the OCXO rate from every
//   PPS edge collected so far and echoes #OCXOCAL, so the rate carried into
//   holdover is the freshest available and is logged next to the arming line.
//   A rate pinned with $ZOCXOCAL=<ppb> is left untouched.
//
// OCXO rate calibration:
//   $ZOCXOCAL?        Report the current rate as
//                       #OCXOCAL,<N|M|F|S>,<ppb>,<ticks_per_s>,<samples>,<span_s>,<pairs>
//                     N = nominal (no measurement yet), M = measured from PPS,
//                     F = pinned by command, S = last measured ppb restored
//                     from EEPROM (ring still empty). ppb is the offset from
//                     nominal. A stored offset is kept until a new ring is
//                     tighter than that store plus datasheet aging (see
//                     OCXO_AGING_* below). A shorter ring does not replace a
//                     longer one while the store is still the better bound.
//   $ZOCXOCAL=AUTO    Resume estimating from PPS (default). Clears the ring and
//                     falls back to nominal until enough edges accumulate.
//                     Does not erase the stored ppb; the next boot still
//                     starts from S.
//   $ZOCXOCAL=<ppb>   Pin the rate to a known offset and stop estimating, e.g.
//                     from a previous dive. $ZOCXOCAL=0 pins nominal 1e7 ticks
//                     per second, reproducing the pre-calibration behaviour for
//                     A/B holdover comparisons.
//
//   Calibration needs roughly a minute of PPS lock before it improves on
//   nominal, and keeps improving until the ring fills (~8.5 min). #OCXOCAL is
//   emitted on the first measurement, whenever the rate moves by >=5 ppb, and
//   once more at the moment holdover is entered, so the applied rate is
//   recoverable from the bag.
//
//   Resolution scales with ring depth, since pairs straddle half the ring:
//   512 samples give a 256 s baseline and a 0.39 ppb grid, but the 64-sample
//   minimum gives only 32 s and 3.1 ppb. Any PPS outage clears the ring, so
//   check <samples> in #OCXOCAL before trusting a rate. Every #OWTT row also
//   carries the receiver's rate and state in rx_ocxo_ppb / rx_ocxo_cal, and
//   the on-die TEMPMON reading in rx_die_c.
//
// On-die temperature (i.MX RT1062 TEMPMON):
//   The Teensy 4.1 silicon has a factory-calibrated junction sensor, already
//   started by the core. It is the MCU die, not the OCXO crystal: useful for
//   tagging a holdover residual with enclosure / diurnal temperature, not for
//   subtracting a tempco. Polled from loop() every few seconds, never from
//   the PPS/epoch path: tempmonGetTemp() busy-waits on VALID (usually already
//   set; ~90 us worst case), and a USB emit can stall much longer than that.
//   A TeensyThreads worker is the wrong tool here — the e-ink thread exists
//   because display() blocks for ~13 s. Emitted as
//     #TEMPMON,<epoch>,<P|H>,<die_c>
//   so a 36-hour bag still has a temperature axis through acoustic dropouts.
//   $ZTEMP? reprints the latest reading.
//
//   $Z commands are handled even while a $Y modem config is pending, so the
//   host may send this right after the $Y receiver/transmitter config.
//
// Timestamped acoustic payload:
//   T<ss>|<seq:hex>|<P/H/W>[|<holdover:hex>]|<GPS-or-TEL-payload>
//
// ss = transmit UTC time-of-minute (seconds, 0-59). Broadcasts are epoch/PPS
// -aligned so the sub-second fraction is always ~0 and is omitted. Full unix
// seconds are redundant (propagation is only a few seconds); the receiver
// resolves the minute against its own UTC. holdover is emitted only when
// non-zero (transmitter free-running); locked transmitters omit the field.
//
// Notes:
//   - #I<delta_us> is the one-way travel time from the envelope TX stamp to
//     the local RX capture (absolute, can exceed 1e6 us).
//   - Ranging (sound velocity, offsets, calibration) is a ROS/host concern.
//   - For reactive round-robin, follower delay must be large enough for:
//       propagation_time + acoustic_packet_duration + guard
//     otherwise the follower may learn too late that its trigger leader spoke.
// $G11.2328,12.12385

#include "owtt_state.h"
#include "owtt_io.h"
#include "owtt_clock.h"
#include "owtt_gnss.h"
#include "owtt_radio.h"
#include "owtt_epd.h"

static void begin_pending_config( BridgeMode new_mode, const char new_own_id[4], const char new_listen_id[4], uint32_t new_period_pps, const char *original_cmd);
static void apply_pending_config();
static void abort_pending_config(const char *reason);
static bool parse_period_pps_strict(const char *p, uint32_t *out_period_pps);
static bool handle_config_command(char *line);
static bool update_latest_gps_from_host(const char *line);
static const char *tx_warning_name(TxWarningReason reason);
static void maybe_emit_tx_warning(TxWarningReason reason);
static bool send_timestamped_broadcast( const uint8_t *application_payload, size_t app_len);
static bool send_latest_gps_broadcast();
static bool update_latest_telemetry_from_host(const char *line);
static bool update_latest_telemetry_from_bytes(const uint8_t *user, size_t un);
static bool send_latest_telemetry_broadcast();
static bool extract_source_id_from_modem_line(const char *line, char *src_id);
static bool parse_timestamp_envelope( const char *raw_payload, size_t raw_n, TimedPayloadInfo *out);
static bool extract_timed_payload_from_modem_frame( const uint8_t *frame, size_t frame_len, TimedPayloadInfo *out);
static void maybe_schedule_round_robin_tx_from_modem_frame( const uint8_t *frame, size_t frame_len);
static void emit_owtt_csv_header();
static bool split_gps_payload( const char *payload, char *lat, size_t lat_size, char *lon, size_t lon_size);
static bool resolve_tx_time_us( uint8_t tx_ss, uint32_t tx_fraction_us, int64_t rx_time_us, int64_t *out_tx_time_us);
static void maybe_emit_receiver_delta_after_modem_frame( const uint8_t *frame, size_t frame_len);
static bool handle_experiment_command(const char *line);
static void handle_host_line(char *line);
static bool handle_pending_config_modem_line(const char *line);


static PendingConfig pending_cfg = {
  false, // active
  MODE_WIRE, // mode
  "000", // own_id
  "000", // listen_id
  1, // period_pps
  0, // deadline_ms
  {0} // original_cmd
};

// Last config outcome shown on the e-ink status page ("ok", abort reason, ...).
static char epd_cfg_line[20] = "ok";



// Latest GPS payload stored on transmitter.
// Payload is stored as raw "<lat>,<lon>", without "$G".
static char latest_gps[GPS_MAX + 1] = {0};


// Latest telemetry payload stored on transmitter.
// Opaque host bytes (DCCL Pose, BeaconTelemetry, …) with no Teensy prefix.
// The Teensy never parses them. When valid, telemetry takes precedence over
// GPS for the scheduled broadcast. payload_valid_epochs controls expiry;
// zero means the stored frame remains valid indefinitely for long experiments.
static char latest_telem[GPS_MAX + 1] = {0};


// Experiment transmission metadata and output policy.
static uint16_t tx_sequence = 0;

static bool tx_warning_enabled = false;


void schedule_first_transmitter_after_config()
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

  strncpy(epd_cfg_line, "pending", sizeof(epd_cfg_line) - 1);
  epd_cfg_line[sizeof(epd_cfg_line) - 1] = '\0';

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

  strncpy(epd_cfg_line, "ok", sizeof(epd_cfg_line) - 1);
  epd_cfg_line[sizeof(epd_cfg_line) - 1] = '\0';

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

  strncpy(epd_cfg_line, reason, sizeof(epd_cfg_line) - 1);
  epd_cfg_line[sizeof(epd_cfg_line) - 1] = '\0';

  host_send_config_error(pending_cfg.original_cmd, reason);
  pending_cfg.active = false;
  tx_auto_due = false;
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
//   T<ss>|<seq:hex>|<P/H/W>[|<holdover:hex>]|<application payload>
//
// Examples:
//   T04|003A|P|59.351034,18.068089          (PPS-locked, no holdover field)
//   T04|003A|H|0000000B|59.351034,18.068089 (holdover, 11 s)
//
// ss is the transmit UTC time-of-minute (seconds, 0-59); broadcasts are
// epoch/PPS-aligned so the sub-second fraction is always ~0 and is omitted.
// The receiver reconstructs the full absolute time with its own UTC clock
// (see resolve_tx_time_us). holdover is emitted only when non-zero (i.e. the
// transmitter is free-running); a locked transmitter omits the field.
static bool send_timestamped_broadcast(
  const uint8_t *application_payload, size_t app_len)
{
  if (application_payload == NULL || app_len < 1) {
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

  (void)tx_time_us;
  (void)tx_fraction_us;

  const int ss = (int)((tx_epoch_second % 60LL + 60LL) % 60LL);
  const char mode = timing_mode_code(timing_mode);

  char header[40];
  int header_len;
  if (holdover_age_s != 0) {
    header_len = snprintf(
      header,
      sizeof(header),
      "T%02d|%04X|%c|%08lX|",
      ss,
      (unsigned int)tx_sequence,
      mode,
      (unsigned long)holdover_age_s
    );
  } else {
    header_len = snprintf(
      header,
      sizeof(header),
      "T%02d|%04X|%c|",
      ss,
      (unsigned int)tx_sequence,
      mode
    );
  }

  if (header_len < 1 ||
      app_len < 1 ||
      (size_t)header_len + app_len > MODEM_PAYLOAD_MAX) {
    maybe_emit_tx_warning(TX_WARN_PAYLOAD_TOO_LONG);
    return false;
  }

  uint8_t payload[MODEM_PAYLOAD_MAX];
  memcpy(payload, header, (size_t)header_len);
  memcpy(payload + header_len, application_payload, app_len);
  const size_t payload_len = (size_t)header_len + app_len;

  uint8_t cmd[4 + MODEM_PAYLOAD_MAX];
  const int cmd_hdr = snprintf((char *)cmd, 5, "$B%02u", (unsigned int)payload_len);
  if (cmd_hdr != 4) {
    maybe_emit_tx_warning(TX_WARN_PAYLOAD_TOO_LONG);
    return false;
  }
  memcpy(cmd + 4, payload, payload_len);

  modem_send_bytes(cmd, 4 + payload_len);
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

  return send_timestamped_broadcast(
    (const uint8_t *)latest_gps, latest_gps_len);
}


static bool update_latest_telemetry_from_host(const char *line)
{
  if (cfg.mode != MODE_TRANSMITTER) {
    return false;
  }

  if (!starts_with(line, "$K")) {
    return false;
  }

  // User payload follows the "$K" prefix. Stored and broadcast as-is.
  const char *user = line + 2;

  size_t un = strlen(user);
  if (un < 1 || un > GPS_MAX) {
#if ENABLE_USB_DEBUG
    Serial.print("WARN: telemetry length invalid: ");
    Serial.println(un);
#endif
    return true;
  }

  memcpy(latest_telem, user, un);
  latest_telem[un] = '\0';
  latest_telem_len = un;

  latest_telem_epoch_count = epoch_count;
  latest_telem_valid = true;
  tx_last_warning = TX_WARN_NONE;

#if ENABLE_USB_DEBUG
  Serial.print("Telemetry updated: ");
  Serial.println(latest_telem);
#endif

  return true;
}


static bool update_latest_telemetry_from_bytes(const uint8_t *user, size_t un)
{
  if (cfg.mode != MODE_TRANSMITTER) {
    return false;
  }
  if (user == NULL || un < 1 || un > GPS_MAX) {
    return true;
  }

  memcpy(latest_telem, user, un);
  latest_telem[un] = 0;
  latest_telem_len = un;

  latest_telem_epoch_count = epoch_count;
  latest_telem_valid = true;
  tx_last_warning = TX_WARN_NONE;
  return true;
}


static bool send_latest_telemetry_broadcast()
{
  if (cfg.mode != MODE_TRANSMITTER ||
      !latest_telem_valid ||
      latest_telem_len < 2) {
    return false;
  }

  return send_timestamped_broadcast(
    (const uint8_t *)latest_telem, latest_telem_len);
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


static bool parse_timestamp_envelope(
  const char *raw_payload,
  size_t raw_n,
  TimedPayloadInfo *out)
{
  if (raw_payload == NULL || out == NULL || raw_n < 8 || raw_payload[0] != 'T') {
    return false;
  }

  // T<ss>|<seq:hex>|<mode>[|<holdover:hex>]|<application>
  // Application may be GPS ASCII or opaque bytes (DCCL). Do not require GPS.
  char *end = NULL;
  const unsigned long ss = strtoul(raw_payload + 1, &end, 10);
  if (end == raw_payload + 1 || end == NULL || *end != '|' || ss > 59UL) {
    return false;
  }
  if ((size_t)(end - raw_payload) >= raw_n) {
    return false;
  }

  const char *seq_start = end + 1;
  const unsigned long seq = strtoul(seq_start, &end, 16);
  if (end == seq_start || end == NULL || *end != '|' || seq > 0xFFFFUL) {
    return false;
  }
  if ((size_t)(end - raw_payload) >= raw_n) {
    return false;
  }

  const char *mode_start = end + 1;
  if ((size_t)(mode_start - raw_payload) + 2 > raw_n ||
      (mode_start[0] != 'P' && mode_start[0] != 'H' && mode_start[0] != 'W') ||
      mode_start[1] != '|') {
    return false;
  }

  const char *application = mode_start + 2;
  unsigned long tx_holdover_age = 0;
  if (mode_start[0] == 'H') {
    const char *age_start = application;
    const unsigned long age = strtoul(age_start, &end, 16);
    if (end == age_start || end == NULL || *end != '|') {
      return false;
    }
    if ((size_t)(end - raw_payload) >= raw_n) {
      return false;
    }
    tx_holdover_age = age;
    application = end + 1;
  }

  const size_t app_off = (size_t)(application - raw_payload);
  if (app_off >= raw_n) {
    return false;
  }
  const size_t app_n = raw_n - app_off;
  if (app_n < 1 || app_n >= sizeof(out->application_payload)) {
    return false;
  }

  out->timestamped = true;
  out->tx_ss = (uint8_t)ss;
  out->tx_fraction_us = 0;
  out->sequence = (uint16_t)seq;
  out->tx_timing_mode = mode_start[0];
  out->tx_holdover_age_s = (uint32_t)tx_holdover_age;
  memcpy(out->application_payload, application, app_n);
  out->application_payload[app_n] = '\0';
  return true;
}


static bool extract_timed_payload_from_modem_frame(
  const uint8_t *frame,
  size_t frame_len,
  TimedPayloadInfo *out)
{
  if (frame == NULL || out == NULL || frame_len < 4) {
    return false;
  }

  memset(out, 0, sizeof(*out));
  strcpy(out->source_id, "000");
  char raw_payload[MODEM_PAYLOAD_MAX + 1] = {0};
  size_t n = 0;
  size_t payload_off = 0;

  if (frame[0] == '#' && frame[1] == 'B') {
    if (frame_len < 7 ||
        !valid_3_digit_id((const char *)frame + 2) ||
        !parse_two_digit_len((const char *)frame + 5, &n)) {
      return false;
    }
    out->source_id[0] = (char)frame[2];
    out->source_id[1] = (char)frame[3];
    out->source_id[2] = (char)frame[4];
    out->source_id[3] = '\0';
    payload_off = 7;
  } else if (frame[0] == '#' && frame[1] == 'U') {
    if (!parse_two_digit_len((const char *)frame + 2, &n)) {
      return false;
    }
    payload_off = 4;
  } else {
    return false;
  }

  if (n > MODEM_PAYLOAD_MAX || frame_len < payload_off + n) {
    return false;
  }

  memcpy(raw_payload, frame + payload_off, n);
  raw_payload[n] = '\0';

  return parse_timestamp_envelope(raw_payload, n, out);
}


static void maybe_schedule_round_robin_tx_from_modem_frame(
  const uint8_t *frame, size_t frame_len)
{
  if (cfg.mode != MODE_TRANSMITTER) {
    return;
  }

  if (is_first_transmitter()) {
    return;
  }

  if (frame == NULL || frame_len < 5) {
    return;
  }

  char src_id[4];
  if (!extract_source_id_from_modem_line((const char *)frame, src_id)) {
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
    "#OWTT_HEADER,seq,src,tx_us,rx_us,tof_us,delta_us,"
    "rx_mode,rx_holdover_age_s,rx_gnss_age_ms,tx_mode,"
    "tx_holdover_age_s,decode_delay_ms,lat,lon,"
    "rx_ocxo_ppb,rx_ocxo_cal,rx_die_c"
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


// Resolve a compact envelope stamp (ss, fraction_us) against the local UTC
// minute. One-way travel time is at most a few seconds, so the TX minute is
// whichever candidate lands closest to the RX capture time (window ±30 s).
static bool resolve_tx_time_us(
  uint8_t tx_ss,
  uint32_t tx_fraction_us,
  int64_t rx_time_us,
  int64_t *out_tx_time_us)
{
  if (out_tx_time_us == NULL || tx_ss > 59U || tx_fraction_us > 999999UL) {
    return false;
  }

  const int64_t rx_second = rx_time_us / 1000000LL;
  const int64_t minute_start = (rx_second - (rx_second % 60LL)) / 60LL * 60LL;

  int64_t best_tx = 0;
  int64_t best_abs = 0;
  bool have_best = false;

  for (int m = -1; m <= 1; ++m) {
    const int64_t cand =
      (minute_start + (int64_t)m * 60LL + (int64_t)tx_ss) * 1000000LL +
      (int64_t)tx_fraction_us;
    const int64_t diff = cand > rx_time_us
      ? cand - rx_time_us
      : rx_time_us - cand;
    if (!have_best || diff < best_abs) {
      best_abs = diff;
      best_tx = cand;
      have_best = true;
    }
  }

  *out_tx_time_us = best_tx;
  return true;
}


static void maybe_emit_receiver_delta_after_modem_frame(
  const uint8_t *frame, size_t frame_len)
{
  // Only attempt OWTT on broadcast/unicast data frames.
  if (frame == NULL || frame_len < 2 ||
      frame[0] != '#' || (frame[1] != 'B' && frame[1] != 'U')) {
    return;
  }

  if (cfg.mode != MODE_RECEIVER) {
    host_send_line("#E,I,NOT_R");
    return;
  }

  TimedPayloadInfo payload_info = {};
  if (!extract_timed_payload_from_modem_frame(frame, frame_len, &payload_info)) {
    host_send_line("#E,I,PARSE");
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

  // Optional debug: which local epoch second + local delta the capture used.
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

  if (!rx_utc_valid) {
    host_send_line("#INA");
    return;
  }

  const int64_t rx_time_us =
    rx_utc_second * (int64_t)EPOCH_US + (int64_t)delta_us;

  int64_t tx_time_us = 0;
  if (!resolve_tx_time_us(
        payload_info.tx_ss,
        payload_info.tx_fraction_us,
        rx_time_us,
        &tx_time_us)) {
    host_send_line("#INA");
    return;
  }

  const int64_t tof_us = rx_time_us - tx_time_us;

  // Reject physically impossible travel times instead of emitting them. A
  // negative TOF or one beyond OWTT_MAX_PLAUSIBLE_TOF_US means the RxS
  // capture was paired with the wrong #B decode (mispairs are quantized in
  // whole TX periods) or a timing transient slipped through — surface it as
  // a diagnostic, never as ranging data.
  if (tof_us < 0 || tof_us > OWTT_MAX_PLAUSIBLE_TOF_US) {
    char err[48];
    snprintf(err, sizeof(err), "#E,I,TOF_IMPLAUSIBLE,%lld", (long long)tof_us);
    host_send_line(err);
    return;
  }

  // #I = absolute one-way travel time (TX stamp -> RX capture), in µs.
  // Unlike the old local-delta #I, this is not bounded by the 1 s epoch.
  snprintf(info, sizeof(info), "#I%lld", (long long)tof_us);
  host_send_line(info);

  char lat[24] = {0};
  char lon[24] = {0};
  split_gps_payload(
    payload_info.application_payload,
    lat, sizeof(lat),
    lon, sizeof(lon)
  );

  const int64_t tx_ss_time_us =
    (int64_t)payload_info.tx_ss * 1000000LL +
    (int64_t)payload_info.tx_fraction_us;

  char die[16];
  format_die_c(die, sizeof(die));

  char row[352];
  snprintf(
    row,
    sizeof(row),
    "#OWTT,%u,%s,%lld,%lld,%lld,%ld,%c,%lu,%lu,%c,%lu,%lu,%s,%s,%ld,%c,%s",
    (unsigned int)payload_info.sequence,
    payload_info.source_id,
    (long long)tx_ss_time_us,
    (long long)rx_time_us,
    (long long)tof_us,
    (long)delta_us,
    rx_timing_mode,
    (unsigned long)rx_holdover_age_s,
    (unsigned long)rx_gnss_age_ms,
    payload_info.tx_timing_mode,
    (unsigned long)payload_info.tx_holdover_age_s,
    (unsigned long)decode_delay_ms,
    lat,
    lon,
    (long)ocxo_rate_ppb,
    ocxo_cal_state_char(),
    die
  );
  host_send_line(row);
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

  if (strcmp(line, "$ZIGNOREPPSAFTER?") == 0) {
    char msg[64];
    snprintf(
      msg,
      sizeof(msg),
      "#IGNOREPPSAFTER,%lu",
      (unsigned long)ignore_pps_after_s
    );
    host_send_line(msg);
    return true;
  }

  static const char ignore_prefix[] = "$ZIGNOREPPSAFTER=";
  if (starts_with(line, ignore_prefix)) {
    const char *value_text = line + strlen(ignore_prefix);
    char *end = NULL;
    const unsigned long requested = strtoul(value_text, &end, 10);
    if (value_text[0] == '\0' || end == NULL || *end != '\0') {
      host_send_line("#IGNOREPPSAFTER,ERROR");
      return true;
    }

    if (requested == 0) {
      ignore_pps_after_s = 0;
      ignore_pps_deadline_ms = 0;
      ignore_pps_armed = false;
    } else {
      ignore_pps_after_s = (uint32_t)requested;
      ignore_pps_deadline_ms =
        millis() + (uint32_t)requested * 1000UL;
      ignore_pps_armed = true;
    }
    char msg[64];
    snprintf(msg, sizeof(msg), "#IGNOREPPSAFTER,%lu", requested);
    host_send_line(msg);

    // Pair the experiment's state change with the rate that will be carried
    // into holdover, so the bag records both together.
    ocxo_cal_refresh_now();
    return true;
  }

  if (strcmp(line, "$ZOCXOCAL?") == 0) {
    ocxo_cal_report();
    return true;
  }

  if (strcmp(line, "$ZOCXOCAL=AUTO") == 0) {
    ocxo_cal_auto = true;
    pps_cal_reset();
    ocxo_apply_rate((uint64_t)EPOCH_TICKS_NOM << 32, OCXO_CAL_NOMINAL);
    ocxo_cal_report();
    return true;
  }

  static const char cal_prefix[] = "$ZOCXOCAL=";
  if (starts_with(line, cal_prefix)) {
    const char *value_text = line + strlen(cal_prefix);
    char *end = NULL;
    const long requested = strtol(value_text, &end, 10);
    if (value_text[0] == '\0' || end == NULL || *end != '\0' ||
        requested > OCXO_RATE_MAX_PPB || requested < -OCXO_RATE_MAX_PPB) {
      host_send_line("#OCXOCAL,ERROR");
      return true;
    }

    // Pinning the rate also stops auto-estimation, so =0 reproduces the
    // original fixed-nominal behaviour exactly for A/B holdover runs.
    ocxo_cal_auto = false;
    pps_cal_reset();
    ocxo_apply_rate(ocxo_step_from_ppb((int32_t)requested), OCXO_CAL_MANUAL);
    ocxo_cal_report();
    return true;
  }

  if (strcmp(line, "$ZTEMP?") == 0) {
    if (!die_c_valid) {
      sample_die_temp();
    }
    if (die_c_valid) {
      emit_tempmon_line();
    } else {
      host_send_line("#TEMPMON,NA");
    }
    return true;
  }

  if (strcmp(line, "$ZOWTTHEADER?") == 0) {
    emit_owtt_csv_header();
    return true;
  }

  if (strcmp(line, "$ZZEROSTART?") == 0) {
    char msg[64];
    if (!bench_epoch_active) {
      host_send_line("#ZEROSTART,OFF");
    } else {
      snprintf(
        msg,
        sizeof(msg),
        "#ZEROSTART,H,%lld",
        (long long)current_epoch_utc_second
      );
      host_send_line(msg);
    }
    return true;
  }

  if (strcmp(line, "$ZZEROSTART=OFF") == 0) {
    bench_epoch_stop();
    host_send_line("#ZEROSTART,OFF");
    return true;
  }

  if (strcmp(line, "$ZZEROSTART") == 0 || starts_with(line, "$ZZEROSTART=")) {
    int64_t unix_second = 0;
    if (line[11] == '=') {
      const char *value_text = line + 12;
      char *end = NULL;
      unix_second = (int64_t)strtoll(value_text, &end, 10);
      if (value_text[0] == '\0' || end == NULL || *end != '\0' || unix_second < 0) {
        host_send_line("#ZEROSTART,ERROR");
        return true;
      }
    }
    bench_epoch_start(unix_second);
    char msg[64];
    snprintf(
      msg,
      sizeof(msg),
      "#ZEROSTART,H,%lld",
      (long long)unix_second
    );
    host_send_line(msg);
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

  // Still waiting for #A / E — do not consume unrelated modem traffic
  // (especially #B/#U), or OWTT ranging goes silent for the whole timeout.
  return false;
}


void check_pending_config_timeout()
{
  if (!pending_cfg.active) {
    return;
  }

  if ((int32_t)(millis() - pending_cfg.deadline_ms) >= 0) {
    abort_pending_config("ADDR_TIMEOUT");
  }
}


void handle_host_frame(const uint8_t *frame, size_t n)
{
  if (frame == NULL || n == 0) {
    return;
  }

  size_t payload_start = 0;
  size_t nbytes = 0;
  if (fa_host_tx_header(frame, n, &payload_start, &nbytes) == 1 &&
      payload_start + nbytes == n) {
    if (pending_cfg.active) {
      host_send_line("#E,Y,BUSY");
      return;
    }
    // $K is host-only (store telemetry). Do not write "$K..." to the modem.
    if (n >= 2 && frame[0] == '$' && frame[1] == 'K') {
      update_latest_telemetry_from_bytes(frame + payload_start, nbytes);
      return;
    }
    modem_send_bytes(frame, n);
    return;
  }

  char line[LINE_MAX];
  if (!frame_as_cstring(frame, n, line, sizeof(line))) {
    return;
  }
  handle_host_line(line);
}


void handle_modem_frame(const uint8_t *frame, size_t n)
{
  if (frame == NULL || n == 0) {
    return;
  }

#if ENABLE_USB_DEBUG
  Serial.print("Received from Succor: ");
  Serial.write(frame, n);
  Serial.println();
#endif

  host_send_frame(frame, n);

  char line[LINE_MAX];
  if (frame_as_cstring(frame, n, line, sizeof(line)) &&
      handle_pending_config_modem_line(line)) {
    return;
  }

  if (cfg.mode == MODE_WIRE) {
    return;
  }

  maybe_emit_receiver_delta_after_modem_frame(frame, n);
  maybe_schedule_round_robin_tx_from_modem_frame(frame, n);
}


void execute_auto_tx_if_due()
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


void sync_owtt_epd_status()
{
  OwttEpdStatus s = {};
  s.mode_char = mode_to_char(cfg.mode);
  copy_id3(s.own_id, cfg.own_id);
  s.timing_char = timing_mode_code(timing_mode);
  s.holdover_age_s = holdover_age_s;
  s.gnss_fresh = gnss_utc_is_fresh();
  s.gps_valid = latest_gps_valid;
  s.telem_valid = latest_telem_valid;
  s.cfg_pending = pending_cfg.active;
  strncpy(s.cfg_line, epd_cfg_line, sizeof(s.cfg_line) - 1);
  s.cfg_line[sizeof(s.cfg_line) - 1] = '\0';
  owtt_epd_set_status(s);
}
