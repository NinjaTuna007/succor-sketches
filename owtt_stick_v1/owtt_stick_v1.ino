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
//   - GPT2 counts the raw 10 MHz OCXO (1 tick = 0.1 us). The true tick rate is
//     measured against PPS (see OCXO RATE CALIBRATION) because the oscillator's
//     calibration tolerance alone is enough to dominate holdover error.
//   - If PPS disappears, GPT2 compare keeps virtual 1-second epochs
//     using the OCXO holdover clock, advanced at the measured rate.
//
// Receiver output order (after a timed GPS/TEL modem line, when UTC is valid):
//   1. Forward raw modem line to host, e.g.
//      #Bxxxnndd...
//      #B00722<lat>,<lon>
//   2. Optional UTC-labelled capture residual (EMIT_RX_UTC_LINE):
//        #J<rx_utc_unix_s>,<delta_us>
//   3. Absolute one-way travel time (TX stamp -> RX capture), for ROS ranging:
//        #I<tof_us>
//   4. Full CSV context row for bags / post-analysis:
//        #OWTT,<seq>,<src>,<tx_us>,<rx_us>,<tof_us>,<delta_us>,...
//   Independently, one #TEMPMON line every few seconds from loop() (see below).
//   If UTC is missing or TOF is implausible, emit #INA / #E,I,... instead of
//   #I/#OWTT (see emit path after extract_timed_payload_from_modem_line).
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
//                       #OCXOCAL,<N|M|F>,<ppb>,<ticks_per_s>,<samples>,<span_s>,<pairs>
//                     N = nominal (no measurement yet), M = measured from PPS,
//                     F = pinned by command. ppb is the offset from nominal.
//   $ZOCXOCAL=AUTO    Resume estimating from PPS (default). Clears the ring and
//                     falls back to nominal until enough edges accumulate.
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

#include <Arduino.h>
#include <ctype.h>

#if !defined(USB_DUAL_SERIAL) && !defined(USB_TRIPLE_SERIAL)
  #error "OWTT_Modem_LOLO_ver3 requires the 'Dual Serial' (or 'Triple Serial') USB type: Arduino IDE > Tools > USB Type > Dual Serial. SerialUSB1 is the GNSS bridge port."
#endif
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "owtt_epd.h"

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

// Baud a stock X20P boots at (factory UART1). The Teensy opens Serial2 here,
// then raises UART1 to GNSS_TARGET_BAUD (RAM-only VALSET) and follows.
// Fresh modules speak 38400; some older sticks may still have 115200 stored —
// maybe_recover_gnss_link() walks both. Staying at 115200 was a TIMEUTC
// latency bottleneck: NAV, config replies and RTCM share this UART, and
// queueing pushed TIMEUTC past the next PPS edge (whole-second ambiguity).
static constexpr uint32_t GNSS_INITIAL_BAUD = 38400;
// Alternate stored baud seen on previously-provisioned sticks.
static constexpr uint32_t GNSS_FALLBACK_BAUD = 115200;
// Max supported UART1 rate on the X20P (u-blox interface limit).
static constexpr uint32_t GNSS_TARGET_BAUD = 921600;
// Link supervisor: if no *valid* UBX/NMEA arrives this long (silent line, or
// a baud mismatch that spews unframed noise/nulls), step the local baud and
// re-apply the RAM config. Byte activity alone is not "healthy".
static constexpr uint32_t GNSS_LINK_SILENT_MS = 3000UL;

// Extra HardwareSerial (Serial2) buffering for the X20P bridge.
// Default Teensy RX is tiny (~64 B). When USB TX backpressures, service_gnss_bridge()
// stops reading Serial2 so bytes aren't dropped on the USB side — but then Serial2's
// own RX overflows and corrupts UBX frames (checksum failures). Larger RX absorbs
// those stalls; larger TX absorbs RTCM bursts host -> X20P.
// (apply helper is defined later — must not be the first function in this .ino or
// Arduino's auto-prototypes are emitted before BridgeMode/LineReader exist.)
static uint8_t gnss_uart_rx_extra[8192];
static uint8_t gnss_uart_tx_extra[4096];
static void gnss_uart_apply_extra_buffers();

// Normal-operation default: keep Serial2 fixed at GNSS_INITIAL_BAUD.
//
// A terminal program can report 9600 as its USB CDC baud even though USB CDC
// itself has no physical baud rate. If this option is true, opening the GNSS
// USB port with `screen` at 9600 would also switch Serial2 to 9600 and make a
// factory 38400-baud X20P appear silent. Enable this only for service tools that must
// intentionally change the receiver UART baud, such as some firmware-update
// workflows.
static constexpr bool GNSS_FOLLOW_USB_BAUD = false;

// Enable UBX-NAV-TIMEUTC output on X20P UART1 at one message per navigation
// epoch. The command is sent to RAM only, so it does not permanently rewrite
// the receiver configuration.
static constexpr bool GNSS_AUTO_ENABLE_TIMEUTC = true;

// UTC-labelled capture (optional debug): absolute UTC second of the epoch the
// RxS flag was captured in, plus the local capture delta within that second.
static constexpr bool EMIT_RX_UTC_LINE = true;

// Teensy 4.1 timing pins
static constexpr uint8_t PIN_OCXO = 14;
static constexpr uint8_t PIN_RXS_CAPTURE   = 15;
static constexpr uint8_t PIN_PPS_CAPTURE   = 40;

// External 10 MHz clock / (0 + 1) = 10 MHz GPT2 tick (one raw OCXO cycle).
static constexpr uint32_t GPT2_PRESCALER_VALUE = 0;

static constexpr size_t LINE_MAX = 256;
static constexpr size_t GPS_MAX  = 64;

// GNSS UTC parser / association limits.
static constexpr uint16_t UBX_MAX_PAYLOAD = 1024;
// PPS↔UTC bind sanity cap on message-after-PPS age. The bind formula itself
// tolerates any after_pps (see maybe_bind_gnss_utc_to_current_epoch), this
// only rejects nonsense from stale epochs / counter races.
static constexpr uint32_t UTC_BIND_MAX_AFTER_PPS_US = 2500000UL;
// Samples that must agree on the max implied label before (re)binding, and
// how many samples the max may go unseen before it is considered stale
// (latency floor shifted / max was a glitch) and the tracker resets.
static constexpr uint8_t UTC_BIND_CONFIRM_SAMPLES = 3;
static constexpr uint8_t UTC_BIND_MAX_UNSEEN_SAMPLES = 60;
static constexpr uint32_t UTC_STATUS_MAX_AGE_MS = 3000UL;
// TOF plausibility ceiling for #I. A genuine one-way travel time must be
// shorter than the leader's TX period (4 s) or pings would overlap; any
// RxS↔#B mispairing (modem buffering decodes across mode churn) is off by
// whole TX periods, so a ceiling below one period rejects every mispair.
// 3 s ≈ 4.5 km at 1500 m/s — beyond these modems' reach anyway.
static constexpr int64_t OWTT_MAX_PLAUSIBLE_TOF_US = 3000000LL;

// GNSS diagnostics. Live debug is disabled at boot and can be enabled from
// the first USB/host port with `$ZGNSSDEBUG=1`. It prints one summary per
// second and never injects debug bytes into the transparent GNSS USB port.
static constexpr uint32_t GNSS_ACTIVITY_FRESH_MS = 2000UL;
static constexpr uint32_t GNSS_DEBUG_PERIOD_MS = 1000UL;
static constexpr size_t GNSS_NMEA_MAX = 160;

// OCXO Holdover constants.
//
// EPOCH_US is microseconds per second in the SI sense: it only ever converts a
// second label into a timestamp. It is *not* interchangeable with the number of
// OCXO ticks in a second, which is measured at runtime; see epoch_ticks() and
// the OCXO RATE CALIBRATION section.
static constexpr uint32_t EPOCH_US       = 1000000UL;

// Nominal OCXO ticks in one true second: 10 MHz / (GPT2_PRESCALER_VALUE + 1).
// Only the starting point; the measured rate replaces it once PPS is available.
static constexpr uint32_t EPOCH_TICKS_NOM =
  10000000UL / (GPT2_PRESCALER_VALUE + 1U);
// 1e9 / n0, exact at 10 MHz (100). Used to convert Q32.32 step <-> ppb.
static constexpr int64_t OCXO_PPB_Q32_SCALE =
  1000000000LL / (int64_t)EPOCH_TICKS_NOM;

// Holdover entry: 1.2 true seconds without an accepted PPS, in GPT2 ticks
// (not microseconds — at 10 MHz those are no longer the same).
static constexpr uint32_t PPS_TIMEOUT_TICKS =
  EPOCH_TICKS_NOM + EPOCH_TICKS_NOM / 5U;

// PPS rate-calibration ring.
//
// A rate estimate resolves one counter tick divided by the pair baseline, so a
// 0.1 us tick over a 256 s baseline is 0.39 ppb, while counting ticks between
// *adjacent* edges would only resolve 100 ppb. MIN_SAMPLES already gives a
// usable ~3.1 ppb estimate about a minute after lock, improving as the ring
// fills. Pairing uses current occupancy, so a dive before the ring is full
// still keeps whatever N >= MIN_SAMPLES is in the ring.
static constexpr uint16_t PPS_CAL_RING_LEN     = 512;
static constexpr uint16_t PPS_CAL_MIN_SAMPLES  = 64;
// Re-estimate every N real edges rather than every edge; the sort is O(n^2).
static constexpr uint16_t PPS_CAL_UPDATE_EDGES = 8;
// Discard pairs spanning longer than this. GPT2 is 32-bit and wraps every
// ~429.5 s at 10 MHz; (b.t - a.t) is an unsigned modular subtract, so a pair
// longer than one wrap is ambiguous. A no-gap full ring pairs at 256 s; 400 s
// leaves room for a few missed edges and keeps d_tick * 2^32 inside uint64.
static constexpr uint32_t PPS_CAL_MAX_PAIR_EPOCHS = 400UL;
static_assert(
  (uint64_t)PPS_CAL_MAX_PAIR_EPOCHS * (uint64_t)EPOCH_TICKS_NOM <
    (1ULL << 32),
  "pair tick delta must fit in uint32");
// Reject estimates further than this from nominal. The AOC2012 allows +-500 ppb
// of calibration tolerance plus +-700 ppb of Vc pullability, so anything beyond
// 2 ppm is a bad capture train rather than a real oscillator.
static constexpr int32_t OCXO_RATE_MAX_PPB = 2000;
// Only announce a new rate when it actually moved, to keep #OCXOCAL out of the
// per-second traffic.
static constexpr int32_t OCXO_CAL_REPORT_PPB = 5;
// A PPS edge is only trusted for calibration while the GNSS vouches for its
// own time solution. An unlocked receiver still emits PPS, but derived from its
// internal oscillator, which would calibrate the OCXO against nothing.
static constexpr uint32_t PPS_CAL_MAX_TACC_NS  = 1000000UL;
static constexpr uint32_t PPS_CAL_UTC_FRESH_MS = 3000UL;

// Config timeout constant
static constexpr uint32_t CONFIG_ADDR_TIMEOUT_MS = 5000;

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

// Last config outcome shown on the e-ink status page ("ok", abort reason, ...).
static char epd_cfg_line[20] = "ok";

// Latest GPS payload stored on transmitter.
// Payload is stored as raw "<lat>,<lon>", without "$G".
static char latest_gps[GPS_MAX + 1] = {0};
static size_t latest_gps_len = 0;
static bool latest_gps_valid = false;

// Payload lifetime in timing epochs. A value of 0 means the last valid GPS or
// telemetry payload remains valid indefinitely. Default is 1 epoch (~1 s with
// PPS) so a dead host / silent $G stream cannot keep a stale position on the
// air. Override at runtime with $ZPAYLOADTTL=<epochs> (ROS leader sets this).
static uint32_t payload_valid_epochs = 1;

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

// Ranging (sound velocity, fixed delays, calibration) is done on the ROS/host
// side from the #I one-way travel time plus the #OWTT context columns.

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
// Bridge bytes dropped because the USB-side reader stalled (local parsing
// always continues; only forwarding is sacrificed).
static uint32_t gnss_bridge_drop_count = 0;
static uint32_t gnss_nmea_line_count = 0;
static uint32_t gnss_last_nmea_ms = 0;
// Last millis() a framed UBX or NMEA message decoded successfully. Used by
// the link supervisor — raw rx_bytes alone can be a stuck-low / wrong-baud
// null stream that must not look healthy.
static uint32_t gnss_last_good_frame_ms = 0;
// When the current local UART baud listen window started (boot, RELINK, or
// local baud change). Silence age must be measured from this when no frame
// has decoded yet — using millis()-since-boot made RELINK fire every loop.
static uint32_t gnss_link_epoch_ms = 0;
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
// Compare interrupt fires once per measured second.
// The transmitter period is handled separately by counting epochs:
//   next_tx_epoch_count = epoch_count + cfg.period_pps
//
// Tick position of the next virtual epoch, Q32.32. The fractional part is what
// lets the holdover train run at the measured rate: a true second is around
// 10000000.47 ticks, so re-truncating to whole ticks every epoch would put a
// 100 ppb rounding error back in and undo the calibration.
//
// The integer part occupies bits 32..63, so uint64 wraparound is exactly the
// mod-2^32 arithmetic the free-running counter needs.
static volatile uint64_t next_compare_q32 = 0;

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

// =======================================================
// ============= OCXO RATE CALIBRATION ===================
// =======================================================
//
// The epoch machinery used to assume the OCXO produced exactly EPOCH_TICKS_NOM
// ticks per second. It does not: the AOC2012 is specified to +-500 ppb of
// initial calibration tolerance (typ. <+-300 ppb), and the field unit measured
// -48 ppb, which is -0.26 m/h of range error at 1481 m/s. PPS re-pinned the
// phase of each epoch every second but nothing ever estimated the *rate*, so
// the full offset reappeared as a ramp the moment PPS was lost.
//
// Every real PPS edge is already timestamped by the same GPT2 capture unit used
// for acoustic arrivals, so the rate is measurable for free while the vehicle is
// on the surface. Holdover then advances at the measured rate.
//
// Estimator: the median of pairwise slopes
//     (t[k + half] - t[k]) / (epoch[k + half] - epoch[k])
// over the retained edges. Two properties matter.
//
// Baseline: each pair spans half the ring, i.e. hundreds of seconds, and a
// slope resolves one tick divided by its baseline. Slopes between *adjacent*
// edges would resolve only 100 ppb, and a -47 ppb offset is still 0.47 ticks
// per second, so a 1 s median stays half-blind. The 32 s short-lock pairs
// already see ~15 ticks of that signal; that is why the finer tick is worth
// it. Pairing still uses current occupancy (half of N), not a full ring.
//
// Robustness: the median (rather than a least-squares fit) keeps one glitched
// capture, a spurious edge, or a PPS outage from moving the rate at all.
//
// Differencing epoch_count rather than ring position means a missed or dropped
// edge shortens the pair by whole seconds instead of biasing the slope.

struct PpsCalSample {
  uint32_t tick;   // GPT2 capture value of the edge
  uint32_t epoch;  // epoch_count at that edge, so gaps are exact
};

static PpsCalSample pps_cal_ring[PPS_CAL_RING_LEN];
static uint16_t pps_cal_head = 0;
static uint16_t pps_cal_count = 0;
static uint16_t pps_cal_since_update = 0;

// Scratch for the median. Static rather than automatic: 2 KB does not belong on
// the main-loop stack.
static uint64_t pps_cal_slopes[PPS_CAL_RING_LEN / 2];

// Measured OCXO ticks in one true second, Q32.32.
static volatile uint64_t epoch_step_q32 = (uint64_t)EPOCH_TICKS_NOM << 32;
// Same measurement rounded to whole ticks, for deciding which epoch a capture
// belongs to. Sub-tick precision is irrelevant there.
static uint32_t epoch_ticks_int = EPOCH_TICKS_NOM;
// Same measurement as a signed fractional offset from nominal, for reporting
// and for converting tick intervals to true microseconds.
static int32_t ocxo_rate_ppb = 0;

enum OcxoCalState : uint8_t {
  OCXO_CAL_NOMINAL  = 0, // no measurement yet, running on EPOCH_TICKS_NOM
  OCXO_CAL_MEASURED = 1, // rate estimated from the PPS ring
  OCXO_CAL_MANUAL   = 2  // rate pinned by $ZOCXOCAL=<ppb>
};

static OcxoCalState ocxo_cal_state = OCXO_CAL_NOMINAL;
static bool ocxo_cal_auto = true;
static uint32_t ocxo_cal_span_s = 0;
static uint16_t ocxo_cal_pairs = 0;

// Explicit prototypes required for Arduino's .ino preprocessor, for the same
// reason as timing_mode_code() below: without them Arduino auto-generates
// prototypes above the PpsCalSample/OcxoCalState declarations.
static void ocxo_apply_rate(uint64_t step_q32, OcxoCalState state);
static const PpsCalSample *pps_cal_at(uint16_t age_index);

// Whole OCXO ticks in one true second.
static inline uint32_t epoch_ticks()
{
  return epoch_ticks_int;
}

// Convert a counter interval to true microseconds. 10 MHz ticks are 0.1 us;
// at the measured rate a 1 s interval is off by ocxo_rate_ppb ns.
static inline uint32_t ticks_to_us(uint32_t ticks)
{
  const int64_t nom_us =
    ((int64_t)ticks * (int64_t)EPOCH_US) / (int64_t)EPOCH_TICKS_NOM;
  const int64_t corr =
    (nom_us * (int64_t)ocxo_rate_ppb) / 1000000000LL;
  return (uint32_t)(nom_us - corr);
}

// ppb = (step - nominal) / nominal * 1e9
//     = diff_q32 * OCXO_PPB_Q32_SCALE / 2^32.
// Rounded rather than truncated so that a rate pinned by $ZOCXOCAL=<ppb>
// reads back unchanged.
static int32_t ocxo_ppb_from_step(uint64_t step_q32)
{
  const int64_t diff_q32 =
    (int64_t)step_q32 - ((int64_t)EPOCH_TICKS_NOM << 32);
  const int64_t scaled = diff_q32 * OCXO_PPB_Q32_SCALE;
  const int64_t half = (int64_t)1 << 31;
  return (int32_t)(((scaled >= 0) ? (scaled + half) : (scaled - half)) /
                   ((int64_t)1 << 32));
}

static uint64_t ocxo_step_from_ppb(int32_t ppb)
{
  const int64_t diff_q32 =
    ((int64_t)ppb * ((int64_t)1 << 32)) / OCXO_PPB_Q32_SCALE;
  return (uint64_t)(((int64_t)EPOCH_TICKS_NOM << 32) + diff_q32);
}

static void ocxo_apply_rate(uint64_t step_q32, OcxoCalState state)
{
  // The compare ISR reads epoch_step_q32 as two words; block it across the
  // update so it can never see a torn value.
  noInterrupts();
  epoch_step_q32 = step_q32;
  interrupts();

  epoch_ticks_int = (uint32_t)((step_q32 + 0x80000000ULL) >> 32);
  ocxo_rate_ppb = ocxo_ppb_from_step(step_q32);
  ocxo_cal_state = state;
}

// N = nominal 1e7, M = measured from PPS, F = pinned by $ZOCXOCAL=<ppb>.
static char ocxo_cal_state_char()
{
  if (ocxo_cal_state == OCXO_CAL_MEASURED) {
    return 'M';
  }
  if (ocxo_cal_state == OCXO_CAL_MANUAL) {
    return 'F';
  }
  return 'N';
}

// MCU junction temperature from the i.MX RT1062 TEMPMON, in centi-degC.
// Written only from loop() / $ZTEMP?, never from handle_epoch_event.
static int32_t die_c_x100 = 0;
static bool die_c_valid = false;
static uint32_t die_c_last_ms = 0;
static constexpr uint32_t DIE_TEMP_PERIOD_MS = 5000UL;

static void format_die_c(char *buf, size_t n)
{
  if (!die_c_valid || buf == NULL || n < 16) {
    if (buf != NULL && n > 0) {
      buf[0] = '\0';
    }
    return;
  }
  const char sign = (die_c_x100 < 0) ? '-' : '+';
  const uint32_t a = (uint32_t)((die_c_x100 < 0) ? -die_c_x100 : die_c_x100);
  snprintf(buf, n, "%c%lu.%02lu", sign, (unsigned long)(a / 100U),
           (unsigned long)(a % 100U));
}

static void emit_tempmon_line()
{
  char die[16];
  format_die_c(die, sizeof(die));
  if (die[0] == '\0') {
    return;
  }
  char msg[48];
  snprintf(
    msg,
    sizeof(msg),
    "#TEMPMON,%lu,%c,%s",
    (unsigned long)epoch_count,
    timing_mode_code(timing_mode),
    die
  );
  host_send_line(msg);
}

static bool sample_die_temp()
{
  // Do not call tempmonGetTemp() unless VALID is already set: that helper
  // busy-waits, and a conversion in flight is ~90 us at the core's 3/32768 s
  // measure period. Skip this pass rather than stall the ranging loop.
  if (!(TEMPMON_TEMPSENSE0 & 0x4U)) {
    return false;
  }
  const float t = tempmonGetTemp();
  die_c_x100 = (int32_t)(t * 100.0f + (t >= 0.0f ? 0.5f : -0.5f));
  die_c_valid = true;
  return true;
}

static void service_die_temp()
{
  const uint32_t now = millis();
  if (die_c_valid && (uint32_t)(now - die_c_last_ms) < DIE_TEMP_PERIOD_MS) {
    return;
  }
  if (!sample_die_temp()) {
    return;
  }
  die_c_last_ms = now;
  emit_tempmon_line();
}

static void ocxo_cal_report()
{
  const uint64_t step = epoch_step_q32;
  const uint32_t frac_e6 =
    (uint32_t)(((step & 0xFFFFFFFFULL) * 1000000ULL) >> 32);

  const char state_char = ocxo_cal_state_char();

  char msg[96];
  snprintf(
    msg,
    sizeof(msg),
    "#OCXOCAL,%c,%ld,%lu.%06lu,%u,%lu,%u",
    state_char,
    (long)ocxo_rate_ppb,
    (unsigned long)(step >> 32),
    (unsigned long)frac_e6,
    (unsigned)pps_cal_count,
    (unsigned long)ocxo_cal_span_s,
    (unsigned)ocxo_cal_pairs
  );
  host_send_line(msg);
}

static void pps_cal_reset()
{
  pps_cal_head = 0;
  pps_cal_count = 0;
  pps_cal_since_update = 0;
  ocxo_cal_span_s = 0;
  ocxo_cal_pairs = 0;
}

// Chronological access: age_index 0 is the oldest retained sample.
static const PpsCalSample *pps_cal_at(uint16_t age_index)
{
  const uint16_t oldest = (uint16_t)(
    (pps_cal_head + PPS_CAL_RING_LEN - pps_cal_count) % PPS_CAL_RING_LEN);
  return &pps_cal_ring[(uint16_t)((oldest + age_index) % PPS_CAL_RING_LEN)];
}

static void pps_cal_push(uint32_t tick, uint32_t epoch)
{
  pps_cal_ring[pps_cal_head].tick = tick;
  pps_cal_ring[pps_cal_head].epoch = epoch;
  pps_cal_head = (uint16_t)((pps_cal_head + 1U) % PPS_CAL_RING_LEN);

  if (pps_cal_count < PPS_CAL_RING_LEN) {
    pps_cal_count++;
  }
}

static bool pps_cal_estimate(uint64_t *out_step_q32)
{
  if (out_step_q32 == NULL || pps_cal_count < PPS_CAL_MIN_SAMPLES) {
    return false;
  }

  const uint16_t half = (uint16_t)(pps_cal_count / 2U);
  uint16_t n = 0;

  for (uint16_t k = 0; k < half; k++) {
    const PpsCalSample *a = pps_cal_at(k);
    const PpsCalSample *b = pps_cal_at((uint16_t)(k + half));

    const uint32_t d_epoch = b->epoch - a->epoch;
    if (d_epoch == 0 || d_epoch > PPS_CAL_MAX_PAIR_EPOCHS) {
      continue;
    }

    const uint32_t d_tick = b->tick - a->tick;
    pps_cal_slopes[n++] = ((uint64_t)d_tick << 32) / (uint64_t)d_epoch;
  }

  if (n < PPS_CAL_MIN_SAMPLES / 2U) {
    return false;
  }

  // Insertion sort: n <= PPS_CAL_RING_LEN/2 and this runs at most once every
  // PPS_CAL_UPDATE_EDGES seconds.
  for (uint16_t i = 1; i < n; i++) {
    const uint64_t v = pps_cal_slopes[i];
    uint16_t j = i;
    while (j > 0 && pps_cal_slopes[j - 1] > v) {
      pps_cal_slopes[j] = pps_cal_slopes[j - 1];
      j--;
    }
    pps_cal_slopes[j] = v;
  }

  const uint64_t median = (n & 1U)
    ? pps_cal_slopes[n / 2U]
    : ((pps_cal_slopes[n / 2U - 1U] >> 1) + (pps_cal_slopes[n / 2U] >> 1));

  const PpsCalSample *oldest = pps_cal_at(0);
  const PpsCalSample *newest = pps_cal_at((uint16_t)(pps_cal_count - 1U));
  ocxo_cal_span_s = newest->epoch - oldest->epoch;
  ocxo_cal_pairs = n;

  *out_step_q32 = median;
  return true;
}

// Recompute from the ring and adopt the result if it is plausible. Returns true
// if a new rate was applied.
static bool ocxo_cal_try_update()
{
  uint64_t step_q32 = 0;
  if (!pps_cal_estimate(&step_q32)) {
    return false;
  }

  // A wild estimate means the capture train is wrong, not the oscillator.
  // Keeping the previous rate is always safer than trusting it.
  const int32_t ppb = ocxo_ppb_from_step(step_q32);
  if (ppb > OCXO_RATE_MAX_PPB || ppb < -OCXO_RATE_MAX_PPB) {
    return false;
  }

  ocxo_apply_rate(step_q32, OCXO_CAL_MEASURED);
  return true;
}

static void pps_cal_on_real_pps(uint32_t t_epoch)
{
  const bool gnss_vouches =
    gnss_utc.valid &&
    gnss_utc.tAcc_ns <= PPS_CAL_MAX_TACC_NS &&
    (uint32_t)(millis() - gnss_utc.received_ms) <= PPS_CAL_UTC_FRESH_MS;

  if (!gnss_vouches) {
    return;
  }

  pps_cal_push(t_epoch, epoch_count);

  if (!ocxo_cal_auto) {
    return;
  }

  if (++pps_cal_since_update < PPS_CAL_UPDATE_EDGES) {
    return;
  }
  pps_cal_since_update = 0;

  const int32_t prev_ppb = ocxo_rate_ppb;
  const bool was_measured = (ocxo_cal_state == OCXO_CAL_MEASURED);

  if (!ocxo_cal_try_update()) {
    return;
  }

  const int32_t moved = (ocxo_rate_ppb > prev_ppb)
    ? (ocxo_rate_ppb - prev_ppb)
    : (prev_ppb - ocxo_rate_ppb);

  if (!was_measured || moved >= OCXO_CAL_REPORT_PPB) {
    ocxo_cal_report();
  }
}

// Fold every edge collected so far into the rate immediately, instead of waiting
// for the next PPS_CAL_UPDATE_EDGES boundary, and report what is in effect.
//
// This recomputes rather than clears. The ring's accumulated baseline is exactly
// what gives the estimate its resolution, so discarding samples would make the
// rate worse and leave it unavailable for the next PPS_CAL_MIN_SAMPLES edges.
//
// A rate pinned with $ZOCXOCAL=<ppb> is deliberately left alone, so arming the
// holdover experiment cannot silently re-enable estimation partway through an
// A/B baseline run.
static void ocxo_cal_refresh_now()
{
  if (ocxo_cal_auto) {
    pps_cal_since_update = 0;
    ocxo_cal_try_update();
  }

  ocxo_cal_report();
}

extern "C" void GPT2_IRQHandler();

static inline uint32_t OCXO_counter_us()
{
  return GPT2_CNT;
}

// Place the virtual-epoch train n_ahead measured seconds after t_anchor. The
// anchor is always a real PPS capture, so the holdover train stays phase-locked
// to the last GNSS edge instead of accumulating the oscillator's rate error.
static inline void gpt2_schedule_epoch_train(uint32_t t_anchor, uint32_t n_ahead)
{
  noInterrupts();
  next_compare_q32 =
    ((uint64_t)t_anchor << 32) + epoch_step_q32 * (uint64_t)n_ahead;
  GPT2_OCR1 = (uint32_t)(next_compare_q32 >> 32);
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

// $ZIGNOREPPSAFTER experiment: after this many seconds from when N was armed,
// stop accepting real PPS so timing drifts into OCXO holdover. 0 / disarmed =
// never ignore (default). Deadline uses wrap-safe millis comparison.
static uint32_t ignore_pps_after_s = 0;
static uint32_t ignore_pps_deadline_ms = 0;
static bool ignore_pps_armed = false;

static bool pps_ignored_now()
{
  if (!ignore_pps_armed || ignore_pps_after_s == 0) {
    return false;
  }
  return (int32_t)(millis() - ignore_pps_deadline_ms) >= 0;
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
    ref -= epoch_ticks();
    second--;
  }

  const uint32_t elapsed_ticks = (uint32_t)(counter_us - ref);

  // This also handles the small window where an epoch interrupt occurred but
  // the main loop has not processed it yet. Whole seconds are counted in the
  // tick domain; only the sub-second remainder becomes microseconds.
  second += (int64_t)(elapsed_ticks / epoch_ticks());
  const uint32_t fraction_us = ticks_to_us(elapsed_ticks % epoch_ticks());

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
        gnss_last_good_frame_ms = millis();

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

static void gpt2_extclk_capture_init()
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
    // Pin 13 is SPI SCK for the e-ink panel; do not toggle LED_BUILTIN here.
  }

  // Holdover virtual PPS compare.
  if (sr & GPT_SR_OF1) {
    cmp_stamp = GPT2_OCR1;
    cmp_pending = true;
    GPT2_SR = GPT_SR_OF1;

    // Accumulate in Q32.32 so the sub-tick part of the measured second carries
    // across epochs instead of being truncated away each time.
    next_compare_q32 += epoch_step_q32;
    GPT2_OCR1 = (uint32_t)(next_compare_q32 >> 32);
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

static void handle_epoch_event(uint32_t t_epoch, bool real_pps)
{
  // Advance by *elapsed hardware time*, not by processed-event count. The
  // pending-PPS slot only holds the latest capture: if the main loop stalls
  // (USB backpressure), several edges collapse into one call and per-call ++
  // would leave the UTC label seconds behind (observed as #UTC,REBIND,diff_s=15
  // after node restarts). E-ink refreshes run on a TeensyThreads worker and
  // no longer stall this path. The capture timestamps are exact, so the
  // epoch delta is exact.
  //
  // elapsed == 0 happens on holdover→PPS relock when the real edge lands
  // within half an epoch of the last virtual edge: same second, re-aligned
  // timestamp, no label advance (per-call ++ used to double-count here).
  uint32_t elapsed = 1;
  if (pps_valid) {
    const uint32_t dt = (uint32_t)(t_epoch - current_epoch_us);
    elapsed = (dt + epoch_ticks() / 2U) / epoch_ticks();
  }

  current_epoch_us = t_epoch;
  pps_valid = true;
  epoch_count += elapsed;

  if (current_epoch_utc_valid) {
    current_epoch_utc_second += (int64_t)elapsed;
  }

  if (real_pps) {
    holdover_age_s = 0;
  } else if (timing_mode == TIMING_HOLDOVER) {
    holdover_age_s += elapsed;
  }

  // Feed the rate estimator before the train is re-anchored below, so a new
  // measurement takes effect on this edge rather than the next one.
  if (real_pps) {
    pps_cal_on_real_pps(t_epoch);
  }

  if (payload_valid_epochs != 0 &&
      latest_gps_valid &&
      (uint32_t)(epoch_count - latest_gps_epoch_count) > payload_valid_epochs) {
    latest_gps_valid = false;
    latest_gps_len = 0;
    tx_last_warning = TX_WARN_NONE;
    // Host $G/$K went stale — paint red "HOST SILENT" without waiting the e-ink gate.
    owtt_epd_request_immediate();
  }

  if (payload_valid_epochs != 0 &&
      latest_telem_valid &&
      (uint32_t)(epoch_count - latest_telem_epoch_count) > payload_valid_epochs) {
    latest_telem_valid = false;
    latest_telem_len = 0;
    tx_last_warning = TX_WARN_NONE;
    // Host $G/$K went stale — paint red "HOST SILENT" without waiting the e-ink gate.
    owtt_epd_request_immediate();
  }

  if (real_pps) {
    last_real_pps_us = t_epoch;
    timing_mode = TIMING_PPS_LOCKED;

    gpt2_schedule_epoch_train(t_epoch, 1);
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
    ref -= epoch_ticks();
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

  pending_delta_us =
    (int32_t)ticks_to_us((uint32_t)(t_rxs - ref_epoch_us));
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

  if (since_last_pps <= PPS_TIMEOUT_TICKS) {
    return;
  }

  uint32_t missed_epochs = since_last_pps / epoch_ticks();
  if (missed_epochs == 0) {
    missed_epochs = 1;
  }

  // Account for each virtual epoch that has already elapsed since the last
  // real PPS. The previous implementation moved current_epoch_us forward
  // without incrementing epoch_count, which also made an absolute UTC label
  // lag by one second after entering holdover.
  timing_mode = TIMING_HOLDOVER;

  // Place every virtual epoch at the measured rate from the last real edge
  // rather than at a nominal 1e7 ticks, so the train inherits GNSS phase and
  // does not walk off at the oscillator's frequency offset.
  for (uint32_t i = 1; i <= missed_epochs; i++) {
    const uint64_t pos_q32 =
      ((uint64_t)last_real_pps_us << 32) + epoch_step_q32 * (uint64_t)i;
    handle_epoch_event((uint32_t)(pos_q32 >> 32), false);
  }

  gpt2_schedule_epoch_train(last_real_pps_us, missed_epochs + 1U);
  gpt2_enable_compare_irq();

  // Record the rate the holdover actually inherits, and the ring depth behind
  // it, while those counters still hold the pre-cut window. $ZIGNOREPPSAFTER
  // reports at arm time, but the estimator keeps revising for the N seconds
  // until the cut, so that earlier line is not necessarily what is in effect
  // here. The reset below would otherwise zero samples/span/pairs.
  ocxo_cal_report();

  // Start a fresh calibration window on the next lock: pairs straddling the
  // outage could span more than the counter's 429.5 s wrap.
  pps_cal_reset();

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

    // Holdover experiment: past the ignore deadline, drop real PPS captures.
    // last_real_pps_us goes stale and maybe_enter_holdover() below switches
    // to the OCXO epoch train, exactly like a physical PPS loss.
    if (!pps_ignored_now()) {
      handle_epoch_event(t, true);
    }
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

  (void)tx_time_us;
  (void)tx_fraction_us;

  const int ss = (int)((tx_epoch_second % 60LL + 60LL) % 60LL);
  const char mode = timing_mode_code(timing_mode);

  char payload[MODEM_PAYLOAD_MAX + 1];
  int payload_len;
  if (holdover_age_s != 0) {
    payload_len = snprintf(
      payload,
      sizeof(payload),
      "T%02d|%04X|%c|%08lX|%s",
      ss,
      (unsigned int)tx_sequence,
      mode,
      (unsigned long)holdover_age_s,
      application_payload
    );
  } else {
    payload_len = snprintf(
      payload,
      sizeof(payload),
      "T%02d|%04X|%c|%s",
      ss,
      (unsigned int)tx_sequence,
      mode,
      application_payload
    );
  }

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
// reports whether it carries a compact TX time stamp we can use for OWTT.
// Only timestamped payloads drive #I; anything else is untimed.
struct TimedPayloadInfo {
  bool timestamped;
  char source_id[4];
  uint8_t tx_ss;              // TX time-of-minute (seconds field 0..59)
  uint32_t tx_fraction_us;    // µs within that second
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

  // T<ss>|<seq:hex>|<mode>[|<holdover:hex>]|<application>
  char *end = NULL;
  const unsigned long ss = strtoul(raw_payload + 1, &end, 10);
  if (end == raw_payload + 1 || end == NULL || *end != '|' || ss > 59UL) {
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

  // Optional holdover field: TX only emits it when free-running (mode H).
  // Do not probe for it under P/W — GPS payloads start with a hex digit and
  // must not be mistaken for a holdover age.
  const char *application = mode_start + 2;
  unsigned long tx_holdover_age = 0;
  if (mode_start[0] == 'H') {
    const char *age_start = application;
    const unsigned long age = strtoul(age_start, &end, 16);
    if (end == age_start || end == NULL || *end != '|') {
      return false;
    }
    tx_holdover_age = age;
    application = end + 1;
  }

  if (!looks_like_gps_coords(application) &&
      !looks_like_telemetry(application)) {
    return false;
  }

  const size_t n = strlen(application);
  if (n >= sizeof(out->application_payload)) {
    return false;
  }

  out->timestamped = true;
  out->tx_ss = (uint8_t)ss;
  out->tx_fraction_us = 0;
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

  // Only envelope-stamped payloads carry a TX time; untimed payloads do not
  // produce #I/#J/#OWTT at all.
  return parse_timestamp_envelope(raw_payload, out);
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

static void maybe_emit_receiver_delta_after_modem_line(const char *line)
{
  // Only attempt OWTT on broadcast/unicast data frames.
  if (line == NULL || line[0] != '#' || (line[1] != 'B' && line[1] != 'U')) {
    return;
  }

  if (cfg.mode != MODE_RECEIVER) {
    // #B reached us but we are not in receiver mode — ranging will stay silent.
    host_send_line("#E,I,NOT_R");
    return;
  }

  TimedPayloadInfo payload_info = {};
  if (!extract_timed_payload_from_modem_line(line, &payload_info)) {
    // Surfaced so a silent parse miss is obvious on the host.
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
    "bridge_drops=%lu,timing=%s,utc_valid=%u,epoch_unix_s=%lld,"
    "holdover_age_s=%lu",
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
  gnss_uart_apply_extra_buffers();
  gnss_uart_baud = (uint32_t)requested;
    ubx_parser_reset();
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
  ubx_parser_reset();
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
static void maybe_send_gnss_boot_config()
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

static void maybe_recover_gnss_link()
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
  ubx_parser_reset();
  gnss_last_good_frame_ms = 0;
  gnss_link_epoch_ms = millis();
}

static void gnss_uart_apply_extra_buffers()
{
  GNSS_SERIAL.addMemoryForRead(gnss_uart_rx_extra, sizeof(gnss_uart_rx_extra));
  GNSS_SERIAL.addMemoryForWrite(gnss_uart_tx_extra, sizeof(gnss_uart_tx_extra));
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

// =======================================================
// ===================== E-INK STATUS ====================
// =======================================================

static void sync_owtt_epd_status()
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
  gnss_uart_apply_extra_buffers();
  gnss_last_good_frame_ms = 0;
  gnss_link_epoch_ms = millis();

  pinMode(PIN_OCXO, INPUT);
  pinMode(PIN_RXS_CAPTURE, INPUT);
  pinMode(PIN_PPS_CAPTURE, INPUT);

  // Pin 13 is SPI SCK for the e-ink panel (shared with LED_BUILTIN).
  // Do not drive it as a GPIO LED.

  gpt2_extclk_capture_init();

  // Give the X20P time to boot before enabling UBX-NAV-TIMEUTC.
  gnss_timeutc_config_due_ms = millis() + 1000UL;

  sync_owtt_epd_status();
  owtt_epd_begin();
  owtt_epd_service();
}

void loop()
{
  char line[LINE_MAX];

  // Keep the X20P bridge responsive and passively parse UTC messages.
  service_gnss_bridge();
  maybe_send_gnss_boot_config();
  maybe_recover_gnss_link();

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

  // E-ink status: coalesce full refreshes (see EPD_MIN_REFRESH_MS).
  sync_owtt_epd_status();
  owtt_epd_service();

  // Die temperature: idle-path only, after timing and serial have run.
  service_die_temp();
}
