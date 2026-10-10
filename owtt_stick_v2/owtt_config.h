#pragma once
// Pins, baud rates, limits, and the shared types.
// The loop is owtt_stick_v2.ino.

#include <Arduino.h>
#include <EEPROM.h>
#include <ctype.h>

#if !defined(USB_DUAL_SERIAL) && !defined(USB_TRIPLE_SERIAL)
  #error "owtt_stick_v2 requires the 'Dual Serial' (or 'Triple Serial') USB type: Arduino IDE > Tools > USB Type > Dual Serial. SerialUSB1 is the GNSS bridge port."
#endif
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "owtt_frame.h"
#include "owtt_ubx.h"

// =======================================================
// ===================== USER CONFIG =====================
// =======================================================


// USB host protocol must stay clean. Do not print asynchronous debug text to
// Serial or SerialUSB1.
#define ENABLE_USB_DEBUG 0
#define USB_MIRROR_HOST_OUTPUT 0
#define HOST_OVER_USB 0


// The first USB CDC interface remains the LoLo/Succorfish protocol port.
// The second USB CDC interface is dedicated to the X20P UART bridge.
#define MODEM_SERIAL    Serial1
#define GNSS_SERIAL     Serial2
#define GNSS_USB_SERIAL SerialUSB1

// TODO setup this

#ifdef HOST_OVER_USB
// e.g in the case of Sticks/Floatsam/PowerPuffin(?)
#define HOST_SERIAL     Serial

#else
// e.g in the case where LoLo is the host
#define HOST_SERIAL Serial4

#endif


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

static constexpr size_t LINE_MAX = OWTT_LINE_MAX;
static constexpr size_t GPS_MAX  = 64;

// GNSS UTC association limits. The UBX frame-length cap lives in owtt_ubx.cpp.
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

struct PendingConfig {
  bool active;

  BridgeMode mode;
  char own_id[4];
  char listen_id[4];
  uint32_t period_pps;

  uint32_t deadline_ms;
  char original_cmd[96];
};

enum TxWarningReason : uint8_t {
  TX_WARN_NONE = 0,
  TX_WARN_NO_PAYLOAD,
  TX_WARN_NO_UTC,
  TX_WARN_PAYLOAD_TOO_LONG
};

// Succorfish broadcast payload limit. Timestamp metadata is included inside
// this limit. GPS coordinates comfortably fit; very large telemetry payloads
// may need to be shortened.
static constexpr size_t MODEM_PAYLOAD_MAX = 64;

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

struct GnssNmeaObserver {
  bool active;
  size_t len;
  char line[GNSS_NMEA_MAX];
};

enum TimingMode : uint8_t {
  TIMING_WAIT_PPS   = 0,
  TIMING_PPS_LOCKED = 1,
  TIMING_HOLDOVER   = 2
};

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

enum OcxoCalState : uint8_t {
  OCXO_CAL_NOMINAL  = 0, // no measurement yet, running on EPOCH_TICKS_NOM
  OCXO_CAL_MEASURED = 1, // rate estimated from the PPS ring
  OCXO_CAL_MANUAL   = 2, // rate pinned by $ZOCXOCAL=<ppb>
  OCXO_CAL_STORED   = 3  // last measured ppb restored from EEPROM; ring may replace it
};

// Latest measured ppb, the span it was measured over, and the GNSS time.
// Boot loads the ppb. A new ring replaces it only when its grid is tighter
// than the stored grid plus Abracon worst-case aging, or when the new span
// is at least as long (a fresh full ring is today's offset).
//
// Abracon AOC2012-Series table (https://abracon.com/datasheets/AOC2012-Series.pdf):
//   Aging Daily ±1.0 ppb, Aging 1st Year ±100 ppb, Aging 20 Years ±500 ppb.
// The daily line is the short-term worst case. Integrating ±1 ppb/day with no
// cap would exceed the yearly and 20-year lines, so the allowance is
// ceil(age / 1 day) ppb, capped at 100 ppb through the first year and at
// 500 ppb thereafter. A gap shorter than a day still costs the whole ±1 ppb:
// the datasheet bounds a day, not an hour. A record with no timestamp is
// treated as the 20-year worst case, so the first real ring may replace it.
static constexpr uint32_t OCXO_EEPROM_MAGIC_V1 = 0x4F435831UL; // 'OCX1' ppb only
static constexpr uint32_t OCXO_EEPROM_MAGIC     = 0x4F435832UL; // 'OCX2'
static constexpr uint32_t OCXO_AGING_PPB_PER_DAY = 1;
static constexpr uint32_t OCXO_AGING_YEAR1_PPB   = 100;
static constexpr uint32_t OCXO_AGING_20Y_PPB     = 500;
static constexpr uint32_t OCXO_AGING_UNKNOWN_PPB = OCXO_AGING_20Y_PPB;
static constexpr uint32_t OCXO_EEPROM_FILL_MS = 60000UL;
static constexpr uint32_t OCXO_EEPROM_FULL_MS = 600000UL;

struct OcxoEepromRec {
  uint32_t magic;
  int32_t ppb;
  uint32_t span_s;
  uint32_t unix_s;
};
static constexpr uint32_t DIE_TEMP_PERIOD_MS = 5000UL;

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
