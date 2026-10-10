#pragma once
#include "owtt_config.h"

// Variables used by more than one module.
// A name that only one module uses stays static in that file.


extern BridgeConfig cfg;

extern size_t latest_gps_len;

extern bool latest_gps_valid;


// Payload lifetime in timing epochs. A value of 0 means the last valid GPS or
// telemetry payload remains valid indefinitely. Default is 1 epoch (~1 s with
// PPS) so a dead host / silent $G stream cannot keep a stale position on the
// air. Override at runtime with $ZPAYLOADTTL=<epochs> (ROS leader sets this).
extern uint32_t payload_valid_epochs;


extern uint32_t latest_gps_epoch_count;

extern size_t latest_telem_len;

extern bool latest_telem_valid;

extern uint32_t latest_telem_epoch_count;


extern TxWarningReason tx_last_warning;


// Ranging (sound velocity, fixed delays, calibration) is done on the ROS/host
// side from the #I one-way travel time plus the #OWTT context columns.

// Automatic TX due flag.
// Set by PPS timing logic, executed later after host commands are serviced.
extern bool tx_auto_due;


extern GnssUtcState gnss_utc;


// Absolute UTC label associated with current_epoch_us. Once initialized from
// a valid UBX-NAV-TIMEUTC message, it advances on real and holdover epochs.
extern bool current_epoch_utc_valid;

extern int64_t current_epoch_utc_second;


// UBX byte parser is owtt_ubx. This file sees a completed NAV-TIMEUTC
// payload in handle_ubx_nav_timeutc, then binds that second to the PPS epoch.

extern uint32_t gnss_uart_baud;

extern uint32_t gnss_timeutc_config_due_ms;

// Last millis() a framed UBX or NMEA message decoded successfully. Used by
// the link supervisor — raw rx_bytes alone can be a stuck-low / wrong-baud
// null stream that must not look healthy.
extern uint32_t gnss_last_good_frame_ms;

// When the current local UART baud listen window started (boot, RELINK, or
// local baud change). Silence age must be measured from this when no frame
// has decoded yet — using millis()-since-boot made RELINK fire every loop.
extern uint32_t gnss_link_epoch_ms;


// =======================================================
// ===================== FRAME ASSEMBLER =================
// =======================================================
//
// Text records are newline-delimited. NM3 *data* is length-prefixed and the
// payload may contain CR/LF/NUL:
//   Host → modem: $B<xx><data>, $U/$M/$E<xxx><yy><data>  (no trailer required)
//   Modem → host: #B<aaa><yy><data>[Q…][D…][T…]<CRLF>
//                 #U<yy><data>[trailer]<CRLF>

// Implementation: owtt_frame.cpp. Two assemblers, one per direction.
extern FrameAssembler host_fa;

extern FrameAssembler modem_fa;


extern TimingMode timing_mode;

// $ZZEROSTART is running the epoch train. Pin-40 edges are dropped until OFF.
extern bool bench_epoch_active;


// Number of OCXO-generated virtual seconds since the last real PPS. Reset to
// zero on every real PPS.
extern uint32_t holdover_age_s;


// It increments on real PPS and on holdover virtual PPS. Keeps tracks of how many transmissions passed -> good for orchestrating multiple transmitters
extern uint32_t epoch_count;


extern uint32_t current_epoch_us;

extern bool pps_valid;


extern bool pending_delta_valid;

// Same measurement as a signed fractional offset from nominal, for reporting
// and for converting tick intervals to true microseconds.
extern int32_t ocxo_rate_ppb;

extern bool ocxo_cal_auto;

extern bool die_c_valid;


// $ZIGNOREPPSAFTER experiment: after this many seconds from when N was armed,
// stop accepting real PPS so timing drifts into OCXO holdover. 0 / disarmed =
// never ignore (default). Deadline uses wrap-safe millis comparison.
extern uint32_t ignore_pps_after_s;

extern uint32_t ignore_pps_deadline_ms;

extern bool ignore_pps_armed;

