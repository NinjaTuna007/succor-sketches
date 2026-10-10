#include "owtt_state.h"

BridgeConfig cfg = {
  MODE_WIRE,
  "000",  // own_id
  "000",  // listen_id
  1, // period_pps
  false, // tx_scheduled
  0 // next_tx_epoch_count
};

size_t latest_gps_len = 0;

bool latest_gps_valid = false;

uint32_t payload_valid_epochs = 1;

uint32_t latest_gps_epoch_count = 0;

size_t latest_telem_len = 0;

bool latest_telem_valid = false;

uint32_t latest_telem_epoch_count = 0;

TxWarningReason tx_last_warning = TX_WARN_NONE;

bool tx_auto_due = false;

GnssUtcState gnss_utc = {
  false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

bool current_epoch_utc_valid = false;

int64_t current_epoch_utc_second = 0;

uint32_t gnss_uart_baud = GNSS_INITIAL_BAUD;

uint32_t gnss_timeutc_config_due_ms = 0;

uint32_t gnss_last_good_frame_ms = 0;

uint32_t gnss_link_epoch_ms = 0;

FrameAssembler host_fa  = {{0}, 0};

FrameAssembler modem_fa = {{0}, 0};

TimingMode timing_mode = TIMING_WAIT_PPS;

bool bench_epoch_active = false;

uint32_t holdover_age_s = 0;

uint32_t epoch_count = 0;

uint32_t current_epoch_us = 0;

bool pps_valid = false;

bool pending_delta_valid = false; // Calculates deltas only if received GPS coords from a transmitter

int32_t ocxo_rate_ppb = 0;

bool ocxo_cal_auto = true;

bool die_c_valid = false;

uint32_t ignore_pps_after_s = 0;

uint32_t ignore_pps_deadline_ms = 0;

bool ignore_pps_armed = false;

