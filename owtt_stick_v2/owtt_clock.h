#pragma once
// GPT2 on the 10 MHz OCXO, PPS lock, holdover, and the die-temperature log.
#include "owtt_config.h"

uint32_t ticks_to_us(uint32_t ticks);
uint64_t ocxo_step_from_ppb(int32_t ppb);
void ocxo_apply_rate(uint64_t step_q32, OcxoCalState state);
char ocxo_cal_state_char();
void ocxo_eeprom_load();
void format_die_c(char *buf, size_t n);
void emit_tempmon_line();
bool sample_die_temp();
void service_die_temp();
void ocxo_cal_report();
void pps_cal_reset();
void ocxo_cal_refresh_now();
uint32_t OCXO_counter_us();
char timing_mode_code(TimingMode mode);
bool utc_timestamp_for_counter( uint32_t counter_us, uint64_t *out_timestamp_us, int64_t *out_epoch_second, uint32_t *out_fraction_us);
void gpt2_extclk_capture_init();
void process_timing_events();
bool consume_delta( int32_t *out_delta_us, bool *out_utc_valid, int64_t *out_utc_second, uint32_t *out_holdover_age_s, uint32_t *out_gnss_age_ms, char *out_timing_mode, uint32_t *out_decode_delay_ms);
const char *timing_mode_name();
void bench_epoch_stop();
void bench_epoch_start(int64_t unix_second);

