#pragma once
// Host commands ($Y $G $K $Z) and the acoustic schedule ($B out, #I back).
#include "owtt_config.h"

void schedule_first_transmitter_after_config();
void check_pending_config_timeout();
void handle_host_frame(const uint8_t *frame, size_t n);
void handle_modem_frame(const uint8_t *frame, size_t n);
void execute_auto_tx_if_due();
void sync_owtt_epd_status();

