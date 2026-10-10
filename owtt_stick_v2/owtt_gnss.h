#pragma once
// X20P UART bridge, UBX-NAV-TIMEUTC, and the PPS-to-UTC bind.
#include "owtt_config.h"

bool gnss_utc_is_fresh();
bool handle_gnss_debug_command(const char *line);
void maybe_emit_gnss_live_debug();
bool handle_gnss_utc_query(const char *line);
void maybe_send_gnss_boot_config();
void maybe_recover_gnss_link();
void gnss_uart_apply_extra_buffers();
void service_gnss_bridge();

