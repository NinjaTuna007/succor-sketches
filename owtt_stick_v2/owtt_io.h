#pragma once
// Host USB and modem UART writes, plus the small string helpers
// ($Y ids, $G coordinate check) those paths share.
#include "owtt_config.h"

void host_send_frame(const uint8_t *data, size_t n);
void host_send_line(const char *line);
bool frame_as_cstring( const uint8_t *frame, size_t n, char *out, size_t out_sz);
void modem_send_bytes(const uint8_t *data, size_t n);
void modem_send_command_no_terminator(const char *cmd);
void modem_set_address(const char id3[4]);
bool is_digit_char(char c);
bool starts_with(const char *s, const char *prefix);
void trim_spaces(char *s);
bool parse_two_digit_len(const char *p, size_t *out_len);
bool valid_3_digit_id(const char *p);
bool parse_id3_u8(const char *p, char out_id[4]);
bool same_id3(const char *a, const char *b);
bool is_first_transmitter();
bool looks_like_gps_coords(const char *s);
void host_send_config_error(const char *cmd, const char *reason);
void copy_id3(char dst[4], const char src[4]);
char mode_to_char(BridgeMode mode);
void host_send_config_ok();

