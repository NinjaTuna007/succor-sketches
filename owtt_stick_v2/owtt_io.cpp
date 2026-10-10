// USB host lines and the modem UART.

#include "owtt_state.h"
#include "owtt_io.h"
#include "owtt_clock.h"
#include "owtt_gnss.h"
#include "owtt_radio.h"


void host_send_frame(const uint8_t *data, size_t n)
{
  if (data == NULL || n == 0) {
    return;
  }
  HOST_SERIAL.write(data, n);
  HOST_SERIAL.write((uint8_t)'\r');
  HOST_SERIAL.write((uint8_t)'\n');

#if ENABLE_USB_DEBUG && USB_MIRROR_HOST_OUTPUT
  Serial.print("HOST SEES: ");
  Serial.write(data, n);
  Serial.println();
#endif
}


void host_send_line(const char *line)
{
  if (line == NULL) {
    return;
  }
  host_send_frame((const uint8_t *)line, strlen(line));
}


bool frame_as_cstring(
  const uint8_t *frame, size_t n, char *out, size_t out_sz)
{
  if (frame == NULL || out == NULL || n + 1 > out_sz) {
    return false;
  }
  for (size_t i = 0; i < n; i++) {
    if (frame[i] == 0) {
      return false;
    }
  }
  memcpy(out, frame, n);
  out[n] = '\0';
  return true;
}


void modem_send_bytes(const uint8_t *data, size_t n)
{
  if (data == NULL || n == 0) {
    return;
  }
  // Succorfish modem commands require no CR/LF terminator. Burst-write so the
  // inter-byte gap stays under the vendor 2 ms "E" threshold.
  MODEM_SERIAL.write(data, n);
  MODEM_SERIAL.flush();

#if ENABLE_USB_DEBUG
  Serial.print("Sent to Succor: ");
  Serial.write(data, n);
  Serial.println();
#endif
}


void modem_send_command_no_terminator(const char *cmd)
{
  if (cmd == NULL) {
    return;
  }
  modem_send_bytes((const uint8_t *)cmd, strlen(cmd));
}


void modem_set_address(const char id3[4])
{
  char cmd[8];
  snprintf(cmd, sizeof(cmd), "$A%s", id3);
  modem_send_command_no_terminator(cmd);
}


// =======================================================
// ===================== SMALL HELPERS ===================
// =======================================================

bool is_digit_char(char c)
{
  return c >= '0' && c <= '9';
}


bool starts_with(const char *s, const char *prefix)
{
  return strncmp(s, prefix, strlen(prefix)) == 0;
}


void trim_spaces(char *s)
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



bool parse_two_digit_len(const char *p, size_t *out_len)
{
  if (!is_digit_char(p[0]) || !is_digit_char(p[1])) {
    return false;
  }

  *out_len = (size_t)((p[0] - '0') * 10 + (p[1] - '0'));
  return true;
}



bool valid_3_digit_id(const char *p)
{
  return is_digit_char(p[0]) && is_digit_char(p[1]) && is_digit_char(p[2]);
}


bool parse_id3_u8(const char *p, char out_id[4])
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


bool same_id3(const char *a, const char *b)
{
  return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}


bool is_first_transmitter()
{
  return same_id3(cfg.listen_id, "000");
}


bool looks_like_gps_coords(const char *s)
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


void host_send_config_error(const char *cmd, const char *reason)
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


void copy_id3(char dst[4], const char src[4])
{
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = '\0';
}


char mode_to_char(BridgeMode mode)
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


void host_send_config_ok()
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

