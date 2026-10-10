#include "owtt_frame.h"

#include <string.h>

static bool fa_is_digit(uint8_t b)
{
  return b >= '0' && b <= '9';
}

static int fa_digits2(const uint8_t *buf, size_t offset, size_t len)
{
  if (offset + 2 > len) {
    return -1;
  }
  if (!fa_is_digit(buf[offset]) || !fa_is_digit(buf[offset + 1])) {
    return -2;
  }
  return (int)(buf[offset] - '0') * 10 + (int)(buf[offset + 1] - '0');
}

int fa_host_tx_header(
  const uint8_t *buf, size_t len, size_t *payload_start, size_t *nbytes)
{
  if (len == 0 || buf[0] != '$') {
    return -1;
  }
  if (len < 2) {
    return 0;
  }
  if (buf[1] == 'B' || buf[1] == 'K') {
    if (len < 4) {
      return 0;
    }
    const int n = fa_digits2(buf, 2, len);
    if (n < 0) {
      return -1;
    }
    *payload_start = 4;
    *nbytes = (size_t)n;
    return 1;
  }
  if (buf[1] == 'U' || buf[1] == 'M' || buf[1] == 'E') {
    if (len < 7) {
      return 0;
    }
    if (!fa_is_digit(buf[2]) || !fa_is_digit(buf[3]) || !fa_is_digit(buf[4])) {
      return -1;
    }
    const int n = fa_digits2(buf, 5, len);
    if (n < 0) {
      return -1;
    }
    *payload_start = 7;
    *nbytes = (size_t)n;
    return 1;
  }
  return -1;
}

static int fa_modem_rx_header(
  const uint8_t *buf, size_t len, size_t *payload_start, size_t *nbytes)
{
  if (len == 0 || buf[0] != '#') {
    return -1;
  }
  if (len < 2) {
    return 0;
  }
  if (buf[1] == 'B') {
    if (len < 7) {
      return 0;
    }
    for (size_t i = 2; i < 7; i++) {
      if (!fa_is_digit(buf[i])) {
        return -1;
      }
    }
    const int n = fa_digits2(buf, 5, len);
    *payload_start = 7;
    *nbytes = (size_t)n;
    return 1;
  }
  if (buf[1] == 'U') {
    if (len < 4) {
      return 0;
    }
    const int n = fa_digits2(buf, 2, len);
    if (n < 0) {
      return -1;
    }
    *payload_start = 4;
    *nbytes = (size_t)n;
    return 1;
  }
  return -1;
}

static void fa_drop_overflow(FrameAssembler &a)
{
  a.len = 0;
}

static bool fa_copy_out(
  const uint8_t *src, size_t n, uint8_t *out, size_t out_sz, size_t *out_len)
{
  if (n >= out_sz) {
    return false;
  }
  memcpy(out, src, n);
  out[n] = 0;
  *out_len = n;
  return true;
}

static bool fa_pop_newline(
  FrameAssembler &a, uint8_t *out, size_t out_sz, size_t *out_len)
{
  size_t nl = a.len;
  bool found = false;
  for (size_t i = 0; i < a.len; i++) {
    if (a.buf[i] == '\n') {
      nl = i;
      found = true;
      break;
    }
  }
  if (!found) {
    return false;
  }
  size_t n = nl;
  if (n > 0 && a.buf[n - 1] == '\r') {
    n--;
  }
  const size_t consumed = nl + 1;
  bool ok = false;
  if (n > 0) {
    ok = fa_copy_out(a.buf, n, out, out_sz, out_len);
  }
  memmove(a.buf, a.buf + consumed, a.len - consumed);
  a.len -= consumed;
  return ok;
}

bool fa_pop_host(
  FrameAssembler &a, uint8_t *out, size_t out_sz, size_t *out_len)
{
  if (a.len == 0) {
    return false;
  }
  size_t payload_start = 0;
  size_t nbytes = 0;
  const int hdr = fa_host_tx_header(a.buf, a.len, &payload_start, &nbytes);
  if (hdr == 0) {
    return false;
  }
  if (hdr == 1) {
    const size_t need = payload_start + nbytes;
    if (a.len < need) {
      return false;
    }
    if (!fa_copy_out(a.buf, need, out, out_sz, out_len)) {
      fa_drop_overflow(a);
      return false;
    }
    memmove(a.buf, a.buf + need, a.len - need);
    a.len -= need;
    return true;
  }
  return fa_pop_newline(a, out, out_sz, out_len);
}

bool fa_pop_modem(
  FrameAssembler &a, uint8_t *out, size_t out_sz, size_t *out_len)
{
  if (a.len == 0) {
    return false;
  }
  size_t payload_start = 0;
  size_t nbytes = 0;
  const int hdr = fa_modem_rx_header(a.buf, a.len, &payload_start, &nbytes);
  if (hdr == 0) {
    return false;
  }
  if (hdr == 1) {
    const size_t after_payload = payload_start + nbytes;
    if (a.len < after_payload) {
      return false;
    }
    size_t nl = a.len;
    bool found = false;
    for (size_t i = after_payload; i < a.len; i++) {
      if (a.buf[i] == '\n') {
        nl = i;
        found = true;
        break;
      }
    }
    if (!found) {
      return false;
    }
    size_t n = nl;
    if (n > 0 && a.buf[n - 1] == '\r') {
      n--;
    }
    const size_t consumed = nl + 1;
    if (!fa_copy_out(a.buf, n, out, out_sz, out_len)) {
      fa_drop_overflow(a);
      return false;
    }
    memmove(a.buf, a.buf + consumed, a.len - consumed);
    a.len -= consumed;
    return true;
  }
  return fa_pop_newline(a, out, out_sz, out_len);
}

void fa_push(FrameAssembler &a, uint8_t b)
{
  if (a.len >= OWTT_LINE_MAX) {
    fa_drop_overflow(a);
  }
  if (a.len < OWTT_LINE_MAX) {
    a.buf[a.len++] = b;
  }
}
