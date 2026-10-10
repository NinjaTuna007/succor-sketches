// Host and modem byte pipes for owtt_stick_v2.
//
// The sketch owns one FrameAssembler per direction and only calls
// fa_push / fa_pop_host / fa_pop_modem. fa_host_tx_header is also public
// because the host handler asks "was this a length-prefixed $B/$K/$U/$M/$E?"
// after a frame has already been popped.
//
// Text records end at newline. NM3 data is length-prefixed and the payload
// may contain CR/LF/NUL:
//   Host → modem: $B<xx><data>, $K<xx><data>, $U/$M/$E<xxx><yy><data>
//   Modem → host: #B<aaa><yy><data>[trailer]<CRLF>
//                 #U<yy><data>[trailer]<CRLF>
#pragma once

#include <stddef.h>
#include <stdint.h>

static constexpr size_t OWTT_LINE_MAX = 256;

struct FrameAssembler {
  uint8_t buf[OWTT_LINE_MAX];
  size_t len;
};

void fa_push(FrameAssembler &a, uint8_t b);
bool fa_pop_host(
  FrameAssembler &a, uint8_t *out, size_t out_sz, size_t *out_len);
bool fa_pop_modem(
  FrameAssembler &a, uint8_t *out, size_t out_sz, size_t *out_len);

// 1 = header ready, 0 = need more bytes, -1 = not a length-prefixed record.
int fa_host_tx_header(
  const uint8_t *buf, size_t len, size_t *payload_start, size_t *nbytes);
