// Passive UBX byte parser for owtt_stick_v2.
//
// Feed every GNSS UART byte. The parser checks framing and the Fletcher
// checksum of every UBX frame. It copies a payload out only for
// UBX-NAV-TIMEUTC (class 0x01, id 0x21, 20 bytes). Binding that payload to
// a PPS epoch stays in the sketch.
#pragma once

#include <stdint.h>

enum OwttUbxFeed : uint8_t {
  OWTT_UBX_NONE = 0,
  OWTT_UBX_FRAME = 1,
  OWTT_UBX_TIMEUTC = 2,
  OWTT_UBX_BAD = 3
};

void owtt_ubx_reset();

// timeutc_payload is filled only when the return is OWTT_UBX_TIMEUTC.
OwttUbxFeed owtt_ubx_feed(uint8_t b, uint8_t timeutc_payload[20]);

uint32_t owtt_ubx_ok_count();
uint32_t owtt_ubx_bad_count();
