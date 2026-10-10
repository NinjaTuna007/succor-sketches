#include "owtt_ubx.h"

#include <string.h>

enum UbxParserState : uint8_t {
  UBX_WAIT_SYNC1 = 0,
  UBX_WAIT_SYNC2,
  UBX_READ_CLASS,
  UBX_READ_ID,
  UBX_READ_LEN1,
  UBX_READ_LEN2,
  UBX_READ_PAYLOAD,
  UBX_READ_CK_A,
  UBX_READ_CK_B
};

struct UbxParser {
  UbxParserState state;
  uint8_t msg_class;
  uint8_t msg_id;
  uint16_t payload_len;
  uint16_t payload_index;
  uint8_t payload[20];
  uint8_t ck_a;
  uint8_t ck_b;
  uint8_t received_ck_a;
};

static constexpr uint16_t UBX_MAX_PAYLOAD = 1024;

static UbxParser gnss_ubx = {
  UBX_WAIT_SYNC1, 0, 0, 0, 0, {0}, 0, 0, 0
};

static uint32_t ok_count = 0;
static uint32_t bad_count = 0;

static void ubx_checksum_add(uint8_t b)
{
  gnss_ubx.ck_a = (uint8_t)(gnss_ubx.ck_a + b);
  gnss_ubx.ck_b = (uint8_t)(gnss_ubx.ck_b + gnss_ubx.ck_a);
}

void owtt_ubx_reset()
{
  gnss_ubx.state = UBX_WAIT_SYNC1;
  gnss_ubx.msg_class = 0;
  gnss_ubx.msg_id = 0;
  gnss_ubx.payload_len = 0;
  gnss_ubx.payload_index = 0;
  gnss_ubx.ck_a = 0;
  gnss_ubx.ck_b = 0;
  gnss_ubx.received_ck_a = 0;
}

uint32_t owtt_ubx_ok_count()
{
  return ok_count;
}

uint32_t owtt_ubx_bad_count()
{
  return bad_count;
}

OwttUbxFeed owtt_ubx_feed(uint8_t b, uint8_t timeutc_payload[20])
{
  switch (gnss_ubx.state) {
    case UBX_WAIT_SYNC1:
      if (b == 0xB5U) {
        gnss_ubx.state = UBX_WAIT_SYNC2;
      }
      return OWTT_UBX_NONE;

    case UBX_WAIT_SYNC2:
      if (b == 0x62U) {
        gnss_ubx.state = UBX_READ_CLASS;
        gnss_ubx.ck_a = 0;
        gnss_ubx.ck_b = 0;
      } else if (b != 0xB5U) {
        gnss_ubx.state = UBX_WAIT_SYNC1;
      }
      return OWTT_UBX_NONE;

    case UBX_READ_CLASS:
      gnss_ubx.msg_class = b;
      ubx_checksum_add(b);
      gnss_ubx.state = UBX_READ_ID;
      return OWTT_UBX_NONE;

    case UBX_READ_ID:
      gnss_ubx.msg_id = b;
      ubx_checksum_add(b);
      gnss_ubx.state = UBX_READ_LEN1;
      return OWTT_UBX_NONE;

    case UBX_READ_LEN1:
      gnss_ubx.payload_len = b;
      ubx_checksum_add(b);
      gnss_ubx.state = UBX_READ_LEN2;
      return OWTT_UBX_NONE;

    case UBX_READ_LEN2:
      gnss_ubx.payload_len |= ((uint16_t)b << 8);
      ubx_checksum_add(b);
      gnss_ubx.payload_index = 0;

      if (gnss_ubx.payload_len > UBX_MAX_PAYLOAD) {
        owtt_ubx_reset();
      } else if (gnss_ubx.payload_len == 0) {
        gnss_ubx.state = UBX_READ_CK_A;
      } else {
        gnss_ubx.state = UBX_READ_PAYLOAD;
      }
      return OWTT_UBX_NONE;

    case UBX_READ_PAYLOAD:
      if (gnss_ubx.payload_index < sizeof(gnss_ubx.payload)) {
        gnss_ubx.payload[gnss_ubx.payload_index] = b;
      }

      ubx_checksum_add(b);
      gnss_ubx.payload_index++;

      if (gnss_ubx.payload_index >= gnss_ubx.payload_len) {
        gnss_ubx.state = UBX_READ_CK_A;
      }
      return OWTT_UBX_NONE;

    case UBX_READ_CK_A:
      gnss_ubx.received_ck_a = b;
      gnss_ubx.state = UBX_READ_CK_B;
      return OWTT_UBX_NONE;

    case UBX_READ_CK_B: {
      const bool checksum_ok =
        gnss_ubx.received_ck_a == gnss_ubx.ck_a &&
        b == gnss_ubx.ck_b;

      if (checksum_ok) {
        ok_count++;
        const bool timeutc =
          gnss_ubx.msg_class == 0x01U &&
          gnss_ubx.msg_id == 0x21U &&
          gnss_ubx.payload_len == 20U;
        if (timeutc && timeutc_payload != nullptr) {
          memcpy(timeutc_payload, gnss_ubx.payload, 20);
        }
        owtt_ubx_reset();
        return timeutc ? OWTT_UBX_TIMEUTC : OWTT_UBX_FRAME;
      }

      bad_count++;
      owtt_ubx_reset();
      return OWTT_UBX_BAD;
    }
  }

  owtt_ubx_reset();
  return OWTT_UBX_NONE;
}
