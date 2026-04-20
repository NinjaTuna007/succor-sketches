// hybrid_microros_counter_sketch_dual_capture.ino
//
// Teensy 4.1 (i.MX RT1062) + micro-ROS
// HYBRID (TX/RX) via compile-time switch.
//
// Key points:
//   - NO PPS interrupt.
//   - PPS timestamp is hardware-captured (GPT2 CAPIN2 -> ICR2 -> IF2).
//   - MODEM_FLAG timestamp is hardware-captured (GPT2 CAPIN1 -> ICR1 -> IF1).
//   - loop() polls GPT2_SR IF1/IF2, reads ICR1/ICR2, publishes only from loop.
//
// RX publishes:
//   /rx/succorfish/pps_time_us   (UInt32)
//   /rx/succorfish/flag_time_us  (UInt32)
//   /rx/succorfish/delta_t       (Int32)
//
// TX publishes:
//   /tx/succorfish/pps_time_us   (UInt32)
//   /tx/succorfish/msg/sent      (String)
//   /tx/succorfish/msg/received  (String)
//
// Node names:
//   TX: succorfish_tx_node
//   RX: succorfish_rx_node

#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/string.h>

// ===================== MODE SELECT =====================
#define MODE_TRANSMITTER 1  // 1 = TX node, 0 = RX node
// =======================================================

// Optional: publish simple overrun counters (useful for proving you are not missing captures)
#define ENABLE_DIAG_COUNTERS 1

// ===================== NODE + NAMESPACE =================
#if MODE_TRANSMITTER
  static const char TOPIC_ROOT[] = "/tx";
  static const char NODE_NAME[]  = "succorfish_tx_node";
#else
  static const char TOPIC_ROOT[] = "/rx";
  static const char NODE_NAME[]  = "succorfish_rx_node";
#endif
// =======================================================

// ===================== PINS ============================
// FLAG capture: GPT2 CAPIN1 known-good on Teensy 4.1 pin 15 (GPIO_AD_B1_03 ALT8)
static constexpr uint8_t PIN_FLAG_CAPTURE = 15;

// PPS capture: GPT2 CAPIN2 on Teensy 4.1 pin 40 (GPIO_AD_B1_04 ALT8)
static constexpr uint8_t PIN_PPS_CAPTURE  = 40;

// Optional scope diagnostic pin:
// set HIGH on PPS capture seen, LOW on FLAG capture consumed (RX)
static constexpr uint8_t PIN_DIAG = 42;
// =======================================================

// ===================== MODEM UART ======================
#define MODEM_SERIAL Serial1
static constexpr uint32_t MODEM_BAUD = 9600;
static constexpr size_t   LINE_MAX   = 256;
// =======================================================

// ===================== TOPICS ==========================
static char topic_pps[96];

#if MODE_TRANSMITTER
static char topic_sent[96];
static char topic_recv[96];
#else
static char topic_flag[96];
static char topic_delta[96];
#endif

#if ENABLE_DIAG_COUNTERS
static char topic_diag[96];
#endif
// =======================================================

// ===================== micro-ROS objects ===============
static rcl_node_t node;
static rclc_support_t support;
static rcl_allocator_t allocator;

// Common publisher
static rcl_publisher_t pub_pps;
static std_msgs__msg__UInt32 msg_pps;

#if MODE_TRANSMITTER
static rcl_publisher_t pub_sent;
static rcl_publisher_t pub_recv;
static std_msgs__msg__String msg_sent;
static std_msgs__msg__String msg_recv;
static char ros_sent_buf[LINE_MAX];
static char ros_recv_buf[LINE_MAX];
#else
static rcl_publisher_t pub_flag;
static rcl_publisher_t pub_delta;
static std_msgs__msg__UInt32 msg_flag;
static std_msgs__msg__Int32  msg_delta;
#endif

#if ENABLE_DIAG_COUNTERS
static rcl_publisher_t pub_diag;
static std_msgs__msg__UInt32 msg_diag;
#endif
// =======================================================

// ===================== Helpers =========================
static void die_blink(uint8_t blinks)
{
  pinMode(LED_BUILTIN, OUTPUT);
  while (1) {
    for (uint8_t i = 0; i < blinks; i++) {
      digitalWrite(LED_BUILTIN, HIGH); delay(120);
      digitalWrite(LED_BUILTIN, LOW);  delay(180);
    }
    delay(800);
  }
}

static void require_ok(rcl_ret_t rc, uint8_t blink_stage)
{
  if (rc != RCL_RET_OK) die_blink(blink_stage);
}

static void make_topic(char *out, size_t out_sz, const char *suffix)
{
  if (!suffix || suffix[0] != '/') die_blink(9);
  int n = snprintf(out, out_sz, "%s%s", TOPIC_ROOT, suffix);
  if (n < 0 || (size_t)n >= out_sz) die_blink(8);
}

#if MODE_TRANSMITTER
static inline void publish_string(rcl_publisher_t *pub,
                                  std_msgs__msg__String *msg,
                                  const char *cstr)
{
  const size_t cap = msg->data.capacity;
  size_t n = strnlen(cstr, cap - 1);
  memcpy(msg->data.data, cstr, n);
  msg->data.data[n] = '\0';
  msg->data.size = n;
  (void)rcl_publish(pub, msg, NULL);
}
#endif
// =======================================================

// ===================== UART RX (TX only) ===============
#if MODE_TRANSMITTER
static char   modem_line[LINE_MAX];
static size_t modem_len = 0;
static bool   saw_cr    = false;

static void publish_modem_line_if_any()
{
  if (modem_len == 0) return;
  modem_line[modem_len] = '\0';
  publish_string(&pub_recv, &msg_recv, modem_line);
  modem_len = 0;
}

static void pump_modem_rx_publish_full_lines()
{
  while (MODEM_SERIAL.available()) {
    char c = (char)MODEM_SERIAL.read();
    if (saw_cr) { saw_cr = false; if (c == '\n') continue; }
    if (c == '\r') { publish_modem_line_if_any(); saw_cr = true; continue; }
    if (c == '\n') { publish_modem_line_if_any(); continue; }
    if (modem_len < (LINE_MAX - 1)) modem_line[modem_len++] = c;
  }
}
#endif
// =======================================================

// ===================== GPT2 dual capture @ 1 MHz =======
//
// GPT2_CNT ticks at 1 MHz => 1 tick = 1 us.
// CAPIN1 -> ICR1 -> IF1 (MODEM_FLAG)  on pin 15 (GPIO_AD_B1_03 ALT8)
// CAPIN2 -> ICR2 -> IF2 (PPS)         on pin 40 (GPIO_AD_B1_04 ALT8)
//
// We do NOT enable GPT2 interrupts; loop polls IF1/IF2.

static void gpt2_dual_capture_init_1mhz()
{
  // clocks on
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON) | CCM_CCGR0_GPT2_SERIAL(CCM_CCGR_ON);

  // ---------- CAPIN1 (FLAG) on pin 15: GPIO_AD_B1_03 ALT8 ----------
  // Daisy selection: keep as 1 (Teensy known-good for this pad)
  IOMUXC_GPT2_IPP_IND_CAPIN1_SELECT_INPUT = 1;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_03 = 8;       // ALT8 = GPT2_CAPTURE1
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_03 = 0x13000; // pulldown + hysteresis

  // ---------- CAPIN2 (PPS) on pin 40: GPIO_AD_B1_04 ALT8 ----------
  // Daisy selection: for CAPIN2 there are multiple pads; for GPIO_AD_B1_04 it is commonly 1.
  // If you ever see IF2 not firing, flip this between 0 and 1 (but your manual indicates AD_B1_04 is a valid option).
  IOMUXC_GPT2_IPP_IND_CAPIN2_SELECT_INPUT = 1;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_04 = 8;       // ALT8 = GPT2_CAPTURE2
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_04 = 0x13000; // pulldown + hysteresis

  // stop/reset
  GPT2_CR = 0;
  GPT2_PR = 0;
  GPT2_SR = 0x3F;  // clear status
  GPT2_IR = 0;     // no interrupts (polling only)

  // prescale 24 MHz -> 1 MHz
  GPT2_PR = 23;

  // start free-run + enable capture on input1 and input2
  GPT2_CR = GPT_CR_EN | GPT_CR_CLKSRC(1) | GPT_CR_FRR | GPT_CR_IM1(1) | GPT_CR_IM2(1);
}
// =======================================================

// ===================== TX send (TX only) ===============
#if MODE_TRANSMITTER
static const char tx_cmd[] = "$P007";

static void modem_send_cmd_and_publish(const char *cmd)
{
  const size_t len = strlen(cmd);
  if (len == 0 || len >= 128) die_blink(7);

  // optional UART restart for phase experiments
  MODEM_SERIAL.end();
  MODEM_SERIAL.begin(MODEM_BAUD);

  MODEM_SERIAL.write((const uint8_t*)cmd, len);
  MODEM_SERIAL.flush();

  publish_string(&pub_sent, &msg_sent, cmd);
}
#endif
// =======================================================

// ===================== Capture state ===================
// Frame one PPS cycle:
//   - PPS capture (IF2) -> pps_us -> arm flag (RX) / trigger TX
//   - FLAG capture (IF1) while armed -> flag_us -> publish delta -> disarm
static uint32_t pps_us = 0;

#if !MODE_TRANSMITTER
static bool armed_for_flag = false;
#endif

#if ENABLE_DIAG_COUNTERS
static uint32_t cnt_pps = 0;
static uint32_t cnt_flag = 0;
static uint32_t cnt_flag_ignored = 0; // flag edges arriving when not armed
static uint32_t cnt_pps_overwrite = 0; // IF2 already set when we come back
static uint32_t cnt_flag_overwrite = 0; // IF1 already set when we come back
#endif
// =======================================================

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(PIN_DIAG, OUTPUT);
  digitalWriteFast(PIN_DIAG, LOW);

  pinMode(PIN_FLAG_CAPTURE, INPUT);
  pinMode(PIN_PPS_CAPTURE, INPUT);

  MODEM_SERIAL.begin(MODEM_BAUD);

  set_microros_transports();

  while (RMW_RET_OK != rmw_uros_ping_agent(500, 1)) {
    digitalWrite(LED_BUILTIN, HIGH); delay(50);
    digitalWrite(LED_BUILTIN, LOW);  delay(450);
  }

  gpt2_dual_capture_init_1mhz();

  allocator = rcl_get_default_allocator();
  require_ok(rclc_support_init(&support, 0, NULL, &allocator), 2);
  require_ok(rclc_node_init_default(&node, NODE_NAME, "", &support), 3);

  // Common topic
  make_topic(topic_pps, sizeof(topic_pps), "/succorfish/pps_time_us");
  require_ok(
    rclc_publisher_init_best_effort(
      &pub_pps, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
      topic_pps),
    4);
  std_msgs__msg__UInt32__init(&msg_pps);

#if MODE_TRANSMITTER
  make_topic(topic_sent, sizeof(topic_sent), "/succorfish/msg/sent");
  make_topic(topic_recv, sizeof(topic_recv), "/succorfish/msg/received");

  require_ok(
    rclc_publisher_init_best_effort(
      &pub_sent, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      topic_sent),
    5);
  require_ok(
    rclc_publisher_init_best_effort(
      &pub_recv, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      topic_recv),
    6);

  std_msgs__msg__String__init(&msg_sent);
  msg_sent.data.data = ros_sent_buf;
  msg_sent.data.size = 0;
  msg_sent.data.capacity = sizeof(ros_sent_buf);

  std_msgs__msg__String__init(&msg_recv);
  msg_recv.data.data = ros_recv_buf;
  msg_recv.data.size = 0;
  msg_recv.data.capacity = sizeof(ros_recv_buf);

#else
  make_topic(topic_flag,  sizeof(topic_flag),  "/succorfish/flag_time_us");
  make_topic(topic_delta, sizeof(topic_delta), "/succorfish/delta_t");

  require_ok(
    rclc_publisher_init_best_effort(
      &pub_flag, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
      topic_flag),
    5);
  require_ok(
    rclc_publisher_init_best_effort(
      &pub_delta, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      topic_delta),
    6);

  std_msgs__msg__UInt32__init(&msg_flag);
  std_msgs__msg__Int32__init(&msg_delta);

  armed_for_flag = false;
#endif

#if ENABLE_DIAG_COUNTERS
  make_topic(topic_diag, sizeof(topic_diag), "/succorfish/diag_counts");
  require_ok(
    rclc_publisher_init_best_effort(
      &pub_diag, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
      topic_diag),
    7);
  std_msgs__msg__UInt32__init(&msg_diag);
#endif
}

void loop()
{
#if MODE_TRANSMITTER
  pump_modem_rx_publish_full_lines();
#endif

#if ENABLE_DIAG_COUNTERS
  // detect potential overwrites (we came back and IF is still set from a previous event)
  static bool last_if2 = false;
  static bool last_if1 = false;
  bool now_if2 = (GPT2_SR & GPT_SR_IF2);
  bool now_if1 = (GPT2_SR & GPT_SR_IF1);
  if (last_if2 && now_if2) cnt_pps_overwrite++;
  if (last_if1 && now_if1) cnt_flag_overwrite++;
  last_if2 = now_if2;
  last_if1 = now_if1;
#endif

  // ---------- PPS capture (IF2 -> ICR2) ----------
  if (GPT2_SR & GPT_SR_IF2) {
    pps_us = GPT2_ICR2;
    GPT2_SR = GPT_SR_IF2; // clear IF2

    digitalWriteFast(PIN_DIAG, HIGH);

    msg_pps.data = pps_us;
    (void)rcl_publish(&pub_pps, &msg_pps, NULL);

#if ENABLE_DIAG_COUNTERS
    cnt_pps++;
    // publish diag occasionally (cheap throttle)
    if ((cnt_pps & 0x3F) == 0) { // every 64 PPS
      // pack counts roughly (you can expand to a custom msg later)
      // Here we just publish cnt_pps as a heartbeat; keep it simple.
      msg_diag.data = cnt_pps;
      (void)rcl_publish(&pub_diag, &msg_diag, NULL);
    }
#endif

#if MODE_TRANSMITTER
    modem_send_cmd_and_publish(tx_cmd);
#else
    armed_for_flag = true;
#endif
  }

#if !MODE_TRANSMITTER
  // ---------- FLAG capture (IF1 -> ICR1) ----------
  if (GPT2_SR & GPT_SR_IF1) {
    if (!armed_for_flag) {
      // We saw a flag edge not associated with the current PPS frame.
      // Clear and count it, but ignore for delta computation.
      (void)GPT2_ICR1;
      GPT2_SR = GPT_SR_IF1;
      digitalWriteFast(PIN_DIAG, LOW);
#if ENABLE_DIAG_COUNTERS
      cnt_flag_ignored++;
#endif
    } else {
      uint32_t flag_us = GPT2_ICR1;
      GPT2_SR = GPT_SR_IF1; // clear IF1
      digitalWriteFast(PIN_DIAG, LOW);

      msg_flag.data = flag_us;
      (void)rcl_publish(&pub_flag, &msg_flag, NULL);

      uint32_t du = (uint32_t)(flag_us - pps_us);
      msg_delta.data = (int32_t)du;
      (void)rcl_publish(&pub_delta, &msg_delta, NULL);

#if ENABLE_DIAG_COUNTERS
      cnt_flag++;
#endif
      armed_for_flag = false;
      digitalToggleFast(LED_BUILTIN);
    }
  }
#endif
}