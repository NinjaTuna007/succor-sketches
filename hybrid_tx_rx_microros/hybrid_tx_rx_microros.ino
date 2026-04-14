// micro-ROS + PPS/FLAG modem helper (Teensy 4.1)
//
// Namespace requirement:
//   - If MODE_TRANSMITTER == 1: ALL topics are under /tx/...
//   - If MODE_TRANSMITTER == 0: ALL topics are under /rx/...
//
// Topics (under the chosen root namespace):
//   /<root>/succorfish/msg/sent      (String)  - exactly what we wrote to modem UART (as-is)
//   /<root>/succorfish/msg/received  (String)  - full modem UART lines (as-is, without CRLF)
//   /<root>/succorfish/delta_t       (Int32)   - RX only, computed on FLAG rising edge
//
// Behavior:
//   - PPS ISR: store last_pps_us + request LED toggle
//   - TX: PPS triggers sending tx_cmd to modem UART + publish it to /.../msg/sent
//   - RX: FLAG rising publishes /.../delta_t once per edge
//   - BOTH: UART parsing publishes ONLY full lines, terminated by <CR><LF> (or tolerant lone '\n')
//
// Notes:
//   - DO NOT Serial.print() after set_microros_transports() (USB serial used by micro-ROS transport).
//   - ISRs do minimal work only (no UART, no rcl_publish).
//   - Modem UART is Serial1 at 9600 baud.
//   - TX jitter fix: restart Serial1 before each send + flush() to force a consistent offset.
//   - Modem requirement: commands must be sent as a contiguous string (avoid inter-byte gaps).

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

// --------------------- MODE SELECT ---------------------
// 1 = transmitter mode (PPS triggers modem broadcast)
// 0 = receiver mode (FLAG rising publishes delta_t)
#define MODE_TRANSMITTER 1
// -------------------------------------------------------

// --------------------- TOPIC ROOT ----------------------
// All topics will be prefixed with this root.
// Must be a compile-time constant C-string.
#if MODE_TRANSMITTER
  static const char TOPIC_ROOT[] = "/tx";
#else
  static const char TOPIC_ROOT[] = "/rx";
#endif
// -------------------------------------------------------

static constexpr uint8_t  PIN_PPS  = 40;
static constexpr uint8_t  PIN_FLAG = 41;

#define MODEM_SERIAL Serial1
static constexpr uint32_t MODEM_BAUD = 9600;

static constexpr size_t   LINE_MAX = 256;   // max published line length (excluding CRLF)

// ---------- micro-ROS objects ----------
static rcl_node_t node;
static rclc_support_t support;
static rcl_allocator_t allocator;

// Publishers
static rcl_publisher_t pub_sent;
static rcl_publisher_t pub_recv;

#if !MODE_TRANSMITTER
static rcl_publisher_t pub_delta;
#endif

// Messages
static std_msgs__msg__String msg_sent;
static std_msgs__msg__String msg_recv;

#if !MODE_TRANSMITTER
static std_msgs__msg__Int32  msg_delta;
#endif

// ---------- preallocated backing stores for ROS strings ----------
static char ros_sent_buf[LINE_MAX];
static char ros_recv_buf[LINE_MAX];

// ---------- topic name storage (must persist for lifetime of publisher) ----------
static char topic_sent[64];
static char topic_recv[64];
#if !MODE_TRANSMITTER
static char topic_delta[64];
#endif

// ---------- ISR-shared state ----------
static volatile uint32_t last_pps_us = 0;
static volatile bool pps_led_toggle_pending = false;

#if MODE_TRANSMITTER
static volatile bool pps_tx_pending = false;
#else
static volatile int32_t delta_us_isr = 0;
static volatile bool    delta_pending = false;
#endif

// ---------- modem RX line assembly ----------
static char   modem_line[LINE_MAX];
static size_t modem_len = 0;
static bool   saw_cr = false;

// ---------- helpers ----------
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

// Build "rooted" topic: out = "<TOPIC_ROOT><suffix>"
static void make_topic(char *out, size_t out_sz, const char *suffix)
{
  if (!suffix || suffix[0] != '/') die_blink(9);
  int n = snprintf(out, out_sz, "%s%s", TOPIC_ROOT, suffix);
  if (n < 0 || (size_t)n >= out_sz) die_blink(8);
}

// --------------------- ISRs ----------------------------
void isr_pps_rising()
{
  last_pps_us = micros();
  pps_led_toggle_pending = true;
#if MODE_TRANSMITTER
  pps_tx_pending = true;
#endif
}

#if !MODE_TRANSMITTER
void isr_flag_rising()
{
  const uint32_t now = micros();
  const uint32_t pps = last_pps_us;
  const uint32_t du  = (uint32_t)(now - pps);
  delta_us_isr = (int32_t)du;
  delta_pending = true;
}
#endif
// -------------------------------------------------------

// Publish helper: copies cstr into a preallocated ROS String message (no mallocs)
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

// Flush current assembled modem line
static void publish_modem_line_if_any()
{
  if (modem_len == 0) return;
  modem_line[modem_len] = '\0';
  publish_string(&pub_recv, &msg_recv, modem_line);
  modem_len = 0;
}

// UART pump: publish only full lines
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

#if MODE_TRANSMITTER
static const char tx_cmd[] = "$P007";

static void modem_send_cmd_and_publish(const char *cmd)
{
  const size_t len = strlen(cmd);
  if (len == 0 || len >= 128) die_blink(7);

  MODEM_SERIAL.end();
  MODEM_SERIAL.begin(MODEM_BAUD);
  MODEM_SERIAL.write((const uint8_t*)cmd, len);
  MODEM_SERIAL.flush();
  publish_string(&pub_sent, &msg_sent, cmd);
}
#endif

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  MODEM_SERIAL.begin(MODEM_BAUD);

  pinMode(PIN_PPS, INPUT);
  pinMode(PIN_FLAG, INPUT);

  set_microros_transports();

  while (RMW_RET_OK != rmw_uros_ping_agent(500, 1)) {
    digitalWrite(LED_BUILTIN, HIGH); delay(50);
    digitalWrite(LED_BUILTIN, LOW);  delay(450);
  }

  attachInterrupt(digitalPinToInterrupt(PIN_PPS), isr_pps_rising, RISING);
#if !MODE_TRANSMITTER
  attachInterrupt(digitalPinToInterrupt(PIN_FLAG), isr_flag_rising, RISING);
#endif

  allocator = rcl_get_default_allocator();
  require_ok(rclc_support_init(&support, 0, NULL, &allocator), 2);

#if MODE_TRANSMITTER
  require_ok(rclc_node_init_default(&node, "succorfish_transmitter", "", &support), 3);
#else
  require_ok(rclc_node_init_default(&node, "succorfish_receiver", "", &support), 3);
#endif

  make_topic(topic_sent, sizeof(topic_sent), "/succorfish/msg/sent");
  make_topic(topic_recv, sizeof(topic_recv), "/succorfish/msg/received");
#if !MODE_TRANSMITTER
  make_topic(topic_delta, sizeof(topic_delta), "/succorfish/delta_t");
#endif

  // --- Best-effort publishers ---
  require_ok(
    rclc_publisher_init_best_effort(&pub_sent, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      topic_sent),
    4
  );
  require_ok(
    rclc_publisher_init_best_effort(&pub_recv, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      topic_recv),
    5
  );
#if !MODE_TRANSMITTER
  require_ok(
    rclc_publisher_init_best_effort(&pub_delta, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      topic_delta),
    6
  );
#endif

  // Init messages and bind static buffers
  std_msgs__msg__String__init(&msg_sent);
  msg_sent.data.data = ros_sent_buf;
  msg_sent.data.capacity = sizeof(ros_sent_buf);
  msg_sent.data.size = 0;

  std_msgs__msg__String__init(&msg_recv);
  msg_recv.data.data = ros_recv_buf;
  msg_recv.data.capacity = sizeof(ros_recv_buf);
  msg_recv.data.size = 0;

#if !MODE_TRANSMITTER
  std_msgs__msg__Int32__init(&msg_delta);
#endif

#if MODE_TRANSMITTER
  publish_string(&pub_sent, &msg_sent, "transmitter_ready");
#else
  publish_string(&pub_sent, &msg_sent, "receiver_ready");
#endif
}

void loop()
{
  if (pps_led_toggle_pending) {
    noInterrupts();
    pps_led_toggle_pending = false;
    interrupts();
    digitalToggleFast(LED_BUILTIN);
  }

#if MODE_TRANSMITTER
  if (pps_tx_pending) {
    noInterrupts();
    pps_tx_pending = false;
    interrupts();
    modem_send_cmd_and_publish(tx_cmd);
  }
#else
  if (delta_pending) {
    noInterrupts();
    int32_t du = delta_us_isr;
    delta_pending = false;
    interrupts();
    msg_delta.data = du;
    (void)rcl_publish(&pub_delta, &msg_delta, NULL);
  }
#endif

  pump_modem_rx_publish_full_lines();
  delay(1);
}
