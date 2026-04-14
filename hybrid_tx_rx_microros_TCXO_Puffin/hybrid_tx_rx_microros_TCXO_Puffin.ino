// hybrid_tx_rx_microros_TCXO_Puffin.ino
//
// Teensy 4.1
// HYBRID (TX/RX) via compile-time switch.
//
// Main idea:
//   - GPT2 runs from external SiT5501 clock on pin 14
//   - GPT2 counter is prescaled to 1 MHz => 1 tick = 1 us
//   - MODEM_FLAG is hardware-captured on GPT2 CAPIN1 (pin 15)
//   - PPS is optionally hardware-captured on GPT2 CAPIN2 (pin 40)
//   - If PPS is disabled, a GPT2 compare event every 1,000,000 ticks is used instead
//
// Trigger source options:
//   USE_PPS_CAPTURE = 1 -> PPS on pin 40 defines the 1-second frame
//   USE_PPS_CAPTURE = 0 -> GPT2 OCR1 compare every 1,000,000 ticks defines the 1-second frame
//
// micro-ROS options:
//   ENABLE_MICROROS = 1 -> build with micro-ROS publishers enabled
//   ENABLE_MICROROS = 0 -> build without micro-ROS, serial debug only

#include <Arduino.h>

// ===================== FEATURE SWITCHES =================
#define MODE_TRANSMITTER    0  // 1 = TX node, 0 = RX node
#define USE_PPS_CAPTURE     1   // 1 = use PPS hardware capture on pin 40
                                 // 0 = use GPT2 compare at 1,000,000 ticks
#define ENABLE_MICROROS     0   // 1 = enable micro-ROS, 0 = no micro-ROS
#define ENABLE_DIAG_COUNTERS 0
#define ENABLE_SERIAL_DEBUG 1
#define SERIAL_DEBUG_BAUD   115200
#define SERIAL_DEBUG_PERIOD_MS 1000

#define RESTART_MODEM_UART_EACH_SEND 0   // 1 = MODEM_SERIAL.end()/begin() every send
                                         // 0 = keep UART continuously open
// =======================================================

#if ENABLE_MICROROS
  #include <micro_ros_arduino.h>
  #include <rcl/rcl.h>
  #include <rclc/rclc.h>
  #include <std_msgs/msg/int32.h>
  #include <std_msgs/msg/u_int32.h>
  #include <std_msgs/msg/string.h>
#endif

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
static constexpr uint8_t PIN_SIT5501_CLOCK = 14;
static constexpr uint8_t PIN_FLAG_CAPTURE   = 15;
static constexpr uint8_t PIN_PPS_CAPTURE    = 40;
// =======================================================

// ===================== MODEM UART ======================
#define MODEM_SERIAL Serial1
static constexpr uint32_t MODEM_BAUD = 9600;
static constexpr size_t   LINE_MAX   = 256;
// =======================================================

// ===================== micro-ROS objects ===============
#if ENABLE_MICROROS
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

static rcl_node_t node;
static rclc_support_t support;
static rcl_allocator_t allocator;

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
#endif
// =======================================================

// ===================== Debug state =====================
static volatile uint32_t dbg_trigger_count = 0;
static volatile uint32_t dbg_prev_trigger_us = 0;
static volatile uint32_t dbg_last_trigger_us = 0;
static volatile int32_t  dbg_last_trigger_delta_us = 0;

static volatile uint32_t dbg_flag_count = 0;
static volatile uint32_t dbg_last_flag_us = 0;
static volatile int32_t  dbg_last_flag_delta_us = 0;

static volatile uint32_t dbg_ignored_flag_count = 0;
static volatile uint32_t dbg_last_sit_counter = 0;
static volatile uint32_t dbg_last_teensy_micros = 0;
// =======================================================

// ===================== Helpers =========================
static void die_blink(uint8_t blinks)
{
  (void)blinks;
  while (1) {
    delay(1000);
  }
}

#if ENABLE_MICROROS
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
#endif

static inline uint32_t sit_counter_us()
{
  return GPT2_CNT;
}
// =======================================================

// ===================== UART RX (TX only) ===============
#if MODE_TRANSMITTER
static char   modem_line[LINE_MAX];
static size_t modem_len = 0;
static bool   saw_cr    = false;

#if ENABLE_MICROROS
static void publish_modem_line_if_any()
{
  if (modem_len == 0) return;
  modem_line[modem_len] = '\0';
  publish_string(&pub_recv, &msg_recv, modem_line);
  modem_len = 0;
}
#else
static void publish_modem_line_if_any()
{
  if (modem_len == 0) return;
  modem_line[modem_len] = '\0';
#if ENABLE_SERIAL_DEBUG
  Serial.printf("MODEM_RX=\"%s\"\n", modem_line);
#endif
  modem_len = 0;
}
#endif

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

// ===================== GPT2 compare fallback state =====
#if !USE_PPS_CAPTURE
static volatile bool gpt2_compare_pending = false;
static volatile uint32_t gpt2_compare_stamp = 0;
static uint32_t gpt2_next_compare = 0;
#endif
// =======================================================

// ===================== GPT2 init =======================
static void gpt2_extclk_dual_capture_init_1mhz()
{
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON) | CCM_CCGR0_GPT2_SERIAL(CCM_CCGR_ON);

  GPT2_CR = 0;
  GPT2_PR = 0;
  GPT2_SR = 0x3F;
  GPT2_IR = 0;

  // External clock on pin 14: GPIO_AD_B1_02 ALT8
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;
  IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT = 1;

  // CAPIN1 (FLAG) on pin 15: GPIO_AD_B1_03 ALT8
  IOMUXC_GPT2_IPP_IND_CAPIN1_SELECT_INPUT = 1;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_03 = 8;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_03 = 0x13000;

  // CAPIN2 (PPS) on pin 40: GPIO_AD_B1_04 ALT8
  IOMUXC_GPT2_IPP_IND_CAPIN2_SELECT_INPUT = 1;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_04 = 8;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_04 = 0x13000;

  // External clock 10 MHz, prescaler /10 => 1 MHz
  GPT2_PR = GPT_PR_PRESCALER(9);

  // Free-run, capture on input1 and input2
  GPT2_CR = GPT_CR_CLKSRC(3) | GPT_CR_FRR | GPT_CR_IM1(1) | GPT_CR_IM2(1);

#if !USE_PPS_CAPTURE
  uint32_t now = GPT2_CNT;
  gpt2_next_compare = now + 1000000UL;
  GPT2_OCR1 = gpt2_next_compare;
  GPT2_IR = GPT_IR_OF1IE;
  NVIC_CLEAR_PENDING(IRQ_GPT2);
  attachInterruptVector(IRQ_GPT2, GPT2_IRQHandler);
  NVIC_ENABLE_IRQ(IRQ_GPT2);
#endif

  GPT2_CR |= GPT_CR_EN;
}

#if !USE_PPS_CAPTURE
extern "C" void GPT2_IRQHandler()
{
  uint32_t sr = GPT2_SR;

  if (sr & GPT_SR_OF1) {
    gpt2_compare_stamp = GPT2_OCR1;
    gpt2_compare_pending = true;

    GPT2_SR = GPT_SR_OF1;

    gpt2_next_compare += 1000000UL;
    GPT2_OCR1 = gpt2_next_compare;
  }

  asm volatile("dsb");
}
#endif
// =======================================================

// ===================== TX send (TX only) ===============
#if MODE_TRANSMITTER
static const char tx_cmd[] = "$P007";

static void modem_send_cmd_and_publish(const char *cmd)
{
  const size_t len = strlen(cmd);
  if (len == 0 || len >= 128) die_blink(7);

#if RESTART_MODEM_UART_EACH_SEND
  MODEM_SERIAL.end();
  MODEM_SERIAL.begin(MODEM_BAUD);
#endif

  MODEM_SERIAL.write((const uint8_t*)cmd, len);
  MODEM_SERIAL.flush();

#if ENABLE_MICROROS
  publish_string(&pub_sent, &msg_sent, cmd);
#else
  #if ENABLE_SERIAL_DEBUG
  Serial.printf("MODEM_TX=\"%s\" | UART_restart=%d\n", cmd, RESTART_MODEM_UART_EACH_SEND);
  #endif
#endif
}
#endif
// =======================================================

// ===================== Capture state ===================
static uint32_t pps_us = 0;

#if !MODE_TRANSMITTER
static bool armed_for_flag = false;
#endif

#if ENABLE_DIAG_COUNTERS
static uint32_t cnt_pps = 0;
static uint32_t cnt_flag = 0;
static uint32_t cnt_flag_ignored = 0;
static uint32_t cnt_pps_overwrite = 0;
static uint32_t cnt_flag_overwrite = 0;
#endif
// =======================================================

// ===================== Serial debug ====================
#if ENABLE_SERIAL_DEBUG
static void print_debug_snapshot()
{
  uint32_t sit_now;
  uint32_t teensy_now;
  uint32_t trigger_count;
  uint32_t prev_trigger_us;
  uint32_t last_trigger_us;
  int32_t  last_trigger_delta_us;
  uint32_t flag_count;
  uint32_t last_flag_us;
  int32_t  last_flag_delta_us;
  uint32_t ignored_flag_count;

  noInterrupts();
  sit_now                = sit_counter_us();
  teensy_now             = micros();
  trigger_count          = dbg_trigger_count;
  prev_trigger_us        = dbg_prev_trigger_us;
  last_trigger_us        = dbg_last_trigger_us;
  last_trigger_delta_us  = dbg_last_trigger_delta_us;
  flag_count             = dbg_flag_count;
  last_flag_us           = dbg_last_flag_us;
  last_flag_delta_us     = dbg_last_flag_delta_us;
  ignored_flag_count     = dbg_ignored_flag_count;
  dbg_last_sit_counter   = sit_now;
  dbg_last_teensy_micros = teensy_now;
  interrupts();

#if USE_PPS_CAPTURE
  Serial.printf(
    "SRC=PPS | SIT_now=%u us | TeensyMicros=%u us | PPS_count=%u | PPS_ts=%u us | prev_PPS_ts=%u us | dPPS=%ld us | FLAG_count=%u | FLAG_ts=%u us | dFLAG=%ld us | FLAG_ignored=%u\n",
    sit_now,
    teensy_now,
    trigger_count,
    last_trigger_us,
    prev_trigger_us,
    (long)last_trigger_delta_us,
    flag_count,
    last_flag_us,
    (long)last_flag_delta_us,
    ignored_flag_count
  );
#else
  Serial.printf(
    "SRC=CMP | SIT_now=%u us | TeensyMicros=%u us | CMP_count=%u | CMP_ts=%u us | prev_CMP_ts=%u us | dCMP=%ld us | FLAG_count=%u | FLAG_ts=%u us | dFLAG=%ld us | FLAG_ignored=%u\n",
    sit_now,
    teensy_now,
    trigger_count,
    last_trigger_us,
    prev_trigger_us,
    (long)last_trigger_delta_us,
    flag_count,
    last_flag_us,
    (long)last_flag_delta_us,
    ignored_flag_count
  );
#endif
}
#endif
// =======================================================

void setup()
{
#if ENABLE_SERIAL_DEBUG
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(500);
  Serial.println("Booting hybrid_tx_rx_microros_TCXO_Puffin...");
  #if ENABLE_MICROROS
  Serial.println("micro-ROS: ENABLED");
  #else
  Serial.println("micro-ROS: DISABLED");
  #endif
  Serial.printf("RESTART_MODEM_UART_EACH_SEND=%d\n", RESTART_MODEM_UART_EACH_SEND);
#endif

#if ENABLE_SERIAL_DEBUG
  uint32_t reset_cause = SRC_SRSR;
  Serial.printf("Reset cause register SRC_SRSR = 0x%08lX\n", (unsigned long)reset_cause);
  if (CrashReport) {
    Serial.print(CrashReport);
  }
#endif

  pinMode(PIN_SIT5501_CLOCK, INPUT);
  pinMode(PIN_FLAG_CAPTURE, INPUT);
  pinMode(PIN_PPS_CAPTURE, INPUT);

  MODEM_SERIAL.begin(MODEM_BAUD);

#if ENABLE_MICROROS
  set_microros_transports();

  #if ENABLE_SERIAL_DEBUG
  Serial.println("Waiting for micro-ROS agent...");
  #endif

  while (RMW_RET_OK != rmw_uros_ping_agent(500, 1)) {
    #if ENABLE_SERIAL_DEBUG
    Serial.println("micro-ROS agent not reachable yet...");
    #endif
    delay(500);
  }

  #if ENABLE_SERIAL_DEBUG
  Serial.println("micro-ROS agent connected.");
  #endif
#endif

  gpt2_extclk_dual_capture_init_1mhz();

#if ENABLE_SERIAL_DEBUG
  Serial.println("GPT2 external-clock capture/compare initialized.");
#endif

#if ENABLE_MICROROS
  allocator = rcl_get_default_allocator();
  require_ok(rclc_support_init(&support, 0, NULL, &allocator), 2);
  require_ok(rclc_node_init_default(&node, NODE_NAME, "", &support), 3);

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
#else
  #if !MODE_TRANSMITTER
  armed_for_flag = false;
  #endif
#endif

#if ENABLE_SERIAL_DEBUG
  Serial.println("Setup done.");
#endif
}

void loop()
{
#if MODE_TRANSMITTER
  pump_modem_rx_publish_full_lines();
#endif

#if ENABLE_DIAG_COUNTERS
  static bool last_if2 = false;
  static bool last_if1 = false;
  bool now_if2 = (GPT2_SR & GPT_SR_IF2);
  bool now_if1 = (GPT2_SR & GPT_SR_IF1);
  if (last_if2 && now_if2) cnt_pps_overwrite++;
  if (last_if1 && now_if1) cnt_flag_overwrite++;
  last_if2 = now_if2;
  last_if1 = now_if1;
#endif

  // ---------- PPS / compare trigger source ----------
#if USE_PPS_CAPTURE
  if (GPT2_SR & GPT_SR_IF2) {
    uint32_t t_trigger = GPT2_ICR2;
    GPT2_SR = GPT_SR_IF2;

    pps_us = t_trigger;

    uint32_t prev = dbg_last_trigger_us;
    dbg_prev_trigger_us = prev;
    dbg_last_trigger_us = t_trigger;
    dbg_last_trigger_delta_us = (prev == 0) ? 0 : (int32_t)(t_trigger - prev);
    dbg_trigger_count++;

#if ENABLE_MICROROS
    msg_pps.data = pps_us;
    (void)rcl_publish(&pub_pps, &msg_pps, NULL);

    #if ENABLE_DIAG_COUNTERS
    cnt_pps++;
    if ((cnt_pps & 0x3F) == 0) {
      msg_diag.data = cnt_pps;
      (void)rcl_publish(&pub_diag, &msg_diag, NULL);
    }
    #endif
#endif

#if MODE_TRANSMITTER
    modem_send_cmd_and_publish(tx_cmd);
#else
    armed_for_flag = true;
#endif
  }
#else
  if (gpt2_compare_pending) {
    noInterrupts();
    uint32_t t_compare = gpt2_compare_stamp;
    gpt2_compare_pending = false;
    interrupts();

    pps_us = t_compare;

    uint32_t prev = dbg_last_trigger_us;
    dbg_prev_trigger_us = prev;
    dbg_last_trigger_us = t_compare;
    dbg_last_trigger_delta_us = (prev == 0) ? 0 : (int32_t)(t_compare - prev);
    dbg_trigger_count++;

#if ENABLE_MICROROS
    msg_pps.data = pps_us;
    (void)rcl_publish(&pub_pps, &msg_pps, NULL);

    #if ENABLE_DIAG_COUNTERS
    cnt_pps++;
    if ((cnt_pps & 0x3F) == 0) {
      msg_diag.data = cnt_pps;
      (void)rcl_publish(&pub_diag, &msg_diag, NULL);
    }
    #endif
#endif

#if MODE_TRANSMITTER
    modem_send_cmd_and_publish(tx_cmd);
#else
    armed_for_flag = true;
#endif
  }
#endif

#if !MODE_TRANSMITTER
  // ---------- FLAG capture (IF1 -> ICR1) ----------
  if (GPT2_SR & GPT_SR_IF1) {
    if (!armed_for_flag) {
      (void)GPT2_ICR1;
      GPT2_SR = GPT_SR_IF1;

      dbg_ignored_flag_count++;

#if ENABLE_DIAG_COUNTERS
      cnt_flag_ignored++;
#endif
    } else {
      uint32_t flag_us = GPT2_ICR1;
      GPT2_SR = GPT_SR_IF1;
      
      uint32_t du = (uint32_t)(flag_us - pps_us);

      dbg_last_flag_us = flag_us;
      dbg_last_flag_delta_us = (int32_t)du;
      dbg_flag_count++;

#if ENABLE_MICROROS
      msg_flag.data = flag_us;
      (void)rcl_publish(&pub_flag, &msg_flag, NULL);

      msg_delta.data = (int32_t)du;
      (void)rcl_publish(&pub_delta, &msg_delta, NULL);
#endif

#if ENABLE_DIAG_COUNTERS
      cnt_flag++;
#endif
      armed_for_flag = false;
    }
  }
#endif

#if ENABLE_SERIAL_DEBUG
  static uint32_t last_print_ms = 0;
  uint32_t now_ms = millis();
  if ((uint32_t)(now_ms - last_print_ms) >= SERIAL_DEBUG_PERIOD_MS) {
    last_print_ms = now_ms;
    print_debug_snapshot();
  }
#endif
}