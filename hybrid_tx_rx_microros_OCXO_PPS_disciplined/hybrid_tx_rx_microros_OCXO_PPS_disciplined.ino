// hybrid_tx_rx_microros_OCXO_holdover_Puffin.ino
//
// Teensy 4.1
// PPS-locked when available, OCXO-holdover when PPS is lost.
//
// Main idea:
//   - GPT2 runs from external OCXO clock on pin 14
//   - GPT2 counter is prescaled to 1 MHz => 1 tick = 1 us
//   - MODEM_FLAG is hardware-captured on GPT2 CAPIN1 (pin 15)
//   - PPS is hardware-captured on GPT2 CAPIN2 (pin 40)
//   - While PPS is present: epochs come from real PPS
//   - When PPS is lost: epochs continue from last PPS-aligned phase using GPT2 OCR1 compare
//   - When PPS returns: re-lock immediately
//
// micro-ROS options:
//   ENABLE_MICROROS = 1 -> build with micro-ROS publishers enabled
//   ENABLE_MICROROS = 0 -> build without micro-ROS, serial debug only

#include <Arduino.h>

// ===================== FEATURE SWITCHES =================
#define MODE_TRANSMITTER     0 // 1 = TX node, 0 = RX node
#define ENABLE_MICROROS      0  // 1 = enable micro-ROS, 0 = no micro-ROS
#define ENABLE_DIAG_COUNTERS 0
#define ENABLE_SERIAL_DEBUG  1
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

// ===================== Holdover config =================
static constexpr uint32_t EPOCH_US       = 1000000UL;
static constexpr uint32_t PPS_TIMEOUT_US = 1200000UL;  // declare PPS lost if >1.2 s since last PPS
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
static volatile uint8_t  dbg_last_trigger_src = 0; // 0=none, 1=PPS, 2=HOLD

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

// ===================== Latched ISR events ==============
static volatile bool pps_pending = false;
static volatile uint32_t pps_stamp = 0;

static volatile bool cmp_pending = false;
static volatile uint32_t cmp_stamp = 0;

#if !MODE_TRANSMITTER
static volatile bool flag_pending = false;
static volatile uint32_t flag_stamp = 0;
#endif

static volatile uint32_t gpt2_next_compare = 0;
// =======================================================

// ===================== Timing state ====================
enum TimingMode : uint8_t {
  TIMING_WAIT_PPS   = 0,
  TIMING_PPS_LOCKED = 1,
  TIMING_HOLDOVER   = 2
};

static TimingMode timing_mode = TIMING_WAIT_PPS;

// This is the second boundary actually used for delta computation.
// It is real PPS when locked, synthetic epoch when in holdover.
static uint32_t current_epoch_us = 0;

// Last real PPS edge seen.
static uint32_t last_real_pps_us = 0;

#if !MODE_TRANSMITTER
static bool armed_for_flag = false;
#endif
// =======================================================

// ===================== GPT2 init =======================
extern "C" void GPT2_IRQHandler();

static inline void gpt2_set_compare_us(uint32_t t_us)
{
  noInterrupts();
  gpt2_next_compare = t_us;
  GPT2_OCR1 = t_us;
  interrupts();
}

static inline void gpt2_enable_compare_irq()
{
  noInterrupts();
  GPT2_IR |= GPT_IR_OF1IE;
  interrupts();
}

static inline void gpt2_disable_compare_irq()
{
  noInterrupts();
  GPT2_IR &= ~GPT_IR_OF1IE;
  interrupts();
}

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

  // Start with FLAG + PPS only. Compare is enabled only during holdover.
  GPT2_IR = GPT_IR_IF1IE | GPT_IR_IF2IE;

  NVIC_CLEAR_PENDING(IRQ_GPT2);
  attachInterruptVector(IRQ_GPT2, GPT2_IRQHandler);
  NVIC_ENABLE_IRQ(IRQ_GPT2);

  GPT2_CR |= GPT_CR_EN;
}

extern "C" void GPT2_IRQHandler()
{
  uint32_t sr = GPT2_SR;

  // FLAG capture on ICR1
  if (sr & GPT_SR_IF1) {
#if !MODE_TRANSMITTER
    flag_stamp = GPT2_ICR1;
    flag_pending = true;
#else
    (void)GPT2_ICR1;
#endif
    GPT2_SR = GPT_SR_IF1;
  }

  // PPS capture on ICR2
  if (sr & GPT_SR_IF2) {
    pps_stamp = GPT2_ICR2;
    pps_pending = true;
    GPT2_SR = GPT_SR_IF2;
  }

  // Compare holdover epoch
  if (sr & GPT_SR_OF1) {
    cmp_stamp = GPT2_OCR1;
    cmp_pending = true;
    GPT2_SR = GPT_SR_OF1;

    gpt2_next_compare += EPOCH_US;
    GPT2_OCR1 = gpt2_next_compare;
  }

  asm volatile("dsb");
}
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

// ===================== Epoch handling ==================
static void handle_epoch_trigger(uint32_t epoch_us, uint8_t src)
{
  current_epoch_us = epoch_us;

  uint32_t prev = dbg_last_trigger_us;
  dbg_prev_trigger_us = prev;
  dbg_last_trigger_us = epoch_us;
  dbg_last_trigger_delta_us = (prev == 0) ? 0 : (int32_t)(epoch_us - prev);
  dbg_trigger_count++;
  dbg_last_trigger_src = src;

#if ENABLE_MICROROS
  msg_pps.data = epoch_us;
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

static void maybe_enter_holdover()
{
  if (timing_mode != TIMING_PPS_LOCKED) return;

  uint32_t now = sit_counter_us();
  uint32_t since_last_pps = (uint32_t)(now - last_real_pps_us);

  if (since_last_pps <= PPS_TIMEOUT_US) return;

  // Advance to the current PPS-aligned second boundary that should already exist,
  // then continue from there with compare-based synthetic epochs.
  uint32_t missed_epochs = since_last_pps / EPOCH_US;
  if (missed_epochs == 0) missed_epochs = 1;

  current_epoch_us = last_real_pps_us + missed_epochs * EPOCH_US;
  gpt2_set_compare_us(current_epoch_us + EPOCH_US);
  gpt2_enable_compare_irq();
  timing_mode = TIMING_HOLDOVER;

#if !MODE_TRANSMITTER
  // We are now living off the synthetic second grid.
  armed_for_flag = true;
#endif
}
// =======================================================

// ===================== Diagnostic counters =============
#if ENABLE_DIAG_COUNTERS
static uint32_t cnt_pps = 0;
static uint32_t cnt_flag = 0;
static uint32_t cnt_flag_ignored = 0;
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
  uint8_t  last_trigger_src;
  uint32_t flag_count;
  uint32_t last_flag_us;
  int32_t  last_flag_delta_us;
  uint32_t ignored_flag_count;
  uint32_t epoch_us_snapshot;
  uint32_t last_real_pps_snapshot;
  TimingMode mode_snapshot;

  noInterrupts();
  sit_now                = sit_counter_us();
  teensy_now             = micros();
  trigger_count          = dbg_trigger_count;
  prev_trigger_us        = dbg_prev_trigger_us;
  last_trigger_us        = dbg_last_trigger_us;
  last_trigger_delta_us  = dbg_last_trigger_delta_us;
  last_trigger_src       = dbg_last_trigger_src;
  flag_count             = dbg_flag_count;
  last_flag_us           = dbg_last_flag_us;
  last_flag_delta_us     = dbg_last_flag_delta_us;
  ignored_flag_count     = dbg_ignored_flag_count;
  epoch_us_snapshot      = current_epoch_us;
  last_real_pps_snapshot = last_real_pps_us;
  mode_snapshot          = timing_mode;
  dbg_last_sit_counter   = sit_now;
  dbg_last_teensy_micros = teensy_now;
  interrupts();

  const char *src_str =
    (last_trigger_src == 1) ? "PPS" :
    (last_trigger_src == 2) ? "HOLD" : "NONE";

  const char *mode_str =
    (mode_snapshot == TIMING_WAIT_PPS)   ? "WAIT_PPS" :
    (mode_snapshot == TIMING_PPS_LOCKED) ? "PPS_LOCK" :
                                           "HOLDOVER";

  Serial.printf(
    "MODE=%s | LAST_SRC=%s | SIT_now=%u us | TeensyMicros=%u us | TRIG_count=%u | EPOCH=%u us | LAST_REAL_PPS=%u us | prev_TRIG=%u us | dTRIG=%ld us | FLAG_count=%u | FLAG_ts=%u us | dFLAG=%ld us | FLAG_ignored=%u\n",
    mode_str,
    src_str,
    sit_now,
    teensy_now,
    trigger_count,
    epoch_us_snapshot,
    last_real_pps_snapshot,
    prev_trigger_us,
    (long)last_trigger_delta_us,
    flag_count,
    last_flag_us,
    (long)last_flag_delta_us,
    ignored_flag_count
  );
}
#endif
// =======================================================

void setup()
{
#if ENABLE_SERIAL_DEBUG
  Serial.begin(115200);
  delay(500);
  Serial.println("Booting hybrid_tx_rx_microros_OCXO_holdover_Puffin...");
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
  Serial.println("GPT2 external-clock capture initialized.");
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

  // 1) If PPS has gone away, enter holdover from the last PPS-aligned phase.
  maybe_enter_holdover();

  // 2) Real PPS wins immediately and re-locks us if it comes back.
  if (pps_pending) {
    noInterrupts();
    uint32_t t_pps = pps_stamp;
    pps_pending = false;
    interrupts();

    last_real_pps_us = t_pps;
    timing_mode = TIMING_PPS_LOCKED;

    // Keep compare phase aligned to this PPS, but disable compare IRQ while locked
    // so we do not generate duplicate epoch triggers.
    gpt2_set_compare_us(t_pps + EPOCH_US);
    gpt2_disable_compare_irq();

    handle_epoch_trigger(t_pps, 1);
  }

  // 3) Synthetic epoch only matters during holdover.
  if (cmp_pending) {
    noInterrupts();
    uint32_t t_cmp = cmp_stamp;
    cmp_pending = false;
    interrupts();

    if (timing_mode == TIMING_HOLDOVER) {
      handle_epoch_trigger(t_cmp, 2);
    }
  }

#if !MODE_TRANSMITTER
  // 4) FLAG capture is always measured against current_epoch_us
  //    whether that epoch came from real PPS or synthetic holdover.
  if (flag_pending) {
    noInterrupts();
    uint32_t flag_us = flag_stamp;
    flag_pending = false;
    interrupts();

    if (!armed_for_flag) {
      dbg_ignored_flag_count++;

#if ENABLE_DIAG_COUNTERS
      cnt_flag_ignored++;
#endif
    } else {
      uint32_t du = (uint32_t)(flag_us - current_epoch_us);

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
