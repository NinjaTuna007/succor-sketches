#include <Arduino.h>

// Teensy 4.1 dual square-wave generator (ISR version).
// - CH A and CH B are 50% duty cycle square waves from IntervalTimer ISRs.
// - Supports very low frequencies (well below PWM hardware floor).
// - CH B can be offset from CH A by a configurable startup offset.
// - Optional 180 deg phase flip can be applied on top of offset.
//
// Serial commands (USB):
//   f <hz>   : set frequency in Hz (both channels)
//   o <us>   : set offset in microseconds (CH B relative to CH A)
//   p <0|1>  : phase flip off/on (adds 180 deg)
//   s        : print current settings
//   h        : print help

static constexpr uint8_t PIN_CH_A = 2;
static constexpr uint8_t PIN_CH_B = 4;
static constexpr uint8_t PIN_HEARTBEAT = LED_BUILTIN;

static constexpr float FREQ_MIN_HZ = 0.1f;
static constexpr float FREQ_MAX_HZ = 100000.0f;
static constexpr uint32_t HEARTBEAT_ON_MS = 900;
static constexpr uint32_t HEARTBEAT_OFF_MS = 100;

struct WaveConfig {
  float freq_hz;
  uint32_t offset_us;
  bool phase_flip;
};

static WaveConfig cfg = {
  1.0f, // 1 Hz default
  100,  // 100 us default offset
  false
};

static IntervalTimer timerA;
static IntervalTimer timerB;

static float half_period_us = 500000.0f;
static uint32_t effective_offset_us = 100;

static bool reconfigure_requested = true;
static bool start_b_pending = false;
static uint32_t start_b_deadline_us = 0;

static bool heartbeat_on = true;
static uint32_t heartbeat_next_ms = 0;

void isr_ch_a_toggle() { digitalToggleFast(PIN_CH_A); }
void isr_ch_b_toggle() { digitalToggleFast(PIN_CH_B); }

static uint32_t phase_us_for_config(const WaveConfig &c)
{
  const float period_f = 1000000.0f / c.freq_hz;
  const uint32_t period_us = (uint32_t)(period_f + 0.5f);
  if (period_us == 0) return 0;

  const uint32_t base = c.offset_us % period_us;
  if (!c.phase_flip) return base;

  const uint32_t half = period_us / 2U;
  return (base + half) % period_us;
}

static void print_status()
{
  const float period_f = 1000000.0f / cfg.freq_hz;
  const uint32_t period_us = (uint32_t)(period_f + 0.5f);

  Serial.print("STATUS,freq_hz=");
  Serial.print(cfg.freq_hz, 4);
  Serial.print(",period_us=");
  Serial.print(period_us);
  Serial.print(",half_period_us=");
  Serial.print(half_period_us, 3);
  Serial.print(",offset_us=");
  Serial.print(cfg.offset_us);
  Serial.print(",phase_flip=");
  Serial.print(cfg.phase_flip ? 1 : 0);
  Serial.print(",effective_offset_us=");
  Serial.println(effective_offset_us);
}

static void print_help()
{
  Serial.println("Commands:");
  Serial.println("  f <hz>   set frequency (0.1 .. 100000)");
  Serial.println("  o <us>   set offset in microseconds");
  Serial.println("  p <0|1>  phase flip off/on (adds 180 deg)");
  Serial.println("  s        show status");
  Serial.println("  h        help");
}

static void heartbeat_update()
{
  const uint32_t now = millis();
  if ((int32_t)(now - heartbeat_next_ms) < 0) return;

  heartbeat_on = !heartbeat_on;
  digitalWriteFast(PIN_HEARTBEAT, heartbeat_on ? HIGH : LOW);
  heartbeat_next_ms = now + (heartbeat_on ? HEARTBEAT_ON_MS : HEARTBEAT_OFF_MS);
}

static void stop_waveforms()
{
  timerA.end();
  timerB.end();
  digitalWriteFast(PIN_CH_A, LOW);
  digitalWriteFast(PIN_CH_B, LOW);
  start_b_pending = false;
}

static void apply_config()
{
  stop_waveforms();

  half_period_us = 500000.0f / cfg.freq_hz;
  effective_offset_us = phase_us_for_config(cfg);

  if (!timerA.begin(isr_ch_a_toggle, half_period_us)) {
    Serial.println("ERR,failed_to_start_timerA");
    return;
  }

  if (effective_offset_us == 0) {
    if (!timerB.begin(isr_ch_b_toggle, half_period_us)) {
      Serial.println("ERR,failed_to_start_timerB");
      return;
    }
  } else {
    start_b_pending = true;
    start_b_deadline_us = micros() + effective_offset_us;
  }

  Serial.println("OK,reconfigured");
  print_status();
}

static void start_b_if_due()
{
  if (!start_b_pending) return;
  if ((int32_t)(micros() - start_b_deadline_us) < 0) return;

  start_b_pending = false;
  if (!timerB.begin(isr_ch_b_toggle, half_period_us)) {
    Serial.println("ERR,failed_to_start_timerB_delayed");
  }
}

static void handle_serial_line(char *line)
{
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0') return;

  if (*line == 's') {
    print_status();
    return;
  }
  if (*line == 'h') {
    print_help();
    return;
  }

  char cmd = *line++;
  while (*line == ' ' || *line == '\t') line++;

  if (cmd == 'f') {
    float hz = atof(line);
    if (!(hz >= FREQ_MIN_HZ && hz <= FREQ_MAX_HZ)) {
      Serial.println("ERR,freq_out_of_range");
      return;
    }
    cfg.freq_hz = hz;
    reconfigure_requested = true;
    return;
  }

  if (cmd == 'o') {
    long v = atol(line);
    if (v < 0) {
      Serial.println("ERR,offset_must_be_non_negative");
      return;
    }
    cfg.offset_us = (uint32_t)v;
    reconfigure_requested = true;
    return;
  }

  if (cmd == 'p') {
    int v = atoi(line);
    if (!(v == 0 || v == 1)) {
      Serial.println("ERR,phase_flip_use_0_or_1");
      return;
    }
    cfg.phase_flip = (v == 1);
    reconfigure_requested = true;
    return;
  }

  Serial.println("ERR,unknown_command");
}

void setup()
{
  pinMode(PIN_CH_A, OUTPUT);
  pinMode(PIN_CH_B, OUTPUT);
  pinMode(PIN_HEARTBEAT, OUTPUT);

  digitalWriteFast(PIN_CH_A, LOW);
  digitalWriteFast(PIN_CH_B, LOW);
  digitalWriteFast(PIN_HEARTBEAT, HIGH);

  heartbeat_on = true;
  heartbeat_next_ms = millis() + HEARTBEAT_ON_MS;

  Serial.begin(115200);
  delay(200);

  Serial.println("BOOT,dual_squarewave_generator_isr");
  print_help();
}

void loop()
{
  heartbeat_update();

  if (reconfigure_requested) {
    reconfigure_requested = false;
    apply_config();
  }

  start_b_if_due();

  static char line[64];
  static size_t len = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[len] = '\0';
      handle_serial_line(line);
      len = 0;
      continue;
    }
    if (len < sizeof(line) - 1) {
      line[len++] = c;
    }
  }
}
