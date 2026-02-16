#include <Arduino.h>

// Teensy 4.1 low-jitter dual square-wave generator (PWM version).
// - CH A and CH B are 50% duty cycle square waves from hardware PWM.
// - CH B can be offset from CH A by a configurable startup offset.
// - Optional 180 deg phase flip can be applied on top of offset.
// - NOTE: On these PWM pins, practical hardware floor is about 17.88 Hz.
//
// Serial commands (USB):
//   f <hz>   : set frequency in Hz (both channels)
//   o <us>   : set offset in microseconds (CH B relative to CH A)
//   p <0|1>  : phase flip off/on (adds 180 deg)
//   s        : print current settings
//   h        : print help

// Use pins on different PWM timers so each channel can be started independently.
// Teensy 4.1:
//   Pin 2 -> FlexPWM4.2
//   Pin 4 -> FlexPWM2.0
static constexpr uint8_t PIN_CH_A = 2;
static constexpr uint8_t PIN_CH_B = 4;
static constexpr uint8_t PIN_HEARTBEAT = LED_BUILTIN;

static constexpr float FREQ_MIN_HZ = 17.88f; // hardware PWM floor is ~17.88 Hz
static constexpr float FREQ_MAX_HZ = 500000.0f; // practical upper bound depends on resolution
static constexpr uint8_t PWM_BITS = 12;
static constexpr uint16_t PWM_50 = (1u << (PWM_BITS - 1)); // 50%
static constexpr uint32_t HEARTBEAT_ON_MS = 900;
static constexpr uint32_t HEARTBEAT_OFF_MS = 100;

struct WaveConfig {
  float freq_hz;
  uint32_t offset_us;
  bool phase_flip;
};

static WaveConfig cfg = {
  1000.0f, // 1 kHz default
  100,     // 100 us default offset
  false
};

static uint32_t effective_offset_us = 100;

static bool reconfigure_requested = true;
static bool heartbeat_on = true;
static uint32_t heartbeat_next_ms = 0;

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
  Serial.print(cfg.freq_hz, 3);
  Serial.print(",period_us=");
  Serial.print(period_us);
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
  Serial.println("  f <hz>   set frequency (17.88 .. 500000)");
  Serial.println("  o <us>   set offset in microseconds");
  Serial.println("  p <0|1>  phase flip off/on (adds 180 deg)");
  Serial.println("  s        show status");
  Serial.println("  h        help");
}

static void pwm_stop_low(uint8_t pin)
{
  analogWrite(pin, 0);
  pinMode(pin, OUTPUT);
  digitalWriteFast(pin, LOW);
}

static void wait_us_blocking(uint32_t delay_us)
{
  const uint32_t start = micros();
  while ((uint32_t)(micros() - start) < delay_us) {
    // Busy wait only during reconfiguration to maximize phase-start precision.
  }
}

static void heartbeat_update()
{
  const uint32_t now = millis();
  if ((int32_t)(now - heartbeat_next_ms) < 0) return;

  heartbeat_on = !heartbeat_on;
  digitalWriteFast(PIN_HEARTBEAT, heartbeat_on ? HIGH : LOW);
  heartbeat_next_ms = now + (heartbeat_on ? HEARTBEAT_ON_MS : HEARTBEAT_OFF_MS);
}

static void apply_config()
{
  pwm_stop_low(PIN_CH_A);
  pwm_stop_low(PIN_CH_B);

  effective_offset_us = phase_us_for_config(cfg);

  analogWriteFrequency(PIN_CH_A, cfg.freq_hz);
  analogWriteFrequency(PIN_CH_B, cfg.freq_hz);

  // Start A, then delay, then start B.
  // This offset only affects startup phase; after start both channels are hardware-driven.
  analogWrite(PIN_CH_A, PWM_50);
  if (effective_offset_us > 0) {
    wait_us_blocking(effective_offset_us);
  }
  analogWrite(PIN_CH_B, PWM_50);

  Serial.println("OK,reconfigured");
  print_status();
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
  analogWriteResolution(PWM_BITS);
  heartbeat_on = true;
  heartbeat_next_ms = millis() + HEARTBEAT_ON_MS;

  Serial.begin(115200);
  delay(200);

  Serial.println("BOOT,dual_squarewave_generator_pwm");
  print_help();
}

void loop()
{
  heartbeat_update();

  if (reconfigure_requested) {
    reconfigure_requested = false;
    apply_config();
  }

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
