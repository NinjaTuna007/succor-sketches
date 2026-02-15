# Square Wave Generators (Teensy 4.1)

These sketches are intended primarily for **testing and bench validation**.
They are useful for generating repeatable timing stimuli, but they are not designed as production-grade signal generators.

## Included Sketches

- `dual_squarewave_generator_pwm/dual_squarewave_generator_pwm.ino`
- `dual_squarewave_generator_isr/dual_squarewave_generator_isr.ino`

## When to Use Each

- `PWM` version:
  - Lowest cycle-to-cycle jitter (hardware PWM driven).
  - Frequency floor is about **17.88 Hz** on the selected pins.
  - Best for higher-frequency timing tests.

- `ISR` version:
  - Supports low frequencies (including around 1 Hz and below).
  - More timing jitter than PWM at higher frequencies.
  - Best for low-frequency functional tests.

## Common Serial Commands

Open USB serial and send:

- `f <hz>` set frequency
- `o <us>` set CH B offset from CH A in microseconds
- `p <0|1>` disable/enable 180 degree phase flip
- `s` print current status
- `h` print help

## Hardware Notes

- Output pins are configured in each sketch (`PIN_CH_A`, `PIN_CH_B`).
- Onboard LED heartbeat is enabled (1 Hz, 90% on / 10% off) to indicate the sketch is running.

## Testing Scope

These generators are intended for:

- timing chain bring-up
- trigger/edge alignment checks
- interface sanity testing

For strict metrology-grade timing guarantees, verify with external instrumentation and a hardware-locked design.
