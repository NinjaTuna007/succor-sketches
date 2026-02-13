# Succor Sketches

Arduino sketches for **one-way travel time ranging** in underwater environments using acoustic modems with GPS PPS (Pulse Per Second) timing synchronization. Designed for Teensy 4.1.

## Overview

This repository contains two complementary sketches for underwater acoustic ranging experiments and testing:

1. **hybrid_fullmetal_sketch** - Standalone timing logger with USB Serial output
2. **hybrid_tx_rx_microros** - micro-ROS enabled version with ROS topic publishing

Both sketches support dual-mode operation (transmitter or receiver) configured at compile time.

## Hardware Requirements

- **Microcontroller**: Teensy 4.1
- **PPS Signal**: Connected to pin 40 (GPS timing reference - acts as accurate clock)
- **FLAG Signal**: Connected to pin 41 (acoustic signal detection in receiver mode)
- **Modem UART**: Serial1 (Hardware UART, 9600 baud)
- **USB Connection**: Serial @ 115200 baud for micro-ROS transport, 2Mbaud for standalone logging
- **GPS Module**: Provides PPS signal for time synchronization across platforms

## Sketches

### 1. hybrid_fullmetal_sketch

Standalone sketch that logs all timing events to USB Serial (2Mbaud). Features non-blocking logging with ring buffer to maintain timing accuracy.

**Features:**
- Non-blocking logger (drops logs if USB can't keep up, never blocks timing)
- Microsecond-precision timestamps
- ISR-based signal handling (minimal ISR work for accuracy)

**Output Format:**
```
PPS,<timestamp_us>                        # PPS pulse detected
TX,<timestamp_us>,<command>               # Transmitter mode: command sent
RX,<timestamp_us>,<modem_response>        # Modem response received
DELTA,<flag_timestamp_us>,<delta_us>      # Receiver mode: FLAG-to-PPS delta
```

### 2. hybrid_tx_rx_microros

micro-ROS enabled version that publishes timing data to ROS topics. Requires a micro-ROS agent running on the host.

**Topics:**

Transmitter mode (`/tx` namespace):
- `/tx/succorfish/msg/sent` (String) - Commands sent to modem
- `/tx/succorfish/msg/received` (String) - Modem responses

Receiver mode (`/rx` namespace):
- `/rx/succorfish/msg/sent` (String) - Status messages
- `/rx/succorfish/msg/received` (String) - Modem messages received
- `/rx/succorfish/delta_t` (Int32) - FLAG-to-PPS timing delta (microseconds)

## Operation Modes

### Transmitter Mode (`MODE_TRANSMITTER = 1`)

- **PPS Rising Edge**: Triggers modem command transmission
- **Default Command**: `$P007` (configurable via `tx_cmd` variable)
- **Timing**: Serial1 is restarted before each transmission to minimize jitter
- **Purpose**: Synchronous acoustic beacon or broadcast

### Receiver Mode (`MODE_TRANSMITTER = 0`)

- **PPS Rising Edge**: Records reference timestamp
- **FLAG Rising Edge**: Captures timing delta from last PPS
- **Purpose**: Acoustic signal arrival time measurement
- **Delta Calculation**: `delta_t = FLAG_timestamp - PPS_timestamp`

## Pin Configuration

```cpp
PIN_PPS  = 40  // PPS input (GPS or timing reference)
PIN_FLAG = 41  // Acoustic signal detection (RX mode only)
```

## Modem Communication

- **Interface**: Hardware Serial1 (UART)
- **Baud Rate**: 9600
- **Protocol**: Commands/responses terminated by CR+LF
- **Line Parsing**: Tolerant of both `\r\n` and lone `\n`
- **Max Line Length**: 256 bytes

## Setup & Usage

### hybrid_fullmetal_sketch

1. Set `MODE_TRANSMITTER` (0 or 1) at line 26
2. Upload to Teensy 4.1
3. Open Serial Monitor at 2Mbaud
4. Connect PPS signal to pin 40
5. (RX mode) Connect FLAG signal to pin 41
6. Monitor CSV-formatted timing logs

### hybrid_tx_rx_microros

1. Install [micro-ROS for Arduino](https://github.com/micro-ROS/micro_ros_arduino)
2. Set `MODE_TRANSMITTER` (0 or 1) at line 35
3. Upload to Teensy 4.1
4. Start micro-ROS agent (Docker):
   ```bash
   docker run -it --rm \
     --net=host --privileged \
     -v /dev:/dev \
     microros/micro-ros-agent:iron \
     serial --dev /dev/ttyACM0 -b 115200 -v6
   ```
   Replace `/dev/ttyACM0` with your Teensy's serial port.

5. Verify topics:
   ```bash
   ros2 topic list
   ros2 topic echo /rx/succorfish/delta_t  # Receiver mode
   ros2 topic echo /tx/succorfish/msg/sent  # Transmitter mode
   ```

## Timing Considerations

- **ISR Priority**: Minimal work only (timestamp capture + flag setting)
- **No Serial in ISR**: All I/O happens in main loop to prevent blocking
- **Jitter Mitigation**: Serial1 restart before each transmission (TX mode)
- **Microsecond Resolution**: Uses `micros()` for all timestamps
- **Wrap-Safe**: Delta calculations handle uint32 wraparound correctly

## Customization

### Change Modem Command (TX Mode)

Edit the `tx_cmd` variable in either sketch:
```cpp
static const char tx_cmd[] = "$P007";  // Replace with your command
```

### Adjust Logging Buffer

In `hybrid_fullmetal_sketch`, modify ring buffer size:
```cpp
static constexpr uint16_t LOG_Q_DEPTH = 64;     // Increase for more buffering
static constexpr uint16_t LOG_LINE_MAX = 320;   // Max line length
```

### Change Topic Namespace

In `hybrid_tx_rx_microros`, edit `TOPIC_ROOT` automatically set by mode, or modify the suffix in `make_topic()` calls.

## Use Cases

### Primary Application: One-Way Travel Time Ranging
These sketches enable **underwater ranging** by measuring acoustic signal travel time between platforms:
- **GPS PPS signals** provide synchronized time references across spatially separated platforms
- **Transmitter** sends acoustic signal synchronized to PPS pulse
- **Receiver** measures time-of-arrival relative to its own GPS PPS
- **Delta_t measurement** represents one-way travel time (converts to range via speed of sound)

### Testing & Validation
- Acoustic modem synchronization testing
- Time-of-arrival measurement accuracy validation
- GPS-synchronized underwater communication experiments
- Multi-platform time synchronization verification
- Acoustic ranging system development and debugging

## Contributing

Contributions welcome. Please maintain the timing-critical design philosophy and avoid blocking operations in time-sensitive code paths.
