# owtt_stick_v2

Same stick firmware as `owtt_stick_v1`: host protocol, OCXO timing, GNSS bridge, and the e-ink panel. v1 is one long file. v2 is the same behavior split into files so you can open the part you are working on.

Start here, then `owtt_stick_v2.ino`. The command and timing writeup is the comment at the top of `owtt_radio.cpp`.

## How to read it

1. This page, for the map.
2. `owtt_stick_v2.ino`. That is the whole loop. If you understand the order of calls there, you understand what the board does each time around.
3. The top of `owtt_radio.cpp` when you need `$Y`, `$G`, `$K`, `$Z`, the on-air timestamp, or `#I`.
4. One other file only when the loop calls into it. The `.h` next to a `.cpp` is the short list of functions other files are allowed to call. Everything else in that `.cpp` is private to it.

`owtt_config.h` is pins, baud rates, and limits. `owtt_state.h` is the variables that more than one file has to share. A variable used by only one file stays in that file.

## What one pass of `loop()` does

The order matters. Timing is latched before serial work so a slow host write cannot move the capture stamp. The stamp itself is taken in the GPT2 interrupt; the loop only reads it.

1. `service_gnss_bridge` copies the X20P UART to the second USB port and feeds the UBX parser. The parser does not remove or change any byte.
2. `process_timing_events` applies a PPS edge, a holdover tick, or a modem receive-flag capture.
3. Host bytes are assembled and handled immediately. `$Y` configures the board. `$G` and `$K` store a payload. `$Z` is a query. Anything else is forwarded to the modem.
4. Modem bytes are forwarded to the host first. In receiver mode the board then adds `#I` and `#OWTT`.
5. `execute_auto_tx_if_due` sends the scheduled `$B` if this epoch is a transmit epoch. Host commands in this same pass already ran, so they win over the schedule.
6. The e-ink status is updated, if the breakout was found at boot. `owtt_epd_begin()` writes and reads the panel board's SPI SRAM. The host port says `#EPD,PRESENT` or `#EPD,ABSENT`. Absent means no painter thread, no 13-second refresh, and no further SPI to the panel. Pin 13 is that panel's SPI clock. Do not use it as the onboard LED.
7. Die temperature is logged on the host port as `#TEMPMON`. That is the MCU, not the OCXO crystal.

## Files

| File | What it owns |
|---|---|
| `owtt_stick_v2.ino` | `setup` and `loop` |
| `owtt_radio.cpp` | `$Y` `$G` `$K` `$Z`, scheduled `$B`, receiver `#I` |
| `owtt_clock.cpp` | 10 MHz counter, PPS, holdover, OCXO calibration, `#TEMPMON` |
| `owtt_gnss.cpp` | X20P bridge and binding UTC to a PPS edge |
| `owtt_io.cpp` | Writes to the host port and the modem UART |
| `owtt_epd.cpp` | Tricolor status panel |
| `owtt_frame.cpp` | Cutting one `$B` / `$K` / `#B` record out of a byte stream |
| `owtt_ubx.cpp` | UBX framing and checksum. UTC binding stays in `owtt_gnss.cpp` |

## Ports

Tools → USB Type → Dual Serial.

| Port | Role |
|---|---|
| `Serial` | Host. `$Y` `$G` `$K` `$Z`, and modem traffic |
| `SerialUSB1` | Transparent GNSS bridge. Do not send host commands here |
| `Serial1` | Modem, 9600 |
| `Serial2` | ZED-X20P UART |

The board boots in wire mode: host bytes go to the modem, modem bytes go to the host, until a `$Y` says otherwise.

v1 is the sketch on the boards today. Compile v2 as much as you like. Flash it only after a bench check of `#I` against v1.
