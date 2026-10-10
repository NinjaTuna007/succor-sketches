// owtt_stick_v2
//
// Same host protocol, timing, and e-ink panel as owtt_stick_v1.
// Read README.md in this folder first. It says which file owns what,
// and why loop() runs in the order below.
// The command and timing writeup is the comment at the top of owtt_radio.cpp.
//
// This file is only setup() and loop().
//   owtt_io      writes to the host USB port and the modem UART
//   owtt_clock   10 MHz OCXO, PPS, holdover, die temperature
//   owtt_gnss    X20P bridge and the UTC label on each epoch
//   owtt_radio   $Y $G $K $Z, the scheduled $B, and receiver #I
//   owtt_epd     the tricolor status panel
//   owtt_frame   how a $B/$K/#B record is cut out of the byte stream
//   owtt_ubx     UBX framing and checksum
//
// Do not flash v2 until a bench #I.

#include "owtt_state.h"
#include "owtt_io.h"
#include "owtt_clock.h"
#include "owtt_gnss.h"
#include "owtt_radio.h"
#include "owtt_epd.h"


// =======================================================
// ===================== SETUP / LOOP ====================
// =======================================================

void setup()
{
  ocxo_eeprom_load();

  // First USB CDC port: LoLo / Succorfish protocol.
  HOST_SERIAL.begin(HOST_BAUD);

  // Second USB CDC port: transparent X20P serial bridge.
  GNSS_USB_SERIAL.begin(GNSS_INITIAL_BAUD);

  MODEM_SERIAL.begin(MODEM_BAUD, SERIAL_8N1);
  GNSS_SERIAL.begin(GNSS_INITIAL_BAUD, SERIAL_8N1);
  gnss_uart_baud = GNSS_INITIAL_BAUD;
  gnss_uart_apply_extra_buffers();
  gnss_last_good_frame_ms = 0;
  gnss_link_epoch_ms = millis();

  pinMode(PIN_OCXO, INPUT);
  pinMode(PIN_RXS_CAPTURE, INPUT);
  pinMode(PIN_PPS_CAPTURE, INPUT);

  // Pin 13 is SPI SCK for the e-ink panel (shared with LED_BUILTIN).
  // Do not drive it as a GPIO LED.

  gpt2_extclk_capture_init();

  // Give the X20P time to boot before enabling UBX-NAV-TIMEUTC.
  gnss_timeutc_config_due_ms = millis() + 1000UL;

  sync_owtt_epd_status();
  owtt_epd_begin();
  owtt_epd_service();
  // One line so a bag shows whether the boot probe found the breakout.
  // Absent: no painter thread and no further SPI to the panel.
  host_send_line(owtt_epd_present() ? "#EPD,PRESENT" : "#EPD,ABSENT");
}


void loop()
{
  uint8_t frame[LINE_MAX];
  size_t n = 0;

  // Keep the X20P bridge responsive and passively parse UTC messages.
  service_gnss_bridge();
  maybe_send_gnss_boot_config();
  maybe_recover_gnss_link();

  // First: latch timing events so delta_t is independent of serial delays.
  process_timing_events();

  check_pending_config_timeout();

  // Host commands get forwarded immediately and do not wait for PPS.
  while (HOST_SERIAL.available() > 0) {
    const int b = HOST_SERIAL.read();
    if (b < 0) {
      break;
    }
    fa_push(host_fa, (uint8_t)b);
    while (fa_pop_host(host_fa, frame, sizeof(frame), &n)) {
      handle_host_frame(frame, n);
    }
  }

  // Modem frames are forwarded first (raw bytes + CRLF); receiver then adds
  // #I<tof_us> and, when available, #J<UTC,delta_t>.
  while (MODEM_SERIAL.available() > 0) {
    const int b = MODEM_SERIAL.read();
    if (b < 0) {
      break;
    }
    fa_push(modem_fa, (uint8_t)b);
    while (fa_pop_modem(modem_fa, frame, sizeof(frame), &n)) {
      handle_modem_frame(frame, n);
    }
  }

  // Service the higher-bandwidth GNSS path again after line processing.
  service_gnss_bridge();

  // Optional human-readable GNSS diagnostics go only to the first USB
  // host port. The transparent GNSS USB port always remains byte-clean.
  maybe_emit_gnss_live_debug();

  // Automatic transmitter broadcast runs after host commands,
  // so manual host commands have priority over scheduled GPS TX.
  execute_auto_tx_if_due();

  // E-ink status: coalesce full refreshes (see EPD_MIN_REFRESH_MS).
  sync_owtt_epd_status();
  owtt_epd_service();

  // Die temperature: idle-path only, after timing and serial have run.
  service_die_temp();
}

