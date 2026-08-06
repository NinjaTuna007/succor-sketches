// OWTT stick e-ink status panel (Adafruit 2.13" tricolor ThinkInk).
//
// Landscape 3-line glance UI. Full-screen fill encodes severity:
//   white = OK, red = needs attention, black = offline / no link.
// Pins match eink_test. Full refreshes are coalesced: dirty flag +
// EPD_MIN_REFRESH_MS (default 180s per Adafruit guidance).
//
// display.display() blocks ~13 s on this panel (BUSY pin unwired). That
// call runs on a TeensyThreads worker so the OWTT main loop never stalls.
#pragma once

#include <Arduino.h>

#ifndef EPD_ENABLE
#define EPD_ENABLE 1
#endif

#ifndef EPD_MIN_REFRESH_MS
#define EPD_MIN_REFRESH_MS 180000UL
#endif

// Floor between forced severity-change refreshes (full frame ~13s).
#ifndef EPD_SEVERITY_REFRESH_MS
#define EPD_SEVERITY_REFRESH_MS 20000UL
#endif

struct OwttEpdStatus {
  char mode_char;           // W / R / T
  char own_id[4];
  char timing_char;         // W / P / H
  uint32_t holdover_age_s;
  bool gnss_fresh;
  bool gps_valid;
  bool telem_valid;
  bool cfg_pending;
  char cfg_line[20];        // "ok", "pending", or abort reason
};

void owtt_epd_begin();
void owtt_epd_set_status(const OwttEpdStatus &status);
void owtt_epd_mark_dirty();
void owtt_epd_service();
