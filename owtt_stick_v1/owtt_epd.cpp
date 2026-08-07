#include "owtt_epd.h"

#if EPD_ENABLE

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ThinkInk.h>
#include <TeensyThreads.h>
#include <string.h>
#include <stdio.h>

static constexpr int EPD_DC    = 10;
static constexpr int EPD_CS    = 4;
static constexpr int SRAM_CS   = 3;
static constexpr int EPD_RESET = 5;
static constexpr int EPD_BUSY  = -1;

static constexpr int EPD_THREAD_STACK = 8192;

static ThinkInk_213_Tricolor_MFGNR display(
  EPD_DC,
  EPD_RESET,
  EPD_CS,
  SRAM_CS,
  EPD_BUSY,
  &SPI
);

static bool epd_ready = false;
static bool epd_dirty = false;
static bool epd_force_next = false;
static uint32_t epd_last_refresh_ms = 0;

static OwttEpdStatus pending_status = {};
static OwttEpdStatus painted_status = {};
static bool have_pending = false;
static bool have_painted = false;

// Background painter. Adafruit's display() busy-waits ~13 s with no BUSY
// pin; running it here keeps the OWTT main loop real-time via preemption.
static volatile bool epd_refresh_request = false;
static volatile bool epd_busy = false;
static OwttEpdStatus paint_request = {};
static int epd_thread_id = -1;

enum EpdSeverity : uint8_t {
  EPD_SEV_OK = 0,          // white fill
  EPD_SEV_ATTENTION = 1,   // red fill
  EPD_SEV_OFFLINE = 2      // black fill (no useful function yet / stuck)
};

static bool cfg_failed(const OwttEpdStatus &s)
{
  return s.cfg_line[0] != '\0' &&
         strcmp(s.cfg_line, "ok") != 0 &&
         strcmp(s.cfg_line, "pending") != 0;
}

static bool cfg_busy(const OwttEpdStatus &s)
{
  return s.cfg_pending || strcmp(s.cfg_line, "pending") == 0;
}

// Transmitter with no live host $G/$K payload — treat like GNSS death for UI.
static bool host_payload_dead(const OwttEpdStatus &s)
{
  return s.mode_char == 'T' && !s.gps_valid && !s.telem_valid;
}

// Live severity (not MCU-crash detection — panel only shows last paint).
static EpdSeverity severity_of(const OwttEpdStatus &s)
{
  // No PPS lock and no fresh GNSS => not operational.
  if (s.timing_char == 'W' && !s.gnss_fresh) {
    return EPD_SEV_OFFLINE;
  }

  // Holdover, waiting for PPS, stale GNSS/host payload, or config trouble.
  if (s.timing_char == 'H' ||
      s.timing_char == 'W' ||
      !s.gnss_fresh ||
      host_payload_dead(s) ||
      cfg_failed(s) ||
      cfg_busy(s)) {
    return EPD_SEV_ATTENTION;
  }

  return EPD_SEV_OK;
}

static bool status_equal(const OwttEpdStatus &a, const OwttEpdStatus &b)
{
  // Omit utc_second: it advances every epoch and must not dirty the panel.
  return a.mode_char == b.mode_char &&
         strcmp(a.own_id, b.own_id) == 0 &&
         a.timing_char == b.timing_char &&
         a.holdover_age_s == b.holdover_age_s &&
         a.gnss_fresh == b.gnss_fresh &&
         a.gps_valid == b.gps_valid &&
         a.telem_valid == b.telem_valid &&
         a.cfg_pending == b.cfg_pending &&
         strcmp(a.cfg_line, b.cfg_line) == 0;
}

// Mode/id/config/timing/GNSS-or-host-payload health are operator-visible —
// don't sit on the 180s lifetime gate.
static bool priority_fields_changed(
  const OwttEpdStatus &painted,
  const OwttEpdStatus &pending)
{
  return painted.mode_char != pending.mode_char ||
         strcmp(painted.own_id, pending.own_id) != 0 ||
         painted.timing_char != pending.timing_char ||
         painted.gnss_fresh != pending.gnss_fresh ||
         painted.gps_valid != pending.gps_valid ||
         painted.telem_valid != pending.telem_valid ||
         painted.cfg_pending != pending.cfg_pending ||
         strcmp(painted.cfg_line, pending.cfg_line) != 0;
}

static void render_status(const OwttEpdStatus &s)
{
  // Native 250x122 landscape.
  display.setRotation(0);
  display.setTextWrap(false);
  display.clearBuffer();

  const EpdSeverity sev = severity_of(s);
  uint16_t bg = EPD_WHITE;
  uint16_t fg = EPD_BLACK;
  if (sev == EPD_SEV_ATTENTION) {
    bg = EPD_RED;
    fg = EPD_BLACK;
  } else if (sev == EPD_SEV_OFFLINE) {
    bg = EPD_BLACK;
    fg = EPD_WHITE;
  }

  display.fillScreen(bg);
  display.setTextSize(3);
  display.setTextColor(fg);

  char line[24];

  // Line 1: mode + own id
  snprintf(line, sizeof(line), "%c %s", s.mode_char, s.own_id);
  display.setCursor(8, 20);
  display.print(line);

  // Line 2: timing
  if (s.timing_char == 'H') {
    snprintf(line, sizeof(line), "HOLD %lus", (unsigned long)s.holdover_age_s);
  } else if (s.timing_char == 'P') {
    snprintf(line, sizeof(line), "PPS");
  } else {
    snprintf(line, sizeof(line), "WAIT");
  }
  display.setCursor(8, 52);
  display.print(line);

  // Line 3: GNSS / config summary (≤13 chars at text size 3 on 250px).
  if (cfg_busy(s)) {
    snprintf(line, sizeof(line), "CFG PEND");
  } else if (cfg_failed(s)) {
    if (strcmp(s.cfg_line, "ADDR_TIMEOUT") == 0) {
      snprintf(line, sizeof(line), "CFG TMO");
    } else if (strcmp(s.cfg_line, "ADDRESS_REJECTED") == 0) {
      snprintf(line, sizeof(line), "CFG REJ");
    } else {
      // Keep it short; unknown reasons get a tight prefix.
      snprintf(line, sizeof(line), "CFG %.7s", s.cfg_line);
    }
  } else if (!s.gnss_fresh) {
    snprintf(line, sizeof(line), "GNSS DEAD");
  } else if (host_payload_dead(s)) {
    // Transmitter with expired/missing host $G/$K — not the X20P link.
    snprintf(line, sizeof(line), "HOST SILENT");
  } else if (s.mode_char == 'T') {
    if (s.telem_valid) {
      snprintf(line, sizeof(line), "GNSS OK TEL");
    } else if (s.gps_valid) {
      snprintf(line, sizeof(line), "GNSS OK GPS");
    } else {
      snprintf(line, sizeof(line), "GNSS OK");
    }
  } else {
    snprintf(line, sizeof(line), "GNSS OK");
  }
  display.setCursor(8, 84);
  display.print(line);
}

static void epd_worker(void * /*arg*/)
{
  for (;;) {
    if (epd_refresh_request) {
      // Snapshot under a brief critical section so service() cannot
      // overwrite paint_request mid-copy.
      noInterrupts();
      const OwttEpdStatus s = paint_request;
      epd_refresh_request = false;
      epd_busy = true;
      interrupts();

      render_status(s);
      display.display();  // ~13 s busy-wait; this thread only

      noInterrupts();
      painted_status = s;
      have_painted = true;
      epd_last_refresh_ms = millis();
      epd_force_next = false;
      // If status moved again while we painted, keep dirty so service
      // can schedule another refresh under the normal cadence gates.
      if (have_pending && !status_equal(painted_status, pending_status)) {
        epd_dirty = true;
      } else {
        epd_dirty = false;
      }
      epd_busy = false;
      interrupts();
    }
    threads.yield();
  }
}

void owtt_epd_begin()
{
  pinMode(EPD_CS, OUTPUT);
  digitalWrite(EPD_CS, HIGH);

  pinMode(SRAM_CS, OUTPUT);
  digitalWrite(SRAM_CS, HIGH);

  pinMode(EPD_RESET, OUTPUT);
  digitalWrite(EPD_RESET, HIGH);
  delay(100);
  digitalWrite(EPD_RESET, LOW);
  delay(20);
  digitalWrite(EPD_RESET, HIGH);
  delay(500);

  SPI.begin();

  display.begin(THINKINK_TRICOLOR);
  epd_ready = true;

  // Caller should owtt_epd_set_status() then owtt_epd_service() for the
  // boot frame. Force the first refresh past the min-interval gate.
  epd_force_next = true;
  epd_last_refresh_ms = 0;
  if (have_pending) {
    epd_dirty = true;
  }

  // Prefer the OWTT main thread when the painter is spinning in delay().
  threads.setTimeSlice(0, 10);
  epd_thread_id = threads.addThread(epd_worker, 0, EPD_THREAD_STACK);
  if (epd_thread_id >= 0) {
    threads.setTimeSlice(epd_thread_id, 1);
  }
}

void owtt_epd_set_status(const OwttEpdStatus &status)
{
  if (have_pending && status_equal(pending_status, status)) {
    return;
  }

  pending_status = status;
  have_pending = true;

  if (!have_painted || !status_equal(painted_status, status)) {
    epd_dirty = true;
  }
}

void owtt_epd_mark_dirty()
{
  epd_dirty = true;
}

void owtt_epd_request_immediate()
{
  epd_dirty = true;
  epd_force_next = true;
}

void owtt_epd_service()
{
  if (!epd_ready || !epd_dirty || !have_pending) {
    return;
  }

  // One in-flight refresh at a time; leave dirty set for a later pass.
  if (epd_busy || epd_refresh_request) {
    return;
  }

  if (have_painted && status_equal(painted_status, pending_status)) {
    epd_dirty = false;
    return;
  }

  const EpdSeverity pending_sev = severity_of(pending_status);
  const EpdSeverity painted_sev =
    have_painted ? severity_of(painted_status) : EPD_SEV_OFFLINE;
  const bool severity_changed = !have_painted || pending_sev != painted_sev;
  const bool priority_changed =
    !have_painted ||
    severity_changed ||
    priority_fields_changed(painted_status, pending_status);
  // Escaping the black boot frame, severity colour, or mode/id/config/timing
  // must not wait the full 180s lifetime gate — only a short floor around one
  // refresh (~20s, longer than the ~13s tricolor paint).
  const uint32_t min_ms = priority_changed
    ? EPD_PRIORITY_REFRESH_MS
    : EPD_MIN_REFRESH_MS;

  if (!epd_force_next) {
    const uint32_t elapsed = (uint32_t)(millis() - epd_last_refresh_ms);
    if (elapsed < min_ms) {
      return;
    }
  }

  // Queue for the worker and return immediately — never call display()
  // from the main loop.
  noInterrupts();
  paint_request = pending_status;
  epd_dirty = false;
  epd_refresh_request = true;
  interrupts();
}

#else  // !EPD_ENABLE

void owtt_epd_begin() {}
void owtt_epd_set_status(const OwttEpdStatus &) {}
void owtt_epd_mark_dirty() {}
void owtt_epd_request_immediate() {}
void owtt_epd_service() {}

#endif
