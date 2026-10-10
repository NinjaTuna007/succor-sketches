// OCXO counter, PPS, holdover epochs, calibration, #TEMPMON.

#include "owtt_state.h"
#include "owtt_io.h"
#include "owtt_clock.h"
#include "owtt_gnss.h"
#include "owtt_radio.h"
#include "owtt_epd.h"

extern "C" void GPT2_IRQHandler();

static uint32_t epoch_ticks();
static int32_t ocxo_ppb_from_step(uint64_t step_q32);
static void ocxo_eeprom_remember(int32_t ppb, uint32_t span_s, uint32_t unix_s);
static void ocxo_eeprom_save();
static uint32_t ocxo_grid_cppb(uint32_t span_s);
static uint32_t ocxo_aging_cppb(uint32_t saved_unix_s, uint32_t now_unix_s);
static bool ocxo_new_span_wins(uint32_t new_span_s);
static const PpsCalSample *pps_cal_at(uint16_t age_index);
static void pps_cal_push(uint32_t tick, uint32_t epoch);
static bool pps_cal_estimate(uint64_t *out_step_q32);
static bool ocxo_cal_try_update();
static void pps_cal_on_real_pps(uint32_t t_epoch);
static void gpt2_schedule_epoch_train(uint32_t t_anchor, uint32_t n_ahead);
static void gpt2_enable_compare_irq();
static void gpt2_disable_compare_irq();
static bool pps_ignored_now();
static void handle_epoch_event(uint32_t t_epoch, bool real_pps);
static uint32_t epoch_ref_for_capture(uint32_t t_capture);
static void handle_rxs_event(uint32_t t_rxs);
static void maybe_enter_holdover();


// UTC label captured together with pending_delta_us.
static bool pending_delta_utc_valid = false;

static int64_t pending_delta_utc_second = 0;


// =======================================================
// ===================== TIMING ==========================
// =======================================================

// PPS rising edge, record the time and process it when have time
static volatile bool pps_pending = false;

// Store timestamp when rising edge hit
static volatile uint32_t pps_stamp = 0;


// Succor flagged a received message, record that time and then process it after passing messages to HOST
static volatile bool recv_flag_pending = false;

// Store timestamp when Succor flag was risen
static volatile uint32_t recv_flag_stamp = 0;


// For HOLDOVER (no PPS)
// Counter reached compare value (1 epoch has passed)
static volatile bool cmp_pending = false;

// Store timestamp and process it when free
static volatile uint32_t cmp_stamp = 0;


// Used for OCXO holdover.
// Compare interrupt fires once per measured second.
// The transmitter period is handled separately by counting epochs:
//   next_tx_epoch_count = epoch_count + cfg.period_pps
//
// Tick position of the next virtual epoch, Q32.32. The fractional part is what
// lets the holdover train run at the measured rate: a true second is around
// 10000000.47 ticks, so re-truncating to whole ticks every epoch would put a
// 100 ppb rounding error back in and undo the calibration.
//
// The integer part occupies bits 32..63, so uint64 wraparound is exactly the
// mod-2^32 arithmetic the free-running counter needs.
static volatile uint64_t next_compare_q32 = 0;

static uint32_t last_real_pps_us = 0; // Time of last synchronisation

static int32_t pending_delta_us = 0;

static uint32_t pending_delta_ms = 0;

static uint32_t pending_delta_holdover_age_s = 0;

static uint32_t pending_delta_gnss_age_ms = 0;

static char pending_delta_timing_mode = 'W';


static PpsCalSample pps_cal_ring[PPS_CAL_RING_LEN];

static uint16_t pps_cal_head = 0;

static uint16_t pps_cal_count = 0;

static uint16_t pps_cal_since_update = 0;


// Scratch for the median. Static rather than automatic: 2 KB does not belong on
// the main-loop stack.
static uint64_t pps_cal_slopes[PPS_CAL_RING_LEN / 2];


// Measured OCXO ticks in one true second, Q32.32.
static volatile uint64_t epoch_step_q32 = (uint64_t)EPOCH_TICKS_NOM << 32;

// Same measurement rounded to whole ticks, for deciding which epoch a capture
// belongs to. Sub-tick precision is irrelevant there.
static uint32_t epoch_ticks_int = EPOCH_TICKS_NOM;


static OcxoCalState ocxo_cal_state = OCXO_CAL_NOMINAL;

static uint32_t ocxo_cal_span_s = 0;

static uint16_t ocxo_cal_pairs = 0;

static uint32_t ocxo_eeprom_saved_ms = 0;

static int32_t ocxo_eeprom_saved_ppb = 0;

static uint32_t ocxo_eeprom_span_s = 0;

static uint32_t ocxo_eeprom_unix_s = 0;

static bool ocxo_eeprom_have_saved = false;

static bool ocxo_eeprom_dirty = false;


// MCU junction temperature from the i.MX RT1062 TEMPMON, in centi-degC.
// Written only from loop() / $ZTEMP?, never from handle_epoch_event.
static int32_t die_c_x100 = 0;

static uint32_t die_c_last_ms = 0;


// Whole OCXO ticks in one true second.
static inline uint32_t epoch_ticks()
{
  return epoch_ticks_int;
}


// Convert a counter interval to true microseconds. 10 MHz ticks are 0.1 us;
// at the measured rate a 1 s interval is off by ocxo_rate_ppb ns.
uint32_t ticks_to_us(uint32_t ticks)
{
  const int64_t nom_us =
    ((int64_t)ticks * (int64_t)EPOCH_US) / (int64_t)EPOCH_TICKS_NOM;
  const int64_t corr =
    (nom_us * (int64_t)ocxo_rate_ppb) / 1000000000LL;
  return (uint32_t)(nom_us - corr);
}


// ppb = (step - nominal) / nominal * 1e9
//     = diff_q32 * OCXO_PPB_Q32_SCALE / 2^32.
// Rounded rather than truncated so that a rate pinned by $ZOCXOCAL=<ppb>
// reads back unchanged.
static int32_t ocxo_ppb_from_step(uint64_t step_q32)
{
  const int64_t diff_q32 =
    (int64_t)step_q32 - ((int64_t)EPOCH_TICKS_NOM << 32);
  const int64_t scaled = diff_q32 * OCXO_PPB_Q32_SCALE;
  const int64_t half = (int64_t)1 << 31;
  return (int32_t)(((scaled >= 0) ? (scaled + half) : (scaled - half)) /
                   ((int64_t)1 << 32));
}


uint64_t ocxo_step_from_ppb(int32_t ppb)
{
  const int64_t diff_q32 =
    ((int64_t)ppb * ((int64_t)1 << 32)) / OCXO_PPB_Q32_SCALE;
  return (uint64_t)(((int64_t)EPOCH_TICKS_NOM << 32) + diff_q32);
}


void ocxo_apply_rate(uint64_t step_q32, OcxoCalState state)
{
  // The compare ISR reads epoch_step_q32 as two words; block it across the
  // update so it can never see a torn value.
  noInterrupts();
  epoch_step_q32 = step_q32;
  interrupts();

  epoch_ticks_int = (uint32_t)((step_q32 + 0x80000000ULL) >> 32);
  ocxo_rate_ppb = ocxo_ppb_from_step(step_q32);
  ocxo_cal_state = state;
}


// N = nominal 1e7, M = measured from PPS, F = pinned by $ZOCXOCAL=<ppb>,
// S = EEPROM restore of the last measured ppb (ring still empty).
char ocxo_cal_state_char()
{
  if (ocxo_cal_state == OCXO_CAL_MEASURED) {
    return 'M';
  }
  if (ocxo_cal_state == OCXO_CAL_MANUAL) {
    return 'F';
  }
  if (ocxo_cal_state == OCXO_CAL_STORED) {
    return 'S';
  }
  return 'N';
}


static void ocxo_eeprom_remember(int32_t ppb, uint32_t span_s, uint32_t unix_s)
{
  ocxo_eeprom_saved_ppb = ppb;
  ocxo_eeprom_span_s = span_s;
  ocxo_eeprom_unix_s = unix_s;
  ocxo_eeprom_have_saved = true;
}


static void ocxo_eeprom_save()
{
  if (ocxo_eeprom_saved_ppb > OCXO_RATE_MAX_PPB ||
      ocxo_eeprom_saved_ppb < -OCXO_RATE_MAX_PPB) {
    return;
  }
  OcxoEepromRec rec;
  rec.magic = OCXO_EEPROM_MAGIC;
  rec.ppb = ocxo_eeprom_saved_ppb;
  rec.span_s = ocxo_eeprom_span_s;
  rec.unix_s = ocxo_eeprom_unix_s;
  EEPROM.put(0, rec);
  ocxo_eeprom_saved_ms = millis();
  ocxo_eeprom_dirty = false;
}


void ocxo_eeprom_load()
{
  OcxoEepromRec rec;
  EEPROM.get(0, rec);
  if (rec.magic == OCXO_EEPROM_MAGIC) {
    if (rec.ppb > OCXO_RATE_MAX_PPB || rec.ppb < -OCXO_RATE_MAX_PPB) {
      return;
    }
    ocxo_apply_rate(ocxo_step_from_ppb(rec.ppb), OCXO_CAL_STORED);
    ocxo_eeprom_remember(rec.ppb, rec.span_s, rec.unix_s);
    return;
  }

  struct OcxoEepromV1 {
    uint32_t magic;
    int32_t ppb;
  } old;
  EEPROM.get(0, old);
  if (old.magic != OCXO_EEPROM_MAGIC_V1) {
    return;
  }
  if (old.ppb > OCXO_RATE_MAX_PPB || old.ppb < -OCXO_RATE_MAX_PPB) {
    return;
  }
  // No span and no date. Apply the ppb so boot is not nominal, and let the
  // first real ring replace it (unknown age == 20-year worst case).
  ocxo_apply_rate(ocxo_step_from_ppb(old.ppb), OCXO_CAL_STORED);
  ocxo_eeprom_remember(old.ppb, 0, 0);
}


// Pair grid in hundredths of a ppb. One 10 MHz tick is 100 ppb over 1 s, and
// pairs straddle half the stored span, so the grid is 200/span_s ppb.
static uint32_t ocxo_grid_cppb(uint32_t span_s)
{
  if (span_s == 0) {
    return 0xFFFFFFFFUL;
  }
  return (20000UL + span_s - 1U) / span_s;
}


// Hundredths of a ppb. See OCXO_AGING_* for the datasheet lines.
static uint32_t ocxo_aging_cppb(uint32_t saved_unix_s, uint32_t now_unix_s)
{
  if (saved_unix_s == 0 || now_unix_s == 0 || now_unix_s < saved_unix_s) {
    return OCXO_AGING_UNKNOWN_PPB * 100U;
  }
  const uint32_t age_s = now_unix_s - saved_unix_s;
  if (age_s == 0) {
    return 0;
  }
  const uint32_t days = (age_s + 86400UL - 1UL) / 86400UL;
  uint32_t ppb = days * OCXO_AGING_PPB_PER_DAY;
  if (days <= 365U) {
    if (ppb > OCXO_AGING_YEAR1_PPB) {
      ppb = OCXO_AGING_YEAR1_PPB;
    }
  } else {
    ppb = OCXO_AGING_YEAR1_PPB + (days - 365U) * OCXO_AGING_PPB_PER_DAY;
    if (ppb > OCXO_AGING_20Y_PPB) {
      ppb = OCXO_AGING_20Y_PPB;
    }
  }
  return ppb * 100U;
}


static bool ocxo_new_span_wins(uint32_t new_span_s)
{
  if (new_span_s == 0) {
    return false;
  }
  if (!ocxo_eeprom_have_saved || ocxo_eeprom_span_s == 0) {
    return true;
  }
  if (new_span_s >= ocxo_eeprom_span_s) {
    return true;
  }
  uint32_t now_unix_s = 0;
  if (gnss_utc.valid && gnss_utc.unix_second > 0 &&
      gnss_utc.unix_second <= 0xFFFFFFFFLL) {
    now_unix_s = (uint32_t)gnss_utc.unix_second;
  }
  // No current GNSS time: do not invent an age. A shorter ring stays put.
  // A stored record with unix 0 is undated; ocxo_aging_cppb treats that as
  // the 20-year worst case, so a tighter new ring can replace it.
  if (now_unix_s == 0) {
    return false;
  }
  const uint32_t old_u =
    ocxo_grid_cppb(ocxo_eeprom_span_s) +
    ocxo_aging_cppb(ocxo_eeprom_unix_s, now_unix_s);
  return ocxo_grid_cppb(new_span_s) < old_u;
}


void format_die_c(char *buf, size_t n)
{
  if (!die_c_valid || buf == NULL || n < 16) {
    if (buf != NULL && n > 0) {
      buf[0] = '\0';
    }
    return;
  }
  const char sign = (die_c_x100 < 0) ? '-' : '+';
  const uint32_t a = (uint32_t)((die_c_x100 < 0) ? -die_c_x100 : die_c_x100);
  snprintf(buf, n, "%c%lu.%02lu", sign, (unsigned long)(a / 100U),
           (unsigned long)(a % 100U));
}


void emit_tempmon_line()
{
  char die[16];
  format_die_c(die, sizeof(die));
  if (die[0] == '\0') {
    return;
  }
  char msg[48];
  snprintf(
    msg,
    sizeof(msg),
    "#TEMPMON,%lu,%c,%s",
    (unsigned long)epoch_count,
    timing_mode_code(timing_mode),
    die
  );
  host_send_line(msg);
}


bool sample_die_temp()
{
  // Do not call tempmonGetTemp() unless VALID is already set: that helper
  // busy-waits, and a conversion in flight is ~90 us at the core's 3/32768 s
  // measure period. Skip this pass rather than stall the ranging loop.
  if (!(TEMPMON_TEMPSENSE0 & 0x4U)) {
    return false;
  }
  const float t = tempmonGetTemp();
  die_c_x100 = (int32_t)(t * 100.0f + (t >= 0.0f ? 0.5f : -0.5f));
  die_c_valid = true;
  return true;
}


void service_die_temp()
{
  const uint32_t now = millis();
  if (die_c_valid && (uint32_t)(now - die_c_last_ms) < DIE_TEMP_PERIOD_MS) {
    return;
  }
  if (!sample_die_temp()) {
    return;
  }
  die_c_last_ms = now;
  emit_tempmon_line();
}


void ocxo_cal_report()
{
  const uint64_t step = epoch_step_q32;
  const uint32_t frac_e6 =
    (uint32_t)(((step & 0xFFFFFFFFULL) * 1000000ULL) >> 32);

  const char state_char = ocxo_cal_state_char();

  char msg[96];
  snprintf(
    msg,
    sizeof(msg),
    "#OCXOCAL,%c,%ld,%lu.%06lu,%u,%lu,%u",
    state_char,
    (long)ocxo_rate_ppb,
    (unsigned long)(step >> 32),
    (unsigned long)frac_e6,
    (unsigned)pps_cal_count,
    (unsigned long)ocxo_cal_span_s,
    (unsigned)ocxo_cal_pairs
  );
  host_send_line(msg);
}


void pps_cal_reset()
{
  pps_cal_head = 0;
  pps_cal_count = 0;
  pps_cal_since_update = 0;
  ocxo_cal_span_s = 0;
  ocxo_cal_pairs = 0;
}


// Chronological access: age_index 0 is the oldest retained sample.
static const PpsCalSample *pps_cal_at(uint16_t age_index)
{
  const uint16_t oldest = (uint16_t)(
    (pps_cal_head + PPS_CAL_RING_LEN - pps_cal_count) % PPS_CAL_RING_LEN);
  return &pps_cal_ring[(uint16_t)((oldest + age_index) % PPS_CAL_RING_LEN)];
}


static void pps_cal_push(uint32_t tick, uint32_t epoch)
{
  pps_cal_ring[pps_cal_head].tick = tick;
  pps_cal_ring[pps_cal_head].epoch = epoch;
  pps_cal_head = (uint16_t)((pps_cal_head + 1U) % PPS_CAL_RING_LEN);

  if (pps_cal_count < PPS_CAL_RING_LEN) {
    pps_cal_count++;
  }
}


static bool pps_cal_estimate(uint64_t *out_step_q32)
{
  if (out_step_q32 == NULL || pps_cal_count < PPS_CAL_MIN_SAMPLES) {
    return false;
  }

  const uint16_t half = (uint16_t)(pps_cal_count / 2U);
  uint16_t n = 0;

  for (uint16_t k = 0; k < half; k++) {
    const PpsCalSample *a = pps_cal_at(k);
    const PpsCalSample *b = pps_cal_at((uint16_t)(k + half));

    const uint32_t d_epoch = b->epoch - a->epoch;
    if (d_epoch == 0 || d_epoch > PPS_CAL_MAX_PAIR_EPOCHS) {
      continue;
    }

    const uint32_t d_tick = b->tick - a->tick;
    pps_cal_slopes[n++] = ((uint64_t)d_tick << 32) / (uint64_t)d_epoch;
  }

  if (n < PPS_CAL_MIN_SAMPLES / 2U) {
    return false;
  }

  // Insertion sort: n <= PPS_CAL_RING_LEN/2 and this runs at most once every
  // PPS_CAL_UPDATE_EDGES seconds.
  for (uint16_t i = 1; i < n; i++) {
    const uint64_t v = pps_cal_slopes[i];
    uint16_t j = i;
    while (j > 0 && pps_cal_slopes[j - 1] > v) {
      pps_cal_slopes[j] = pps_cal_slopes[j - 1];
      j--;
    }
    pps_cal_slopes[j] = v;
  }

  const uint64_t median = (n & 1U)
    ? pps_cal_slopes[n / 2U]
    : ((pps_cal_slopes[n / 2U - 1U] >> 1) + (pps_cal_slopes[n / 2U] >> 1));

  const PpsCalSample *oldest = pps_cal_at(0);
  const PpsCalSample *newest = pps_cal_at((uint16_t)(pps_cal_count - 1U));
  ocxo_cal_span_s = newest->epoch - oldest->epoch;
  ocxo_cal_pairs = n;

  *out_step_q32 = median;
  return true;
}


// Recompute from the ring and adopt the result if it is plausible. Returns true
// if a new rate was applied.
static bool ocxo_cal_try_update()
{
  uint64_t step_q32 = 0;
  if (!pps_cal_estimate(&step_q32)) {
    return false;
  }

  // A wild estimate means the capture train is wrong, not the oscillator.
  // Keeping the previous rate is always safer than trusting it.
  const int32_t ppb = ocxo_ppb_from_step(step_q32);
  if (ppb > OCXO_RATE_MAX_PPB || ppb < -OCXO_RATE_MAX_PPB) {
    return false;
  }
  if (!ocxo_new_span_wins(ocxo_cal_span_s)) {
    return false;
  }

  ocxo_apply_rate(step_q32, OCXO_CAL_MEASURED);
  ocxo_eeprom_saved_ppb = ppb;
  ocxo_eeprom_span_s = ocxo_cal_span_s;
  if (gnss_utc.valid && gnss_utc.unix_second > 0 &&
      gnss_utc.unix_second <= 0xFFFFFFFFLL) {
    ocxo_eeprom_unix_s = (uint32_t)gnss_utc.unix_second;
  }
  ocxo_eeprom_have_saved = true;
  ocxo_eeprom_dirty = true;
  return true;
}


static void pps_cal_on_real_pps(uint32_t t_epoch)
{
  const bool gnss_vouches =
    gnss_utc.valid &&
    gnss_utc.tAcc_ns <= PPS_CAL_MAX_TACC_NS &&
    (uint32_t)(millis() - gnss_utc.received_ms) <= PPS_CAL_UTC_FRESH_MS;

  if (!gnss_vouches) {
    return;
  }

  pps_cal_push(t_epoch, epoch_count);

  if (!ocxo_cal_auto) {
    return;
  }

  if (++pps_cal_since_update < PPS_CAL_UPDATE_EDGES) {
    return;
  }
  pps_cal_since_update = 0;

  const int32_t prev_ppb = ocxo_rate_ppb;
  const bool was_measured = (ocxo_cal_state == OCXO_CAL_MEASURED);

  if (!ocxo_cal_try_update()) {
    return;
  }

  const int32_t moved = (ocxo_rate_ppb > prev_ppb)
    ? (ocxo_rate_ppb - prev_ppb)
    : (prev_ppb - ocxo_rate_ppb);

  if (!was_measured || moved >= OCXO_CAL_REPORT_PPB) {
    ocxo_cal_report();
  }

  if (!ocxo_eeprom_dirty) {
    return;
  }
  const uint32_t min_ms = (pps_cal_count >= PPS_CAL_RING_LEN)
    ? OCXO_EEPROM_FULL_MS
    : OCXO_EEPROM_FILL_MS;
  const bool due = (ocxo_eeprom_saved_ms == 0) ||
    ((uint32_t)(millis() - ocxo_eeprom_saved_ms) >= min_ms);
  if (due) {
    ocxo_eeprom_save();
  }
}


// Fold every edge collected so far into the rate immediately, instead of waiting
// for the next PPS_CAL_UPDATE_EDGES boundary, and report what is in effect.
//
// This recomputes rather than clears. The ring's accumulated baseline is exactly
// what gives the estimate its resolution, so discarding samples would make the
// rate worse and leave it unavailable for the next PPS_CAL_MIN_SAMPLES edges.
//
// A rate pinned with $ZOCXOCAL=<ppb> is deliberately left alone, so arming the
// holdover experiment cannot silently re-enable estimation partway through an
// A/B baseline run.
void ocxo_cal_refresh_now()
{
  if (ocxo_cal_auto) {
    pps_cal_since_update = 0;
    ocxo_cal_try_update();
  }

  ocxo_cal_report();
}


uint32_t OCXO_counter_us()
{
  return GPT2_CNT;
}


// Place the virtual-epoch train n_ahead measured seconds after t_anchor. The
// anchor is always a real PPS capture, so the holdover train stays phase-locked
// to the last GNSS edge instead of accumulating the oscillator's rate error.
static inline void gpt2_schedule_epoch_train(uint32_t t_anchor, uint32_t n_ahead)
{
  noInterrupts();
  next_compare_q32 =
    ((uint64_t)t_anchor << 32) + epoch_step_q32 * (uint64_t)n_ahead;
  GPT2_OCR1 = (uint32_t)(next_compare_q32 >> 32);
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


char timing_mode_code(TimingMode mode)
{
  switch (mode) {
    case TIMING_PPS_LOCKED:
      return 'P';
    case TIMING_HOLDOVER:
      return 'H';
    case TIMING_WAIT_PPS:
    default:
      return 'W';
  }
}


static bool pps_ignored_now()
{
  if (!ignore_pps_armed || ignore_pps_after_s == 0) {
    return false;
  }
  return (int32_t)(millis() - ignore_pps_deadline_ms) >= 0;
}


// Build an absolute UTC timestamp from the UTC-labelled epoch and the OCXO
// counter. This does not use serial-message arrival time for the fractional
// timestamp. The serial UTC message labels the second; GPT2 supplies the
// microseconds inside it.
bool utc_timestamp_for_counter(
  uint32_t counter_us,
  uint64_t *out_timestamp_us,
  int64_t *out_epoch_second,
  uint32_t *out_fraction_us)
{
  if (out_timestamp_us == NULL ||
      out_epoch_second == NULL ||
      out_fraction_us == NULL ||
      !pps_valid ||
      !current_epoch_utc_valid) {
    return false;
  }

  int64_t second = current_epoch_utc_second;
  uint32_t ref = current_epoch_us;

  // If a capture belongs to the immediately previous epoch, move both the
  // counter reference and UTC label back together.
  if ((int32_t)(counter_us - ref) < 0) {
    ref -= epoch_ticks();
    second--;
  }

  const uint32_t elapsed_ticks = (uint32_t)(counter_us - ref);

  // This also handles the small window where an epoch interrupt occurred but
  // the main loop has not processed it yet. Whole seconds are counted in the
  // tick domain; only the sub-second remainder becomes microseconds.
  second += (int64_t)(elapsed_ticks / epoch_ticks());
  const uint32_t fraction_us = ticks_to_us(elapsed_ticks % epoch_ticks());

  *out_epoch_second = second;
  *out_fraction_us = fraction_us;
  *out_timestamp_us =
    (uint64_t)second * (uint64_t)EPOCH_US + (uint64_t)fraction_us;
  return true;
}


void gpt2_extclk_capture_init()
{
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON) |
               CCM_CCGR0_GPT2_SERIAL(CCM_CCGR_ON);

  GPT2_CR = 0;
  GPT2_PR = 0;
  GPT2_SR = 0x3F;
  GPT2_IR = 0;

  // External clock on pin 14:
  // GPIO_AD_B1_02 -> GPT2_CLK
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;
  IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT = 1;

  // Receive Flag capture on pin 15:
  // GPIO_AD_B1_03 -> GPT2_CAPTURE1
  IOMUXC_GPT2_IPP_IND_CAPIN1_SELECT_INPUT = 1;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_03 = 8;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_03 = 0x13000;

  // PPS capture on pin 40:
  // GPIO_AD_B1_04 -> GPT2_CAPTURE2
  IOMUXC_GPT2_IPP_IND_CAPIN2_SELECT_INPUT = 1;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_04 = 8;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_04 = 0x13000;

  GPT2_PR = GPT_PR_PRESCALER(GPT2_PRESCALER_VALUE);

  // CLKSRC(3): external clock
  // FRR: free-running
  // IM1(1): capture 1 rising edge, Receive Flag
  // IM2(1): capture 2 rising edge, PPS
  GPT2_CR = GPT_CR_CLKSRC(3) |
            GPT_CR_FRR |
            GPT_CR_IM1(1) |
            GPT_CR_IM2(1);

  GPT2_IR = GPT_IR_IF1IE | GPT_IR_IF2IE;

  NVIC_CLEAR_PENDING(IRQ_GPT2);
  attachInterruptVector(IRQ_GPT2, GPT2_IRQHandler);
  NVIC_ENABLE_IRQ(IRQ_GPT2);

  GPT2_CR |= GPT_CR_EN;
}


extern "C" void GPT2_IRQHandler()
{
  uint32_t sr = GPT2_SR;

  // Receive Flag capture: acoustic packet/header detected.
  if (sr & GPT_SR_IF1) {
    recv_flag_stamp = GPT2_ICR1;
    recv_flag_pending = true;
    GPT2_SR = GPT_SR_IF1;
  }

  // PPS capture
  if (sr & GPT_SR_IF2) {
    pps_stamp = GPT2_ICR2;
    pps_pending = true;
    GPT2_SR = GPT_SR_IF2;
    // Pin 13 is SPI SCK for the e-ink panel; do not toggle LED_BUILTIN here.
  }

  // Holdover virtual PPS compare.
  if (sr & GPT_SR_OF1) {
    cmp_stamp = GPT2_OCR1;
    cmp_pending = true;
    GPT2_SR = GPT_SR_OF1;

    // Accumulate in Q32.32 so the sub-tick part of the measured second carries
    // across epochs instead of being truncated away each time.
    next_compare_q32 += epoch_step_q32;
    GPT2_OCR1 = (uint32_t)(next_compare_q32 >> 32);
  }

  asm volatile("dsb");
}


static void handle_epoch_event(uint32_t t_epoch, bool real_pps)
{
  // Advance by *elapsed hardware time*, not by processed-event count. The
  // pending-PPS slot only holds the latest capture: if the main loop stalls
  // (USB backpressure), several edges collapse into one call and per-call ++
  // would leave the UTC label seconds behind (observed as #UTC,REBIND,diff_s=15
  // after node restarts). E-ink refreshes run on a TeensyThreads worker and
  // no longer stall this path. The capture timestamps are exact, so the
  // epoch delta is exact.
  //
  // elapsed == 0 happens on holdover→PPS relock when the real edge lands
  // within half an epoch of the last virtual edge: same second, re-aligned
  // timestamp, no label advance (per-call ++ used to double-count here).
  uint32_t elapsed = 1;
  if (pps_valid) {
    const uint32_t dt = (uint32_t)(t_epoch - current_epoch_us);
    elapsed = (dt + epoch_ticks() / 2U) / epoch_ticks();
  }

  current_epoch_us = t_epoch;
  pps_valid = true;
  epoch_count += elapsed;

  if (current_epoch_utc_valid) {
    current_epoch_utc_second += (int64_t)elapsed;
  }

  if (real_pps) {
    holdover_age_s = 0;
  } else if (timing_mode == TIMING_HOLDOVER) {
    holdover_age_s += elapsed;
  }

  // Feed the rate estimator before the train is re-anchored below, so a new
  // measurement takes effect on this edge rather than the next one.
  if (real_pps) {
    pps_cal_on_real_pps(t_epoch);
  }

  if (payload_valid_epochs != 0 &&
      latest_gps_valid &&
      (uint32_t)(epoch_count - latest_gps_epoch_count) > payload_valid_epochs) {
    latest_gps_valid = false;
    latest_gps_len = 0;
    tx_last_warning = TX_WARN_NONE;
    // Host $G/$K went stale — paint red "HOST SILENT" without waiting the e-ink gate.
    owtt_epd_request_immediate();
  }

  if (payload_valid_epochs != 0 &&
      latest_telem_valid &&
      (uint32_t)(epoch_count - latest_telem_epoch_count) > payload_valid_epochs) {
    latest_telem_valid = false;
    latest_telem_len = 0;
    tx_last_warning = TX_WARN_NONE;
    // Host $G/$K went stale — paint red "HOST SILENT" without waiting the e-ink gate.
    owtt_epd_request_immediate();
  }

  if (real_pps) {
    last_real_pps_us = t_epoch;
    timing_mode = TIMING_PPS_LOCKED;

    gpt2_schedule_epoch_train(t_epoch, 1);
    gpt2_disable_compare_irq();
  }

  if (cfg.mode == MODE_TRANSMITTER && cfg.tx_scheduled) {
    if ((int32_t)(epoch_count - cfg.next_tx_epoch_count) >= 0) {
      tx_auto_due = true;

      if (is_first_transmitter()) {
        cfg.next_tx_epoch_count = epoch_count + cfg.period_pps;
        cfg.tx_scheduled = true;
      } else {
        cfg.tx_scheduled = false;
      }
    }
  }
}


static uint32_t epoch_ref_for_capture(uint32_t t_capture)
{
  uint32_t ref = current_epoch_us;

  // If the captured flag timestamp is before the current epoch edge,
  // then the flag actually belongs to the previous epoch.
  //
  // This can happen when both PPS and receive flag are pending, and the
  // main loop processes PPS first.
  if ((int32_t)(t_capture - ref) < 0) {
    ref -= epoch_ticks();
  }

  return ref;
}


static void handle_rxs_event(uint32_t t_rxs)
{
  // Only receiver mode produces #I timing outputs.
  if (cfg.mode != MODE_RECEIVER) {
    return;
  }

  if (!pps_valid) {
    pending_delta_valid = false;
    pending_delta_utc_valid = false;
    return;
  }

  uint32_t ref_epoch_us = epoch_ref_for_capture(t_rxs);

  pending_delta_us =
    (int32_t)ticks_to_us((uint32_t)(t_rxs - ref_epoch_us));
  pending_delta_ms = millis();
  pending_delta_valid = true;
  pending_delta_holdover_age_s = holdover_age_s;
  pending_delta_timing_mode = timing_mode_code(timing_mode);
  pending_delta_gnss_age_ms = gnss_utc.valid
    ? (uint32_t)(millis() - gnss_utc.received_ms)
    : 0xFFFFFFFFUL;

  pending_delta_utc_valid = current_epoch_utc_valid;
  pending_delta_utc_second = current_epoch_utc_second;

  // epoch_ref_for_capture() can select the previous epoch when PPS and RxS
  // interrupts are processed together in the main loop.
  if (pending_delta_utc_valid && ref_epoch_us != current_epoch_us) {
    pending_delta_utc_second--;
  }

#if ENABLE_USB_DEBUG
  Serial.print("Rx flag captured delta_us=");
  Serial.print(pending_delta_us);
  Serial.print(", ref_epoch_us=");
  Serial.println(ref_epoch_us);
#endif
}


static void maybe_enter_holdover()
{
  if (timing_mode != TIMING_PPS_LOCKED) {
    return;
  }

  uint32_t now = OCXO_counter_us();
  uint32_t since_last_pps = (uint32_t)(now - last_real_pps_us);

  if (since_last_pps <= PPS_TIMEOUT_TICKS) {
    return;
  }

  uint32_t missed_epochs = since_last_pps / epoch_ticks();
  if (missed_epochs == 0) {
    missed_epochs = 1;
  }

  // Account for each virtual epoch that has already elapsed since the last
  // real PPS. The previous implementation moved current_epoch_us forward
  // without incrementing epoch_count, which also made an absolute UTC label
  // lag by one second after entering holdover.
  timing_mode = TIMING_HOLDOVER;

  // Place every virtual epoch at the measured rate from the last real edge
  // rather than at a nominal 1e7 ticks, so the train inherits GNSS phase and
  // does not walk off at the oscillator's frequency offset.
  for (uint32_t i = 1; i <= missed_epochs; i++) {
    const uint64_t pos_q32 =
      ((uint64_t)last_real_pps_us << 32) + epoch_step_q32 * (uint64_t)i;
    handle_epoch_event((uint32_t)(pos_q32 >> 32), false);
  }

  gpt2_schedule_epoch_train(last_real_pps_us, missed_epochs + 1U);
  gpt2_enable_compare_irq();

  // Record the rate the holdover actually inherits, and the ring depth behind
  // it, while those counters still hold the pre-cut window. $ZIGNOREPPSAFTER
  // reports at arm time, but the estimator keeps revising for the N seconds
  // until the cut, so that earlier line is not necessarily what is in effect
  // here. The reset below would otherwise zero samples/span/pairs.
  ocxo_cal_report();

  // Start a fresh calibration window on the next lock: pairs straddling the
  // outage could span more than the counter's 429.5 s wrap.
  pps_cal_reset();

#if ENABLE_USB_DEBUG
  Serial.println("TIMING: entered OCXO holdover");
#endif
}


void process_timing_events()
{
  // Real PPS prioritized. If PPS comes back, it re-locks.
  if (pps_pending) {
    noInterrupts();
    uint32_t t = pps_stamp;
    pps_pending = false;
    interrupts();

    // Holdover experiment, or a bench $ZZEROSTART epoch: drop real PPS
    // captures so the OCXO train keeps the timebase until it is cleared.
    if (!pps_ignored_now() && !bench_epoch_active) {
      handle_epoch_event(t, true);
    }
  }

  // Check if PPS has disappeared -> switch to OCXO holdover
  maybe_enter_holdover();

  // Virtual PPS from GPT2 compare while in holdover.
  if (cmp_pending) {
    noInterrupts();
    uint32_t t = cmp_stamp;
    cmp_pending = false;
    interrupts();

    if (timing_mode == TIMING_HOLDOVER) {
      handle_epoch_event(t, false);
    }
  }

  // Receive flag capture after epoch bookkeeping.
  // The delta calculation chooses the correct epoch based on the captured
  // timestamp, not simply the newest processed PPS.

  if (recv_flag_pending) {
    noInterrupts();
    uint32_t t = recv_flag_stamp;
    recv_flag_pending = false;
    interrupts();

    handle_rxs_event(t);
  }
}


bool consume_delta(
  int32_t *out_delta_us,
  bool *out_utc_valid,
  int64_t *out_utc_second,
  uint32_t *out_holdover_age_s,
  uint32_t *out_gnss_age_ms,
  char *out_timing_mode,
  uint32_t *out_decode_delay_ms)
{
  if (out_delta_us == NULL ||
      out_utc_valid == NULL ||
      out_utc_second == NULL ||
      out_holdover_age_s == NULL ||
      out_gnss_age_ms == NULL ||
      out_timing_mode == NULL ||
      out_decode_delay_ms == NULL) {
    return false;
  }

  if (!pending_delta_valid) {
    return false;
  }

  *out_delta_us = pending_delta_us;
  *out_utc_valid = pending_delta_utc_valid;
  *out_utc_second = pending_delta_utc_second;
  *out_holdover_age_s = pending_delta_holdover_age_s;
  *out_gnss_age_ms = pending_delta_gnss_age_ms;
  *out_timing_mode = pending_delta_timing_mode;
  *out_decode_delay_ms = (uint32_t)(millis() - pending_delta_ms);

  pending_delta_valid = false;
  pending_delta_utc_valid = false;
  return true;
}


// =======================================================
// ===================== HOST / MODEM LOGIC ==============
// =======================================================

const char *timing_mode_name()
{
  switch (timing_mode) {
    case TIMING_WAIT_PPS:
      return "WAIT_PPS";
    case TIMING_PPS_LOCKED:
      return "PPS_LOCKED";
    case TIMING_HOLDOVER:
      return "HOLDOVER";
  }

  return "UNKNOWN";
}


void bench_epoch_stop()
{
  bench_epoch_active = false;
  timing_mode = TIMING_WAIT_PPS;
  pps_valid = false;
  current_epoch_utc_valid = false;
  holdover_age_s = 0;
  tx_auto_due = false;
  gpt2_disable_compare_irq();
  noInterrupts();
  cmp_pending = false;
  interrupts();
}


void bench_epoch_start(int64_t unix_second)
{
  const uint32_t now = OCXO_counter_us();
  noInterrupts();
  cmp_pending = false;
  current_epoch_us = now;
  last_real_pps_us = now;
  pps_valid = true;
  current_epoch_utc_second = unix_second;
  current_epoch_utc_valid = true;
  timing_mode = TIMING_HOLDOVER;
  holdover_age_s = 0;
  bench_epoch_active = true;
  interrupts();
  gpt2_schedule_epoch_train(now, 1);
  gpt2_enable_compare_irq();
  schedule_first_transmitter_after_config();
}

