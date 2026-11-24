#include "board_uptime.h"
#include "daq_acq_clock.h"
#include "board_watchdog.h"
#include "DAQ-Threads.h"

#include "daq_board.h"
#include "daq_dac.h"
#include "main.h"
#include "stm32u5xx_hal_gpio.h"
#include "sd_card.h"
#include "daq_log_time.h"
#include "sedsnet_config.h"
#include "mcp3564r.h"
#include "daq_calibration.h"
#include "daq_rates.h"
#include "daq_network_schedule.h"
#include "daq_timestamp.h"
#include "daq_filter.h"
#include "daq_chamber_pressure.h"
#include "daq_input_faults.h"
#include "daq_median.h"
#include "daq_channel_stats.h"
#include "daq_over_range.h"
#include <string.h>
#include <math.h>
TX_THREAD daq_thread;

#define DAQ_THREAD_STACK_SIZE (32U * 1024U)
#define DAQ_SAMPLE_PERIOD_MS DAQ_ACQUISITION_PERIOD_MS
#define DAQ_SAMPLE_PERIOD_TICKS ((DAQ_SAMPLE_PERIOD_MS * TX_TIMER_TICKS_PER_SECOND) / 1000U)
#define DAQ_INPUT_VOLTAGE_LOW_V 8.5f
#define DAQ_INPUT_VOLTAGE_RECOVERED_V 9.0f
#define DAQ_ENABLE_DUMMY_CAN_TELEMETRY 0U
#define DAQ_RAW_BATCH_MAX DAQ_RAW_BATCH_CAPACITY

volatile uint32_t g_daq_sample_ok_count = 0U;
volatile uint32_t g_daq_sample_fail_count = 0U;
volatile uint32_t g_daq_init_fail_count = 0U;
volatile uint32_t g_daq_loadcell_publish_ok_count = 0U;
volatile uint32_t g_daq_loadcell_publish_fail_count = 0U;
volatile uint32_t g_daq_kg50_publish_ok_count = 0U;
volatile uint32_t g_daq_kg50_publish_fail_count = 0U;
volatile uint32_t g_daq_temperature_publish_ok_count = 0U;
volatile uint32_t g_daq_temperature_publish_fail_count = 0U;
volatile uint32_t g_daq_raw_samples_drained_count = 0U;
volatile uint32_t g_daq_nonzero_raw_sample_count = 0U;
volatile uint32_t g_daq_sd_raw_batch_drop_count = 0U;
volatile uint32_t g_daq_sample_overrun_count = 0U;
volatile uint32_t g_daq_cycle_elapsed_ticks = 0U;
volatile uint32_t g_daq_cycle_max_ticks = 0U;
volatile uint32_t g_daq_first_overrun_ticks = 0U;
volatile uint32_t g_daq_first_overrun_sample = 0U;
volatile uint32_t g_daq_sampling_max_ticks = 0U;
volatile uint32_t g_daq_drain_max_ticks = 0U;
volatile uint32_t g_daq_publish_max_ticks = 0U;
volatile uint32_t g_daq_adc_service_max_ticks = 0U;

/* Wall-time profiling, including preemption and mutex waits. Stages are board
 * sampling, raw drain/enqueue, publishing, and ADC service. Read deltas of the
 * totals/counts to locate rate limits without stopping acquisition. */
volatile uint32_t g_daq_stage_ticks[4] = {0};
volatile uint32_t g_daq_stage_max_ticks[4] = {0};
volatile uint32_t g_daq_stage_count[4] = {0};

static ULONG daq_profile_stage(unsigned stage, ULONG started)
{
  const ULONG now = tx_time_get();
  const uint32_t elapsed = (uint32_t)(now - started);
  g_daq_stage_ticks[stage] += elapsed;
  if (elapsed > g_daq_stage_max_ticks[stage])
    g_daq_stage_max_ticks[stage] = elapsed;
  g_daq_stage_count[stage]++;
  switch (stage) {
    case 0U: g_daq_sampling_max_ticks = g_daq_stage_max_ticks[stage]; break;
    case 1U: g_daq_drain_max_ticks = g_daq_stage_max_ticks[stage]; break;
    case 2U: g_daq_publish_max_ticks = g_daq_stage_max_ticks[stage]; break;
    default: g_daq_adc_service_max_ticks = g_daq_stage_max_ticks[stage]; break;
  }
  return now;
}

static ULONG g_daq_thread_stack[DAQ_THREAD_STACK_SIZE / sizeof(ULONG)];

typedef struct
{
  int64_t code_sum[10];
  float sum[10];
  float voltage_sum[10];
  uint16_t count[10];
} daq_loadcell_window_t;

/* Published (network/display) values completed during one drain. SD keeps
 * every raw conversion; these are the median-filtered report bins
 * (daq_median.h, rates in daq_rates.h). Times are bin centres, uptime ms. */
typedef struct { float value; uint64_t captured_ms; uint8_t ready; } daq_bin_output_t;
typedef struct { daq_bin_output_t kg1000, kg50, chamber_v; } daq_loadcell_bins_t;

/* Latest median-filtered auxiliary MCP3564R channel values (CH1..CH7 that
 * are not a fast channel or the selected KG50), written by acquisition and
 * read by the analog reporter under the IRQ lock. */
static float g_daq_aux_value[8];
static uint32_t g_daq_aux_ms[8];
static uint8_t g_daq_aux_valid;

/* Per-channel noise/drift statistics for DAQ_INPUT_HEALTH v2. */
static daq_channel_stats_t g_daq_stats[DAQ_STATS_CHANNELS];

static void daq_store_aux(unsigned channel, float value, uint32_t now_ms)
{
  const uint32_t saved = __get_PRIMASK();
  __disable_irq();
  g_daq_aux_value[channel] = value;
  g_daq_aux_ms[channel] = now_ms;
  g_daq_aux_valid |= (uint8_t)(1U << channel);
  __set_PRIMASK(saved);
}

static void daq_report_bin(daq_bin_us_t *bin, daq_median_t *median, float value, uint64_t sample_us,
                           uint32_t period_us, daq_bin_output_t *output)
{
  daq_bin_out_t out;
  const float filtered = daq_median_add(median, value);
  if (daq_bin_us_add(bin, filtered, sample_us, period_us, &out) && output != NULL)
    *output = (daq_bin_output_t){out.mean, out.centre_us / 1000U, 1U};
}

/* An over-range conversion: never enters the median or the mean; it only
 * dates its bin, which reports +/-INFINITY if nothing in it was in range. */
static void daq_report_over(daq_bin_us_t *bin, int8_t direction, uint64_t sample_us, uint32_t period_us,
                            daq_bin_output_t *output)
{
  daq_bin_out_t out;
  if (daq_bin_us_add_over(bin, direction, sample_us, period_us, &out) && output != NULL)
    *output = (daq_bin_output_t){out.mean, out.centre_us / 1000U, 1U};
}

/* One window's value of `channel` for the fault checks: the over-range
 * marker when most of its conversions were over range, else the mean of the
 * in-range ones (ADC-pin volts, or `voltage_sum` volts); NaN without any. */
static float daq_window_value(const daq_loadcell_window_t *window, const daq_ovr_window_t *ovr,
                              unsigned channel, int use_voltage)
{
  const int bit = daq_ovr_bit(channel);
  const uint16_t over = bit >= 0 ? ovr->count[bit] : 0U;
  if (over != 0U && over >= window->count[channel]) return ovr->dir[bit] < 0 ? -INFINITY : INFINITY;
  if (window->count[channel] == 0U) return NAN;
  return (use_voltage ? window->voltage_sum[channel] : window->sum[channel]) / window->count[channel];
}

#define DAQ_VDDA_LOG_STEP_V 0.005f
#define DAQ_VDDA_LOG_PERIOD_MS 100U
static uint16_t daq_drain_ext_adc(daq_snapshot_t *snapshot,
                                  sd_raw_adc_record_t *records,
                                  uint16_t capacity,
                                  const daq_calibration_t *calibration,
                                  daq_loadcell_window_t *window,
                                  uint8_t kg50_channel, daq_loadcell_bins_t *bins,
                                  const daq_supply_sample_t *supply, daq_ovr_window_t *ovr)
{
  mcp3564r_sample_t sample = {
    .sample_valid = snapshot->ext_adc_sample_valid,
    .channel = snapshot->ext_adc_channel,
    .monotonic_ms = snapshot->ext_adc_sample_us / 1000U,
    .sample_us = snapshot->ext_adc_sample_us,
    .code = snapshot->ext_adc_code,
    .voltage_v = snapshot->ext_adc_voltage_v,
    .raw_value = snapshot->ext_adc_loadcell_kg1000,
    .temperature_c = snapshot->ext_adc_temp_c,
    .temperature_code = snapshot->ext_adc_temp_code,
  };
  static daq_filter_t filters[2];
  static daq_calibration_t filter_calibration;
  if (memcmp(&filter_calibration, calibration, sizeof(*calibration)) != 0)
  {
    memset(filters, 0, sizeof(filters));
    filter_calibration = *calibration;
  }
  uint16_t count = 0U;
  const uint64_t unix_now = daq_log_unix_ms_cached();
  const uint64_t mono_now = board_uptime_ticks();
  static uint32_t raw_sequence;
  /* Report filters: medians then microsecond bins. SD raw records keep the
   * unfiltered value and the same microsecond time, so an SD reader repeating
   * daq_median_add/daq_bin_us_add in record order reproduces every network
   * value bit for bit. */
  static daq_median_t kg1000_median, kg50_median, chamber_median, aux_median[8];
  static daq_bin_us_t kg1000_bin, kg50_bin, chamber_bin;
  static uint8_t medians_ready;
  static uint8_t kg50_bins_channel = 1U;
  if (!medians_ready)
  {
    daq_median_init(&kg1000_median, DAQ_FAST_MEDIAN);
    daq_median_init(&chamber_median, DAQ_FAST_MEDIAN);
    daq_median_init(&kg50_median, DAQ_KG50_MEDIAN);
    for (unsigned i = 0U; i < 8U; ++i) daq_median_init(&aux_median[i], DAQ_AUX_MEDIAN);
    medians_ready = 1U;
  }
  if (kg50_channel != kg50_bins_channel)
  {
    kg50_bin = (daq_bin_us_t){0};
    daq_median_init(&kg50_median, DAQ_KG50_MEDIAN);
    kg50_bins_channel = kg50_channel;
  }
  *window = (daq_loadcell_window_t){0};
  daq_ovr_window_reset(ovr);
  if (bins != NULL) *bins = (daq_loadcell_bins_t){0};

  /* A supply reading takes the last three records of this batch. */
  const uint16_t adc_capacity = supply != NULL && capacity >= 3U ? (uint16_t)(capacity - 3U) : capacity;
  for (;;)
  {
    if (sample.sample_valid != 0U && (sample.channel < 8U || sample.channel == 9U) && count < adc_capacity)
    {
      const uint8_t channel = sample.channel;
      const float raw = sample.raw_value;
      /* Over range (daq_over_range.h): SD keeps the code with its flag; the
       * value never reaches a filter, median, mean or statistic, and a
       * report made only of such conversions is the +/-INFINITY marker. */
      const int8_t over = daq_adc_over_range(sample.code);
      const float marker = over < 0 ? -INFINITY : INFINITY;
      const int ovr_bit = daq_ovr_bit(channel);
      if (ovr_bit >= 0) daq_ovr_window_note(ovr, (unsigned)ovr_bit, over);
      const float corrected = over != 0 ? marker : channel < 2U ? daq_filter_add(&filters[channel],
          daq_calibration_temperature_raw(calibration, channel, raw, sample.temperature_c),
          (uint32_t)sample.monotonic_ms, calibration->filter_tau_ms[channel]) : mcp3564r_connector_voltage(channel, sample.voltage_v);
      const uint64_t captured_uptime = board_uptime_project(mono_now, (uint32_t)sample.monotonic_ms);
      const uint64_t captured_us = captured_uptime * 1000U + sample.sample_us % 1000U;
      daq_bin_output_t *none = NULL;
      if (channel == 0U) {
        if (over != 0) daq_report_over(&kg1000_bin, over, captured_us, DAQ_FAST_REPORT_US, bins ? &bins->kg1000 : none);
        else daq_report_bin(&kg1000_bin, &kg1000_median, raw, captured_us, DAQ_FAST_REPORT_US, bins ? &bins->kg1000 : none);
      }
      else if (channel == kg50_channel) {
        if (over != 0) daq_report_over(&kg50_bin, over, captured_us, DAQ_KG50_REPORT_US, bins ? &bins->kg50 : none);
        else daq_report_bin(&kg50_bin, &kg50_median, raw, captured_us, DAQ_KG50_REPORT_US, bins ? &bins->kg50 : none);
      }
      else if (daq_chamber_is_channel(channel))
      {
        /* The chamber's 20 ms bin also feeds its DAQ_SDADC_VOLTAGES entry. */
        daq_bin_output_t chamber_out = {0};
        if (over != 0) daq_report_over(&chamber_bin, over, captured_us, DAQ_FAST_REPORT_US, &chamber_out);
        else daq_report_bin(&chamber_bin, &chamber_median, corrected, captured_us, DAQ_FAST_REPORT_US, &chamber_out);
        if (chamber_out.ready)
        {
          if (bins != NULL) bins->chamber_v = chamber_out;
          daq_store_aux(channel, chamber_out.value, (uint32_t)sample.monotonic_ms);
        }
      }
      else if (channel < 8U)
        daq_store_aux(channel, over != 0 ? marker : daq_median_add(&aux_median[channel], corrected),
                      (uint32_t)sample.monotonic_ms);
      const int stats = daq_stats_index(channel, kg50_channel);
      if (stats >= 0 && over == 0)
        daq_channel_stats_add(&g_daq_stats[stats], channel == 9U || channel < 2U ? raw : corrected,
                              (uint32_t)sample.monotonic_ms);
      if (records != NULL)
      {
        records[count].channel = channel;
        records[count].monotonic_ms = (uint32_t)sample.monotonic_ms;
        records[count].sample_us = (uint16_t)(sample.sample_us % 1000U);
        records[count].record_sequence = ++raw_sequence;
        records[count].network_unix_ms = daq_sample_network_ms(unix_now, mono_now, captured_uptime);
        records[count].raw_adc_code = sample.code;
        records[count].over_range = over;
        /* Over range: raw_value stays the linear conversion of the code, so
         * nothing is lost; calibrated_value is the +/-inf marker. */
        records[count].raw_value = daq_chamber_is_channel(channel) && over == 0 ? corrected
            : daq_chamber_is_channel(channel) ? mcp3564r_connector_voltage(channel, sample.voltage_v) : raw;
        records[count].adc_temperature_c = sample.temperature_c;
        records[count].adc_temperature_code = sample.temperature_code;
        records[count].calibrated_value = over != 0 ? marker : daq_chamber_is_channel(channel) ? daq_chamber_pressure_psi(corrected) : channel > 1U ? corrected : channel == 1U
            ? daq_calibration_apply_kg50(calibration, corrected)
            : calibration->kg1000_slope * corrected + calibration->kg1000_intercept;
      }
      if (sample.code != 0) g_daq_nonzero_raw_sample_count++;
      if (over == 0)
      {
        window->code_sum[channel] += sample.code;
        window->sum[channel] += raw;
        window->voltage_sum[channel] += sample.voltage_v;
        window->count[channel]++;
      }
      count++;
    }
    if (count >= adc_capacity || mcp3564r_pending_samples() == 0U ||
        mcp3564r_get_sample(&sample) != TX_SUCCESS || sample.sample_valid == 0U) break;
  }
  if (supply != NULL && records != NULL && adc_capacity != capacity)
  {
    const uint64_t captured_uptime = board_uptime_project(mono_now, supply->monotonic_ms);
    /* VDDA joins a supply reading when it moved by 5 mV or more, or 100 ms
     * after the last one: a quiet rail costs no launch-window SD bandwidth. */
    static float vdda_written = NAN;
    static uint32_t vdda_written_ms;
    const uint8_t vdda_due = isnan(vdda_written) != isnan(supply->vdda_v) ||
        fabsf(supply->vdda_v - vdda_written) >= DAQ_VDDA_LOG_STEP_V ||
        (uint32_t)(supply->monotonic_ms - vdda_written_ms) >= DAQ_VDDA_LOG_PERIOD_MS;
    if (vdda_due) { vdda_written = supply->vdda_v; vdda_written_ms = supply->monotonic_ms; }
    for (unsigned i = 0U; i < (vdda_due ? 3U : 2U); ++i)
    {
      records[count] = (sd_raw_adc_record_t){
        .network_unix_ms = daq_sample_network_ms(unix_now, mono_now, captured_uptime),
        .monotonic_ms = supply->monotonic_ms, .record_sequence = ++raw_sequence,
        .raw_adc_code = i < 2U ? supply->code[i] : supply->vrefint_code,
        .raw_value = i < 2U ? supply->pin_v[i] : supply->vdda_v,
        .calibrated_value = i == 0U ? supply->input_voltage_v : i == 1U ? supply->input_current_a : supply->vdda_v,
        .adc_temperature_c = NAN, .adc_temperature_code = 0,
        .channel = i == 0U ? SD_RAW_CHANNEL_INPUT_VOLTAGE : i == 1U ? SD_RAW_CHANNEL_INPUT_CURRENT : SD_RAW_CHANNEL_VDDA};
      ++count;
    }
  }
  /* Legacy snapshot fields describe only the 1000 kg input. */
  snapshot->ext_adc_sample_valid = window->count[0] != 0U;
  if (window->count[0] != 0U)
  {
    snapshot->ext_adc_code = (int32_t)(window->code_sum[0] / window->count[0]);
    snapshot->ext_adc_loadcell_kg1000 = window->sum[0] / window->count[0];
    snapshot->ext_adc_voltage_v = window->voltage_sum[0] / window->count[0];
  }
  g_daq_raw_samples_drained_count += count;
  return count;
}

volatile uint32_t g_daq_sd_snapshot_row_drop_count;

static void daq_store_snapshot_csv(const daq_snapshot_t *snapshot,
                                   const daq_calibration_t *calibration,
                                   const daq_loadcell_window_t *window, sd_capture_t capture)
{
#if (DISABLE_SD_CARD == 0U)
  if (!snapshot->analog_sample_fresh) return;
  /* SD has its own bounded static owners. Never retain this acquisition/network
   * work block or re-publish its network data while storage is backpressured. */
  float values[SD_OPTIONAL_SNAPSHOT_ROWS];
  /* input_voltage_v/input_current_a are raw records (daq_drain_ext_adc). */
  values[0] = snapshot->adc1_aux_v;
  memcpy(&values[1], snapshot->analog_inputs_v, sizeof(snapshot->analog_inputs_v));
  memcpy(&values[9], snapshot->current_sense_v, sizeof(snapshot->current_sense_v));
  memcpy(&values[11], snapshot->current_sense_a, sizeof(snapshot->current_sense_a));
  memcpy(&values[13], snapshot->power_monitor_v, sizeof(snapshot->power_monitor_v));
  memcpy(&values[15], snapshot->analog_outputs_v, sizeof(snapshot->analog_outputs_v));
  values[17] = snapshot->ext_adc_sample_valid ? (float)snapshot->ext_adc_code : NAN;
  values[18] = snapshot->ext_adc_sample_valid ? snapshot->ext_adc_voltage_v : NAN;
  values[19] = snapshot->ext_adc_temp_c;
  values[20] = window->count[1] ? window->voltage_sum[1] / window->count[1] : NAN;
  if (sd_card_enqueue_optional_snapshot_captured(values, snapshot->monotonic_ms, calibration, capture) != SD_CARD_STATUS_OK)
    { sd_card_capture_fault(capture); g_daq_sd_snapshot_row_drop_count += SD_OPTIONAL_SNAPSHOT_ROWS; }
#else
  (void)snapshot; (void)calibration; (void)window; (void)capture;
#endif
}

extern volatile uint32_t g_daq_analog_publish_ok_count;
extern volatile uint32_t g_daq_analog_publish_fail_count;
#ifdef TELEMETRY_ENABLED
static daq_network_schedule_t g_daq_network;
static TX_THREAD g_network_thread;
static ULONG g_network_stack[8192U / sizeof(ULONG)];
volatile uint32_t g_daq_network_offered[DAQ_NETWORK_STREAMS];
volatile uint32_t g_daq_network_coalesced[DAQ_NETWORK_STREAMS];
volatile uint32_t g_daq_network_sent[DAQ_NETWORK_STREAMS];
volatile uint32_t g_daq_network_failed[DAQ_NETWORK_STREAMS];
#ifdef SEDS_FIRMWARE_SIM_TEST
/* These classify scheduler submissions, separately from CAN's transport retries. */
volatile uint32_t g_sim_daq_publication_attempts;
volatile uint32_t g_sim_daq_publication_io_refusals;
volatile uint32_t g_sim_daq_publication_other_refusals;
#endif
volatile uint32_t g_daq_network_invalid;

static void daq_network_offer(unsigned stream, uint32_t type, const float *values,
                               size_t count, uint64_t captured_ms)
{
  const uint32_t saved = __get_PRIMASK();
  __disable_irq();
  const int replaced = daq_network_schedule_offer(&g_daq_network, stream, type, values, count, captured_ms);
  if (replaced < 0) ++g_daq_network_invalid;
  else {
    ++g_daq_network_offered[stream];
    if (replaced) ++g_daq_network_coalesced[stream];
  }
  __set_PRIMASK(saved);
}

static void daq_network_process_once(uint32_t now_ms)
{
  daq_network_sample_t sample;
  unsigned stream;
  const uint32_t saved = __get_PRIMASK();
  __disable_irq();
  const int ready = daq_network_schedule_take(&g_daq_network, now_ms, &stream, &sample);
  __set_PRIMASK(saved);
  if (!ready) return;
  /* One SDK call per aggregate slot. No lock is held across network work and
   * a refused sample does not trigger a retry burst; the next offer is fresh. */
#ifdef SEDS_FIRMWARE_SIM_TEST
  ++g_sim_daq_publication_attempts;
#endif
  const SedsResult result = log_telemetry_captured(sample.type, sample.values,
      sample.count, sizeof(float), sample.captured_ms);
  if (result == SEDS_OK) ++g_daq_network_sent[stream];
  else ++g_daq_network_failed[stream];
#ifdef SEDS_FIRMWARE_SIM_TEST
  if (result == SEDS_IO) ++g_sim_daq_publication_io_refusals;
  else if (result != SEDS_OK) ++g_sim_daq_publication_other_refusals;
#endif
  if (stream == DAQ_NETWORK_KG1000) {
    if (result == SEDS_OK) ++g_daq_loadcell_publish_ok_count;
    else ++g_daq_loadcell_publish_fail_count;
  } else if (stream == DAQ_NETWORK_KG50) {
    if (result == SEDS_OK) ++g_daq_kg50_publish_ok_count;
    else ++g_daq_kg50_publish_fail_count;
  } else if (stream == DAQ_NETWORK_TEMPERATURE) {
    if (result == SEDS_OK) ++g_daq_temperature_publish_ok_count;
    else ++g_daq_temperature_publish_fail_count;
  } else {
    if (result == SEDS_OK) ++g_daq_analog_publish_ok_count;
    else ++g_daq_analog_publish_fail_count;
  }
}

static void daq_network_thread_entry(ULONG argument)
{
  (void)argument;
  for (;;) {
    daq_network_process_once(HAL_GetTick());
    /* Independent of acquisition: the DAC lease and state gate still run if
     * sampling stalls. Rate-limited to 50 ms inside. */
    daq_dac_service(HAL_GetTick());
    tx_thread_sleep((TX_TIMER_TICKS_PER_SECOND + 999U) / 1000U);
  }
}
#endif

volatile uint32_t g_daq_analog_publish_ok_count;
volatile uint32_t g_daq_analog_publish_fail_count;

static void daq_publish_analog_packet(uint32_t type, const float *values, size_t count,
                                     uint64_t captured_ms)
{
#ifdef TELEMETRY_ENABLED
  const unsigned stream = type == SEDS_DT_DAQ_SDADC_VOLTAGES ? DAQ_NETWORK_SDADC :
      type == SEDS_DT_DAQ_SAR_VOLTAGES ? DAQ_NETWORK_SAR :
      type == SEDS_DT_DAQ_CURRENT_SENSE ? DAQ_NETWORK_CURRENT : DAQ_NETWORK_POWER;
  daq_network_offer(stream, type, values, count, captured_ms);
#else
  (void)type; (void)values; (void)count; (void)captured_ms;
#endif
}

static void daq_publish_analog(const daq_snapshot_t *snapshot, const daq_loadcell_window_t *window)
{
  static uint32_t last_report;
  (void)window;
  const uint32_t now = (uint32_t)snapshot->monotonic_ms;
  if ((uint32_t)(now - last_report) >= DAQ_ANALOG_REPORT_PERIOD_MS)
  {
    last_report = now;
    /* CH2..CH7 connector volts: the latest 3-sample median of each channel's
     * auxiliary slots (about 14 Hz); the chamber channel publishes its fast
     * 20 ms bin. NaN once a value is older than DAQ_AUX_STALE_MS. The samples
     * carry HAL-tick milliseconds (daq_acq_clock.h), so their age is taken on
     * the HAL tick too, not on the telemetry (ThreadX) clock of `now`: the two
     * start apart, and a sample "newer" than a lagging now wrapped to stale. */
    float values[6];
    const uint32_t aux_now = HAL_GetTick();
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    for (unsigned i = 0; i < 6U; ++i)
      values[i] = (g_daq_aux_valid & (1U << (i + 2U))) &&
          daq_acq_sample_fresh(aux_now, g_daq_aux_ms[i + 2U], DAQ_AUX_STALE_MS, DAQ_AUX_AHEAD_MS)
          ? g_daq_aux_value[i + 2U] : NAN;
    __set_PRIMASK(saved);
    daq_publish_analog_packet(SEDS_DT_DAQ_SDADC_VOLTAGES, values, 6U, snapshot->monotonic_ms);
  }
  if (snapshot->analog_sample_fresh)
  {
    const float current[] = {snapshot->current_sense_v[0], snapshot->current_sense_v[1],
                            snapshot->current_sense_a[0], snapshot->current_sense_a[1]};
    const float power[] = {snapshot->power_monitor_v[0], snapshot->power_monitor_v[1],
                          snapshot->input_voltage_v, snapshot->input_current_a};
    daq_publish_analog_packet(SEDS_DT_DAQ_SAR_VOLTAGES, snapshot->analog_inputs_v, 8U, snapshot->monotonic_ms);
    daq_publish_analog_packet(SEDS_DT_DAQ_CURRENT_SENSE, current, 4U, snapshot->monotonic_ms);
    daq_publish_analog_packet(SEDS_DT_DAQ_POWER_MONITOR, power, 4U, snapshot->monotonic_ms);
  }
}

/* One acquisition cycle's live values. Acquisition offers them straight to
 * the latest-value network slots. Nothing here is queued or written to SD:
 * the SD card keeps every conversion as a raw record instead. Load cells are
 * uncalibrated ADC-pin volts; GroundStation owns display calibration, and SD
 * raw records separately keep the on-board calibrated values. */
#define DAQ_REPORT_KG1000 1U
#define DAQ_REPORT_KG50 2U
#define DAQ_REPORT_TEMPERATURE 4U
typedef struct {
  uint64_t monotonic_ms, kg1000_ms, kg50_ms;
  float kg1000, kg50, temperature;
  float chamber[2];
  uint8_t chamber_valid;
  uint32_t flags;
  uint8_t kg50_input;
} daq_report_t;

/* Acquisition publishes only into fixed latest-value slots. SD queue or
 * writer delays must never hold the live path behind historical reports. */
static void daq_offer_live_report(const daq_report_t *report)
{
#ifdef TELEMETRY_ENABLED
  if (report->flags & DAQ_REPORT_KG1000) {
    if (report->chamber_valid && !isnan(report->chamber[0]) && !isnan(report->chamber[1])) {
      const float combined[] = {report->kg1000, report->chamber[0], report->chamber[1]};
      daq_network_offer(DAQ_NETWORK_KG1000, SEDS_DT_DAQ_LOADCELL_CHAMBER,
                        combined, 3U, report->kg1000_ms);
    } else daq_network_offer(DAQ_NETWORK_KG1000, SEDS_DT_KG1000,
                            &report->kg1000, 1U, report->kg1000_ms);
  }
  if (report->flags & DAQ_REPORT_KG50) {
    const float tagged[] = {(float)report->kg50_input, report->kg50};
    daq_network_offer(DAQ_NETWORK_KG50,
        report->kg50_input == 0U ? SEDS_DT_KG50 : SEDS_DT_DAQ_KG50_SELECTED,
        report->kg50_input == 0U ? &report->kg50 : tagged,
        report->kg50_input == 0U ? 1U : 2U, report->kg50_ms);
  }
  if (report->flags & DAQ_REPORT_TEMPERATURE)
    daq_network_offer(DAQ_NETWORK_TEMPERATURE, SEDS_DT_DAQ_ADC_TEMPERATURE,
                      &report->temperature, 1U, report->monotonic_ms);
#else
  (void)report;
#endif
}

#if (DAQ_ENABLE_DUMMY_CAN_TELEMETRY != 0U)
static void daq_publish_dummy_can_telemetry(void)
{
  const float kg1000 = 0.0f;

  (void)log_telemetry_asynchronous(SEDS_DT_KG1000,
                                   &kg1000,
                                   1U,
                                   sizeof(kg1000));
}
#endif

/* Live input faults (daq_input_faults.h). Acquisition observes each window;
 * the telemetry thread (board_health_poll) takes reports. Both run with IRQs
 * masked for a few microseconds. */
static daq_input_faults_t g_daq_input_faults;
static uint8_t g_daq_input_faults_ready;
static uint32_t g_daq_input_health_sent_ms;

/* `supply` is the VMAIN reading taken since the last window (every analog
 * sample, and every DAQ_SUPPLY_LAUNCH_PERIOD_MS in the launch window), timed
 * by its own HAL tick; NULL falls back to the snapshot's 100 ms reading. */
static void daq_observe_input_faults(const daq_snapshot_t *snapshot,
                                     const daq_loadcell_window_t *window, uint8_t kg50_input,
                                     const daq_supply_sample_t *supply, daq_ovr_window_t *ovr)
{
  const uint32_t now = (uint32_t)snapshot->monotonic_ms;
  const unsigned kg50 = kg50_input == 1U ? 9U : 1U;
  daq_input_faults_t *faults = &g_daq_input_faults;
#if DAQ_CHAMBER_SCAN_MASK != 0U
  /* Launch T0 for the chamber burn window; read only with a chamber reading
   * (every 2 ms window: the chamber is a fast channel) and outside the IRQ
   * lock (it takes its own). */
  uint32_t since_t0 = DAQ_CHAMBER_NO_LAUNCH;
#if (DISABLE_SD_CARD == 0U)
  const float chamber_v = daq_window_value(window, ovr, DAQ_CHAMBER_ADC_CHANNEL, 1);
  if (!isnan(chamber_v) && !DAQ_FAULT_CHAMBER_ALWAYS_ON)
    since_t0 = sd_card_launch_since_t0_ms();
#endif
#endif
  /* Die temperature: the TEMP conversions are consumed by the driver. */
  static uint32_t temperature_over_seen;
  const uint32_t temperature_over = g_mcp3564r_diag.temperature_over_range;
  const uint32_t temperature_new = temperature_over - temperature_over_seen;
  temperature_over_seen = temperature_over;
  ovr->count[DAQ_OVR_BIT_TEMPERATURE] = temperature_new > UINT16_MAX ? UINT16_MAX : (uint16_t)temperature_new;
  ovr->last[DAQ_OVR_BIT_TEMPERATURE] = g_mcp3564r_diag.temperature_over_range_last;
  const float kg1000_v = daq_window_value(window, ovr, 0U, 0);
  const float kg50_v = daq_window_value(window, ovr, kg50, 0);
  const uint32_t saved = __get_PRIMASK();
  __disable_irq();
  if (!g_daq_input_faults_ready) {
    daq_input_faults_reset(faults);
    g_daq_input_faults_ready = 1U;
  }
  if (supply != NULL) {
    daq_input_faults_supply(faults, supply->input_voltage_v, supply->monotonic_ms);
    daq_input_faults_vdda(faults, supply->vdda_v, supply->monotonic_ms);
  }
  else if (snapshot->analog_sample_fresh) daq_input_faults_supply(faults, snapshot->input_voltage_v, now);
  daq_input_faults_over_range(faults, ovr);
  /* Over range reaches the rail checks as +/-INFINITY: at or past the rail. */
  if (!isnan(kg1000_v)) daq_input_faults_kg1000(faults, kg1000_v, now);
  daq_input_faults_kg50_input(faults, kg50_input);
  if (!isnan(kg50_v)) daq_input_faults_kg50(faults, kg50_v, now);
  /* P6 legs: CH3 is converted 10 ms after CH2 in the auxiliary rotation
   * (daq_acq_aux_items), one pair every 50 ms. */
  static float p6_plus_v;
  static uint32_t p6_plus_ms;
  static uint8_t p6_plus_valid;
  if (window->count[2] != 0U)
  {
    p6_plus_v = window->voltage_sum[2] / window->count[2];
    p6_plus_ms = now;
    p6_plus_valid = 1U;
  }
  if (window->count[3] != 0U && p6_plus_valid && (uint32_t)(now - p6_plus_ms) <= DAQ_P6_LEG_PAIR_MS)
    daq_input_faults_p6_legs(faults, p6_plus_v, window->voltage_sum[3] / window->count[3], now);
#if DAQ_EXCITATION_ADC_CHANNEL >= 0
  if (window->count[DAQ_EXCITATION_ADC_CHANNEL] != 0U)
    daq_input_faults_excitation(faults, mcp3564r_connector_voltage(DAQ_EXCITATION_ADC_CHANNEL,
        window->voltage_sum[DAQ_EXCITATION_ADC_CHANNEL] / window->count[DAQ_EXCITATION_ADC_CHANNEL]));
#endif
#if DAQ_CHAMBER_SCAN_MASK != 0U
  if (isinf(chamber_v))
    daq_input_faults_chamber_over_range(faults, now, since_t0);
  else if (!isnan(chamber_v))
    daq_input_faults_chamber(faults, mcp3564r_connector_voltage(DAQ_CHAMBER_ADC_CHANNEL, chamber_v), now,
        since_t0);
#endif
  __set_PRIMASK(saved);
}

int daq_input_health_take(uint32_t now_ms, uint32_t words[DAQ_INPUT_HEALTH_WORDS])
{
  const uint32_t saved = __get_PRIMASK();
  __disable_irq();
  const uint32_t since = now_ms - g_daq_input_health_sent_ms;
  const int due = g_daq_input_faults_ready &&
      (since >= DAQ_INPUT_HEALTH_PERIOD_MS ||
       (g_daq_input_faults.onset != 0U && since >= DAQ_INPUT_HEALTH_MIN_SPACING_MS));
  if (due) {
    daq_input_faults_report(&g_daq_input_faults, words);
    for (unsigned i = 0U; i < DAQ_STATS_CHANNELS; ++i)
      words[DAQ_INPUT_HEALTH_STATS_WORD + i] = daq_channel_stats_word(&g_daq_stats[i]);
    words[DAQ_INPUT_HEALTH_FRAME_WORD] = daq_input_health_pair(g_mcp3564r_diag.frames_skipped, g_mcp3564r_diag.slots_skipped);
    words[DAQ_INPUT_HEALTH_FRAME_WORD + 1U] = daq_input_health_pair(g_mcp3564r_diag.scans_timed_out,
        g_mcp3564r_diag.recoveries + g_mcp3564r_diag.status_faults);
    words[DAQ_INPUT_HEALTH_FRAME_WORD + 2U] = daq_input_health_pair(g_mcp3564r_diag.fast_done_max_us,
        g_mcp3564r_diag.slot_done_max_us);
    uint32_t vref_source = 0U;
    const float vref = daq_calibration_mcp_vref(&vref_source);
    words[DAQ_INPUT_HEALTH_VREF_WORD] = daq_input_faults_mv(vref) | (vref_source << 16U);
    g_daq_input_health_sent_ms = now_ms;
  }
  __set_PRIMASK(saved);
  return due;
}

/* Offers one KG1000 bin (with the chamber bin of the same centre, if any)
 * and clears it. */
static void daq_offer_kg1000(daq_report_t *report, daq_bin_output_t *kg1000, const daq_bin_output_t *chamber)
{
  daq_report_t one = {.monotonic_ms = report->monotonic_ms, .kg1000 = kg1000->value,
                      .kg1000_ms = kg1000->captured_ms, .flags = DAQ_REPORT_KG1000};
  /* An over-range chamber bin travels as +/-INFINITY in both fields. */
  if (chamber != NULL && !isnan(chamber->value)) {
    one.chamber[1] = chamber->value;
    one.chamber[0] = daq_chamber_pressure_psi(chamber->value);
    /* Valid: a finite pressure, or the marker; a finite voltage whose
     * pressure overflows is not. */
    one.chamber_valid = isinf(chamber->value) || isfinite(one.chamber[0]);
  }
  daq_offer_live_report(&one);
  kg1000->ready = 0U;
}

static void daq_enqueue_analog(const daq_snapshot_t *snapshot,
                               const daq_calibration_t *calibration,
                               const daq_loadcell_window_t *window, sd_capture_t capture);

void daq_thread_entry(ULONG initial_input)
{
  daq_snapshot_t snapshot;
  uint8_t power_loss_latched = 0U;
  uint8_t daq_ready = 1U;
  uint32_t last_temperature_report_ms = 0U;
#if (DISABLE_SD_CARD == 0U)
  uint32_t last_supply_ms = 0U;
#endif
  daq_bin_output_t chamber_bin = {0}, kg1000_pending = {0};
  uint8_t kg1000_wait = 0U;
#if (DISABLE_SD_CARD == 0U)
  sd_raw_adc_record_t raw_records[DAQ_RAW_BATCH_MAX];
#endif

  (void)initial_input;

  if (daq_board_init() != TX_SUCCESS)
  {
    g_daq_init_fail_count++;
    daq_ready = 0U;
  }
  else
  {
    (void)daq_board_ext_adc_start_dma();
  }

  for (;;)
  {
    const ULONG cycle_started = tx_time_get();
    board_watchdog_progress(BOARD_WATCHDOG_ACQUISITION);
#if (DAQ_ENABLE_DUMMY_CAN_TELEMETRY != 0U)
    daq_publish_dummy_can_telemetry();
#endif

    if (daq_ready == 0U)
    {
      tx_thread_sleep(DAQ_SAMPLE_PERIOD_TICKS);
      continue;
    }

    ULONG stage_started = tx_time_get();
    if (daq_board_sample(&snapshot) != TX_SUCCESS)
    {
      g_daq_sample_fail_count++;
      tx_thread_sleep(DAQ_SAMPLE_PERIOD_TICKS);
      continue;
    }

#if (DISABLE_SD_CARD == 0U)
    /* Launch window (with its 0.5 s pre-roll): the MCP3564R LAUNCH profile,
     * fast pair at OSR 256. Applied by the frame engine at the next frame. */
    const uint8_t launch_window = sd_card_launch_window_active();
    mcp3564r_request_profile(launch_window ? DAQ_ACQ_PROFILE_LAUNCH : DAQ_ACQ_PROFILE_FILL);
    /* Launch window: VMAIN/IMON every DAQ_SUPPLY_LAUNCH_PERIOD_MS, beside the
     * load-cell records, so supply dips line up with load-cell faults. */
    if (!snapshot.analog_sample_fresh &&
        (uint32_t)((uint32_t)snapshot.monotonic_ms - last_supply_ms) >= DAQ_SUPPLY_LAUNCH_PERIOD_MS &&
        launch_window)
    {
      (void)daq_board_sample_supply();
      last_supply_ms = (uint32_t)snapshot.monotonic_ms;
    }
    if (snapshot.analog_sample_fresh) last_supply_ms = (uint32_t)snapshot.monotonic_ms;
    daq_supply_sample_t supply;
    const uint8_t supply_fresh = daq_board_take_supply(&supply);
#endif
    stage_started = daq_profile_stage(0U, stage_started);
    g_daq_sample_ok_count++;
    daq_calibration_t calibration;
#if (DISABLE_SD_CARD == 0U)
    const sd_capture_t capture = sd_card_capture_begin(NULL, &calibration);
#else
    const sd_capture_t capture = 0U;
    calibration = daq_calibration_current();
#endif
    daq_loadcell_window_t window;
    daq_ovr_window_t ovr;
    daq_loadcell_bins_t bins;
    daq_report_t report = {.monotonic_ms = snapshot.monotonic_ms,
                           .kg50_input = mcp3564r_kg50_input()};
    const uint8_t kg50_channel = report.kg50_input == 1U ? 9U : 1U;

#if (DISABLE_SD_CARD == 0U)
    const uint16_t raw_count = daq_drain_ext_adc(&snapshot,
                                                 raw_records,
                                                 DAQ_RAW_BATCH_MAX, &calibration, &window,
                                                 kg50_channel, &bins, supply_fresh ? &supply : NULL, &ovr);
#else
    (void)daq_drain_ext_adc(&snapshot, NULL, DAQ_RAW_BATCH_MAX, &calibration, &window,
                            kg50_channel, &bins, NULL, &ovr);
#endif

    stage_started = daq_profile_stage(1U, stage_started);
    const uint32_t report_ms = (uint32_t)snapshot.monotonic_ms;
    if ((uint32_t)(report_ms - last_temperature_report_ms) >= DAQ_TEMPERATURE_REPORT_PERIOD_MS)
    {
      last_temperature_report_ms = report_ms;
      report.temperature = snapshot.ext_adc_temp_c;
      report.flags |= DAQ_REPORT_TEMPERATURE;
    }
    /* The chamber PT shares the fast pair's 20 ms bins and network slot:
     * a KG1000 bin goes out with the chamber bin of the same centre. Both are
     * converted in the same scan, but their bins can close in consecutive
     * drains, so a KG1000 bin waits at most one cycle for its chamber bin.
     * Absence is never a fabricated zero: without a matching chamber bin the
     * scalar KG1000 is sent. SEDSnet receives the mean of each 20 ms bin of
     * 3-sample-median raw conversions; SD keeps those conversions
     * unfiltered, so the SD raw records are the single stored copy. */
    if (bins.chamber_v.ready) chamber_bin = bins.chamber_v;
    if (bins.kg1000.ready)
    {
      if (kg1000_pending.ready) daq_offer_kg1000(&report, &kg1000_pending, NULL);
      kg1000_pending = bins.kg1000;
      kg1000_wait = 0U;
    }
    if (kg1000_pending.ready)
    {
      const uint8_t matched = chamber_bin.ready && chamber_bin.captured_ms == kg1000_pending.captured_ms;
      if (matched || DAQ_CHAMBER_SCAN_MASK == 0U || kg1000_wait >= 1U ||
          (chamber_bin.ready && chamber_bin.captured_ms > kg1000_pending.captured_ms))
        daq_offer_kg1000(&report, &kg1000_pending, matched ? &chamber_bin : NULL);
      else ++kg1000_wait;
    }
#if (DISABLE_SD_CARD == 0U)
    daq_observe_input_faults(&snapshot, &window, report.kg50_input, supply_fresh ? &supply : NULL, &ovr);
#else
    daq_observe_input_faults(&snapshot, &window, report.kg50_input, NULL, &ovr);
#endif
    if (bins.kg50.ready)
    {
      report.kg50 = bins.kg50.value;
      report.kg50_ms = bins.kg50.captured_ms;
      report.flags |= DAQ_REPORT_KG50;
    }
    daq_offer_live_report(&report);
#if (DISABLE_SD_CARD == 0U)
    if ((raw_count != 0U) &&
        sd_card_enqueue_raw_adc_samples_captured(raw_records, raw_count, &calibration, capture) != SD_CARD_STATUS_OK)
      { sd_card_capture_fault(capture); ++g_daq_sd_raw_batch_drop_count; }
#endif
    daq_enqueue_analog(&snapshot, &calibration, &window, capture);
#if (DISABLE_SD_CARD == 0U)
    sd_card_capture_release(capture);
#endif

    stage_started = daq_profile_stage(2U, stage_started);
    (void)daq_board_ext_adc_start_dma();
    (void)daq_profile_stage(3U, stage_started);

    if ((power_loss_latched == 0U) && (snapshot.input_voltage_v <= DAQ_INPUT_VOLTAGE_LOW_V))
    {
      power_loss_latched = 1U;
#if (DISABLE_SD_CARD == 0U)
      (void)sd_card_notify_power_loss();
#endif
    }
    else if (snapshot.input_voltage_v >= DAQ_INPUT_VOLTAGE_RECOVERED_V)
    {
      power_loss_latched = 0U; /* Re-arm after recovery, with hysteresis. */
    }

    /* A power-loss notification asks the SD worker to flush. It must not
     * permanently throttle acquisition/network reporting to 10 Hz after
     * one low-voltage observation, including a startup transient. */

    /* Include acquisition/publish work in the configured period. Sleeping for a
     * whole period after doing that work silently reduces the sample rate. */
    const ULONG elapsed = tx_time_get() - cycle_started;
    g_daq_cycle_elapsed_ticks = elapsed;
    if (elapsed > g_daq_cycle_max_ticks) g_daq_cycle_max_ticks = elapsed;
    if (elapsed < DAQ_SAMPLE_PERIOD_TICKS)
    {
      tx_thread_sleep(DAQ_SAMPLE_PERIOD_TICKS - elapsed);
    }
    else if (elapsed > DAQ_SAMPLE_PERIOD_TICKS)
    {
      if (g_daq_sample_overrun_count == 0U) {
        g_daq_first_overrun_ticks = elapsed;
        g_daq_first_overrun_sample = g_daq_sample_ok_count;
      }
      g_daq_sample_overrun_count++;
      tx_thread_sleep(1U);
    }
    else
    {
      /* Finishing at the deadline is on time. Do not add another tick to
       * every such cycle; still yield to the other ready I/O workers. */
      tx_thread_relinquish();
    }
  }
}

/* Bound slow telemetry work independently of the load-cell drain. */
typedef struct {
  daq_snapshot_t snapshot;
  daq_calibration_t calibration;
  sd_capture_t capture;
  daq_loadcell_window_t window;
} daq_analog_work_t;
_Static_assert(sizeof(daq_analog_work_t) == 384U, "Capture token must not grow the analog pool");
static TX_THREAD g_analog_thread;
static TX_QUEUE g_analog_queue;
static TX_BLOCK_POOL g_analog_pool;
static ULONG g_analog_stack[8192U / sizeof(ULONG)];
static ULONG g_analog_queue_storage[3];
static ULONG g_analog_pool_storage[(3U * (sizeof(daq_analog_work_t) + sizeof(void *)) + sizeof(ULONG) - 1U) / sizeof(ULONG)];
volatile uint32_t g_daq_analog_work_drop_count;

static void daq_enqueue_analog(const daq_snapshot_t *snapshot,
                               const daq_calibration_t *calibration,
                               const daq_loadcell_window_t *window, sd_capture_t capture)
{
  static daq_loadcell_window_t accumulated;
  for (unsigned i = 2U; i < 8U; ++i) {
    accumulated.count[i] += window->count[i];
    accumulated.voltage_sum[i] += window->voltage_sum[i];
  }
  if (!snapshot->analog_sample_fresh) return;
  daq_analog_work_t *work;
  if (tx_block_allocate(&g_analog_pool, (VOID **)&work, TX_NO_WAIT) == TX_SUCCESS) {
    work->snapshot = *snapshot;
    work->calibration = *calibration; work->capture = capture;
#if (DISABLE_SD_CARD == 0U)
    sd_card_capture_retain(capture);
#endif
    work->window = accumulated;
    work->window.count[1] = window->count[1];
    work->window.voltage_sum[1] = window->voltage_sum[1];
    const ULONG message = (ULONG)(uintptr_t)work;
    if (tx_queue_send(&g_analog_queue, (VOID *)&message, TX_NO_WAIT) != TX_SUCCESS) {
#if (DISABLE_SD_CARD == 0U)
      sd_card_capture_fault(work->capture); sd_card_capture_release(work->capture);
#endif
      (void)tx_block_release(work);
      g_daq_analog_work_drop_count++;
    }
  } else {
#if (DISABLE_SD_CARD == 0U)
    sd_card_capture_fault(capture);
#endif
    g_daq_analog_work_drop_count++;
  }
  memset(&accumulated, 0, sizeof(accumulated));
}

static void daq_analog_thread_entry(ULONG argument)
{
  (void)argument;
  for (;;) {
    ULONG message;
    if (tx_queue_receive(&g_analog_queue, &message, TX_WAIT_FOREVER) != TX_SUCCESS) continue;
    daq_analog_work_t *work = (daq_analog_work_t *)(uintptr_t)message;
    daq_publish_analog(&work->snapshot, &work->window);
    /* One nonblocking SD handoff; network publication above is never retried. */
    daq_store_snapshot_csv(&work->snapshot, &work->calibration, &work->window, work->capture);
#if (DISABLE_SD_CARD == 0U)
    sd_card_capture_release(work->capture);
#endif
    (void)tx_block_release(work);
  }
}

UINT create_daq_thread(void)
{
  UINT status = tx_block_pool_create(&g_analog_pool, "Analog snapshots", sizeof(daq_analog_work_t),
                                     g_analog_pool_storage, sizeof(g_analog_pool_storage));
  if (status != TX_SUCCESS) return status;
  status = tx_queue_create(&g_analog_queue, "Analog snapshots", TX_1_ULONG,
                           g_analog_queue_storage, sizeof(g_analog_queue_storage));
  if (status != TX_SUCCESS) return status;
  /* The SD writer can remain ready while draining a slow card. A lower-priority
   * analog worker would never run, even while the DAQ sleeps. Share the same
   * bounded round-robin slice as acquisition and storage instead. */
  status = tx_thread_create(&g_analog_thread, "Analog reporting", daq_analog_thread_entry, 0U,
                            g_analog_stack, sizeof(g_analog_stack), DAQ_IO_THREAD_PRIORITY,
                            DAQ_IO_THREAD_PRIORITY,
                            (TX_TIMER_TICKS_PER_SECOND * DAQ_IO_THREAD_SLICE_MS + 999U) / 1000U,
                            TX_AUTO_START);
  if (status != TX_SUCCESS) return status;
#ifdef TELEMETRY_ENABLED
  status = tx_thread_create(&g_network_thread, "Live sensor network", daq_network_thread_entry, 0U,
                            g_network_stack, sizeof(g_network_stack), DAQ_IO_THREAD_PRIORITY,
                            DAQ_IO_THREAD_PRIORITY,
                            (TX_TIMER_TICKS_PER_SECOND * DAQ_IO_THREAD_SLICE_MS + 999U) / 1000U,
                            TX_AUTO_START);
  if (status != TX_SUCCESS) return status;
#endif
  return tx_thread_create(&daq_thread,
                          "DAQ Thread",
                          daq_thread_entry,
                          0U,
                          g_daq_thread_stack,
                          sizeof(g_daq_thread_stack),
                          DAQ_ACQUISITION_THREAD_PRIORITY,
                          DAQ_ACQUISITION_THREAD_PRIORITY,
                          (TX_TIMER_TICKS_PER_SECOND * DAQ_IO_THREAD_SLICE_MS + 999U) / 1000U,
                          TX_AUTO_START);
}
