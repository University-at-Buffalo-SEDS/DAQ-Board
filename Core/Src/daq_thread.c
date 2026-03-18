#include "DAQ-Threads.h"

#include "daq_board.h"
#include "main.h"
#include "sd_card.h"
#include "telemetry.h"

#include <stdio.h>
#include <string.h>

TX_THREAD daq_thread;

#define DAQ_THREAD_STACK_SIZE (8U * 1024U)
#define DAQ_SAMPLE_PERIOD_TICKS 10U
#define DAQ_TELEMETRY_DOWNSAMPLE 10U
#define DAQ_INPUT_VOLTAGE_LOW_V 8.5f

static ULONG g_daq_thread_stack[DAQ_THREAD_STACK_SIZE / sizeof(ULONG)];

typedef struct
{
  uint32_t sample_count;
  float input_voltage_sum;
  float input_current_sum;
  float adc1_aux_sum;
  float analog_input_sum[4];
  float analog_output_sum[2];
  float ext_adc_code_sum;
  float ext_adc_temp_sum;
} daq_average_accumulator_t;

static void daq_average_reset(daq_average_accumulator_t *acc)
{
  memset(acc, 0, sizeof(*acc));
}

static void daq_average_push(daq_average_accumulator_t *acc,
                             const daq_snapshot_t *snapshot)
{
  acc->sample_count++;
  acc->input_voltage_sum += snapshot->input_voltage_v;
  acc->input_current_sum += snapshot->input_current_a;
  acc->adc1_aux_sum += snapshot->adc1_aux_v;
  acc->analog_input_sum[0] += snapshot->analog_inputs_v[0];
  acc->analog_input_sum[1] += snapshot->analog_inputs_v[1];
  acc->analog_input_sum[2] += snapshot->analog_inputs_v[2];
  acc->analog_input_sum[3] += snapshot->analog_inputs_v[3];
  acc->analog_output_sum[0] += snapshot->analog_outputs_v[0];
  acc->analog_output_sum[1] += snapshot->analog_outputs_v[1];
  acc->ext_adc_code_sum += (float)snapshot->ext_adc_code;
  acc->ext_adc_temp_sum += snapshot->ext_adc_temp_c;
}

static void daq_store_snapshot_csv(const daq_snapshot_t *snapshot)
{
  (void)sd_card_enqueue_csv_row("input_voltage_v", snapshot->monotonic_ms, snapshot->input_voltage_v);
  (void)sd_card_enqueue_csv_row("input_current_a", snapshot->monotonic_ms, snapshot->input_current_a);
  (void)sd_card_enqueue_csv_row("adc1_aux_v", snapshot->monotonic_ms, snapshot->adc1_aux_v);
  (void)sd_card_enqueue_csv_row("analog_in_1_v", snapshot->monotonic_ms, snapshot->analog_inputs_v[0]);
  (void)sd_card_enqueue_csv_row("analog_in_2_v", snapshot->monotonic_ms, snapshot->analog_inputs_v[1]);
  (void)sd_card_enqueue_csv_row("analog_in_3_v", snapshot->monotonic_ms, snapshot->analog_inputs_v[2]);
  (void)sd_card_enqueue_csv_row("analog_in_4_v", snapshot->monotonic_ms, snapshot->analog_inputs_v[3]);
  (void)sd_card_enqueue_csv_row("analog_out_1_v", snapshot->monotonic_ms, snapshot->analog_outputs_v[0]);
  (void)sd_card_enqueue_csv_row("analog_out_2_v", snapshot->monotonic_ms, snapshot->analog_outputs_v[1]);
  if (snapshot->ext_adc_sample_valid != 0U)
  {
    (void)sd_card_enqueue_csv_row("mcp3564r_code", snapshot->monotonic_ms, (float)snapshot->ext_adc_code);
    (void)sd_card_enqueue_csv_row("mcp3564r_temp_c", snapshot->monotonic_ms, snapshot->ext_adc_temp_c);
  }
}

static void daq_publish_averages(const daq_average_accumulator_t *acc,
                                 const daq_snapshot_t *snapshot)
{
  char line[224];
  float scale;
  float vin_avg;
  float iin_avg;
  int len;

  if (acc->sample_count == 0U)
  {
    return;
  }

  scale = 1.0f / (float)acc->sample_count;
  vin_avg = acc->input_voltage_sum * scale;
  iin_avg = acc->input_current_sum * scale;

  (void)log_telemetry_asynchronous(SEDS_DT_BATTERY_VOLTAGE,
                                   &vin_avg,
                                   1U,
                                   sizeof(vin_avg));
  (void)log_telemetry_asynchronous(SEDS_DT_BATTERY_CURRENT,
                                   &iin_avg,
                                   1U,
                                   sizeof(iin_avg));

  len = snprintf(line,
                 sizeof(line),
                 "daq_avg vin=%.3fV iin=%.3fA aux=%.3fV adc4=[%.3f %.3f %.3f %.3f] "
                 "dac=[%.3f %.3f] ext_code=%.1f ext_temp=%.2fC n=%lu",
                 (double)vin_avg,
                 (double)iin_avg,
                 (double)(acc->adc1_aux_sum * scale),
                 (double)(acc->analog_input_sum[0] * scale),
                 (double)(acc->analog_input_sum[1] * scale),
                 (double)(acc->analog_input_sum[2] * scale),
                 (double)(acc->analog_input_sum[3] * scale),
                 (double)(acc->analog_output_sum[0] * scale),
                 (double)(acc->analog_output_sum[1] * scale),
                 (double)(acc->ext_adc_code_sum * scale),
                 (double)(acc->ext_adc_temp_sum * scale),
                 (unsigned long)acc->sample_count);

  if (len > 0)
  {
    (void)log_telemetry_string_asynchronous(SEDS_DT_MESSAGE_DATA, line);
  }

  if (snapshot->input_voltage_v <= DAQ_INPUT_VOLTAGE_LOW_V)
  {
    (void)log_telemetry_string_asynchronous(SEDS_DT_WARNING,
                                            "input voltage low; flushing sd journal");
  }
}

void daq_thread_entry(ULONG initial_input)
{
  daq_snapshot_t snapshot;
  daq_average_accumulator_t averages;
  uint8_t power_loss_latched = 0U;

  (void)initial_input;

  if (daq_board_init() != TX_SUCCESS)
  {
    (void)log_telemetry_string_asynchronous(SEDS_DT_GENERIC_ERROR, "daq init failed");
    return;
  }

  daq_average_reset(&averages);

  for (;;)
  {
    if (daq_board_sample(&snapshot) != TX_SUCCESS)
    {
      (void)log_telemetry_string_asynchronous(SEDS_DT_WARNING, "daq sample failed");
      tx_thread_sleep(DAQ_SAMPLE_PERIOD_TICKS);
      continue;
    }

    daq_store_snapshot_csv(&snapshot);
    daq_average_push(&averages, &snapshot);

    if (averages.sample_count >= DAQ_TELEMETRY_DOWNSAMPLE)
    {
      daq_publish_averages(&averages, &snapshot);
      daq_average_reset(&averages);
      (void)sd_card_request_flush();
    }

    if ((power_loss_latched == 0U) && (snapshot.input_voltage_v <= DAQ_INPUT_VOLTAGE_LOW_V))
    {
      power_loss_latched = 1U;
      (void)log_telemetry_string_asynchronous(SEDS_DT_WARNING, "input voltage low; flushing sd journal");
      (void)sd_card_notify_power_loss();
    }

    if (power_loss_latched != 0U)
    {
      tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10U);
      continue;
    }

    (void)daq_board_ext_adc_start_dma();
    tx_thread_sleep(DAQ_SAMPLE_PERIOD_TICKS);
  }
}

UINT create_daq_thread(void)
{
  return tx_thread_create(&daq_thread,
                          "DAQ Thread",
                          daq_thread_entry,
                          0U,
                          g_daq_thread_stack,
                          sizeof(g_daq_thread_stack),
                          6U,
                          6U,
                          TX_NO_TIME_SLICE,
                          TX_AUTO_START);
}
