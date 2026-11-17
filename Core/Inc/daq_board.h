#ifndef DAQ_BOARD_H
#define DAQ_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "stm32u5xx_hal.h"
#include "tx_api.h"

typedef struct
{
  uint64_t monotonic_ms;
  float input_voltage_v;
  float input_current_a;
  float adc1_aux_v;
  float analog_inputs_v[8]; /* SAR_IN1..8, connector volts. */
  float current_sense_v[2];
  float current_sense_a[2];
  float power_monitor_v[2];
  uint8_t analog_sample_fresh;
  float analog_outputs_v[2];
  uint8_t ext_adc_channel;
  uint8_t ext_adc_sample_valid;
  uint8_t ext_adc_dma_busy;
  uint64_t ext_adc_monotonic_ms;
  int32_t ext_adc_code;
  float ext_adc_voltage_v;
  float ext_adc_loadcell_kg1000; /* Uncalibrated KG1000 input, ADC-pin volts (not kg). */
  float ext_adc_temp_c;
  int32_t ext_adc_temp_code;
} daq_snapshot_t;

/* One ADC1 reading of the board supply: VMAIN through the VMON divider and
 * the LM74202 IMON current, with their ADC1 codes and pin volts. */
typedef struct
{
  uint32_t monotonic_ms; /* HAL_GetTick at the reading */
  float input_voltage_v;
  float input_current_a;
  float pin_v[2];   /* VMON, IMON */
  int32_t code[2];  /* VMON, IMON; -1 when the conversion failed */
} daq_supply_sample_t;

UINT daq_board_init(void);
UINT daq_board_sample(daq_snapshot_t *snapshot);
/* Read VMON/IMON now, outside the 100 ms analog cadence (launch window). */
UINT daq_board_sample_supply(void);
/* The supply reading taken since the last call, if any (each one once). */
uint8_t daq_board_take_supply(daq_supply_sample_t *supply);
HAL_StatusTypeDef daq_board_ext_adc_start_dma(void);

#ifdef __cplusplus
}
#endif

#endif /* DAQ_BOARD_H */
