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
  float analog_inputs_v[4];
  float analog_outputs_v[2];
  uint8_t ext_adc_sample_valid;
  uint8_t ext_adc_dma_busy;
  int32_t ext_adc_code;
  float ext_adc_temp_c;
} daq_snapshot_t;

UINT daq_board_init(void);
UINT daq_board_sample(daq_snapshot_t *snapshot);
HAL_StatusTypeDef daq_board_ext_adc_start_dma(void);

#ifdef __cplusplus
}
#endif

#endif /* DAQ_BOARD_H */
