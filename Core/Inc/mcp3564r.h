#ifndef MCP3564R_H
#define MCP3564R_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx_hal.h"
#include "tx_api.h"

typedef struct
{
  uint8_t config0_reg;
  uint8_t config1_reg;
  uint8_t config2_reg;
  uint8_t config3_reg;
  uint8_t irq_reg;
  uint8_t mux_reg;
  uint32_t scan_reg;
  uint32_t timer_reg;
  uint32_t offsetCal_reg;
  uint32_t gainCal_reg;
} mcp3564r_config_t;

typedef struct
{
  uint8_t sample_valid;
  uint8_t dma_busy;
  uint8_t queued_samples;
  int32_t code;
  float voltage_v;
  float loadcell_kg1000;
  float temperature_c;
  uint32_t overrun_count;
} mcp3564r_sample_t;

extern const mcp3564r_config_t MCP3564R_DEFAULT_CONFIG;

UINT mcp3564r_init(SPI_HandleTypeDef *spi);
HAL_StatusTypeDef mcp3564r_start_dma(void);
void mcp3564r_dma_complete(void);
void mcp3564r_dma_error(void);
UINT mcp3564r_get_sample(mcp3564r_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* MCP3564R_H */
