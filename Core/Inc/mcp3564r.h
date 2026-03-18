#ifndef MCP3564R_H
#define MCP3564R_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx_hal.h"
#include "tx_api.h"

typedef struct
{
  uint8_t sample_valid;
  uint8_t dma_busy;
  int32_t code;
  float temperature_c;
} mcp3564r_sample_t;

UINT mcp3564r_init(SPI_HandleTypeDef *spi);
HAL_StatusTypeDef mcp3564r_start_dma(void);
void mcp3564r_dma_complete(void);
void mcp3564r_dma_error(void);
UINT mcp3564r_get_sample(mcp3564r_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* MCP3564R_H */
