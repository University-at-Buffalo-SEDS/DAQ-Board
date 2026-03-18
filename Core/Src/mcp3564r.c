#include "mcp3564r.h"

#include <string.h>

typedef struct
{
  SPI_HandleTypeDef *spi;
  volatile uint8_t dma_busy;
  volatile uint8_t sample_valid;
  uint8_t tx_frame[4];
  uint8_t rx_frame[4];
  int32_t latest_code;
  float latest_temp_c;
} mcp3564r_context_t;

static mcp3564r_context_t g_mcp3564r = {0};

UINT mcp3564r_init(SPI_HandleTypeDef *spi)
{
  if (spi == NULL)
  {
    return TX_PTR_ERROR;
  }

  memset(&g_mcp3564r, 0, sizeof(g_mcp3564r));
  g_mcp3564r.spi = spi;
  return TX_SUCCESS;
}

HAL_StatusTypeDef mcp3564r_start_dma(void)
{
  if (g_mcp3564r.spi == NULL)
  {
    return HAL_ERROR;
  }

  if (g_mcp3564r.dma_busy != 0U)
  {
    return HAL_BUSY;
  }

  g_mcp3564r.tx_frame[0] = 0x41U;
  g_mcp3564r.tx_frame[1] = 0x00U;
  g_mcp3564r.tx_frame[2] = 0x00U;
  g_mcp3564r.tx_frame[3] = 0x00U;
  g_mcp3564r.dma_busy = 1U;

  if ((g_mcp3564r.spi->hdmatx == NULL) || (g_mcp3564r.spi->hdmarx == NULL))
  {
    g_mcp3564r.dma_busy = 0U;
    return HAL_ERROR;
  }

  return HAL_SPI_TransmitReceive_DMA(g_mcp3564r.spi,
                                     g_mcp3564r.tx_frame,
                                     g_mcp3564r.rx_frame,
                                     sizeof(g_mcp3564r.tx_frame));
}

void mcp3564r_dma_complete(void)
{
  const uint32_t raw24 = ((uint32_t)g_mcp3564r.rx_frame[1] << 16)
                       | ((uint32_t)g_mcp3564r.rx_frame[2] << 8)
                       | ((uint32_t)g_mcp3564r.rx_frame[3]);

  g_mcp3564r.latest_code = (int32_t)raw24;
  g_mcp3564r.latest_temp_c = (float)raw24 / 65536.0f;
  g_mcp3564r.sample_valid = 1U;
  g_mcp3564r.dma_busy = 0U;
}

void mcp3564r_dma_error(void)
{
  g_mcp3564r.dma_busy = 0U;
}

UINT mcp3564r_get_sample(mcp3564r_sample_t *sample)
{
  if (sample == NULL)
  {
    return TX_PTR_ERROR;
  }

  sample->sample_valid = g_mcp3564r.sample_valid;
  sample->dma_busy = g_mcp3564r.dma_busy;
  sample->code = g_mcp3564r.latest_code;
  sample->temperature_c = g_mcp3564r.latest_temp_c;

  return TX_SUCCESS;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == g_mcp3564r.spi)
  {
    mcp3564r_dma_complete();
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == g_mcp3564r.spi)
  {
    mcp3564r_dma_error();
  }
}
