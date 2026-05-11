#include "mcp3564r.h"

#include "main.h"

#include <string.h>

#define MCP3564R_CMD_RESET (0x78U)
#define MCP3564R_CMD_START (0x68U)
#define MCP3564R_CMD_WRITE (0x42U)
#define MCP3564R_CMD_READ (0x41U)
#define MCP3564R_CONFIG0_ADDR (0x01U)
#define MCP3564R_SCAN_ADDR (0x07U)
#define MCP3564R_TIMER_ADDR (0x08U)
#define MCP3564R_OFFSETCAL_ADDR (0x09U)
#define MCP3564R_GAINCAL_ADDR (0x0AU)

#define MCP3564R_VREF_V (2.2104f)
#define MCP3564R_FULL_SCALE_COUNTS (16777216.0f)
#define MCP3564R_INPUT_DIVIDER_GAIN (2.0f)
#define MCP3564R_PGA_GAIN (16.0f)
#define MCP3564R_DMA_FRAME_SIZE (4U)
#define MCP3564R_DCACHE_LINE_SIZE (32U)
#define MCP3564R_USE_DMA_READ (1U)
#define MCP3564R_WRITE_EXTENDED_CONFIG (1U)
#define MCP3564R_SAMPLE_QUEUE_DEPTH (128U)

#ifndef MCP3564R_DEFAULT_START_OFFSET_US
#define MCP3564R_DEFAULT_START_OFFSET_US (200U)
#endif

#define EN_12V HAL_GPIO_WritePin(EN_12V_GPIO_Port, EN_12V_Pin, GPIO_PIN_SET)

#if defined(__GNUC__)
#define MCP3564R_DMA_ALIGN __attribute__((aligned(MCP3564R_DCACHE_LINE_SIZE)))
#else
#define MCP3564R_DMA_ALIGN
#endif

typedef struct
{
  uint32_t raw24;
} mcp3564r_sample_entry_t;

typedef struct
{
  SPI_HandleTypeDef *spi;
  volatile uint8_t dma_busy;
  volatile uint8_t sample_valid;
  volatile uint8_t conversion_active;
  volatile uint8_t start_sent;
  volatile uint8_t queue_head;
  volatile uint8_t queue_tail;
  volatile uint8_t queue_count;
  volatile uint32_t overrun_count;
  uint32_t start_offset_us;
  uint32_t latest_raw24;
  mcp3564r_sample_entry_t queue[MCP3564R_SAMPLE_QUEUE_DEPTH];
} mcp3564r_context_t;

static mcp3564r_context_t g_mcp3564r = {0};
#if (MCP3564R_USE_DMA_READ != 0U)
static uint8_t g_mcp3564r_tx_frame[MCP3564R_DCACHE_LINE_SIZE] MCP3564R_DMA_ALIGN;
static uint8_t g_mcp3564r_rx_frame[MCP3564R_DCACHE_LINE_SIZE] MCP3564R_DMA_ALIGN;
extern DCACHE_HandleTypeDef hdcache1;
#endif
extern TIM_HandleTypeDef htim2;

const mcp3564r_config_t MCP3564R_DEFAULT_CONFIG = {
  .config0_reg = 0b10000010,
  .config1_reg = 0b00011000,
  .config2_reg = 0b11001111,
  .config3_reg = 0b11000000,
  .irq_reg = 0b00000011,
  .mux_reg = 0b00011000,
  .scan_reg = 0x000000,
  .timer_reg = 0x000000,
  .offsetCal_reg = 0x000000,
  .gainCal_reg = 0x000000,
};

static void mcp3564r_select(void)
{
  HAL_GPIO_WritePin(ADC_NCS_GPIO_Port, ADC_NCS_Pin, GPIO_PIN_RESET);
}

static void mcp3564r_deselect(void)
{
  HAL_GPIO_WritePin(ADC_NCS_GPIO_Port, ADC_NCS_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef mcp3564r_transmit_byte(uint8_t byte)
{
  HAL_StatusTypeDef status;
  uint8_t dummy_data = 0U;

  mcp3564r_select();
  status = HAL_SPI_TransmitReceive(g_mcp3564r.spi, &byte, &dummy_data, 1U, 1000U);
  mcp3564r_deselect();

  return status;
}

static HAL_StatusTypeDef mcp3564r_reset_device(void)
{
  return mcp3564r_transmit_byte(MCP3564R_CMD_RESET);
}

#if (MCP3564R_WRITE_EXTENDED_CONFIG != 0U)
static HAL_StatusTypeDef mcp3564r_write_reg24(uint8_t addr, uint32_t value)
{
  uint8_t dummy_data = 0U;
  uint8_t write_cmd = MCP3564R_CMD_WRITE | (addr << 2);
  uint8_t bytes[3];
  HAL_StatusTypeDef status;

  bytes[0] = (uint8_t)((value >> 16) & 0xFFU);
  bytes[1] = (uint8_t)((value >> 8) & 0xFFU);
  bytes[2] = (uint8_t)(value & 0xFFU);

  mcp3564r_select();

  status = HAL_SPI_TransmitReceive(g_mcp3564r.spi, &write_cmd, &dummy_data, 1U, 1000U);
  if (status == HAL_OK)
  {
    status = HAL_SPI_TransmitReceive(g_mcp3564r.spi, &bytes[0], &dummy_data, 1U, 1000U);
  }
  if (status == HAL_OK)
  {
    status = HAL_SPI_TransmitReceive(g_mcp3564r.spi, &bytes[1], &dummy_data, 1U, 1000U);
  }
  if (status == HAL_OK)
  {
    status = HAL_SPI_TransmitReceive(g_mcp3564r.spi, &bytes[2], &dummy_data, 1U, 1000U);
  }

  mcp3564r_deselect();

  return status;
}
#endif

static HAL_StatusTypeDef mcp3564r_write_config(const mcp3564r_config_t *config)
{
  uint8_t dummy_data = 0U;
  uint8_t initial_write_cmd = MCP3564R_CMD_WRITE | (MCP3564R_CONFIG0_ADDR << 2);
  HAL_StatusTypeDef status;

  status = mcp3564r_reset_device();
  if (status != HAL_OK)
  {
    return status;
  }

  mcp3564r_select();

  status = HAL_SPI_TransmitReceive(g_mcp3564r.spi, &initial_write_cmd, &dummy_data, 1U, 1000U);
  if (status == HAL_OK)
  {
    status = HAL_SPI_TransmitReceive(g_mcp3564r.spi, (uint8_t *)&config->config0_reg, &dummy_data, 1U, 1000U);
  }
  if (status == HAL_OK)
  {
    status = HAL_SPI_TransmitReceive(g_mcp3564r.spi, (uint8_t *)&config->config1_reg, &dummy_data, 1U, 1000U);
  }
  if (status == HAL_OK)
  {
    status = HAL_SPI_TransmitReceive(g_mcp3564r.spi, (uint8_t *)&config->config2_reg, &dummy_data, 1U, 1000U);
  }
  if (status == HAL_OK)
  {
    status = HAL_SPI_TransmitReceive(g_mcp3564r.spi, (uint8_t *)&config->config3_reg, &dummy_data, 1U, 1000U);
  }
  if (status == HAL_OK)
  {
    status = HAL_SPI_TransmitReceive(g_mcp3564r.spi, (uint8_t *)&config->irq_reg, &dummy_data, 1U, 1000U);
  }
  if (status == HAL_OK)
  {
    status = HAL_SPI_TransmitReceive(g_mcp3564r.spi, (uint8_t *)&config->mux_reg, &dummy_data, 1U, 1000U);
  }

  mcp3564r_deselect();

#if (MCP3564R_WRITE_EXTENDED_CONFIG != 0U)
  if (status == HAL_OK)
  {
    status = mcp3564r_write_reg24(MCP3564R_SCAN_ADDR, config->scan_reg);
  }
  if (status == HAL_OK)
  {
    status = mcp3564r_write_reg24(MCP3564R_TIMER_ADDR, config->timer_reg);
  }
  if (status == HAL_OK)
  {
    status = mcp3564r_write_reg24(MCP3564R_OFFSETCAL_ADDR, config->offsetCal_reg);
  }
  if (status == HAL_OK)
  {
    status = mcp3564r_write_reg24(MCP3564R_GAINCAL_ADDR, config->gainCal_reg);
  }
#else
  (void)config->scan_reg;
  (void)config->timer_reg;
  (void)config->offsetCal_reg;
  (void)config->gainCal_reg;
#endif

  return status;
}

static HAL_StatusTypeDef mcp3564r_send_start_once(void)
{
  if (g_mcp3564r.start_sent != 0U)
  {
    return HAL_OK;
  }

  const HAL_StatusTypeDef status = mcp3564r_transmit_byte(MCP3564R_CMD_START);
  if (status == HAL_OK)
  {
    g_mcp3564r.start_sent = 1U;
  }

  return status;
}

static int32_t mcp3564r_sign_extend_24(uint32_t raw24)
{
  if ((raw24 & 0x800000U) != 0U)
  {
    raw24 |= 0xFF000000U;
  }

  return (int32_t)raw24;
}

static float mcp3564r_code_to_loadcell_kg1000(int32_t code)
{
  return (((float)code * MCP3564R_VREF_V) / MCP3564R_FULL_SCALE_COUNTS)
       * MCP3564R_INPUT_DIVIDER_GAIN
       / MCP3564R_PGA_GAIN;
}

static void mcp3564r_store_raw24(uint32_t raw24)
{
  mcp3564r_sample_entry_t entry;

  entry.raw24 = raw24 & 0x00FFFFFFU;
  g_mcp3564r.latest_raw24 = entry.raw24;

  if (g_mcp3564r.queue_count >= MCP3564R_SAMPLE_QUEUE_DEPTH)
  {
    g_mcp3564r.queue_tail = (uint8_t)((g_mcp3564r.queue_tail + 1U) % MCP3564R_SAMPLE_QUEUE_DEPTH);
    g_mcp3564r.queue_count--;
    g_mcp3564r.overrun_count++;
  }

  g_mcp3564r.queue[g_mcp3564r.queue_head] = entry;
  g_mcp3564r.queue_head = (uint8_t)((g_mcp3564r.queue_head + 1U) % MCP3564R_SAMPLE_QUEUE_DEPTH);
  g_mcp3564r.queue_count++;
  g_mcp3564r.sample_valid = 1U;
}

#if (MCP3564R_USE_DMA_READ == 0U)
static HAL_StatusTypeDef mcp3564r_read_data_blocking(void)
{
  uint8_t dummy_data = 0U;
  uint8_t read_cmd = MCP3564R_CMD_READ;
  uint8_t tx_buf24[3] = {0x00U, 0x00U, 0x00U};
  uint8_t rx_buf24[3] = {0x00U, 0x00U, 0x00U};
  HAL_StatusTypeDef status;

  status = mcp3564r_send_start_once();
  if (status != HAL_OK)
  {
    return status;
  }

  HAL_Delay((g_mcp3564r.start_offset_us + 999U) / 1000U);

  mcp3564r_select();

  status = HAL_SPI_TransmitReceive(g_mcp3564r.spi, &read_cmd, &dummy_data, 1U, 1000U);
  if (status == HAL_OK)
  {
    status = HAL_SPI_TransmitReceive(g_mcp3564r.spi, tx_buf24, rx_buf24, 3U, 1000U);
  }

  mcp3564r_deselect();

  if (status == HAL_OK)
  {
    const uint32_t raw24 = ((uint32_t)rx_buf24[0] << 16)
                         | ((uint32_t)rx_buf24[1] << 8)
                         | ((uint32_t)rx_buf24[2]);
    mcp3564r_store_raw24(raw24);
  }

  return status;
}
#endif

#if (MCP3564R_USE_DMA_READ != 0U)
static void mcp3564r_dcache_clean(const void *data, size_t len)
{
  if ((data == NULL) || (len == 0U))
  {
    return;
  }

  (void)HAL_DCACHE_CleanByAddr(&hdcache1, (const uint32_t *)data, (uint32_t)len);
}

static void mcp3564r_dcache_invalidate(const void *data, size_t len)
{
  if ((data == NULL) || (len == 0U))
  {
    return;
  }

  (void)HAL_DCACHE_InvalidateByAddr(&hdcache1, (const uint32_t *)data, (uint32_t)len);
}
#endif

static HAL_StatusTypeDef mcp3564r_start_read_dma(void)
{
#if (MCP3564R_USE_DMA_READ != 0U)
  uint8_t dummy_data = 0U;
  uint8_t read_cmd = MCP3564R_CMD_READ;

  if ((g_mcp3564r.spi == NULL) || (g_mcp3564r.dma_busy != 0U))
  {
    return HAL_BUSY;
  }

  if ((g_mcp3564r.spi->hdmatx == NULL) || (g_mcp3564r.spi->hdmarx == NULL))
  {
    return HAL_ERROR;
  }

  memset(g_mcp3564r_tx_frame, 0, MCP3564R_DMA_FRAME_SIZE);
  memset(g_mcp3564r_rx_frame, 0, MCP3564R_DMA_FRAME_SIZE);

  mcp3564r_select();
  const HAL_StatusTypeDef command_status =
    HAL_SPI_TransmitReceive(g_mcp3564r.spi, &read_cmd, &dummy_data, 1U, 1000U);
  if (command_status != HAL_OK)
  {
    mcp3564r_deselect();
    return command_status;
  }

  g_mcp3564r.dma_busy = 1U;
  g_mcp3564r.conversion_active = 0U;
  mcp3564r_dcache_clean(g_mcp3564r_tx_frame, sizeof(g_mcp3564r_tx_frame));
  mcp3564r_dcache_invalidate(g_mcp3564r_rx_frame, sizeof(g_mcp3564r_rx_frame));

  const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(g_mcp3564r.spi,
                                                               g_mcp3564r_tx_frame,
                                                               g_mcp3564r_rx_frame,
                                                               3U);
  if (status != HAL_OK)
  {
    mcp3564r_deselect();
    g_mcp3564r.dma_busy = 0U;
  }

  return status;
#else
  return HAL_ERROR;
#endif
}

static HAL_StatusTypeDef mcp3564r_arm_start_offset_timer(void)
{
  if (g_mcp3564r.start_offset_us == 0U)
  {
    return mcp3564r_start_read_dma();
  }

  (void)HAL_TIM_Base_Stop_IT(&htim2);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_SET_AUTORELOAD(&htim2, g_mcp3564r.start_offset_us - 1U);
  (void)HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

  return HAL_TIM_Base_Start_IT(&htim2);
}

static HAL_StatusTypeDef mcp3564r_start_cycle(void)
{
  HAL_StatusTypeDef status = mcp3564r_send_start_once();
  if (status != HAL_OK)
  {
    return status;
  }

  g_mcp3564r.conversion_active = 1U;
  status = mcp3564r_arm_start_offset_timer();
  if (status != HAL_OK)
  {
    g_mcp3564r.conversion_active = 0U;
  }

  return status;
}

UINT mcp3564r_init(SPI_HandleTypeDef *spi)
{
  if (spi == NULL)
  {
    return TX_PTR_ERROR;
  }

  memset(&g_mcp3564r, 0, sizeof(g_mcp3564r));
  g_mcp3564r.spi = spi;
  g_mcp3564r.start_offset_us = MCP3564R_DEFAULT_START_OFFSET_US;

  EN_12V;
  HAL_Delay(10U);

  if (mcp3564r_reset_device() != HAL_OK)
  {
    return TX_NOT_DONE;
  }

  if (mcp3564r_write_config(&MCP3564R_DEFAULT_CONFIG) != HAL_OK)
  {
    return TX_NOT_DONE;
  }

  if (mcp3564r_start_cycle() != HAL_OK)
  {
    return TX_NOT_DONE;
  }

  return TX_SUCCESS;
}

void mcp3564r_set_start_offset_us(uint32_t offset_us)
{
  g_mcp3564r.start_offset_us = offset_us;
}

HAL_StatusTypeDef mcp3564r_start_dma(void)
{
#if (MCP3564R_USE_DMA_READ == 0U)
  if (g_mcp3564r.spi == NULL)
  {
    return HAL_ERROR;
  }

  return mcp3564r_read_data_blocking();
#else
  if (g_mcp3564r.spi == NULL)
  {
    return HAL_ERROR;
  }

  if ((g_mcp3564r.dma_busy != 0U) || (g_mcp3564r.conversion_active != 0U))
  {
    return HAL_BUSY;
  }

  return mcp3564r_start_cycle();
#endif
}

void mcp3564r_dma_complete(void)
{
#if (MCP3564R_USE_DMA_READ != 0U)
  mcp3564r_dcache_invalidate(g_mcp3564r_rx_frame, sizeof(g_mcp3564r_rx_frame));

  const uint32_t raw24 = ((uint32_t)g_mcp3564r_rx_frame[0] << 16)
                       | ((uint32_t)g_mcp3564r_rx_frame[1] << 8)
                       | ((uint32_t)g_mcp3564r_rx_frame[2]);

  mcp3564r_store_raw24(raw24);
  g_mcp3564r.dma_busy = 0U;
  mcp3564r_deselect();
  (void)mcp3564r_start_cycle();
#endif
}

void mcp3564r_dma_error(void)
{
  g_mcp3564r.dma_busy = 0U;
  mcp3564r_deselect();
  (void)mcp3564r_start_cycle();
}

void mcp3564r_timer_elapsed_callback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    (void)HAL_TIM_Base_Stop_IT(&htim2);
    (void)mcp3564r_start_read_dma();
  }
}

UINT mcp3564r_get_sample(mcp3564r_sample_t *sample)
{
  uint32_t primask;
  uint32_t raw24;
  uint8_t sample_valid;
  uint8_t dma_busy;
  uint8_t queued_samples;
  uint32_t overrun_count;

  if (sample == NULL)
  {
    return TX_PTR_ERROR;
  }

  memset(sample, 0, sizeof(*sample));
  raw24 = 0U;
  sample_valid = 0U;

  primask = __get_PRIMASK();
  __disable_irq();

  dma_busy = g_mcp3564r.dma_busy;
  queued_samples = g_mcp3564r.queue_count;
  overrun_count = g_mcp3564r.overrun_count;

  if (g_mcp3564r.queue_count != 0U)
  {
    const mcp3564r_sample_entry_t entry = g_mcp3564r.queue[g_mcp3564r.queue_tail];

    g_mcp3564r.queue_tail = (uint8_t)((g_mcp3564r.queue_tail + 1U) % MCP3564R_SAMPLE_QUEUE_DEPTH);
    g_mcp3564r.queue_count--;

    raw24 = entry.raw24;
    sample_valid = 1U;
    queued_samples = g_mcp3564r.queue_count;
  }
  else
  {
    raw24 = g_mcp3564r.latest_raw24;
  }

  if (primask == 0U)
  {
    __enable_irq();
  }

  const int32_t code = mcp3564r_sign_extend_24(raw24);

  sample->sample_valid = sample_valid;
  sample->dma_busy = dma_busy;
  sample->queued_samples = queued_samples;
  sample->overrun_count = overrun_count;
  sample->code = code;
  sample->voltage_v = mcp3564r_code_to_loadcell_kg1000(code);
  sample->loadcell_kg1000 = sample->voltage_v;
  sample->temperature_c = 0.0f;

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
