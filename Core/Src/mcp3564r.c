#include "mcp3564r.h"

#include "main.h"
#include "daq_rates.h"

#include <string.h>
#include <math.h>

#define MCP3564R_CMD_RESET (0x78U)
#define MCP3564R_CMD_START (0x68U)
#define MCP3564R_CMD_WRITE (0x42U)
#define MCP3564R_CMD_READ (0x41U)
#define MCP3564R_CONFIG0_ADDR (0x01U)
#define MCP3564R_SCAN_ADDR (0x07U)
#define MCP3564R_TIMER_ADDR (0x08U)
#define MCP3564R_OFFSETCAL_ADDR (0x09U)
#define MCP3564R_GAINCAL_ADDR (0x0AU)

#define MCP3564R_DMA_FRAME_SIZE (4U)
#define MCP3564R_DCACHE_LINE_SIZE (32U)
#define MCP3564R_USE_DMA_READ (1U)
#define MCP3564R_WRITE_EXTENDED_CONFIG (1U)
#define MCP3564R_SAMPLE_QUEUE_DEPTH (128U)

#define MCP3564R_FIRST_CONVERSION_US DAQ_ADC_FIRST_CONVERSION_US
#define MCP3564R_DEFAULT_START_OFFSET_US DAQ_ADC_READ_INTERVAL_US

#define EN_12V HAL_GPIO_WritePin(EN_12V_GPIO_Port, EN_12V_Pin, GPIO_PIN_SET)

#if defined(__GNUC__)
#define MCP3564R_DMA_ALIGN __attribute__((aligned(MCP3564R_DCACHE_LINE_SIZE)))
#else
#define MCP3564R_DMA_ALIGN
#endif

typedef struct
{
  uint32_t raw32;
  uint32_t monotonic_ms;
  float temperature_c;
  int32_t temperature_code;
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
  volatile uint8_t first_conversion_pending;
  uint32_t start_offset_us;
  uint32_t latest_raw32;
  float temperature_c;
  int32_t temperature_code;
  uint32_t temperature_ms;
  uint8_t temperature_valid;
  mcp3564r_sample_entry_t queue[MCP3564R_SAMPLE_QUEUE_DEPTH];
} mcp3564r_context_t;

static mcp3564r_context_t g_mcp3564r = {0};
volatile uint32_t g_mcp3564r_init_status = UINT32_MAX;
#if (MCP3564R_USE_DMA_READ != 0U)
static uint8_t g_mcp3564r_tx_frame[MCP3564R_DCACHE_LINE_SIZE] MCP3564R_DMA_ALIGN;
static uint8_t g_mcp3564r_rx_frame[MCP3564R_DCACHE_LINE_SIZE] MCP3564R_DMA_ALIGN;
extern DCACHE_HandleTypeDef hdcache1;
#endif
extern TIM_HandleTypeDef htim2;

const mcp3564r_config_t MCP3564R_DEFAULT_CONFIG = {
  .config0_reg = MCP3564R_BOARD_CONFIG0,
  /* PRE=MCLK/1; conversion filter selected in daq_rates.h. */
  .config1_reg = (DAQ_ADC_OSR_BITS << 2U),
  .config2_reg = MCP3564R_BOARD_CONFIG2,
  .config3_reg = 0b11110000, /* 32-bit output with channel ID and signed 25-bit code. */
  .irq_reg = 0b00000011,
  .mux_reg = 0b00001000, /* Ignored while SCAN is enabled (datasheet 5.15.1). */
  .scan_reg = MCP3564R_BOARD_SCAN,
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

/* DATA_FORMAT=11: CH_ID in bits 31:28, sign in bit 24. */
static int32_t mcp3564r_decode_code(uint32_t raw32)
{
  uint32_t code = raw32 & 0x01FFFFFFU;
  if ((code & 0x01000000U) != 0U) code |= 0xFE000000U;
  return (int32_t)code;
}

static void mcp3564r_store_raw32(uint32_t raw32)
{
  mcp3564r_sample_entry_t entry;

  if ((raw32 >> 28U) == 12U)
  {
    const float temperature = mcp3564r_code_to_temperature(mcp3564r_decode_code(raw32));
    g_mcp3564r.temperature_valid = isfinite(temperature) && temperature >= -40.0f && temperature <= 125.0f;
    g_mcp3564r.temperature_c = temperature;
    g_mcp3564r.temperature_code = mcp3564r_decode_code(raw32);
    g_mcp3564r.temperature_ms = HAL_GetTick();
    return;
  }
  /* Never route a diagnostic/unconfigured channel as a load cell. */
  if ((raw32 >> 28U) > 1U) return;
  entry.raw32 = raw32;
  entry.monotonic_ms = HAL_GetTick();
  entry.temperature_c = g_mcp3564r.temperature_valid &&
      (uint32_t)(entry.monotonic_ms - g_mcp3564r.temperature_ms) <= 2000U
      ? g_mcp3564r.temperature_c : NAN;
  entry.temperature_code = g_mcp3564r.temperature_code;
  g_mcp3564r.latest_raw32 = entry.raw32;

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
  uint8_t tx_buf32[4] = {0};
  uint8_t rx_buf32[4] = {0};
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
    status = HAL_SPI_TransmitReceive(g_mcp3564r.spi, tx_buf32, rx_buf32, 4U, 1000U);
  }

  mcp3564r_deselect();

  if (status == HAL_OK && (dummy_data & 0x04U) == 0U)
  {
    const uint32_t raw32 = ((uint32_t)rx_buf32[0] << 24)
                         | ((uint32_t)rx_buf32[1] << 16)
                         | ((uint32_t)rx_buf32[2] << 8)
                         | ((uint32_t)rx_buf32[3]);
    mcp3564r_store_raw32(raw32);
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

static HAL_StatusTypeDef mcp3564r_arm_start_offset_timer(void);

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

  /* STATUS DR_STATUS is active-low. Polling faster than the scan conversion
   * rate must not enqueue the previous conversion again. */
  if ((dummy_data & 0x04U) != 0U)
  {
    mcp3564r_deselect();
    g_mcp3564r.first_conversion_pending = 0U;
    return mcp3564r_arm_start_offset_timer();
  }

  g_mcp3564r.dma_busy = 1U;
  g_mcp3564r.conversion_active = 0U;
  mcp3564r_dcache_clean(g_mcp3564r_tx_frame, sizeof(g_mcp3564r_tx_frame));
  mcp3564r_dcache_invalidate(g_mcp3564r_rx_frame, sizeof(g_mcp3564r_rx_frame));

  const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(g_mcp3564r.spi,
                                                               g_mcp3564r_tx_frame,
                                                               g_mcp3564r_rx_frame,
                                                               4U);
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
  const uint32_t delay_us = (g_mcp3564r.first_conversion_pending != 0U)
                                ? MCP3564R_FIRST_CONVERSION_US
                                : g_mcp3564r.start_offset_us;

  if (delay_us == 0U)
  {
    return mcp3564r_start_read_dma();
  }

  (void)HAL_TIM_Base_Stop_IT(&htim2);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_SET_AUTORELOAD(&htim2, delay_us - 1U);
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
  g_mcp3564r.first_conversion_pending = 1U;

  EN_12V;
  HAL_Delay(10U);

  if (mcp3564r_reset_device() != HAL_OK)
  {
    g_mcp3564r_init_status = 1U;
    return TX_NOT_DONE;
  }

  if (mcp3564r_write_config(&MCP3564R_DEFAULT_CONFIG) != HAL_OK)
  {
    g_mcp3564r_init_status = 2U;
    return TX_NOT_DONE;
  }

  if (mcp3564r_start_cycle() != HAL_OK)
  {
    g_mcp3564r_init_status = 3U;
    return TX_NOT_DONE;
  }

  g_mcp3564r_init_status = 0U;
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

  const uint32_t raw32 = ((uint32_t)g_mcp3564r_rx_frame[0] << 24)
                       | ((uint32_t)g_mcp3564r_rx_frame[1] << 16)
                       | ((uint32_t)g_mcp3564r_rx_frame[2] << 8)
                       | ((uint32_t)g_mcp3564r_rx_frame[3]);

  mcp3564r_store_raw32(raw32);
  g_mcp3564r.first_conversion_pending = 0U;
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
    if (mcp3564r_start_read_dma() != HAL_OK)
      g_mcp3564r.conversion_active = 0U;
  }
}

UINT mcp3564r_get_sample(mcp3564r_sample_t *sample)
{
  uint32_t primask;
  uint32_t raw32;
  uint8_t sample_valid;
  uint8_t dma_busy;
  uint8_t queued_samples;
  uint32_t overrun_count;
  uint32_t monotonic_ms = 0U;

  if (sample == NULL)
  {
    return TX_PTR_ERROR;
  }

  memset(sample, 0, sizeof(*sample));
  sample->temperature_c = NAN;
  raw32 = 0U;
  sample_valid = 0U;

  primask = __get_PRIMASK();
  __disable_irq();

  sample->temperature_c = g_mcp3564r.temperature_valid &&
      (uint32_t)(HAL_GetTick() - g_mcp3564r.temperature_ms) <= 2000U
      ? g_mcp3564r.temperature_c : NAN;
  sample->temperature_code = g_mcp3564r.temperature_code;
  dma_busy = g_mcp3564r.dma_busy;
  queued_samples = g_mcp3564r.queue_count;
  overrun_count = g_mcp3564r.overrun_count;

  if (g_mcp3564r.queue_count != 0U)
  {
    const mcp3564r_sample_entry_t entry = g_mcp3564r.queue[g_mcp3564r.queue_tail];

    g_mcp3564r.queue_tail = (uint8_t)((g_mcp3564r.queue_tail + 1U) % MCP3564R_SAMPLE_QUEUE_DEPTH);
    g_mcp3564r.queue_count--;

    raw32 = entry.raw32;
    sample->temperature_c = entry.temperature_c;
    sample->temperature_code = entry.temperature_code;
    monotonic_ms = entry.monotonic_ms;
    sample_valid = 1U;
    queued_samples = g_mcp3564r.queue_count;
  }
  else
  {
    raw32 = g_mcp3564r.latest_raw32;
  }

  if (primask == 0U)
  {
    __enable_irq();
  }

  const int32_t code = mcp3564r_decode_code(raw32);

  sample->channel = (uint8_t)(raw32 >> 28U);
  sample->sample_valid = sample_valid;
  sample->dma_busy = dma_busy;
  sample->queued_samples = queued_samples;
  sample->overrun_count = overrun_count;
  sample->monotonic_ms = monotonic_ms;
  sample->code = code;
  sample->voltage_v = mcp3564r_code_to_voltage(code);
  sample->raw_value = mcp3564r_code_to_raw_value(code);


  return TX_SUCCESS;
}

uint8_t mcp3564r_pending_samples(void)
{
  uint32_t primask = __get_PRIMASK();
  uint8_t count;
  __disable_irq();
  count = g_mcp3564r.queue_count;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return count;
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
