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

#define MCP3564R_DMA_FRAME_SIZE (5U)
#define MCP3564R_DMA_ALIGNMENT (32U)
#define MCP3564R_USE_DMA_READ (1U)
#define MCP3564R_WRITE_EXTENDED_CONFIG (1U)
#define MCP3564R_SAMPLE_QUEUE_DEPTH (128U)
/* Fault deadline, not sample cadence: SPI2=SYSCLK160MHz/16 ->5bytes/4us. */
#define MCP3564R_DMA_TIMEOUT_MS (20U)

#define MCP3564R_TEMP_TIMEOUT_MS 50U

#define EN_12V HAL_GPIO_WritePin(EN_12V_GPIO_Port, EN_12V_Pin, GPIO_PIN_SET)

#if defined(__GNUC__) && (defined(__arm__) || defined(__thumb__))
#define MCP3564R_DMA_ALIGN __attribute__((section(".bss.adc_dma"), aligned(MCP3564R_DMA_ALIGNMENT)))
#elif defined(__GNUC__)
/* Native HAL fixtures use host memory; actual ARM placement is linker-tested. */
#define MCP3564R_DMA_ALIGN __attribute__((aligned(MCP3564R_DMA_ALIGNMENT)))
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
  volatile uint8_t drdy_armed;
  volatile uint8_t recovery_needed;
  volatile uint32_t dma_started_ms;
  volatile uint8_t queue_head;
  volatile uint8_t queue_tail;
  volatile uint8_t queue_count;
  volatile uint32_t overrun_count;
  uint32_t latest_raw32;
  float temperature_c;
  int32_t temperature_code;
  uint32_t temperature_ms;
  uint8_t temperature_valid;
  volatile uint8_t temperature_scan_active;
  volatile uint8_t temperature_scan_complete;
  volatile uint8_t scan_config_valid;
  uint32_t temperature_attempt_ms;
  volatile uint8_t aux_scan_active;
  volatile uint8_t p6_scan_active;
  volatile uint16_t fast_scan_seen;
  volatile uint8_t aux_seen;
  uint32_t aux_attempt_ms;
  mcp3564r_sample_entry_t queue[MCP3564R_SAMPLE_QUEUE_DEPTH];
} mcp3564r_context_t;

static mcp3564r_context_t g_mcp3564r = {0};
static volatile uint8_t g_kg50_input_requested;
static volatile uint8_t g_kg50_input_active;
void mcp3564r_request_kg50_input(uint8_t input) { if (input <= 1U) g_kg50_input_requested = input; }
uint8_t mcp3564r_kg50_input(void) { return g_kg50_input_active; }
static uint32_t mcp3564r_fast_scan(uint8_t input) {
  if (input == 0U) return MCP3564R_BOARD_SCAN;
  return g_mcp3564r.p6_scan_active ? 0x000200U : (0x000001U | DAQ_CHAMBER_SCAN_MASK);
}

volatile uint32_t g_mcp3564r_init_status = UINT32_MAX;
volatile uint32_t g_mcp3564r_dma_timeout_count;
volatile uint32_t g_mcp3564r_12v_enable_readback = UINT32_MAX;
#if (MCP3564R_USE_DMA_READ != 0U)
static uint8_t g_mcp3564r_tx_frame[MCP3564R_DMA_ALIGNMENT] MCP3564R_DMA_ALIGN;
static uint8_t g_mcp3564r_rx_frame[MCP3564R_DMA_ALIGNMENT] MCP3564R_DMA_ALIGN;
#endif


const mcp3564r_config_t MCP3564R_DEFAULT_CONFIG = {
  .config0_reg = MCP3564R_BOARD_CONFIG0,
  /* PRE=MCLK/1; conversion filter selected in daq_rates.h. */
  .config1_reg = (DAQ_ADC_OSR_BITS << 2U),
  .config2_reg = MCP3564R_BOARD_CONFIG2,
  .config3_reg = 0b11110000, /* 32-bit output with channel ID and signed 25-bit code. */
  .irq_reg = 0x06U, /* IRQ driven high inactive, fast commands on, EN_STP off. */
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

static HAL_StatusTypeDef mcp3564r_write_reg8(uint8_t addr, uint8_t value)
{
  uint8_t command = MCP3564R_CMD_WRITE | (addr << 2U);
  uint8_t ignored;
  mcp3564r_select();
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(g_mcp3564r.spi, &command, &ignored, 1U, 1000U);
  if (status == HAL_OK)
    status = HAL_SPI_TransmitReceive(g_mcp3564r.spi, &value, &ignored, 1U, 1000U);
  mcp3564r_deselect();
  return status;
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
    /* A TEMP code measured with the load-cell clock is not a valid die reading. */
    if (g_mcp3564r.temperature_scan_active == 0U) return;
    const float temperature = mcp3564r_code_to_temperature(mcp3564r_decode_code(raw32));
    g_mcp3564r.temperature_valid = isfinite(temperature) && temperature >= -40.0f && temperature <= 125.0f;
    g_mcp3564r.temperature_c = temperature;
    g_mcp3564r.temperature_code = mcp3564r_decode_code(raw32);
    g_mcp3564r.temperature_ms = HAL_GetTick();
    g_mcp3564r.temperature_scan_complete = 1U;
    return;
  }
  /* Never route a diagnostic/unconfigured channel as a load cell. */
  const unsigned channel = raw32 >> 28U;
  if (g_mcp3564r.temperature_scan_active != 0U || (channel > 7U && channel != 9U)) return;
  if (g_mcp3564r.aux_scan_active != 0U)
  {
    if ((MCP3564R_AUX_SCAN & (1U << channel)) == 0U) return;
    g_mcp3564r.aux_seen |= (uint8_t)(1U << channel);
  }
  else if ((mcp3564r_fast_scan(g_kg50_input_active) & (1U << channel)) == 0U) return;
  if (g_mcp3564r.aux_scan_active == 0U) g_mcp3564r.fast_scan_seen |= (uint16_t)(1U << channel);
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

/* STM32U585 DCACHE1 bypasses internal SRAM (ST DCACHE training, page2).
 * The linker pins both DMA buffers inside startup-zeroed internal SRAM.
 * Never issue polling DCACHE maintenance from the DRDY/DMA interrupts. */
/* Thread configuration owns SPI while disarmed; ISR ownership is a single
 * atomic DMA claim. No blocking SPI transaction or configuration write is
 * issued from an ISR; the DMA owns its opcode and data clocks together. */
static void mcp3564r_disarm(void)
{
  g_mcp3564r.drdy_armed = 0U;
  g_mcp3564r.conversion_active = 0U;
  __HAL_GPIO_EXTI_CLEAR_FALLING_IT(ADC_IRQ_Pin);
}

static void mcp3564r_require_recovery(void)
{
  mcp3564r_disarm();
  g_mcp3564r.recovery_needed = 1U;
  /* Keep dma_busy owned until acquisition-thread HAL_SPI_Abort quiesces DMA. */
}

static HAL_StatusTypeDef mcp3564r_start_read_dma(void)
{
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  if (g_mcp3564r.spi == NULL || !g_mcp3564r.drdy_armed ||
      !g_mcp3564r.scan_config_valid || !g_mcp3564r.conversion_active ||
      g_mcp3564r.recovery_needed || g_mcp3564r.dma_busy)
  {
    if (mask == 0U) __enable_irq();
    return HAL_BUSY;
  }
  if (g_mcp3564r.spi->hdmatx == NULL || g_mcp3564r.spi->hdmarx == NULL)
  {
    mcp3564r_require_recovery();
    if (mask == 0U) __enable_irq();
    return HAL_ERROR;
  }
  g_mcp3564r.dma_started_ms = HAL_GetTick();
  g_mcp3564r.dma_busy = 1U;
  if (mask == 0U) __enable_irq();

  memset(g_mcp3564r_tx_frame, 0, sizeof(g_mcp3564r_tx_frame));
  memset(g_mcp3564r_rx_frame, 0, sizeof(g_mcp3564r_rx_frame));
  g_mcp3564r_tx_frame[0] = MCP3564R_CMD_READ;
  __DMB(); /* Publish uncached SRAM command/data before DMA ownership. */
  mcp3564r_select();
  const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(g_mcp3564r.spi,
      g_mcp3564r_tx_frame, g_mcp3564r_rx_frame, MCP3564R_DMA_FRAME_SIZE);
  if (status != HAL_OK) mcp3564r_require_recovery();
  return status;
}

/* Standby/configuration may leave an old ADCDATA/IRQ indication. Consume it
 * only in the acquisition thread before START, never mistake it for the first
 * settled conversion of the new scan. */
static HAL_StatusTypeDef mcp3564r_clear_stale_ready(void)
{
  uint8_t tx[MCP3564R_DMA_FRAME_SIZE] = {MCP3564R_CMD_READ};
  uint8_t rx[MCP3564R_DMA_FRAME_SIZE];
  mcp3564r_select();
  const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(g_mcp3564r.spi,
      tx, rx, MCP3564R_DMA_FRAME_SIZE, 1000U);
  mcp3564r_deselect();
  return status;
}

static HAL_StatusTypeDef mcp3564r_start_cycle(void)
{
  if (!g_mcp3564r.scan_config_valid || g_mcp3564r.recovery_needed) return HAL_ERROR;
  HAL_StatusTypeDef status = mcp3564r_clear_stale_ready();
  if (status == HAL_OK) status = mcp3564r_send_start_once();
  if (status != HAL_OK) { mcp3564r_disarm(); return status; }
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  __HAL_GPIO_EXTI_CLEAR_FALLING_IT(ADC_IRQ_Pin);
  g_mcp3564r.conversion_active = 1U;
  g_mcp3564r.drdy_armed = 1U;
  if (mask == 0U) __enable_irq();
  /* Recover a genuine edge occurring between START and arm. This is one
   * bounded attempt, not an ISR retry/poll loop. STATUS validates freshness. */
  if (HAL_GPIO_ReadPin(ADC_IRQ_GPIO_Port, ADC_IRQ_Pin) == GPIO_PIN_RESET)
    return mcp3564r_start_read_dma();
  return HAL_OK;
}

/* Acquisition thread only: disarm DRDY before changing the sequencer and
 * reject switching mid-transfer. Never hold PRIMASK across SPI writes. */
static HAL_StatusTypeDef mcp3564r_switch_scan(uint8_t mode)
{
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  if (g_mcp3564r.dma_busy != 0U)
  {
    if (mask == 0U) __enable_irq();
    return HAL_BUSY;
  }
  mcp3564r_disarm();
  g_mcp3564r.scan_config_valid = 0U;
  if (mask == 0U) __enable_irq();

  g_mcp3564r.scan_config_valid = 0U;
  const uint8_t temperature = mode == 1U;
  const uint8_t requested_input = g_kg50_input_requested;
  g_mcp3564r.temperature_scan_active = temperature;
  g_mcp3564r.aux_scan_active = mode == 2U;
  g_mcp3564r.p6_scan_active = mode == 3U && requested_input == 1U;
  g_mcp3564r.fast_scan_seen = 0U;
  g_mcp3564r.aux_seen = 0U;
  g_mcp3564r.temperature_scan_complete = 0U;
  g_mcp3564r.start_sent = 0U;
  /* Standby prevents a conversion spanning the configuration change. */
  HAL_StatusTypeDef status = mcp3564r_write_reg8(1U, MCP3564R_BOARD_CONFIG0);
  if (status == HAL_OK)
    status = mcp3564r_write_reg8(2U, temperature ? MCP3564R_TEMPERATURE_CONFIG1
                                               : MCP3564R_DEFAULT_CONFIG.config1_reg);
  if (status == HAL_OK)
    status = mcp3564r_write_reg8(3U, g_mcp3564r.p6_scan_active ? MCP3564R_P6_CONFIG2 : MCP3564R_BOARD_CONFIG2);
  if (status == HAL_OK)
    status = mcp3564r_write_reg24(MCP3564R_SCAN_ADDR,
                                temperature ? MCP3564R_TEMPERATURE_SCAN :
                                mode == 2U ? MCP3564R_AUX_SCAN : mcp3564r_fast_scan(requested_input));
  if (status != HAL_OK) return status;
  if (mode == 0U || mode == 3U) g_kg50_input_active = requested_input;
  g_mcp3564r.scan_config_valid = 1U;
  return mcp3564r_start_cycle();
}

UINT mcp3564r_init(SPI_HandleTypeDef *spi)
{
  if (spi == NULL)
  {
    return TX_PTR_ERROR;
  }

  memset(&g_mcp3564r, 0, sizeof(g_mcp3564r));
  g_kg50_input_active = 0U;
  g_mcp3564r.spi = spi;

  EN_12V;
  HAL_Delay(10U);
  /* PC4/+12VA_EN drives U17 SHDN directly (schematic sheets2/3).
   * High enables the analog rail; only verify the GPIO level here. A voltage
   * measurement at the connector remains a separate hardware observation. */
  g_mcp3564r_12v_enable_readback =
    HAL_GPIO_ReadPin(EN_12V_GPIO_Port, EN_12V_Pin) == GPIO_PIN_SET ? 1U : 0U;
  if (g_mcp3564r_12v_enable_readback == 0U)
  {
    g_mcp3564r_init_status = 4U;
    return TX_NOT_DONE;
  }

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

  g_mcp3564r.scan_config_valid = 1U;
  if (mcp3564r_start_cycle() != HAL_OK)
  {
    g_mcp3564r.scan_config_valid = 0U;
    g_mcp3564r_init_status = 3U;
    return TX_NOT_DONE;
  }

  g_mcp3564r.scan_config_valid = 1U;
  g_mcp3564r.temperature_attempt_ms = HAL_GetTick();
  g_mcp3564r.aux_attempt_ms = HAL_GetTick();
  g_mcp3564r_init_status = 0U;
  return TX_SUCCESS;
}

HAL_StatusTypeDef mcp3564r_start_dma(void)
{

  if (g_mcp3564r.spi == NULL)
  {
    return HAL_ERROR;
  }

  /* Lost completion IRQ must not permanently own the ADC. Claim the fault
   * atomically against a late ISR and preserve ownership until thread abort. */
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  if (g_mcp3564r.dma_busy && !g_mcp3564r.recovery_needed &&
      (uint32_t)(HAL_GetTick() - g_mcp3564r.dma_started_ms) >= MCP3564R_DMA_TIMEOUT_MS)
  {
    g_mcp3564r_dma_timeout_count++;
    mcp3564r_require_recovery();
  }
  if (mask == 0U) __enable_irq();
  if (g_mcp3564r.recovery_needed != 0U)
  {
    /* HAL may still own a partially launched DMA after an asynchronous error.
     * Do not release buffers/CS or reconfigure until abort confirms quiescence. */
    if (HAL_SPI_Abort(g_mcp3564r.spi) != HAL_OK) return HAL_ERROR;
    mcp3564r_deselect();
    g_mcp3564r.dma_busy = 0U;
    g_mcp3564r.scan_config_valid = 0U;
    /* A POR/CRC fault may have reset FORMAT, IRQ and FASTCMD too. Restore
     * every base register before selecting the desired scan again. */
    if (mcp3564r_reset_device() != HAL_OK ||
        mcp3564r_write_config(&MCP3564R_DEFAULT_CONFIG) != HAL_OK)
      return HAL_ERROR;
    g_mcp3564r.start_sent = 0U;
    g_mcp3564r.recovery_needed = 0U;
  }
  const uint32_t now = HAL_GetTick();
  if (g_kg50_input_requested != g_kg50_input_active)
    return mcp3564r_switch_scan(0U);
  if (g_mcp3564r.scan_config_valid == 0U)
    return mcp3564r_switch_scan(g_mcp3564r.temperature_scan_active ? 1U : g_mcp3564r.aux_scan_active ? 2U : g_mcp3564r.p6_scan_active ? 3U : 0U);
  if (g_mcp3564r.aux_scan_active != 0U)
  {
    if (g_mcp3564r.aux_seen == MCP3564R_AUX_SCAN ||
        (uint32_t)(now - g_mcp3564r.aux_attempt_ms) >= MCP3564R_AUX_TIMEOUT_MS)
      return mcp3564r_switch_scan(0U);
  }
  else if (g_mcp3564r.temperature_scan_active != 0U)
  {
    if (g_mcp3564r.temperature_scan_complete != 0U ||
        (uint32_t)(now - g_mcp3564r.temperature_attempt_ms) >= MCP3564R_TEMP_TIMEOUT_MS)
      return mcp3564r_switch_scan(0U);
  }
  else if ((uint32_t)(now - g_mcp3564r.temperature_attempt_ms) >= MCP3564R_TEMPERATURE_INTERVAL_MS)
  {
    const HAL_StatusTypeDef status = mcp3564r_switch_scan(1U);
    if (status != HAL_BUSY) g_mcp3564r.temperature_attempt_ms = now;
    return status;
  }

  else if (MCP3564R_AUX_SCAN != 0U &&
           (uint32_t)(now - g_mcp3564r.aux_attempt_ms) >= MCP3564R_AUX_INTERVAL_MS)
  {
    const HAL_StatusTypeDef status = mcp3564r_switch_scan(2U);
    if (status != HAL_BUSY) g_mcp3564r.aux_attempt_ms = now;
    return status;
  }

  /* CONFIG2 gain is shared. Alternate settled single-channel fast scans:
   * CH0 + chamber pressure at x1, P6 differential channel 9 at x16.
   * Finish every channel in the current scan before resetting its filter;
   * switching after only CH0 can starve the following pressure conversion.
   * Never rewrite SPI registers
   * from the DMA ISR or change gain underneath an in-flight conversion. */
  if (g_kg50_input_active == 1U && g_mcp3564r.temperature_scan_active == 0U &&
      g_mcp3564r.aux_scan_active == 0U &&
      (g_mcp3564r.fast_scan_seen & mcp3564r_fast_scan(g_kg50_input_active)) ==
          mcp3564r_fast_scan(g_kg50_input_active))
    return mcp3564r_switch_scan(g_mcp3564r.p6_scan_active ? 0U : 3U);

  if (g_mcp3564r.dma_busy != 0U) return HAL_BUSY;
  if (g_mcp3564r.drdy_armed && g_mcp3564r.conversion_active)
  {
    if (HAL_GPIO_ReadPin(ADC_IRQ_GPIO_Port, ADC_IRQ_Pin) == GPIO_PIN_RESET)
      return mcp3564r_start_read_dma();
    return HAL_OK;
  }
  return mcp3564r_start_cycle();
}

void mcp3564r_dma_complete(void)
{
  if (g_mcp3564r.dma_busy == 0U || g_mcp3564r.recovery_needed != 0U) return;
  __DMB(); /* Observe completed DMA writes before consuming STATUS/data. */
  const uint8_t status = g_mcp3564r_rx_frame[0];
  /* STATUS address01/ack, active-low POR/CRC faults, active-low DR_STATUS.
   * 0x00/0xff or a different device must never become a valid conversion. */
  if ((status & 0xF8U) != 0x18U || (status & 0x03U) != 0x03U)
  {
    mcp3564r_require_recovery();
    return;
  }
  if ((status & 0x04U) == 0U)
  {
    const uint32_t raw32 = ((uint32_t)g_mcp3564r_rx_frame[1] << 24)
                         | ((uint32_t)g_mcp3564r_rx_frame[2] << 16)
                         | ((uint32_t)g_mcp3564r_rx_frame[3] << 8)
                         | ((uint32_t)g_mcp3564r_rx_frame[4]);
    mcp3564r_store_raw32(raw32);
  }
  mcp3564r_deselect();
  if ((g_mcp3564r.temperature_scan_active && g_mcp3564r.temperature_scan_complete) ||
      (g_mcp3564r.aux_scan_active && g_mcp3564r.aux_seen == MCP3564R_AUX_SCAN))
    mcp3564r_disarm();
  /* Publish idle only after every callback state/buffer/CS access is done. */
  g_mcp3564r.dma_busy = 0U;
}

void mcp3564r_dma_error(void)
{
  if (g_mcp3564r.dma_busy != 0U) mcp3564r_require_recovery();
}

void mcp3564r_data_ready_callback(uint16_t pin)
{
  if (pin == ADC_IRQ_Pin && HAL_GPIO_ReadPin(ADC_IRQ_GPIO_Port, ADC_IRQ_Pin) == GPIO_PIN_RESET)
    (void)mcp3564r_start_read_dma();
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
  sample->voltage_v = mcp3564r_code_to_voltage(code) /
      (sample->channel == 9U ? MCP3564R_P6_GAIN : 1.0f);
  sample->raw_value = sample->channel < 2U ? mcp3564r_code_to_raw_value(code) : sample->voltage_v;


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

uint32_t board_health_adc_overruns(void) { return g_mcp3564r.overrun_count; }
