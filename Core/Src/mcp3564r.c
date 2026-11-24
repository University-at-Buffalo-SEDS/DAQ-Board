#include "mcp3564r.h"

#include "main.h"
#include "daq_rates.h"
#include "daq_acq_clock.h"
#include "daq_over_range.h"

#include <string.h>
#include <math.h>

/* MCP3564R frame engine (daq_acq_schedule.h has the schedule itself).
 *
 * Interrupts, in order within one frame:
 *   TIM2 CC1  frame tick: START scan A (KG1000 + chamber), 1-byte DMA
 *   EXTI10    DRDY: timestamp, 5-byte DMA read
 *   SPI2 EOT  read done: queue the sample; after scan A's last channel,
 *             write the slot configuration (10-byte DMA, ADC in standby)
 *   TIM2 CC2  slot start: START scan B
 *   EXTI10 / SPI2 EOT  slot read; then write scan A's configuration for the
 *             next frame, so the next tick only has to send START.
 *
 * TIM2, SPI2 and GPDMA run at NVIC priority 0 and cannot preempt each other;
 * EXTI10 (priority 5) claims the SPI with interrupts masked. One-shot SCAN
 * cycles return the ADC to standby, so every configuration write happens
 * between conversions. The acquisition thread only drains the sample queue
 * and handles faults that need blocking SPI (mcp3564r_start_dma). */

#define MCP3564R_CMD_RESET (0x78U)
#define MCP3564R_CMD_START (0x68U)
#define MCP3564R_CMD_READ (0x41U)

#define MCP3564R_DMA_FRAME_SIZE (5U)
#define MCP3564R_DMA_ALIGNMENT (32U)
#define MCP3564R_SAMPLE_QUEUE_DEPTH (128U)
/* Fault deadline, not sample cadence: a 10-byte burst takes 8 us. */
#define MCP3564R_DMA_TIMEOUT_MS (20U)
/* The die temperature is measured only in FILL; LAUNCH keeps the last value
 * for the length of a launch window instead of reporting NaN. */
#define MCP3564R_TEMPERATURE_FRESH_MS 2000U
#define MCP3564R_TEMPERATURE_LAUNCH_FRESH_MS 60000U

#define EN_12V HAL_GPIO_WritePin(EN_12V_GPIO_Port, EN_12V_Pin, GPIO_PIN_SET)

#if defined(__GNUC__) && (defined(__arm__) || defined(__thumb__))
#define MCP3564R_DMA_ALIGN __attribute__((section(".bss.adc_dma"), aligned(MCP3564R_DMA_ALIGNMENT)))
#elif defined(__GNUC__)
/* Native HAL fixtures use host memory; actual ARM placement is linker-tested. */
#define MCP3564R_DMA_ALIGN __attribute__((aligned(MCP3564R_DMA_ALIGNMENT)))
#else
#define MCP3564R_DMA_ALIGN
#endif

/* 16 bytes: tools/loadcell_plot.py reads this ring over SWD. */
typedef struct
{
  uint32_t raw32;
  uint32_t t_us;   /* TIM2 microseconds at the conversion midpoint */
  uint32_t frame;
  uint32_t meta;   /* bit 0: LAUNCH profile, bit 1: slot scan */
} mcp3564r_sample_entry_t;

enum { ST_STOPPED = 0, ST_IDLE, ST_FAST_START, ST_FAST_CONV, ST_SLOT_CONFIG,
       ST_SLOT_WAIT, ST_SLOT_START, ST_SLOT_CONV, ST_FAST_CONFIG, ST_RESTART_CONFIG };
enum { OP_NONE = 0, OP_READ, OP_CONFIG, OP_START };

typedef struct
{
  SPI_HandleTypeDef *spi;
  volatile uint8_t state, op, dma_busy, recovery_needed;
  volatile uint8_t pending_drdy;
  volatile uint32_t pending_drdy_us, drdy_us;
  volatile uint32_t dma_started_ms;
  /* Current frame. */
  uint32_t frame, frame_start_us;
  uint8_t profile, kg50, seen;
  daq_acq_scan_t scan;        /* scan in progress */
  uint8_t loaded_profile;     /* profile of the scan A configuration loaded */
  uint32_t slot_start_us;
  uint32_t hal_offset_us;     /* TIM2 - offset = HAL microseconds (mod 2^32) */
  uint64_t now_state_us;      /* thread-side 64-bit extension */
  /* Sample queue. */
  volatile uint8_t queue_head, queue_tail, queue_count;
  volatile uint32_t overrun_count;
  uint32_t latest_raw32;
  /* Die temperature, thread side. */
  float temperature_c;
  int32_t temperature_code;
  uint32_t temperature_ms;
  uint8_t temperature_valid;
  mcp3564r_sample_entry_t queue[MCP3564R_SAMPLE_QUEUE_DEPTH];
} mcp3564r_context_t;

static mcp3564r_context_t g_mcp3564r;
static volatile uint8_t g_kg50_input_requested;
static volatile uint8_t g_kg50_input_active;
static volatile uint8_t g_profile_requested;
mcp3564r_diag_t g_mcp3564r_diag;
volatile uint32_t g_mcp3564r_init_status = UINT32_MAX;
volatile uint32_t g_mcp3564r_dma_timeout_count;
volatile uint32_t g_mcp3564r_12v_enable_readback = UINT32_MAX;
static uint8_t g_mcp3564r_tx_frame[MCP3564R_DMA_ALIGNMENT] MCP3564R_DMA_ALIGN;
static uint8_t g_mcp3564r_rx_frame[MCP3564R_DMA_ALIGNMENT] MCP3564R_DMA_ALIGN;

void mcp3564r_request_kg50_input(uint8_t input) { if (input <= 1U) g_kg50_input_requested = input; }
uint8_t mcp3564r_kg50_input(void) { return g_kg50_input_active; }
void mcp3564r_request_profile(uint8_t profile) { if (profile <= DAQ_ACQ_PROFILE_LAUNCH) g_profile_requested = profile; }
uint8_t mcp3564r_profile(void) { return g_mcp3564r.loaded_profile; }

static void mcp3564r_select(void) { HAL_GPIO_WritePin(ADC_NCS_GPIO_Port, ADC_NCS_Pin, GPIO_PIN_RESET); }
static void mcp3564r_deselect(void) { HAL_GPIO_WritePin(ADC_NCS_GPIO_Port, ADC_NCS_Pin, GPIO_PIN_SET); }

/* ---- Acquisition timer (TIM2) ------------------------------------------- */
/* Status flags are rc_w0: writing 0 clears, 1 leaves a flag unchanged. */
#ifndef ACQ_TIM_CLEAR
#define ACQ_TIM_CLEAR(flags) (TIM2->SR = ~(uint32_t)(flags))
#endif
static uint32_t acq_now(void) { return TIM2->CNT; }
static uint32_t acq_since_frame(void) { return acq_now() - g_mcp3564r.frame_start_us; }
static void acq_note_max(volatile uint32_t *max, uint32_t value) { if (value > *max) *max = value; }

static void acq_timer_start(void)
{
  /* Free-running 1 MHz from the same APB1 timer clock as the HAL tick. */
  TIM2->CR1 = 0U;
  TIM2->DIER = 0U;
  TIM2->CCMR1 = 0U; /* CC1/CC2 frozen output compare: interrupt only */
  TIM2->CCER = 0U;
  TIM2->PSC = HAL_RCC_GetPCLK1Freq() / 1000000U - 1U;
  TIM2->ARR = 0xFFFFFFFFU;
  TIM2->EGR = TIM_EGR_UG;
  TIM2->SR = 0U;
  TIM2->CR1 = TIM_CR1_CEN;
  /* HAL microseconds = tick * 1000 + TIM6 count, read with TIM2 together. */
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  const uint32_t tim2 = TIM2->CNT;
  uint32_t count = TIM6->CNT;
  const int pending = (TIM6->SR & TIM_SR_UIF) != 0U;
  if (pending) count = TIM6->CNT;
  const uint64_t hal_us = daq_acq_hal_us(HAL_GetTick(), count, pending);
  if (mask == 0U) __enable_irq();
  g_mcp3564r.hal_offset_us = tim2 - (uint32_t)hal_us;
  g_mcp3564r.now_state_us = hal_us;
}

static void acq_engine_run(void)
{
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  g_mcp3564r.state = ST_IDLE;
  TIM2->CCR1 = TIM2->CNT + DAQ_ACQ_FRAME_US;
  ACQ_TIM_CLEAR(TIM_SR_CC1IF | TIM_SR_CC2IF);
  TIM2->DIER = TIM_DIER_CC1IE;
  if (mask == 0U) __enable_irq();
}

static void acq_engine_stop(void)
{
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  TIM2->DIER = 0U;
  g_mcp3564r.state = ST_STOPPED;
  if (mask == 0U) __enable_irq();
}

/* ---- SPI ------------------------------------------------------------------ */
static void mcp3564r_require_recovery(void)
{
  g_mcp3564r.recovery_needed = 1U;
  /* Keep dma_busy owned until the thread's HAL_SPI_Abort quiesces DMA. */
}

/* Caller owns dma_busy (set it with interrupts masked or from a priority-0
 * handler that already owned the SPI). */
static void acq_spi(uint8_t op, const uint8_t *tx, uint16_t length)
{
  if (g_mcp3564r.spi == NULL || g_mcp3564r.spi->hdmatx == NULL || g_mcp3564r.spi->hdmarx == NULL)
  {
    mcp3564r_require_recovery();
    return;
  }
  g_mcp3564r.op = op;
  g_mcp3564r.dma_started_ms = HAL_GetTick();
  memset(g_mcp3564r_rx_frame, 0, length);
  memcpy(g_mcp3564r_tx_frame, tx, length);
  __DMB(); /* Publish uncached SRAM command/data before DMA ownership. */
  mcp3564r_select();
  if (HAL_SPI_TransmitReceive_DMA(g_mcp3564r.spi, g_mcp3564r_tx_frame, g_mcp3564r_rx_frame, length) != HAL_OK)
    mcp3564r_require_recovery();
}

static int acq_claim(void)
{
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  const int claimed = !g_mcp3564r.dma_busy && !g_mcp3564r.recovery_needed;
  if (claimed) g_mcp3564r.dma_busy = 1U;
  if (mask == 0U) __enable_irq();
  return claimed;
}

static void acq_send_start(uint8_t state)
{
  static const uint8_t start = MCP3564R_CMD_START;
  g_mcp3564r.state = state;
  acq_spi(OP_START, &start, 1U);
}

static void acq_send_config(const daq_acq_scan_t *scan, uint8_t state)
{
  uint8_t burst[DAQ_ACQ_CONFIG_BURST_BYTES];
  daq_acq_config_burst(scan, burst);
  g_mcp3564r.state = state;
  acq_spi(OP_CONFIG, burst, DAQ_ACQ_CONFIG_BURST_BYTES);
}

static void acq_read(uint32_t drdy_us)
{
  static const uint8_t read[MCP3564R_DMA_FRAME_SIZE] = {MCP3564R_CMD_READ};
  g_mcp3564r.drdy_us = drdy_us;
  acq_spi(OP_READ, read, MCP3564R_DMA_FRAME_SIZE);
}

/* Scan A's configuration for the frame after this one, in the requested
 * profile. The next tick then only sends START. */
static void acq_load_next_fast(void)
{
  g_mcp3564r.loaded_profile = g_profile_requested;
  const daq_acq_scan_t next = daq_acq_fast_scan(g_mcp3564r.loaded_profile);
  acq_send_config(&next, ST_FAST_CONFIG);
}

/* ---- Sample queue (interrupt producer, thread consumer) -------------------- */
static void acq_push(uint32_t raw32, uint32_t t_us)
{
  mcp3564r_sample_entry_t entry = {raw32, t_us, g_mcp3564r.frame,
      (g_mcp3564r.profile == DAQ_ACQ_PROFILE_LAUNCH ? 1U : 0U) |
      (g_mcp3564r.scan.item != DAQ_ACQ_ITEM_NONE ? 2U : 0U)};
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  if (g_mcp3564r.queue_count >= MCP3564R_SAMPLE_QUEUE_DEPTH)
  {
    g_mcp3564r.queue_tail = (uint8_t)((g_mcp3564r.queue_tail + 1U) % MCP3564R_SAMPLE_QUEUE_DEPTH);
    g_mcp3564r.queue_count--;
    g_mcp3564r.overrun_count++;
  }
  g_mcp3564r.queue[g_mcp3564r.queue_head] = entry;
  g_mcp3564r.queue_head = (uint8_t)((g_mcp3564r.queue_head + 1U) % MCP3564R_SAMPLE_QUEUE_DEPTH);
  g_mcp3564r.queue_count++;
  g_mcp3564r.latest_raw32 = raw32;
  if (mask == 0U) __enable_irq();
}

/* ---- Frame state machine ---------------------------------------------------- */
static void acq_scan_done(void)
{
  const uint32_t done = g_mcp3564r.drdy_us - g_mcp3564r.frame_start_us;
  if (g_mcp3564r.state == ST_FAST_CONV)
  {
    acq_note_max(&g_mcp3564r_diag.fast_done_max_us, done);
    const uint8_t item = daq_acq_slot_item(g_mcp3564r.profile, g_mcp3564r.kg50, g_mcp3564r.frame);
    const daq_acq_scan_t slot = daq_acq_slot_scan(g_mcp3564r.profile, item);
    uint32_t start = slot.offset_us;
    const uint32_t earliest = acq_since_frame() + DAQ_ACQ_CONFIG_US + DAQ_ACQ_SETTLE_US;
    if (earliest > start) start = earliest;
    if (!daq_acq_slot_fits(&slot, start))
    {
      /* Scan A ran long: drop this slot, never the next frame's scan A. */
      ++g_mcp3564r_diag.slots_skipped;
      if (g_mcp3564r.loaded_profile != g_profile_requested) acq_load_next_fast();
      else { g_mcp3564r.state = ST_IDLE; g_mcp3564r.dma_busy = 0U; }
      return;
    }
    g_mcp3564r.scan = slot;
    g_mcp3564r.slot_start_us = g_mcp3564r.frame_start_us + start;
    acq_send_config(&slot, ST_SLOT_CONFIG);
    return;
  }
  acq_note_max(&g_mcp3564r_diag.slot_done_max_us, done);
  acq_load_next_fast();
}

static void acq_read_done(void)
{
  const uint8_t status = g_mcp3564r_rx_frame[0];
  /* DS20006391C section6.2/Figure6-1: STAT[5:4] is address01;
   * STAT[3] is the COMPLEMENT of address bit0, hence zero. Healthy replies
   * are0x13 (ready) or0x17 (not ready), not0x1b/0x1f. Keep rejecting wrong
   * addresses, high-impedance patterns, and active-low POR/CRC faults. */
  if ((status & 0xF8U) != 0x10U || (status & 0x03U) != 0x03U)
  {
    ++g_mcp3564r_diag.status_faults;
    mcp3564r_require_recovery();
    return;
  }
  g_mcp3564r.dma_busy = 0U;
  if ((status & 0x04U) != 0U) return; /* not ready: a spurious edge */
  const uint32_t raw32 = ((uint32_t)g_mcp3564r_rx_frame[1] << 24) | ((uint32_t)g_mcp3564r_rx_frame[2] << 16) |
                         ((uint32_t)g_mcp3564r_rx_frame[3] << 8) | (uint32_t)g_mcp3564r_rx_frame[4];
  const unsigned channel = raw32 >> 28U;
  if ((g_mcp3564r.scan.scan & (1UL << channel)) == 0U || (g_mcp3564r.seen & (1U << (channel & 7U))) != 0U)
  {
    ++g_mcp3564r_diag.unexpected;
    return;
  }
  g_mcp3564r.seen |= (uint8_t)(1U << (channel & 7U));
  acq_push(raw32, daq_acq_sample_time_us(&g_mcp3564r.scan, g_mcp3564r.drdy_us));
  unsigned seen = 0U;
  for (uint8_t bits = g_mcp3564r.seen; bits; bits &= (uint8_t)(bits - 1U)) ++seen;
  if (seen < g_mcp3564r.scan.channels) return;
  if (acq_claim()) acq_scan_done();
}

static void acq_begin_scan(const daq_acq_scan_t *scan, uint8_t state, uint32_t due_us)
{
  g_mcp3564r.scan = *scan;
  g_mcp3564r.seen = 0U;
  acq_note_max(&g_mcp3564r_diag.start_late_max_us, acq_now() - due_us);
  acq_send_start(state);
}

void mcp3564r_dma_complete(void)
{
  if (g_mcp3564r.dma_busy == 0U || g_mcp3564r.recovery_needed != 0U) return;
  __DMB(); /* Observe completed DMA writes before consuming STATUS/data. */
  mcp3564r_deselect();
  const uint8_t op = g_mcp3564r.op;
  g_mcp3564r.op = OP_NONE;
  switch (op)
  {
    case OP_READ:
      acq_read_done();
      break;
    case OP_START:
      g_mcp3564r.state = g_mcp3564r.state == ST_FAST_START ? ST_FAST_CONV : ST_SLOT_CONV;
      g_mcp3564r.dma_busy = 0U;
      break;
    case OP_CONFIG:
      if (g_mcp3564r.state == ST_SLOT_CONFIG)
      {
        g_mcp3564r.state = ST_SLOT_WAIT;
        g_mcp3564r.dma_busy = 0U;
        TIM2->CCR2 = g_mcp3564r.slot_start_us;
        ACQ_TIM_CLEAR(TIM_SR_CC2IF);
        TIM2->DIER |= TIM_DIER_CC2IE;
        /* Due already (scan A ran late): start now rather than a timer wrap. */
        if ((int32_t)(g_mcp3564r.slot_start_us - acq_now()) <= 0) TIM2->EGR = TIM_EGR_CC2G;
      }
      else if (g_mcp3564r.state == ST_RESTART_CONFIG)
      {
        const daq_acq_scan_t fast = daq_acq_fast_scan(g_mcp3564r.loaded_profile);
        acq_begin_scan(&fast, ST_FAST_START, g_mcp3564r.frame_start_us);
      }
      else
      {
        g_mcp3564r.state = ST_IDLE;
        g_mcp3564r.dma_busy = 0U;
      }
      break;
    default:
      g_mcp3564r.dma_busy = 0U;
      break;
  }
  /* A DRDY edge seen while the SPI was busy. */
  if (g_mcp3564r.pending_drdy && (g_mcp3564r.state == ST_FAST_CONV || g_mcp3564r.state == ST_SLOT_CONV) &&
      acq_claim())
  {
    g_mcp3564r.pending_drdy = 0U;
    acq_read(g_mcp3564r.pending_drdy_us);
  }
}

void mcp3564r_dma_error(void)
{
  if (g_mcp3564r.dma_busy != 0U) mcp3564r_require_recovery();
}

static void acq_frame_tick(void)
{
  g_mcp3564r.frame_start_us = TIM2->CCR1;
  TIM2->CCR1 = g_mcp3564r.frame_start_us + DAQ_ACQ_FRAME_US;
  ++g_mcp3564r_diag.frames;
  const uint8_t state = g_mcp3564r.state;
  if (state == ST_STOPPED || g_mcp3564r.recovery_needed) return;
  if (!acq_claim())
  {
    ++g_mcp3564r_diag.frames_skipped; /* SPI still busy: the thread checks for a stall */
    return;
  }
  ++g_mcp3564r.frame;
  g_mcp3564r.profile = g_mcp3564r.loaded_profile;
  g_mcp3564r.kg50 = g_kg50_input_requested;
  g_kg50_input_active = g_mcp3564r.kg50;
  g_mcp3564r.pending_drdy = 0U;
  TIM2->DIER &= ~TIM_DIER_CC2IE;
  if (state == ST_IDLE)
  {
    const daq_acq_scan_t fast = daq_acq_fast_scan(g_mcp3564r.profile);
    acq_begin_scan(&fast, ST_FAST_START, g_mcp3564r.frame_start_us);
    return;
  }
  /* A scan still waiting for DRDY at the next tick lost a conversion (or its
   * edge). The configuration burst puts the ADC in standby first, then the
   * frame restarts a few microseconds late. */
  ++g_mcp3564r_diag.scans_timed_out;
  g_mcp3564r.loaded_profile = g_profile_requested;
  g_mcp3564r.profile = g_mcp3564r.loaded_profile;
  const daq_acq_scan_t fast = daq_acq_fast_scan(g_mcp3564r.profile);
  g_mcp3564r.scan = fast;
  acq_send_config(&fast, ST_RESTART_CONFIG);
}

void mcp3564r_timer_irq(void)
{
  const uint32_t sr = TIM2->SR;
  if ((sr & TIM_SR_CC1IF) != 0U && (TIM2->DIER & TIM_DIER_CC1IE) != 0U)
  {
    ACQ_TIM_CLEAR(TIM_SR_CC1IF);
    acq_frame_tick();
  }
  if ((sr & TIM_SR_CC2IF) != 0U)
  {
    ACQ_TIM_CLEAR(TIM_SR_CC2IF);
    if ((TIM2->DIER & TIM_DIER_CC2IE) != 0U && g_mcp3564r.state == ST_SLOT_WAIT && acq_claim())
    {
      TIM2->DIER &= ~TIM_DIER_CC2IE;
      const daq_acq_scan_t slot = g_mcp3564r.scan;
      acq_begin_scan(&slot, ST_SLOT_START, g_mcp3564r.slot_start_us);
    }
  }
}

void mcp3564r_data_ready_callback(uint16_t pin)
{
  if (pin != ADC_IRQ_Pin) return;
  const uint32_t now = acq_now();
  if (HAL_GPIO_ReadPin(ADC_IRQ_GPIO_Port, ADC_IRQ_Pin) != GPIO_PIN_RESET) return;
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  const uint8_t state = g_mcp3564r.state;
  int claimed = 0;
  if (state == ST_FAST_CONV || state == ST_SLOT_CONV)
  {
    if (!g_mcp3564r.dma_busy && !g_mcp3564r.recovery_needed) { g_mcp3564r.dma_busy = 1U; claimed = 1; }
    else { g_mcp3564r.pending_drdy = 1U; g_mcp3564r.pending_drdy_us = now; }
  }
  if (mask == 0U) __enable_irq();
  if (claimed) acq_read(now);
}

/* ---- Thread-context device access (engine stopped) ------------------------- */
static HAL_StatusTypeDef mcp3564r_blocking(const uint8_t *tx, uint16_t length)
{
  uint8_t rx[DAQ_ACQ_CONFIG_BURST_BYTES];
  if (length > sizeof(rx)) return HAL_ERROR;
  mcp3564r_select();
  const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(g_mcp3564r.spi, (uint8_t *)tx, rx, length, 1000U);
  mcp3564r_deselect();
  return status;
}

static HAL_StatusTypeDef mcp3564r_device_setup(void)
{
  static const uint8_t reset = MCP3564R_CMD_RESET;
  static const uint8_t read[MCP3564R_DMA_FRAME_SIZE] = {MCP3564R_CMD_READ};
  uint8_t burst[DAQ_ACQ_CONFIG_BURST_BYTES];
  g_mcp3564r.loaded_profile = g_profile_requested;
  const daq_acq_scan_t fast = daq_acq_fast_scan(g_mcp3564r.loaded_profile);
  daq_acq_config_burst(&fast, burst);
  HAL_StatusTypeDef status = mcp3564r_blocking(&reset, 1U);
  if (status == HAL_OK) status = mcp3564r_blocking(burst, sizeof(burst));
  /* Consume any stale ADCDATA/IRQ indication before the first frame. */
  if (status == HAL_OK) status = mcp3564r_blocking(read, sizeof(read));
  return status;
}

UINT mcp3564r_init(SPI_HandleTypeDef *spi)
{
  if (spi == NULL) return TX_PTR_ERROR;
  memset(&g_mcp3564r, 0, sizeof(g_mcp3564r));
  g_kg50_input_active = g_kg50_input_requested;
  g_mcp3564r.spi = spi;
  g_mcp3564r.temperature_c = NAN;

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
  if (mcp3564r_device_setup() != HAL_OK)
  {
    g_mcp3564r_init_status = 2U;
    return TX_NOT_DONE;
  }
  acq_timer_start();
  acq_engine_run();
  g_mcp3564r_init_status = 0U;
  return TX_SUCCESS;
}

HAL_StatusTypeDef mcp3564r_start_dma(void)
{
  if (g_mcp3564r.spi == NULL) return HAL_ERROR;
  /* A lost completion interrupt must not own the ADC forever. */
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
    acq_engine_stop();
    /* HAL may still own a partially launched DMA after an asynchronous error.
     * Do not release buffers/CS or reconfigure until abort confirms quiescence. */
    if (HAL_SPI_Abort(g_mcp3564r.spi) != HAL_OK) return HAL_ERROR;
    mcp3564r_deselect();
    g_mcp3564r.dma_busy = 0U;
    g_mcp3564r.op = OP_NONE;
    /* A POR/CRC fault may have reset FORMAT, IRQ and FASTCMD too. */
    if (mcp3564r_device_setup() != HAL_OK) return HAL_ERROR;
    g_mcp3564r.recovery_needed = 0U;
    ++g_mcp3564r_diag.recoveries;
    acq_engine_run();
    return HAL_OK;
  }
  /* A DRDY edge lost while the pin stayed low (EXTI is edge-triggered). */
  if (HAL_GPIO_ReadPin(ADC_IRQ_GPIO_Port, ADC_IRQ_Pin) == GPIO_PIN_RESET)
    mcp3564r_data_ready_callback(ADC_IRQ_Pin);
  return HAL_OK;
}

/* ---- Thread-side sample access ----------------------------------------------- */
uint64_t mcp3564r_now_us(void)
{
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  const uint64_t now = daq_acq_extend_us(&g_mcp3564r.now_state_us, acq_now() - g_mcp3564r.hal_offset_us);
  if (mask == 0U) __enable_irq();
  return now;
}

/* DATA_FORMAT=11: CH_ID in bits 31:28, sign in bit 24. */
static int32_t mcp3564r_decode_code(uint32_t raw32)
{
  uint32_t code = raw32 & 0x01FFFFFFU;
  if ((code & 0x01000000U) != 0U) code |= 0xFE000000U;
  return (int32_t)code;
}

UINT mcp3564r_get_sample(mcp3564r_sample_t *sample)
{
  if (sample == NULL) return TX_PTR_ERROR;
  memset(sample, 0, sizeof(*sample));
  const uint64_t now_us = mcp3564r_now_us();
  mcp3564r_sample_entry_t entry = {0};
  uint8_t valid = 0U;
  for (;;)
  {
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    valid = g_mcp3564r.queue_count != 0U;
    if (valid)
    {
      entry = g_mcp3564r.queue[g_mcp3564r.queue_tail];
      g_mcp3564r.queue_tail = (uint8_t)((g_mcp3564r.queue_tail + 1U) % MCP3564R_SAMPLE_QUEUE_DEPTH);
      g_mcp3564r.queue_count--;
    }
    else entry.raw32 = g_mcp3564r.latest_raw32;
    sample->queued_samples = g_mcp3564r.queue_count;
    sample->dma_busy = g_mcp3564r.dma_busy;
    sample->overrun_count = g_mcp3564r.overrun_count;
    if (mask == 0U) __enable_irq();
    if (!valid || (entry.raw32 >> 28U) != DAQ_ACQ_ITEM_TEMPERATURE) break;
    /* TEMP conversions update the die temperature; they are not samples. */
    const int32_t code = mcp3564r_decode_code(entry.raw32);
    const float temperature = mcp3564r_code_to_temperature(code);
    /* An over-range TEMP code is never a temperature (daq_over_range.h). */
    const int8_t over = daq_adc_over_range(code);
    g_mcp3564r_diag.temperature_over_range_last = over;
    if (over != 0) ++g_mcp3564r_diag.temperature_over_range;
    g_mcp3564r.temperature_valid = over == 0 && isfinite(temperature) && temperature >= -40.0f && temperature <= 125.0f;
    g_mcp3564r.temperature_c = temperature;
    g_mcp3564r.temperature_code = code;
    g_mcp3564r.temperature_ms = (uint32_t)(daq_acq_project_us(now_us, entry.t_us - g_mcp3564r.hal_offset_us) / 1000U);
  }
  const uint32_t now_ms = (uint32_t)(now_us / 1000U);
  const uint32_t fresh = g_mcp3564r.loaded_profile == DAQ_ACQ_PROFILE_LAUNCH
      ? MCP3564R_TEMPERATURE_LAUNCH_FRESH_MS : MCP3564R_TEMPERATURE_FRESH_MS;
  sample->temperature_c = g_mcp3564r.temperature_valid && (uint32_t)(now_ms - g_mcp3564r.temperature_ms) <= fresh
      ? g_mcp3564r.temperature_c : NAN;
  sample->temperature_code = g_mcp3564r.temperature_code;
  const int32_t code = mcp3564r_decode_code(entry.raw32);
  sample->channel = (uint8_t)(entry.raw32 >> 28U);
  sample->sample_valid = valid;
  sample->code = code;
  sample->voltage_v = mcp3564r_code_to_voltage(code) / (sample->channel == DAQ_ACQ_ITEM_P6 ? MCP3564R_P6_GAIN : 1.0f);
  /* Every channel, KG1000 and KG50 included, reports ADC-pin volts. */
  sample->raw_value = sample->voltage_v;
  if (valid)
  {
    sample->sample_us = daq_acq_project_us(now_us, entry.t_us - g_mcp3564r.hal_offset_us);
    sample->monotonic_ms = sample->sample_us / 1000U;
    sample->frame = entry.frame;
    sample->profile = (uint8_t)(entry.meta & 1U);
    sample->slot = (uint8_t)((entry.meta >> 1U) & 1U);
  }
  return TX_SUCCESS;
}

uint8_t mcp3564r_pending_samples(void)
{
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  const uint8_t count = g_mcp3564r.queue_count;
  if (mask == 0U) __enable_irq();
  return count;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == g_mcp3564r.spi) mcp3564r_dma_complete();
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == g_mcp3564r.spi) mcp3564r_dma_error();
}

uint32_t board_health_adc_overruns(void) { return g_mcp3564r.overrun_count; }
