/* Event-driven host simulation of the MCP3564R frame engine: the production
 * Core/Src/mcp3564r.c against a modelled TIM2, SPI2 DMA, DRDY pin and ADC.
 * Interrupt handlers run to completion in event order (TIM2/SPI/DMA share one
 * NVIC priority on the board, and EXTI claims the SPI with interrupts
 * masked). The acquisition thread runs every 2 ms and drains the queue.
 *
 * Usage: acq_engine_sim <seconds> <slow_ppm> <drop_drdy_every> <launch_from_ms>
 *                       <launch_until_ms> <kg50_input> [input_waveform]
 * Prints one line per sample: "S <channel> <sample_us> <frame> <profile> <slot> <code>"
 * then "D <name> <value>" diagnostics. Options (comma-separated):
 *   artefact     add a code offset to every conversion that directly follows
 *                a scan with a different gain or clock (worst-case settling
 *                artefact), for the spectral test;
 *   badstatus=N  the Nth ADCDATA read returns STATUS 0xFF (device fault);
 *   stall=N      the Nth SPI DMA never completes;
 *   spurious=N   a DRDY edge with no data every N conversions;
 *   en12v_low    the +12VA enable readback stays low;
 *   trace        print "C <scan> <config1> <config2>" at each START. */
#include "main.h"
#include "mcp3564r.h"
uint32_t board_health_adc_overruns(void);
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

TIM_TypeDef g_tim2, g_tim6;
SPI_HandleTypeDef g_spi;
static DMA_HandleTypeDef g_dma_tx, g_dma_rx;
static uint64_t now_us;
uint32_t g_primask;

/* ---- ADC model ---------------------------------------------------------- */
static uint8_t reg[16];
static uint32_t scan_reg;
static int converting, pin_low, data_ready;
static uint32_t data_word;
static unsigned conv_index, conv_total;
static uint8_t conv_list[16];
static uint64_t conv_end;
static double slow = 1.0;
static unsigned drop_every, conversions;
static int artefact, trace, en12v_low;
static unsigned bad_status_at, stall_at, spurious_every, reads, dmas, blocking_spi;
static uint8_t last_scan_cfg2, last_scan_cfg1, prev_scan_cfg2, prev_scan_cfg1;

static unsigned osr_of(uint8_t c1)
{
  static const unsigned table[16] = {32, 64, 128, 256, 512, 1024, 2048, 4096,
                                     8192, 16384, 20480, 24576, 40960, 49152, 81920, 98304};
  return table[(c1 >> 2) & 15U];
}

static double conv_us_one(void)
{
  const unsigned osr = osr_of(reg[2]);
  const unsigned settle = osr <= 512U ? 3U * osr : osr + 1024U;
  const unsigned pre = 1U << (reg[2] >> 6);
  const unsigned az = (reg[3] & 4U) ? 2U : 1U;
  return (double)settle * az * pre * 4.0 / 16.0 * slow;
}

static void adc_start(void)
{
  conv_total = 0;
  /* Descending channel order, as the Renode model and the board docs. */
  for (int bit = 15; bit >= 0; --bit)
    if (scan_reg & (1UL << bit)) conv_list[conv_total++] = (uint8_t)(bit == 9 ? 9 : bit == 12 ? 12 : bit);
  conv_index = 0;
  converting = conv_total != 0;
  prev_scan_cfg1 = last_scan_cfg1; prev_scan_cfg2 = last_scan_cfg2;
  last_scan_cfg1 = reg[2]; last_scan_cfg2 = reg[3];
  conv_end = now_us + (uint64_t)llround(conv_us_one());
}

static uint64_t last_clock_change_us;
static uint32_t sample_code(unsigned channel)
{
  /* Channel identity in the code, plus a constant. The artefact model adds
   * (1) an offset to a conversion that directly follows a gain/OSR change and
   * (2) a disturbance decaying with a 3 ms time constant after every scan on
   * the slow MCLK/4 clock (temperature), on every channel. */
  int32_t code = 100000 + (int32_t)channel * 1000;
  if (artefact && conv_index == 0 && (prev_scan_cfg1 != reg[2] || prev_scan_cfg2 != reg[3])) code += 50;
  if (artefact && last_clock_change_us)
    code += (int32_t)lround(40.0 * exp(-(double)(now_us - last_clock_change_us) / 3000.0));
  return ((uint32_t)channel << 28) | ((uint32_t)code & 0x01FFFFFFU);
}

static void adc_finish_conversion(void)
{
  const unsigned channel = conv_list[conv_index];
  data_word = sample_code(channel);
  if ((reg[2] >> 6) != 0U) last_clock_change_us = now_us;
  data_ready = 1;
  ++conversions;
  const int drop = drop_every && conversions % drop_every == 0;
  if (spurious_every && conversions % spurious_every == 0) {
    /* An edge with no new data: the read must see "not ready" and ignore it. */
    const int saved = data_ready; data_ready = 0; pin_low = 1;
    mcp3564r_data_ready_callback(ADC_IRQ_Pin);
    data_ready = saved; pin_low = 0;
  }
  if (!drop && !pin_low) { pin_low = 1; mcp3564r_data_ready_callback(ADC_IRQ_Pin); }
  else if (drop) pin_low = 1; /* level low but no edge delivered */
  ++conv_index;
  if (conv_index < conv_total) conv_end = now_us + (uint64_t)llround(conv_us_one());
  else converting = 0; /* one-shot: standby */
}

/* ---- SPI model ---------------------------------------------------------- */
static uint8_t *dma_tx, *dma_rx;
static uint16_t dma_len;
static int dma_active;
static uint64_t dma_end;

static void spi_apply(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
  memset(rx, 0, len);
  if (tx[0] == 0x78U) { memset(reg, 0, sizeof reg); scan_reg = 0; converting = 0; return; }
  if (tx[0] == 0x68U) {
    if (trace) printf("C %u %u %u\n", scan_reg, reg[2], reg[3]);
    adc_start(); rx[0] = 0x17; return;
  }
  if (tx[0] == 0x41U) {
    ++reads;
    rx[0] = (uint8_t)(0x13U | (data_ready ? 0U : 4U));
    if (bad_status_at && reads == bad_status_at) { rx[0] = 0xFF; return; }
    for (int i = 0; i < 4; ++i) rx[1 + i] = (uint8_t)(data_word >> (24 - 8 * i));
    data_ready = 0; pin_low = 0;
    return;
  }
  if (tx[0] == 0x46U && len == 10U) {
    for (int i = 0; i < 6; ++i) reg[1 + i] = tx[1 + i];
    scan_reg = ((uint32_t)tx[7] << 16) | ((uint32_t)tx[8] << 8) | tx[9];
    if ((reg[1] & 3U) != 3U) converting = 0; /* standby stops a conversion */
    return;
  }
  fprintf(stderr, "unexpected SPI command %02x len %u\n", tx[0], len);
  exit(3);
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t t)
{
  (void)h; (void)t;
  if (dma_active) { fprintf(stderr, "blocking SPI during DMA\n"); exit(4); }
  ++blocking_spi;
  spi_apply(tx, rx, len);
  return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *h, uint8_t *tx, uint8_t *rx, uint16_t len)
{
  (void)h;
  if (dma_active) { fprintf(stderr, "overlapping SPI DMA\n"); exit(5); }
  dma_active = 1; dma_tx = tx; dma_rx = rx; dma_len = len;
  dma_end = now_us + 1U + (len * 8U + 9U) / 10U; /* 10 MHz SPI + setup */
  if (stall_at && ++dmas == stall_at) dma_end = UINT64_MAX; /* lost completion */
  return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *h) { (void)h; dma_active = 0; return HAL_OK; }

/* ---- HAL glue ----------------------------------------------------------- */
uint32_t HAL_GetTick(void) { return (uint32_t)(now_us / 1000U); }
void HAL_Delay(uint32_t ms) { (void)ms; }
uint32_t HAL_RCC_GetPCLK1Freq(void) { return 160000000U; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state) { (void)port; (void)pin; (void)state; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
  (void)port;
  if (pin == ADC_IRQ_Pin) return pin_low ? GPIO_PIN_RESET : GPIO_PIN_SET;
  return en12v_low ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

static void sync_timers(void)
{
  g_tim2.CNT = (uint32_t)(now_us + 12345U); /* TIM2 offset from HAL time */
  g_tim6.CNT = (uint32_t)(now_us % 1000U);
  g_tim6.SR = 0;
}

static void timer_irq_if_pending(void)
{
  if (g_tim2.EGR & TIM_EGR_CC2G) { g_tim2.EGR = 0; g_tim2.SR |= TIM_SR_CC2IF; }
  if (g_tim2.EGR & TIM_EGR_UG) g_tim2.EGR = 0;
  if ((g_tim2.SR & (TIM_SR_CC1IF | TIM_SR_CC2IF)) & (g_tim2.DIER & (TIM_DIER_CC1IE | TIM_DIER_CC2IE)))
    mcp3564r_timer_irq();
  if (g_tim2.EGR & TIM_EGR_CC2G) { g_tim2.EGR = 0; g_tim2.SR |= TIM_SR_CC2IF; mcp3564r_timer_irq(); }
}

int main(int argc, char **argv)
{
  if (argc < 7) return 2;
  const double seconds = atof(argv[1]);
  slow = 1.0 + atof(argv[2]) / 1e6;
  drop_every = (unsigned)atoi(argv[3]);
  const uint64_t launch_from = (uint64_t)atoll(argv[4]) * 1000U;
  const uint64_t launch_until = (uint64_t)atoll(argv[5]) * 1000U;
  const uint8_t kg50 = (uint8_t)atoi(argv[6]);
  if (argc > 7) {
    char options[256];
    snprintf(options, sizeof options, "%s", argv[7]);
    for (char *opt = strtok(options, ","); opt; opt = strtok(NULL, ",")) {
      if (!strcmp(opt, "artefact")) artefact = 1;
      else if (!strcmp(opt, "trace")) trace = 1;
      else if (!strcmp(opt, "en12v_low")) en12v_low = 1;
      else if (!strncmp(opt, "badstatus=", 10)) bad_status_at = (unsigned)atoi(opt + 10);
      else if (!strncmp(opt, "stall=", 6)) stall_at = (unsigned)atoi(opt + 6);
      else if (!strncmp(opt, "spurious=", 9)) spurious_every = (unsigned)atoi(opt + 9);
    }
  }
  g_spi.hdmatx = &g_dma_tx; g_spi.hdmarx = &g_dma_rx;
  now_us = 5000;
  sync_timers();
  mcp3564r_request_kg50_input(kg50);
  if (mcp3564r_init(&g_spi) != TX_SUCCESS) {
    printf("D init_failed 1\nD spi_transfers %u\n", dmas + blocking_spi);
    return 0;
  }
  const uint64_t end = (uint64_t)(seconds * 1e6) + now_us;
  uint64_t next_thread = now_us + 2000U;
  while (now_us < end) {
    /* Next event: CC1, CC2, DMA end, conversion end, thread. */
    uint64_t next = next_thread;
    const uint32_t cnt = g_tim2.CNT;
    uint64_t cc1 = UINT64_MAX, cc2 = UINT64_MAX;
    if (g_tim2.DIER & TIM_DIER_CC1IE) cc1 = now_us + (uint32_t)(g_tim2.CCR1 - cnt);
    if (g_tim2.DIER & TIM_DIER_CC2IE) cc2 = now_us + (uint32_t)(g_tim2.CCR2 - cnt);
    if (cc1 < next) next = cc1;
    if (cc2 < next) next = cc2;
    if (dma_active && dma_end < next) next = dma_end;
    if (converting && conv_end < next) next = conv_end;
    now_us = next;
    sync_timers();
    if (dma_active && dma_end == now_us) {
      dma_active = 0;
      spi_apply(dma_tx, dma_rx, dma_len);
      mcp3564r_dma_complete();
      timer_irq_if_pending();
      continue;
    }
    if (converting && conv_end == now_us) { adc_finish_conversion(); continue; }
    if (cc1 == now_us) { g_tim2.SR |= TIM_SR_CC1IF; timer_irq_if_pending(); continue; }
    if (cc2 == now_us) { g_tim2.SR |= TIM_SR_CC2IF; timer_irq_if_pending(); continue; }
    if (next_thread == now_us) {
      next_thread += 2000U;
      const int launch = now_us >= launch_from && now_us < launch_until;
      mcp3564r_request_profile(launch ? DAQ_ACQ_PROFILE_LAUNCH : DAQ_ACQ_PROFILE_FILL);
      (void)mcp3564r_start_dma();
      timer_irq_if_pending();
      while (mcp3564r_pending_samples()) {
        mcp3564r_sample_t s;
        mcp3564r_get_sample(&s);
        if (!s.sample_valid) continue;
        printf("S %u %" PRIu64 " %u %u %u %d\n", s.channel, s.sample_us, s.frame, s.profile, s.slot, s.code);
      }
    }
  }
  printf("D frames %u\nD frames_skipped %u\nD slots_skipped %u\nD scans_timed_out %u\n"
         "D status_faults %u\nD unexpected %u\nD recoveries %u\nD fast_done_max_us %u\n"
         "D slot_done_max_us %u\nD start_late_max_us %u\nD overruns %u\n",
         g_mcp3564r_diag.frames, g_mcp3564r_diag.frames_skipped, g_mcp3564r_diag.slots_skipped,
         g_mcp3564r_diag.scans_timed_out, g_mcp3564r_diag.status_faults, g_mcp3564r_diag.unexpected,
         g_mcp3564r_diag.recoveries, g_mcp3564r_diag.fast_done_max_us, g_mcp3564r_diag.slot_done_max_us,
         g_mcp3564r_diag.start_late_max_us, board_health_adc_overruns());
  printf("D dma_timeouts %u\n", g_mcp3564r_dma_timeout_count);
  return 0;
}
