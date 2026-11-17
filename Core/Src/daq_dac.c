#include "daq_dac.h"

#include "daq_adc4.h"
#include "daq_dac_control.h"
#include "daq_log_time.h"
#include "flight_state_cache.h"
#include "main.h"
#include "sedsnet_config.h"
#if (DISABLE_SD_CARD == 0U)
#include "sd_card.h"
#endif
#ifdef TELEMETRY_ENABLED
#include "telemetry.h"
#endif

#include <math.h>
#include <string.h>

extern DAC_HandleTypeDef hdac1;

/* Channel c uses DAC1 OUTc+1, a dedicated trigger timer and a GPDMA channel
 * left free by CubeMX (channels 0..3 belong to SPI2/ADC). The DMA re-reads
 * its own self-linked list item, so the table repeats without interrupts. */
static DMA_Channel_TypeDef *const k_dma[DAQ_DAC_CHANNELS] = {GPDMA1_Channel10, GPDMA1_Channel11};
static TIM_TypeDef *const k_tim[DAQ_DAC_CHANNELS] = {TIM7, TIM15};
static const uint32_t k_trigger[DAQ_DAC_CHANNELS] = {DAC_TRIGGER_T7_TRGO, DAC_TRIGGER_T15_TRGO};
static const uint32_t k_hal_channel[DAQ_DAC_CHANNELS] = {DAC_CHANNEL_1, DAC_CHANNEL_2};
static const uint32_t k_request[DAQ_DAC_CHANNELS] = {GPDMA1_REQUEST_DAC1_CH1, GPDMA1_REQUEST_DAC1_CH2};
static const uint32_t k_adc_channel[DAQ_DAC_CHANNELS] = {ADC_CHANNEL_DAC1CH1_ADC4, ADC_CHANNEL_DAC1CH2_ADC4};

#define DAQ_DAC_SERVICE_PERIOD_MS 50U
#define DAQ_DAC_STATUS_IDLE_MS 1000U
#define DAQ_DAC_STATUS_ACTIVE_MS 200U
#define DAQ_DAC_SD_ROW_MS 100U
#define DAQ_DAC_DMA_FLAGS 0x7F00U /* CFCR: every channel event flag. */

/* Internal SRAM is not behind DCACHE1, so plain .bss is DMA-coherent. */
static uint32_t g_table[DAQ_DAC_CHANNELS][DAQ_DAC_TABLE_LEN] __attribute__((aligned(4)));
/* Linked-list item, in update order: CBR1, CSAR, CLLR. */
static uint32_t g_lli[DAQ_DAC_CHANNELS][3] __attribute__((aligned(4)));
static daq_dac_ctrl_t g_ctrl;
static daq_dac_command_t g_pending[DAQ_DAC_CHANNELS];
static volatile uint8_t g_pending_mask;
static volatile uint8_t g_pending_all_off;
static uint32_t g_pending_all_sequence;
static volatile uint8_t g_kill_reason;
static volatile uint8_t g_hw_active;
static volatile uint8_t g_hw_ready;
static volatile uint8_t g_readback_ok;
static volatile float g_output_v[DAQ_DAC_CHANNELS] = {NAN, NAN};
static volatile float g_readback_v[DAQ_DAC_CHANNELS] = {NAN, NAN};

volatile uint32_t g_daq_dac_active_mask __attribute__((used));
volatile uint32_t g_daq_dac_commands __attribute__((used));
volatile uint32_t g_daq_dac_malformed __attribute__((used));
volatile uint32_t g_daq_dac_refusals __attribute__((used));
volatile uint32_t g_daq_dac_starts __attribute__((used));
volatile uint32_t g_daq_dac_stops __attribute__((used));
volatile uint32_t g_daq_dac_emergency_offs __attribute__((used));
volatile uint32_t g_daq_dac_status_sent __attribute__((used));
volatile uint32_t g_daq_dac_status_failed __attribute__((used));

static uint32_t timer_clock_hz(unsigned c)
{
  const uint32_t pclk = c == 0U ? HAL_RCC_GetPCLK1Freq() : HAL_RCC_GetPCLK2Freq();
  const uint32_t pre = c == 0U ? ((RCC->CFGR2 & RCC_CFGR2_PPRE1) >> RCC_CFGR2_PPRE1_Pos)
                               : ((RCC->CFGR2 & RCC_CFGR2_PPRE2) >> RCC_CFGR2_PPRE2_Pos);
  /* APB prescaler 1 clocks the timers at PCLK, otherwise at twice PCLK. */
  return pre < 4U ? pclk : 2U * pclk;
}

/* Idempotent and short: safe from any thread. Disabling the channel puts the
 * pin in high impedance; the board's 1 M pulldown takes the buffer input to 0 V. */
static void hw_stop(unsigned c)
{
  const uint32_t shift = c == 0U ? 0U : 16U;
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  if (g_hw_active & (1U << c)) k_tim[c]->CR1 &= ~TIM_CR1_CEN;
  DAC1->CR &= ~((DAC_CR_EN1 | DAC_CR_DMAEN1 | DAC_CR_TEN1 | DAC_CR_TSEL1) << shift);
  DMA_Channel_TypeDef *dma = k_dma[c];
  if (dma->CCR & DMA_CCR_EN) {
    dma->CCR |= DMA_CCR_SUSP;
    for (unsigned i = 0U; i < 1000U && (dma->CSR & DMA_CSR_SUSPF) == 0U; ++i) {}
  }
  dma->CCR = DMA_CCR_RESET;
  dma->CFCR = DAQ_DAC_DMA_FLAGS;
  g_hw_active &= (uint8_t)~(1U << c);
  g_output_v[c] = NAN;
  g_readback_v[c] = NAN;
  __set_PRIMASK(mask);
  ++g_daq_dac_stops;
}

static bool hw_start(unsigned c, const daq_dac_plan_t *plan)
{
  hw_stop(c);
  if (!g_hw_ready) return false;
  DAC_ChannelConfTypeDef config = {0};
  config.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  config.DAC_DMADoubleDataMode = DISABLE;
  config.DAC_SignedFormat = DISABLE;
  config.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  config.DAC_Trigger = plan->samples > 1U ? k_trigger[c] : DAC_TRIGGER_NONE;
  config.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  config.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_BOTH; /* Pin and ADC4 readback. */
  config.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  config.DAC_AutonomousMode = DAC_AUTONOMOUS_MODE_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &config, k_hal_channel[c]) != HAL_OK) return false;
  if (HAL_DAC_SetValue(&hdac1, k_hal_channel[c], DAC_ALIGN_12B_R, g_table[c][0]) != HAL_OK) return false;

  if (plan->samples > 1U) {
    TIM_TypeDef *tim = k_tim[c];
    if (c == 0U) __HAL_RCC_TIM7_CLK_ENABLE(); else __HAL_RCC_TIM15_CLK_ENABLE();
    tim->CR1 = 0U;
    tim->PSC = plan->psc;
    tim->ARR = plan->arr;
    tim->CR2 = (tim->CR2 & ~TIM_CR2_MMS) | TIM_CR2_MMS_1; /* Update event -> TRGO. */
    tim->EGR = TIM_EGR_UG;
    tim->SR = 0U;

    DMA_Channel_TypeDef *dma = k_dma[c];
    const uint32_t bytes = (uint32_t)plan->samples * sizeof(uint32_t);
    const uint32_t node = (uint32_t)(uintptr_t)g_lli[c];
    g_lli[c][0] = bytes;
    g_lli[c][1] = (uint32_t)(uintptr_t)g_table[c];
    g_lli[c][2] = DMA_CLLR_UB1 | DMA_CLLR_USA | DMA_CLLR_ULL | (node & DMA_CLLR_LA);
    dma->CCR = DMA_CCR_RESET;
    dma->CFCR = DAQ_DAC_DMA_FLAGS;
    dma->CLBAR = node & 0xFFFF0000U;
    dma->CTR1 = DMA_CTR1_SINC | (2U << DMA_CTR1_SDW_LOG2_Pos) | (2U << DMA_CTR1_DDW_LOG2_Pos);
    dma->CTR2 = (k_request[c] << DMA_CTR2_REQSEL_Pos) | DMA_CTR2_DREQ;
    dma->CBR1 = bytes;
    dma->CSAR = (uint32_t)(uintptr_t)g_table[c];
    dma->CDAR = (uint32_t)(uintptr_t)(c == 0U ? &DAC1->DHR12R1 : &DAC1->DHR12R2);
    dma->CLLR = g_lli[c][2];
    dma->CCR = DMA_CCR_EN;
    DAC1->CR |= DAC_CR_DMAEN1 << (c == 0U ? 0U : 16U);
  }
  if (HAL_DAC_Start(&hdac1, k_hal_channel[c]) != HAL_OK) { hw_stop(c); return false; }

  /* An abort or flight-state change that raced this start wins: it already
   * stopped the hardware, or it stops it before the timer runs. */
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  const bool killed = g_kill_reason != 0U;
  if (!killed) {
    g_hw_active |= (uint8_t)(1U << c);
    if (plan->samples > 1U) k_tim[c]->CR1 |= TIM_CR1_CEN;
  }
  __set_PRIMASK(mask);
  if (killed) hw_stop(c);
  else ++g_daq_dac_starts;
  return true;
}

UINT daq_dac_init(void)
{
  /* Never resume an output across a reset: stop both, whatever CubeMX left. */
  g_hw_ready = hdac1.Instance == DAC1 && hdac1.State == HAL_DAC_STATE_READY;
  for (unsigned c = 0U; c < DAQ_DAC_CHANNELS; ++c) hw_stop(c);
  daq_dac_ctrl_init(&g_ctrl);
  g_daq_dac_active_mask = 0U;
  return TX_SUCCESS;
}

UINT daq_dac_sample(daq_dac_sample_t *sample, uint8_t read_back)
{
  if (sample == NULL) return TX_PTR_ERROR;
  for (unsigned c = 0U; c < DAQ_DAC_CHANNELS; ++c) {
    if ((g_hw_active & (1U << c)) == 0U) { sample->analog_outputs_v[c] = NAN; continue; }
    const float output = daq_dac_code_to_volts(HAL_DAC_GetValue(&hdac1, k_hal_channel[c]));
    sample->analog_outputs_v[c] = output;
    if (!read_back) continue;
    float readback = NAN;
    const bool ok = daq_adc4_read_internal(k_adc_channel[c], &readback) == TX_SUCCESS;
    g_readback_ok = ok ? 1U : 0U;
    /* Store only while still active, so a concurrent stop leaves NaN. */
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    if (g_hw_active & (1U << c)) { g_output_v[c] = output; g_readback_v[c] = ok ? readback : NAN; }
    __set_PRIMASK(mask);
  }
  return TX_SUCCESS;
}

void daq_dac_submit_command(const uint8_t *payload, size_t len)
{
  daq_dac_command_t command;
  if (daq_dac_command_decode(payload, len, &command) != 0) { ++g_daq_dac_malformed; return; }
  ++g_daq_dac_commands;
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  if (command.channel == DAQ_DAC_CHANNEL_ALL) {
    g_pending_mask = 0U; /* Older per-channel requests are superseded. */
    g_pending_all_off = 1U;
    g_pending_all_sequence = command.sequence;
  } else {
    g_pending[command.channel] = command;
    g_pending_mask |= (uint8_t)(1U << command.channel);
  }
  __set_PRIMASK(mask);
}

void daq_dac_emergency_off(uint8_t reason)
{
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  if (g_kill_reason == 0U) g_kill_reason = reason;
  g_pending_mask = 0U; /* Requests from before the abort never start later. */
  g_pending_all_off = 0U;
  __set_PRIMASK(mask);
  for (unsigned c = 0U; c < DAQ_DAC_CHANNELS; ++c) hw_stop(c);
  ++g_daq_dac_emergency_offs;
}

#ifdef TELEMETRY_ENABLED
/* flight_state_cache.c calls this before applying any received FLIGHT_STATE.
 * Leaving Startup/Idle stops the outputs before the new state is visible. */
SedsResult flight_state_cache_apply_network_update(const SedsPacketView *packet)
{
  if (packet != NULL && packet->payload != NULL && packet->payload_len == 1U) {
    const uint8_t state = flight_state_from_wire(packet->payload[0], NULL);
    if (daq_dac_state_gate(true, state) != DAQ_DAC_RESULT_OK)
      daq_dac_emergency_off(DAQ_DAC_OFF_FLIGHT_STATE);
  }
  return SEDS_OK;
}
#endif

static void apply_actions(const uint8_t actions[DAQ_DAC_CHANNELS])
{
  for (unsigned c = 0U; c < DAQ_DAC_CHANNELS; ++c) {
    if (actions[c] == DAQ_DAC_ACTION_STOP) hw_stop(c);
    else if (actions[c] == DAQ_DAC_ACTION_START) {
      daq_dac_plan_t plan;
      const bool planned = daq_dac_plan(&g_ctrl.ch[c].config, timer_clock_hz(c), &plan, g_table[c]) == 0;
      const bool ok = planned && hw_start(c, &plan);
      if (!ok) hw_stop(c);
      daq_dac_ctrl_started(&g_ctrl, c, &plan, ok);
    }
  }
}

#if (DISABLE_SD_CARD == 0U)
enum { SD_ACTIVE, SD_WAVEFORM, SD_AMPLITUDE, SD_OFFSET, SD_FREQUENCY, SD_STEPS, SD_DWELL,
       SD_SEQUENCE, SD_RESULT, SD_REASON, SD_OUTPUT, SD_READBACK, SD_NAMES };
static const char *const k_sd_names[DAQ_DAC_CHANNELS][SD_NAMES] = {
  {"dac_ch1_active", "dac_ch1_waveform", "dac_ch1_amplitude_v", "dac_ch1_offset_v",
   "dac_ch1_frequency_hz", "dac_ch1_steps", "dac_ch1_dwell_ms", "dac_ch1_sequence",
   "dac_ch1_result", "dac_ch1_off_reason", "dac_ch1_output_v", "dac_ch1_readback_v"},
  {"dac_ch2_active", "dac_ch2_waveform", "dac_ch2_amplitude_v", "dac_ch2_offset_v",
   "dac_ch2_frequency_hz", "dac_ch2_steps", "dac_ch2_dwell_ms", "dac_ch2_sequence",
   "dac_ch2_result", "dac_ch2_off_reason", "dac_ch2_output_v", "dac_ch2_readback_v"},
};

/* One row per field at each change: with the timer-quantised frequency or
 * dwell, these rows reconstruct the generated reference offline. Sequence
 * rows are exact below 2^24. */
static void log_event(unsigned c, uint64_t timestamp_ms)
{
  const daq_dac_channel_state_t *ch = &g_ctrl.ch[c];
  const float values[SD_OUTPUT] = {
    (float)ch->active, (float)ch->config.waveform, ch->config.amplitude_v, ch->config.offset_v,
    ch->actual_frequency_hz, (float)ch->config.steps, (float)ch->actual_dwell_ms,
    (float)(ch->sequence & 0xFFFFFFU), (float)ch->last_result, (float)ch->off_reason};
  for (unsigned i = 0U; i < SD_OUTPUT; ++i)
    (void)sd_card_enqueue_csv_row(k_sd_names[c][i], timestamp_ms, values[i], NULL);
}
#endif

void daq_dac_service(uint32_t now_ms)
{
  static uint32_t last_service_ms, last_status_ms, last_sd_ms;
  static uint8_t started;
  if (started && (uint32_t)(now_ms - last_service_ms) < DAQ_DAC_SERVICE_PERIOD_MS) return;
  started = 1U;
  last_service_ms = now_ms;

  daq_dac_command_t pending[DAQ_DAC_CHANNELS];
  uint32_t mask = __get_PRIMASK(); __disable_irq();
  const uint8_t kill = g_kill_reason;
  const uint8_t pending_mask = g_pending_mask;
  const uint8_t all_off = g_pending_all_off;
  const uint32_t all_sequence = g_pending_all_sequence;
  memcpy(pending, g_pending, sizeof(pending));
  g_kill_reason = 0U; g_pending_mask = 0U; g_pending_all_off = 0U;
  __set_PRIMASK(mask);

  uint8_t actions[DAQ_DAC_CHANNELS];
  if (kill) {
    daq_dac_ctrl_force_off(&g_ctrl, kill, actions);
    for (unsigned c = 0U; c < DAQ_DAC_CHANNELS; ++c) hw_stop(c);
  }
  const bool has_state = flight_state_cache_has_value();
  const uint8_t state = has_state ? flight_state_cache_value() : DAQ_DAC_STATE_UNKNOWN;
  if (all_off) {
    const daq_dac_command_t off = {.channel = DAQ_DAC_CHANNEL_ALL, .sequence = all_sequence};
    (void)daq_dac_ctrl_command(&g_ctrl, &off, has_state, state, now_ms, actions);
    apply_actions(actions);
  }
  for (unsigned c = 0U; c < DAQ_DAC_CHANNELS; ++c) {
    if ((pending_mask & (1U << c)) == 0U) continue;
    const daq_dac_result_t result =
        daq_dac_ctrl_command(&g_ctrl, &pending[c], has_state, state, now_ms, actions);
    if (result != DAQ_DAC_RESULT_OK) ++g_daq_dac_refusals;
    apply_actions(actions);
  }
  daq_dac_ctrl_tick(&g_ctrl, has_state, state, now_ms, actions);
  apply_actions(actions);

  const uint8_t active = daq_dac_ctrl_active_mask(&g_ctrl);
  g_daq_dac_active_mask = active;
  const uint8_t changed = daq_dac_take_changes(&g_ctrl);

#if (DISABLE_SD_CARD == 0U)
  const uint64_t timestamp = daq_log_monotonic_ms();
  for (unsigned c = 0U; c < DAQ_DAC_CHANNELS; ++c)
    if (changed & (1U << c)) log_event(c, timestamp);
  if (active && (uint32_t)(now_ms - last_sd_ms) >= DAQ_DAC_SD_ROW_MS) {
    last_sd_ms = now_ms;
    for (unsigned c = 0U; c < DAQ_DAC_CHANNELS; ++c) {
      if ((active & (1U << c)) == 0U) continue;
      (void)sd_card_enqueue_csv_row(k_sd_names[c][SD_OUTPUT], timestamp, g_output_v[c], NULL);
      (void)sd_card_enqueue_csv_row(k_sd_names[c][SD_READBACK], timestamp, g_readback_v[c], NULL);
    }
  }
#endif

  const uint32_t period = active ? DAQ_DAC_STATUS_ACTIVE_MS : DAQ_DAC_STATUS_IDLE_MS;
  if (!changed && (uint32_t)(now_ms - last_status_ms) < period) return;
  last_status_ms = now_ms;
  float output[DAQ_DAC_CHANNELS], readback[DAQ_DAC_CHANNELS];
  for (unsigned c = 0U; c < DAQ_DAC_CHANNELS; ++c) { output[c] = g_output_v[c]; readback[c] = g_readback_v[c]; }
  const daq_dac_result_t gate = daq_dac_state_gate(has_state, state);
  const uint8_t flags = (g_hw_ready ? 1U : 0U) | (g_readback_ok ? 2U : 0U);
  uint8_t status[DAQ_DAC_STATUS_BYTES];
  daq_dac_status_encode(&g_ctrl, state,
      gate == DAQ_DAC_RESULT_OK ? 0U : gate == DAQ_DAC_RESULT_REFUSED_STATE ? 1U : 2U,
      flags, now_ms, output, readback, status);
#ifdef TELEMETRY_ENABLED
  if (log_telemetry_asynchronous(SEDS_DT_DAQ_DAC_STATUS, status, sizeof(status), 1U) == SEDS_OK)
    ++g_daq_dac_status_sent;
  else ++g_daq_dac_status_failed;
#endif
}
