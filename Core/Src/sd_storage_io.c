#include "sd_storage_io.h"
#include "fx_api.h"
#include "main.h"
#include <stddef.h>
#include <stdint.h>

extern SD_HandleTypeDef hsd1;
#define SD_IO_IRQ 1UL
#define SD_IO_DONE 2UL
#define SD_IO_ERROR 4UL
#define SD_IO_EVENTS (SD_IO_IRQ | SD_IO_DONE | SD_IO_ERROR)
#define SD_IO_TIMEOUT_MS 2000U
#define SD_IO_MAX_SECTORS 8U
static TX_EVENT_FLAGS_GROUP g_sd_io_events;
static uint8_t g_sd_io_initialized;
static volatile uint8_t g_sd_io_active, g_sd_io_reinit;
static volatile ULONG g_sd_io_result;
volatile uint32_t g_sd_dma_reads, g_sd_dma_writes, g_sd_dma_errors;
volatile uint32_t g_sd_dma_timeouts, g_sd_dma_recoveries, g_sd_dma_stale_irqs;
volatile uint32_t g_sd_dma_max_wait_ms;

static ULONG sd_io_ticks(uint32_t milliseconds)
{
  return (ULONG)(((uint64_t)milliseconds * TX_TIMER_TICKS_PER_SECOND + 999U) / 1000U);
}

static void sd_io_disarm(void)
{
  HAL_NVIC_DisableIRQ(SDMMC1_IRQn);
  hsd1.Instance->MASK = 0U;
  HAL_NVIC_ClearPendingIRQ(SDMMC1_IRQn);
}

static void sd_io_clear_events(void)
{
  ULONG discarded;
  (void)tx_event_flags_get(&g_sd_io_events, SD_IO_EVENTS, TX_OR_CLEAR, &discarded, TX_NO_WAIT);
}

/* No blocking HAL abort/CMD12 here. A reset must quiesce internal DMA before
 * returning buffer ownership; the caller then retries card initialization. */
static void sd_io_recover(void)
{
  sd_io_disarm();
  hsd1.Instance->IDMACTRL = SDMMC_DISABLE_IDMA;
  hsd1.Instance->DCTRL = 0U;
  __DSB();
  __HAL_RCC_SDMMC1_FORCE_RESET();
  __HAL_RCC_SDMMC1_RELEASE_RESET();
  __DSB();
  hsd1.ErrorCode |= HAL_SD_ERROR_TIMEOUT;
  hsd1.State = HAL_SD_STATE_RESET;
  hsd1.Context = SD_CONTEXT_NONE;
  g_sd_io_reinit = 1U;
  g_sd_io_active = 0U;
  ++g_sd_dma_recoveries;
  sd_io_clear_events();
}

UINT sd_storage_io_init(void)
{
  if (g_sd_io_initialized) return TX_SUCCESS;
  const UINT status = tx_event_flags_create(&g_sd_io_events, "SD DMA events");
  if (status != TX_SUCCESS) return status;
  g_sd_io_initialized = 1U;
  HAL_NVIC_DisableIRQ(SDMMC1_IRQn);
  return TX_SUCCESS;
}

uint8_t sd_storage_io_requires_reinit(void)
{
  return g_sd_io_reinit;
}

/* IRQ work is constant and contains no card command, polling, buffer copying
 * or formatting. HAL_SD_IRQHandler can send CMD12 synchronously, therefore it
 * must be deferred to the lower-priority writer rather than invoked here. */
void sd_storage_io_irq(void)
{
  HAL_NVIC_DisableIRQ(SDMMC1_IRQn);
  if (!g_sd_io_initialized || !g_sd_io_active) {
    ++g_sd_dma_stale_irqs;
    return;
  }
  (void)tx_event_flags_set(&g_sd_io_events, SD_IO_IRQ, TX_OR);
}

static void sd_io_completed(SD_HandleTypeDef *card, ULONG result)
{
  if (card != &hsd1 || !g_sd_io_active) return;
  g_sd_io_result |= result;
  (void)tx_event_flags_set(&g_sd_io_events, result, TX_OR);
}
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *card) { sd_io_completed(card, SD_IO_DONE); }
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *card) { sd_io_completed(card, SD_IO_DONE); }
void HAL_SD_ErrorCallback(SD_HandleTypeDef *card) { sd_io_completed(card, SD_IO_ERROR); }
void HAL_SD_AbortCallback(SD_HandleTypeDef *card) { sd_io_completed(card, SD_IO_ERROR); }

static UINT sd_io_ready(void)
{
  const uint32_t started = HAL_GetTick();
  do {
    const HAL_SD_CardStateTypeDef state = HAL_SD_GetCardState(&hsd1);
    if (state == HAL_SD_CARD_TRANSFER) return FX_SUCCESS;
    if (state == HAL_SD_CARD_ERROR) return FX_IO_ERROR;
    if ((uint32_t)(HAL_GetTick() - started) >= SD_IO_TIMEOUT_MS) {
      ++g_sd_dma_timeouts; return FX_IO_ERROR;
    }
    /* At least one millisecond at either1kHz or50kHz ThreadX tick rates. */
    tx_thread_sleep(sd_io_ticks(1U));
  } while (1);
}

static UINT sd_io_transfer(UCHAR *buffer, ULONG sector, ULONG count, uint8_t write)
{
  if (!g_sd_io_initialized || !buffer || ((uintptr_t)buffer & 31U) ||
      count == 0U || count > SD_IO_MAX_SECTORS) return FX_IO_ERROR;
#if defined(__arm__) || defined(__thumb__)
  /* U585 DCACHE1 bypasses internal SRAM. CPU copies and SDMMC IDMA share this
   * already aligned buffer coherently; no polling cache operations in IRQs. */
  if ((uintptr_t)buffer < 0x20000000UL ||
      (uintptr_t)buffer > 0x200C0000UL - (uintptr_t)count * 512U) return FX_IO_ERROR;
#endif
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  if (g_sd_io_active) { __set_PRIMASK(saved); return FX_IO_ERROR; }
  g_sd_io_active = 1U;
  __set_PRIMASK(saved);
  sd_io_disarm();
  sd_io_clear_events();
  if (g_sd_io_reinit) {
    if (hsd1.State != HAL_SD_STATE_READY || hsd1.ErrorCode != HAL_SD_ERROR_NONE) {
      g_sd_io_active = 0U; return FX_IO_ERROR;
    }
    g_sd_io_reinit = 0U; /* Caller successfully reinitialized the card. */
  }
  if (sd_io_ready() != FX_SUCCESS) { ++g_sd_dma_errors; sd_io_recover(); return FX_IO_ERROR; }
  __HAL_SD_CLEAR_FLAG(&hsd1, SDMMC_STATIC_FLAGS);
  g_sd_io_result = 0U;
  __DMB();
  const uint32_t started = HAL_GetTick();
  const HAL_StatusTypeDef launched = write ?
      HAL_SD_WriteBlocks_DMA(&hsd1, buffer, sector, count) :
      HAL_SD_ReadBlocks_DMA(&hsd1, buffer, sector, count);
  if (launched != HAL_OK) { ++g_sd_dma_errors; sd_io_recover(); return FX_IO_ERROR; }
  if (write) ++g_sd_dma_writes; else ++g_sd_dma_reads;
  /* disarm cleared stale pending state before launch. Do not clear again:
   * a fast transfer may already have raised its genuine completion IRQ. */
  HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
  for (;;) {
    const uint32_t elapsed = HAL_GetTick() - started;
    if (elapsed >= SD_IO_TIMEOUT_MS) { ++g_sd_dma_timeouts; sd_io_recover(); return FX_IO_ERROR; }
    ULONG events = 0U;
    const UINT waited = tx_event_flags_get(&g_sd_io_events, SD_IO_EVENTS, TX_OR_CLEAR,
        &events, sd_io_ticks(SD_IO_TIMEOUT_MS - elapsed));
    if (waited != TX_SUCCESS) { ++g_sd_dma_timeouts; sd_io_recover(); return FX_IO_ERROR; }
    if (events & SD_IO_IRQ) {
      /* TASK context only; vendor multiblock STOP polling cannot hold up
       * acquisition/network tasks or prolong an interrupt handler. */
      HAL_SD_IRQHandler(&hsd1);
    }
    if ((events | g_sd_io_result) & SD_IO_ERROR || hsd1.ErrorCode != HAL_SD_ERROR_NONE) {
      ++g_sd_dma_errors; sd_io_recover(); return FX_IO_ERROR;
    }
    if ((events | g_sd_io_result) & SD_IO_DONE) break;
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
  }
  sd_io_disarm();
  if (sd_io_ready() != FX_SUCCESS) { ++g_sd_dma_errors; sd_io_recover(); return FX_IO_ERROR; }
  __DMB();
  const uint32_t elapsed = HAL_GetTick() - started;
  if (elapsed > g_sd_dma_max_wait_ms) g_sd_dma_max_wait_ms = elapsed;
  g_sd_io_active = 0U;
  sd_io_clear_events();
  return FX_SUCCESS;
}

UINT sd_storage_io_read(UCHAR *buffer, ULONG sector, ULONG count)
{
  return sd_io_transfer(buffer, sector, count, 0U);
}
UINT sd_storage_io_write(const UCHAR *buffer, ULONG sector, ULONG count)
{
  return sd_io_transfer((UCHAR *)(uintptr_t)buffer, sector, count, 1U);
}
