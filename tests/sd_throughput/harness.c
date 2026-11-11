/* SD logging throughput model: the production writer (Core/Src/sd_card.c),
 * the production IDMA adapter (Core/Src/sd_storage_io.c) and real FileX run on
 * the host against a virtual-time SD card. Producers offer raw ADC records and
 * loadcell rows at fixed rates while the card injects long busy stalls.
 *
 * Time is virtual and single-CPU. The writer is the lowest-priority thread:
 * its CPU work (charged per formatted record) is stretched by the share other
 * threads take, and it blocks during card transfers and busy time while the
 * producers keep running. Each write command costs a fixed card program time
 * plus a per-byte term; PROG_FIXED_US defaults to the 6 ms that reproduces the
 * bench (about 1.7k records/s accepted at 2410 offered with the 4 KiB writer).
 * That figure is fitted, not measured: tests/test_sd_throughput.py also runs a
 * 2-15 ms range. Not cycle accurate; no claim about any individual card. */
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>

#include "sd_card.c"
#include "sd_storage_io.c"

#ifndef OFFER_RAW_HZ
#define OFFER_RAW_HZ 2410U
#endif
#ifndef OFFER_SCALAR_HZ
#define OFFER_SCALAR_HZ 500U
#endif
#ifndef STALL_MS
#define STALL_MS 400U
#endif
#ifndef STALL_PERIOD_MS
#define STALL_PERIOD_MS 4000U
#endif
#ifndef RUN_MS
#define RUN_MS 20000U
#endif
/* Card/bus model. 20 MHz 4-bit SDMMC = 10 bytes/us on the bus. */
#ifndef BUS_BYTES_PER_US
#define BUS_BYTES_PER_US 10U
#endif
#ifndef CMD_US
#define CMD_US 60U
#endif
#ifndef PROG_FIXED_US
#define PROG_FIXED_US 6000U
#endif
#ifndef PROG_BYTES_PER_US
#define PROG_BYTES_PER_US 10U
#endif
/* Writer CPU per formatted row and its share of the CPU while runnable. */
#ifndef RAW_RECORD_US
#define RAW_RECORD_US 40U
#endif
#ifndef CSV_ROW_US
#define CSV_ROW_US 60U
#endif
#ifndef WRITER_CPU_SHARE_PCT
#define WRITER_CPU_SHARE_PCT 70U
#endif
#define CARD_SECTORS (UINT64_C(4) * 1024U * 1024U * 1024U / 512U)

TX_THREAD sd_writer_thread;
uint32_t g_model_primask;
SD_HandleTypeDef hsd1;
static SDMMC_TypeDef g_sdmmc;
static uint64_t g_now_us, g_end_us, g_offer_start_us, g_offer_stop_us;
static jmp_buf g_stop;
static uint8_t g_drained, g_warmup_sent;
static uint32_t g_backlog_peak;
static uint8_t *g_card;

/* Card state */
static uint64_t g_xfer_end_us, g_busy_until_us, g_next_stall_us;
static uint8_t g_xfer_pending, g_xfer_write;
static uint32_t g_cmd_writes, g_cmd_reads, g_sectors_written, g_stalls;
static uint32_t g_write_hist[8]; /* sectors per write command: 1,2-7,8,9-15,16-31,32-63,64-127,128+ */

/* Producers */
static uint64_t g_next_raw_us, g_next_scalar_us, g_raw_offered, g_scalar_offered, g_raw_seq;
static uint64_t g_raw_credit_milli;
static uint32_t g_charged_raw, g_charged_csv;

static void producers_until(uint64_t t);
static void maybe_start(void);
static void advance_to(uint64_t t)
{
  maybe_start();
  if (t <= g_now_us) return;
  producers_until(t);
  g_now_us = t;
  if (g_now_us >= g_end_us) longjmp(g_stop, 1);
}
static void advance(uint64_t us) { advance_to(g_now_us + us); }

/* Charge the writer's formatting CPU since the last observation point. */
static void settle_cpu(void)
{
  if (g_model_primask) return;
  const uint32_t raw = g_sd_raw_records_written_count, csv = g_sd_csv_rows_written_count;
  const uint64_t work = (uint64_t)(raw - g_charged_raw) * RAW_RECORD_US +
      (uint64_t)(csv - g_charged_csv) * CSV_ROW_US;
  g_charged_raw = raw; g_charged_csv = csv;
  if (work) advance(work * 100U / WRITER_CPU_SHARE_PCT);
}

uint32_t HAL_GetTick(void) { settle_cpu(); return (uint32_t)(g_now_us / 1000U); }
uint64_t board_uptime_ticks(void) { return g_now_us / 1000U; }
ULONG tx_time_get(void) { settle_cpu(); return (ULONG)(g_now_us / 1000U); }
TX_THREAD *tx_thread_identify(void) { return &sd_writer_thread; }
UINT tx_thread_sleep(ULONG ticks)
{
  settle_cpu();
  if (ticks == 1U && g_drained && !g_sd_io_active && !g_xfer_pending) longjmp(g_stop, 2);
  /* ThreadX wakes on a tick boundary: sleep(1) ends at the next tick. */
  advance_to((g_now_us / 1000U + ticks) * 1000U);
  return TX_SUCCESS;
}

/* ---- ThreadX objects (single CPU, never contended) ---- */
UINT tx_queue_create(TX_QUEUE *q, CHAR *name, UINT size, VOID *start, ULONG bytes)
{
  (void)name; q->tx_queue_message_size = size;
  q->tx_queue_start = start; q->tx_queue_capacity = (UINT)(bytes / (size * sizeof(ULONG)));
  q->tx_queue_end = q->tx_queue_start + q->tx_queue_capacity * size;
  q->tx_queue_read = q->tx_queue_write = q->tx_queue_start; q->tx_queue_enqueued = 0U;
  return TX_SUCCESS;
}
UINT tx_queue_send(TX_QUEUE *q, VOID *message, ULONG wait)
{
  (void)wait;
  if (q->tx_queue_enqueued == q->tx_queue_capacity) return TX_QUEUE_FULL;
  memcpy(q->tx_queue_write, message, q->tx_queue_message_size * sizeof(ULONG));
  q->tx_queue_write += q->tx_queue_message_size;
  if (q->tx_queue_write == q->tx_queue_end) q->tx_queue_write = q->tx_queue_start;
  ++q->tx_queue_enqueued; return TX_SUCCESS;
}
UINT tx_queue_receive(TX_QUEUE *q, VOID *message, ULONG wait)
{
  (void)wait;
  if (!q->tx_queue_enqueued) return TX_QUEUE_EMPTY;
  memcpy(message, q->tx_queue_read, q->tx_queue_message_size * sizeof(ULONG));
  q->tx_queue_read += q->tx_queue_message_size;
  if (q->tx_queue_read == q->tx_queue_end) q->tx_queue_read = q->tx_queue_start;
  --q->tx_queue_enqueued; return TX_SUCCESS;
}
/* Block count follows ThreadX on Cortex-M: one 4-byte owner word per block.
 * Each host block carries its owning pool in a 16-byte header. */
UINT tx_block_pool_create(TX_BLOCK_POOL *p, CHAR *name, ULONG block, VOID *start, ULONG bytes)
{
  (void)name; (void)start; block = (block + 3U) & ~3UL;
  p->tx_block_pool_size = bytes; p->tx_block_pool_block_size = block;
  p->tx_block_pool_total = p->tx_block_pool_available = (UINT)(bytes / (block + 4U));
  p->free_list = calloc(p->tx_block_pool_total, sizeof(*p->free_list));
  for (UINT i = 0; i < p->tx_block_pool_total; ++i) {
    UCHAR *raw = malloc(block + 16U);
    *(TX_BLOCK_POOL **)(void *)raw = p; p->free_list[i] = raw + 16U;
  }
  p->free_top = p->tx_block_pool_total;
  return TX_SUCCESS;
}
UINT tx_block_allocate(TX_BLOCK_POOL *p, VOID **block, ULONG wait)
{
  (void)wait;
  if (!p->free_top) return TX_NO_MEMORY;
  *block = p->free_list[--p->free_top]; --p->tx_block_pool_available;
  return TX_SUCCESS;
}
UINT tx_block_release(VOID *block)
{
  TX_BLOCK_POOL *p = *(TX_BLOCK_POOL **)(void *)((UCHAR *)block - 16U);
  p->free_list[p->free_top++] = block; ++p->tx_block_pool_available;
  return TX_SUCCESS;
}
UINT tx_event_flags_create(TX_EVENT_FLAGS_GROUP *g, CHAR *name) { (void)name; g->flags = 0U; return TX_SUCCESS; }
UINT tx_event_flags_set(TX_EVENT_FLAGS_GROUP *g, ULONG flags, UINT option)
{ (void)option; g->flags |= flags; return TX_SUCCESS; }
static void card_complete_transfer(void);
UINT tx_event_flags_get(TX_EVENT_FLAGS_GROUP *g, ULONG requested, UINT option, ULONG *actual, ULONG wait)
{
  if (!(g->flags & requested) && wait != TX_NO_WAIT) {
    settle_cpu();
    card_complete_transfer(); /* Only the IDMA wait blocks on an event group. */
  }
  if (!(g->flags & requested)) return TX_NO_EVENTS;
  *actual = g->flags & requested;
  if (option == TX_OR_CLEAR) g->flags &= ~requested;
  return TX_SUCCESS;
}

/* ---- SD card ---- */
HAL_StatusTypeDef HAL_SD_Init(SD_HandleTypeDef *h)
{
  h->Instance = &g_sdmmc; h->State = HAL_SD_STATE_READY; h->ErrorCode = 0U;
  h->SdCard.LogBlockNbr = (uint32_t)CARD_SECTORS; h->SdCard.LogBlockSize = 512U;
  h->CID[0] = 0x1234U;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_SD_DeInit(SD_HandleTypeDef *h) { h->State = HAL_SD_STATE_RESET; return HAL_OK; }
HAL_StatusTypeDef HAL_SD_ConfigWideBusOperation(SD_HandleTypeDef *h, uint32_t w) { (void)h; (void)w; return HAL_OK; }
HAL_StatusTypeDef HAL_SD_GetCardInfo(SD_HandleTypeDef *h, HAL_SD_CardInfoTypeDef *info) { *info = h->SdCard; return HAL_OK; }
HAL_SD_CardStateTypeDef HAL_SD_GetCardState(SD_HandleTypeDef *h)
{
  (void)h; settle_cpu(); advance(CMD_US);
  return g_now_us < g_busy_until_us ? HAL_SD_CARD_PROGRAMMING : HAL_SD_CARD_TRANSFER;
}
static uint32_t write_bucket(uint32_t n)
{
  return n == 1 ? 0 : n < 8 ? 1 : n == 8 ? 2 : n < 16 ? 3 : n < 32 ? 4 : n < 64 ? 5 : n < 128 ? 6 : 7;
}
HAL_StatusTypeDef HAL_SD_WriteBlocks_DMA(SD_HandleTypeDef *h, uint8_t *data, uint32_t sector, uint32_t count)
{
  (void)h;
  if ((uint64_t)sector + count > CARD_SECTORS || g_xfer_pending) return HAL_ERROR;
  memcpy(g_card + (uint64_t)sector * 512U, data, (size_t)count * 512U);
  g_xfer_end_us = g_now_us + CMD_US + (uint64_t)count * 512U / BUS_BYTES_PER_US;
  uint64_t busy = PROG_FIXED_US + (uint64_t)count * 512U / PROG_BYTES_PER_US;
  if (g_next_stall_us && g_now_us >= g_next_stall_us) {
    busy += (uint64_t)STALL_MS * 1000U; ++g_stalls;
    g_next_stall_us += (uint64_t)STALL_PERIOD_MS * 1000U;
  }
  g_busy_until_us = g_xfer_end_us + busy;
  g_xfer_pending = 1U; g_xfer_write = 1U;
  ++g_cmd_writes; g_sectors_written += count; ++g_write_hist[write_bucket(count)];
  return HAL_OK;
}
HAL_StatusTypeDef HAL_SD_ReadBlocks_DMA(SD_HandleTypeDef *h, uint8_t *data, uint32_t sector, uint32_t count)
{
  (void)h;
  if ((uint64_t)sector + count > CARD_SECTORS || g_xfer_pending) return HAL_ERROR;
  memcpy(data, g_card + (uint64_t)sector * 512U, (size_t)count * 512U);
  g_xfer_end_us = g_now_us + CMD_US + (uint64_t)count * 512U / BUS_BYTES_PER_US;
  g_xfer_pending = 1U; g_xfer_write = 0U; ++g_cmd_reads;
  return HAL_OK;
}
static void card_complete_transfer(void)
{
  if (!g_xfer_pending) return;
  advance_to(g_xfer_end_us);
  sd_storage_io_irq();
}
void HAL_SD_IRQHandler(SD_HandleTypeDef *h)
{
  if (!g_xfer_pending) return;
  g_xfer_pending = 0U;
  if (g_xfer_write) HAL_SD_TxCpltCallback(h); else HAL_SD_RxCpltCallback(h);
}
void HAL_NVIC_EnableIRQ(IRQn_Type irq) { (void)irq; }
void HAL_NVIC_DisableIRQ(IRQn_Type irq) { (void)irq; }
void HAL_NVIC_ClearPendingIRQ(IRQn_Type irq) { (void)irq; }

/* ---- Board services the writer reads ---- */
daq_calibration_t daq_calibration_current(void) { return g_sd_calibration; }
bool sd_file_stream_claim(sd_file_request_t *request, uint32_t *ticket) { (void)request; (void)ticket; return false; }
uint8_t *sd_file_stream_block(void) { return NULL; }
static uint8_t g_dl_handle[sizeof(FX_FILE)];
void *sd_file_stream_handle(void) { return g_dl_handle; }
void sd_file_stream_handle_open(bool open) { (void)open; }
void sd_file_stream_finish(uint32_t ticket, const sd_file_result_t *result) { (void)ticket; (void)result; }
bool sd_file_stream_session_active(void) { return false; }
bool sd_file_stream_state_allowed(void) { return false; }

/* ---- Producers: the acquisition thread every 2 ms, loadcell rows at 250 Hz each ---- */
static const uint8_t k_channels[] = {0U, 1U, 6U, 9U, 2U, 3U};
static void offer_raw(uint64_t t)
{
  g_raw_credit_milli += (uint64_t)OFFER_RAW_HZ * DAQ_ACQUISITION_PERIOD_MS;
  uint16_t count = (uint16_t)(g_raw_credit_milli / 1000U);
  g_raw_credit_milli -= (uint64_t)count * 1000U;
  if (!count) return;
  sd_raw_adc_record_t batch[SD_RAW_BATCH_MAX];
  if (count > SD_RAW_BATCH_MAX) count = SD_RAW_BATCH_MAX;
  for (uint16_t i = 0; i < count; ++i) {
    const uint64_t seq = ++g_raw_seq;
    batch[i] = (sd_raw_adc_record_t){0U, (uint32_t)(t / 1000U), (uint32_t)seq,
        (int32_t)(4123456 - (int32_t)(seq % 9000U) * 917), 0.123456789f + (float)(seq % 977U) * 1.0e-5f,
        12.3456789f + (float)(seq % 613U) * 0.01f, 25.0625f, 41234, k_channels[seq % sizeof k_channels]};
  }
  g_raw_offered += count;
  if (sd_raw_backlog_records() > g_backlog_peak) g_backlog_peak = sd_raw_backlog_records();
  (void)sd_card_enqueue_raw_adc_samples(batch, count, &g_sd_calibration);
}
static void producers_until(uint64_t t)
{
  if (!g_offer_start_us) return;
  const uint64_t saved_now = g_now_us;
  while (1) {
    const uint64_t next = g_next_raw_us < g_next_scalar_us ? g_next_raw_us : g_next_scalar_us;
    if (next > t || next >= g_offer_stop_us) break;
    g_now_us = next > saved_now ? next : saved_now;
    const uint32_t mask = g_model_primask; g_model_primask = 0U;
    if (next == g_next_raw_us) {
      offer_raw(next); g_next_raw_us += DAQ_ACQUISITION_PERIOD_MS * 1000U;
    } else {
      const uint64_t period = 2U * 1000000U / (OFFER_SCALAR_HZ ? OFFER_SCALAR_HZ : 1U);
      (void)sd_card_enqueue_csv_row("kg1000_network", g_now_us / 1000U, 12.5f, &g_sd_calibration);
      (void)sd_card_enqueue_csv_row("kg50_network", g_now_us / 1000U, 3.25f, &g_sd_calibration);
      g_scalar_offered += 2U; g_next_scalar_us += period;
    }
    g_model_primask = mask;
  }
  g_now_us = saved_now;
}

/* ---- Verification of the bytes on the card through FileX ---- */
static int verify_log(uint64_t *rows, uint64_t *raw_rows, uint64_t *bad)
{
  FX_FILE file; ULONG got;
  if (fx_file_open(&g_sd_media, &file, g_sd_regular_log.name, FX_OPEN_FOR_READ) != FX_SUCCESS) return -1;
  static char chunk[65536]; char line[512]; size_t used = 0U; uint64_t expect = 1U;
  *rows = *raw_rows = *bad = 0U;
  uint8_t header = 1U;
  while (fx_file_read(&file, chunk, sizeof chunk, &got) == FX_SUCCESS && got) {
    for (ULONG i = 0; i < got; ++i) {
      if (used + 1U >= sizeof line) { ++*bad; used = 0U; continue; }
      line[used++] = chunk[i];
      if (chunk[i] != '\n') continue;
      line[used] = '\0'; used = 0U;
      if (header) { header = 0U; if (strcmp(line, SD_CSV_HEADER) != 0) ++*bad; continue; }
      if (line[0] == '#') continue;
      ++*rows;
      if (strlen(line) < 3 || line[strlen(line) - 2] != '\r') { ++*bad; continue; }
      unsigned commas = 0U; for (char *p = line; *p; ++p) commas += *p == ',';
      if (commas != 12U) { ++*bad; continue; }
      if (strstr(line, ",kg1000_network,") || strstr(line, ",kg50_network,")) continue;
      /* Raw rows keep their acquisition sequence in the final column. */
      const char *last = strrchr(line, ',');
      const uint64_t seq = strtoull(last + 1, NULL, 10);
      ++*raw_rows;
      if (seq < expect) ++*bad; /* duplicated or reordered */
      expect = seq + 1U;
    }
  }
  if (used) ++*bad; /* torn final line */
  /* Optional copy of the exact bytes for format comparison. */
  const char *dump = getenv("SD_THROUGHPUT_DUMP");
  if (dump && fx_file_seek(&file, 0U) == FX_SUCCESS) {
    FILE *out = fopen(dump, "wb");
    while (out && fx_file_read(&file, chunk, sizeof chunk, &got) == FX_SUCCESS && got) fwrite(chunk, 1U, got, out);
    if (out) fclose(out);
  }
  (void)fx_file_close(&file);
  return 0;
}

static uint32_t g_raw_dropped0, g_line_dropped0, g_raw_enq0, g_drain_flush;
static uint64_t g_drained_us;
static void maybe_start(void)
{
  /* Warm-up: one row opens the session file (and, on this freshly formatted
   * card, creates its directory: a one-time 64-sector cluster clear). The
   * measured load starts once that file is open, as on a provisioned card. */
  if (!g_offer_start_us && g_sd_ready && !g_warmup_sent) {
    g_warmup_sent = 1U;
    const uint32_t mask = g_model_primask; g_model_primask = 0U;
    (void)sd_card_enqueue_csv_row("kg1000_network", g_now_us / 1000U, 1.0f, &g_sd_calibration);
    g_model_primask = mask;
  }
  if (!g_offer_start_us && g_sd_ready && g_sd_regular_log.open && g_sd_queue.tx_queue_enqueued == 0U) {
    g_offer_start_us = g_now_us + 100000U;
    g_offer_stop_us = g_offer_start_us + (uint64_t)RUN_MS * 1000U;
    g_next_raw_us = g_next_scalar_us = g_offer_start_us;
    if (!OFFER_SCALAR_HZ) g_next_scalar_us = UINT64_MAX;
    if (STALL_MS) g_next_stall_us = g_offer_start_us + (uint64_t)STALL_PERIOD_MS * 1000U / 2U;
    g_raw_dropped0 = g_sd_raw_records_dropped_count; g_line_dropped0 = g_sd_line_drop_count;
    g_raw_enq0 = g_sd_raw_records_enqueued_count;
  }
  if (g_offer_start_us && g_now_us >= g_offer_stop_us && !g_drained) {
    const uint8_t empty = g_sd_raw_queue.tx_queue_enqueued == 0U && !g_sd_raw_pending &&
        g_sd_queue.tx_queue_enqueued == 0U && !g_sd_line_pending &&
        g_sd_raw_records_enqueued_count == g_sd_raw_records_written_count + g_sd_raw_records_io_lost_count;
    if (empty && !g_drained_us) { g_drained_us = g_now_us; g_drain_flush = g_sd_flush_count; }
    if (g_drained_us && g_sd_flush_count > g_drain_flush + 1U) g_drained = 1U;
  }
}

int main(void)
{
  g_card = mmap(NULL, (size_t)(CARD_SECTORS * 512U), PROT_READ | PROT_WRITE,
                MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
  if (g_card == MAP_FAILED) return 2;
  hsd1.Instance = &g_sdmmc;
  g_end_us = (uint64_t)(RUN_MS + 60000U) * 1000U;
  if (sd_card_init(NULL) != TX_SUCCESS) return 3;
  const int phase = setjmp(g_stop);
  if (phase == 0) sd_card_writer_thread_entry(0U);
  const uint64_t offered_us = g_offer_stop_us - g_offer_start_us;
  const uint32_t raw_dropped = g_sd_raw_records_dropped_count - g_raw_dropped0;
  const uint32_t line_dropped = g_sd_line_drop_count - g_line_dropped0;
  const uint32_t accepted = g_sd_raw_records_enqueued_count - g_raw_enq0;
  printf("drained=%u\n", phase == 2);
  printf("offered_raw=%llu offered_raw_per_s=%.0f\n", (unsigned long long)g_raw_offered,
         g_raw_offered * 1e6 / (double)offered_us);
  printf("accepted_raw=%u raw_dropped=%u line_dropped=%u essential_telemetry_dropped=%u\n",
         accepted, raw_dropped, line_dropped, g_sd_essential_telemetry_dropped);
  printf("written_raw=%u written_csv=%u io_lost=%u write_errors=%u\n", g_sd_raw_records_written_count,
         g_sd_csv_rows_written_count, g_sd_raw_records_io_lost_count, g_sd_write_error_count);
  printf("written_raw_per_s=%.0f\n", (g_sd_raw_records_written_count) * 1e6 /
         (double)((g_drained_us ? g_drained_us : g_now_us) - g_offer_start_us));
  printf("raw_slots_peak=%u raw_pool_blocks=%u stalls=%u max_write_ms=%u max_flush_ms=%u flushes=%u\n",
         g_sd_raw_slots_peak, g_sd_raw_pool.tx_block_pool_total, g_stalls, g_sd_max_write_ms,
         g_sd_max_flush_ms, g_sd_flush_count);
  printf("raw_backlog_peak=%u optional_admit_limit=%u\n", g_backlog_peak, (unsigned)SD_OPTIONAL_RAW_ADMIT_LIMIT);
  printf("cmd_writes=%u cmd_reads=%u sectors_written=%u sectors_per_write=%.1f\n", g_cmd_writes, g_cmd_reads,
         g_sectors_written, g_cmd_writes ? (double)g_sectors_written / g_cmd_writes : 0.0);
  printf("write_hist=%u,%u,%u,%u,%u,%u,%u,%u\n", g_write_hist[0], g_write_hist[1], g_write_hist[2],
         g_write_hist[3], g_write_hist[4], g_write_hist[5], g_write_hist[6], g_write_hist[7]);
  if (phase == 2) {
    uint64_t rows, raw_rows, bad;
    const int v = verify_log(&rows, &raw_rows, &bad);
    printf("verify=%d file_rows=%llu file_raw_rows=%llu file_bad=%llu\n", v, (unsigned long long)rows,
           (unsigned long long)raw_rows, (unsigned long long)bad);
  }
  return 0;
}
