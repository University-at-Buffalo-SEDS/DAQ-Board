#include "sd_card.h"
#include "daq_chamber_pressure.h"
#include "daq_rates.h"

#include "fx_api.h"
#include "main.h"
#include "daq_log_time.h"
#include "sd_float_format.h"
#include "sd_record_format.h"
#include "sd_raw_record_format.h"
#include "sd_file_limit.h"
#include "sd_storage_config.h"
#include "sd_storage_io.h"
#include "daq_timestamp.h"
#include "sd_calendar.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

extern SD_HandleTypeDef hsd1;
UINT _fx_partition_offset_calculate(void *partition_sector, UINT partition,
                                    ULONG *partition_start,
                                    ULONG *partition_size);

/* Report rows also accumulate while the card stalls; 24 slots overflowed
 * during ordinary flushes even when raw recording kept up. */
#define SD_QUEUE_DEPTH 1024U
#define SD_RAW_QUEUE_DEPTH 1024U
#define SD_RAW_BATCH_MAX DAQ_RAW_BATCH_CAPACITY
#define SD_LINE_MAX 384U
#define SD_MEDIA_CACHE_SIZE (8U * 512U)
#define SD_TRANSFER_SECTORS 8U
#define SD_SECTOR_SIZE 512U
#define SD_FLAG_FLUSH_REQUEST 0x00000001UL
#define SD_FLAG_POWER_LOSS 0x00000002UL
#define SD_PROVISION_MARKER "SEDSDAQ.ID"

#if defined(__GNUC__)
#define SD_ALIGN_32 __attribute__((aligned(32)))
#else
#define SD_ALIGN_32
#endif

typedef struct
{
  uint16_t len;
  uint32_t route_epoch;
  daq_calibration_t calibration;
  uint64_t session;
  char line[];
} sd_line_slot_t;

typedef struct
{
  uint16_t count, capacity;
  uint32_t route_epoch;
  uint64_t uptime_reference;
  daq_calibration_t calibration;
  uint64_t session;
  sd_raw_adc_record_t samples[];
} sd_raw_slot_t;

/* Compact acquisition rows share immutable calibration/session metadata. The
 * writer expands them to the exact same CSV only after dequeueing ownership. */
#define SD_SCALAR_SLOT_TAG UINT16_MAX
#define SD_SCALAR_BATCH_MAX 32U
#define SD_RAW_SLOT_CAPACITY ((SD_RAW_BATCH_MAX) > 32U ? (SD_RAW_BATCH_MAX) : 32U)
typedef struct {
  uint64_t network_ms;
  uint32_t monotonic_ms, sequence_delta;
  float value;
  uint8_t sensor;
} sd_scalar_record_t;
typedef struct {
  uint16_t len;
  uint32_t route_epoch;
  daq_calibration_t calibration;
  uint64_t session;
  uint16_t count, capacity;
  uint64_t uptime_reference, sequence_base;
  sd_scalar_record_t records[];
} sd_scalar_slot_t;
_Static_assert(offsetof(sd_scalar_slot_t, count) == offsetof(sd_line_slot_t, line),
               "Scalar and CSV slots must share their routing prefix");
/* Every pool allocation is a constant-time fixed-block pop. Reserve padding
 * for8-byte alignment even though ThreadX guarantees only ULONG alignment. */
#define SD_RAW_BLOCK_PAYLOAD (offsetof(sd_raw_slot_t, samples) + SD_RAW_SLOT_CAPACITY * sizeof(sd_raw_adc_record_t))
#define SD_SCALAR_BLOCK_PAYLOAD (offsetof(sd_scalar_slot_t, records) + SD_SCALAR_BATCH_MAX * sizeof(sd_scalar_record_t) + 1U)
#define SD_CSV_BLOCK_PAYLOAD (offsetof(sd_line_slot_t, line) + SD_LINE_MAX + 1U)
#define SD_LINE_BLOCK_PAYLOAD ((SD_SCALAR_BLOCK_PAYLOAD > SD_CSV_BLOCK_PAYLOAD) ? SD_SCALAR_BLOCK_PAYLOAD : SD_CSV_BLOCK_PAYLOAD)
#define SD_POOL_ALIGNMENT_RESERVE (sizeof(void *) + 7U)



static TX_QUEUE g_sd_queue;
static ULONG g_sd_queue_storage[SD_QUEUE_DEPTH];
static TX_QUEUE g_sd_raw_queue;
static ULONG g_sd_raw_queue_storage[SD_RAW_QUEUE_DEPTH];
/* Only a queued producer tail may be extended. The writer clears ownership
 * atomically when it removes that exact slot from the queue. */
static sd_raw_slot_t *g_sd_raw_append_tail;
static sd_scalar_slot_t *g_sd_scalar_append_tail;
static const char *const g_sd_scalar_sensors[] = {
  "kg1000_network", "kg50_network", "kg50_p6_network_volts",
  "chamber_pressure_raw_volts_psi", "valve_pressure_psi"
};
/* The receive callback copies one fixed message; CSV and file operations stay
 * in the SD writer. Include the acquisition's session across a launch change. */
typedef struct {
  pressure_sd_batch_t batch;
  uint64_t session;
  uint32_t route_epoch;
} sd_pressure_message_t;
#define SD_PRESSURE_MESSAGE_WORDS ((sizeof(sd_pressure_message_t) + sizeof(ULONG) - 1U) / sizeof(ULONG))
_Static_assert(SD_PRESSURE_MESSAGE_WORDS <= 16U, "ThreadX pressure message bounds");
static TX_QUEUE g_sd_pressure_queue;
/* 800 pressure samples cover3.2s of SD-card programming/flush latency. */
#define SD_PRESSURE_QUEUE_BATCHES 200U
static ULONG g_sd_pressure_queue_storage[SD_PRESSURE_QUEUE_BATCHES * SD_PRESSURE_MESSAGE_WORDS];
static uint8_t g_sd_pressure_pending;
static uint32_t g_sd_pressure_pending_epoch;
static uint8_t g_sd_line_pending;
static uint32_t g_sd_line_pending_epoch;
volatile uint32_t g_sd_pressure_records_enqueued;
volatile uint32_t g_sd_pressure_records_dropped;
volatile uint32_t g_sd_pressure_records_written;
volatile uint32_t g_sd_pressure_records_flushed;

static TX_EVENT_FLAGS_GROUP g_sd_flags;
/* Numeric acquisition rows use compact batches instead of full CSV strings.
 * Reinvest that RAM in raw acquisition and pressure stall coverage. */
static TX_BLOCK_POOL g_sd_line_pool, g_sd_raw_pool;
static UCHAR g_sd_line_pool_storage[62U * 1024U] SD_ALIGN_32;
static UCHAR g_sd_raw_pool_storage[332U * 1024U] SD_ALIGN_32;
/* Keep at least the acquisition configuration's original minimum reserve.
 * The fixed-block pool additionally covers measured multi-second
 * card stalls at the observed2.5ksamples/s rate when queued tails coalesce. */
_Static_assert(DAQ_CHAMBER_SCAN_MASK == 0U ||
    ((DAQ_SD_RAW_BUFFER_MS * DAQ_ADC_READ_RATE_HZ + 999U) / 1000U) * sizeof(sd_raw_adc_record_t) +
    DAQ_SD_RAW_QUEUE_DEPTH * (offsetof(sd_raw_slot_t, samples) + 16U) +
    SD_RAW_BATCH_MAX * sizeof(sd_raw_adc_record_t) <= sizeof(g_sd_raw_pool_storage),
    "Chamber sample rate exceeds the existing800ms SD raw byte reserve");
_Static_assert(sizeof(g_sd_line_pool_storage) <= 180U * 1024U, "SD telemetry RAM budget");
_Static_assert(sizeof(g_sd_raw_pool_storage) <= 340U * 1024U, "SD raw RAM budget");

static FX_MEDIA g_sd_media;
static FX_FILE g_sd_file;
static FX_FILE g_sd_telemetry_file;
static uint8_t g_sd_raw_reserve_disabled, g_sd_telemetry_reserve_disabled;
static uint8_t g_telemetry_file_open;
static daq_calibration_t g_sd_telemetry_calibration;
static char g_sd_telemetry_filename[64];
static UCHAR g_sd_media_cache[SD_MEDIA_CACHE_SIZE] SD_ALIGN_32;
static UCHAR g_sd_transfer_buffer[SD_TRANSFER_SECTORS * SD_SECTOR_SIZE] SD_ALIGN_32;
static UCHAR g_sd_write_buffer[4096U];
static size_t g_sd_write_buffer_len = 0U;
static UCHAR g_sd_telemetry_write_buffer[4096U];
static size_t g_sd_telemetry_write_buffer_len = 0U;
static char g_sd_filename[64];
volatile uint32_t g_sd_ready = 0U;
/* Debugger/simulator progress: 1 card init, 2 bus setup, 3 mount,
 * 4 open log, 5 ready, 6 formatting. */
volatile uint32_t g_sd_init_stage = 0U;
static uint8_t g_sd_hardware_ready = 0U;
static uint8_t g_sd_media_mounted;
static uint32_t g_sd_mounted_cid[4], g_sd_mounted_blocks;
volatile uint32_t g_sd_card_identity_mismatches;
static uint8_t g_sd_manual_no_format;
static volatile uint32_t g_sd_retry_pending_token, g_sd_retry_active_token;
static uint32_t g_sd_retry_accepted_token, g_sd_retry_completed_token;
static uint32_t g_sd_retry_recent[8], g_sd_retry_recent_next;
static uint64_t g_sd_last_sector_write_ms, g_sd_last_media_flush_ms;
static uint8_t g_sd_write_progress_valid, g_sd_flush_progress_valid;
static uint8_t g_file_open = 0U;
static daq_calibration_t g_sd_calibration = {1.0f, 0.0f, 1.0f, 0.0f, {0.0f, 1.0f}};
static volatile uint32_t g_sd_calibration_generation = 1U;
static daq_calibration_t g_sd_open_calibration;
static uint8_t g_sd_services_initialized = 0U;
static uint64_t g_sd_session, g_sd_last_launch_session, g_sd_close_unix_ms;
static uint64_t g_sd_raw_session, g_sd_telemetry_session;
static uint64_t g_sd_close_local_ms;
static uint8_t g_sd_launch_closed, g_sd_deadline_anchored;
static uint8_t g_sd_launch_flush_pending;
/* Orders session transitions across the independent CSV and pressure queues.
 * A late pressure batch must drain before a newer ordinary CSV row rotates
 * their shared file. These are bounded queue generations, not sample rates. */
static uint32_t g_sd_route_epoch;

/* Caller holds the short IRQ lock; only the local extended uptime is read.
 * No router/network-clock lock is acquired from a receive callback. */
static uint64_t sd_route_session_locked(uint64_t local)
{
  if (!g_sd_launch_closed && g_sd_deadline_anchored && local >= g_sd_close_local_ms) {
    g_sd_launch_closed = 1U;
    ++g_sd_route_epoch;
  }
  return g_sd_deadline_anchored && !g_sd_launch_closed ? g_sd_session : 0U;
}

static uint64_t sd_run_snapshot(void)
{
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  const uint64_t result = sd_route_session_locked(daq_log_monotonic_ms());
  __set_PRIMASK(mask);
  return result;
}

static uint8_t sd_launch_finished(void)
{
  /* Map UTC once; later loss/correction cannot extend the monotonic cutoff.
   * Without a UTC mapping, ordinary logging continues until the session can
   * be dated honestly. Retained expired sessions never open a launch file. */
  uint32_t mask = __get_PRIMASK(); __disable_irq();
  const uint8_t needs_utc = g_sd_close_unix_ms != 0U && !g_sd_deadline_anchored && !g_sd_launch_closed;
  __set_PRIMASK(mask);
  const uint64_t utc = needs_utc ? daq_log_unix_ms() : 0U;
  const uint64_t local = daq_log_monotonic_ms();
  mask = __get_PRIMASK(); __disable_irq();
  if (g_sd_close_unix_ms != 0U && !g_sd_deadline_anchored && !g_sd_launch_closed && utc != 0U) {
    g_sd_close_local_ms = local + (utc < g_sd_close_unix_ms ? g_sd_close_unix_ms - utc : 0U);
    g_sd_deadline_anchored = 1U;
    ++g_sd_route_epoch;
  }
  (void)sd_route_session_locked(local);
  const uint8_t finished = g_sd_launch_closed;
  __set_PRIMASK(mask);
  return finished;
}

void sd_card_set_launch_clock(uint64_t session, uint64_t close_unix_ms)
{
  const uint64_t now = session != 0U ? daq_log_unix_ms() : 0U;
  const uint64_t local = daq_log_monotonic_ms();
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  const uint64_t before = sd_route_session_locked(local);
  if (session == 0U) {
    g_sd_session = 0U; g_sd_close_unix_ms = 0U;
    g_sd_close_local_ms = 0U; g_sd_launch_closed = 0U; g_sd_deadline_anchored = 0U;
  } else if (session > g_sd_last_launch_session) {
    g_sd_last_launch_session = session;
    g_sd_session = session; g_sd_close_unix_ms = close_unix_ms;
    g_sd_deadline_anchored = now != 0U;
    g_sd_launch_closed = now != 0U && now >= close_unix_ms;
    g_sd_close_local_ms = now != 0U ? local + (now < close_unix_ms ? close_unix_ms - now : 0U) : 0U;
  } else if (g_sd_session == session && !g_sd_launch_closed && close_unix_ms != g_sd_close_unix_ms) {
    /* Receiver revision ordering has established a genuinely newer T0. Move
     * the existing monotonic mapping by the absolute delta, never by a fresh
     * wall/UTC reading. A closed/tombstoned session never enters this branch. */
    const uint64_t prior = g_sd_close_unix_ms;
    g_sd_close_unix_ms = close_unix_ms;
    if (g_sd_deadline_anchored) {
      if (close_unix_ms < prior) {
        const uint64_t earlier = prior - close_unix_ms;
        g_sd_close_local_ms = g_sd_close_local_ms > earlier ? g_sd_close_local_ms - earlier : 0U;
      } else {
        const uint64_t later = close_unix_ms - prior;
        if (UINT64_MAX - g_sd_close_local_ms < later) g_sd_launch_closed = 1U;
        else g_sd_close_local_ms += later;
      }
    } else if (now != 0U) {
      g_sd_close_local_ms = local + (now < close_unix_ms ? close_unix_ms - now : 0U);
      g_sd_deadline_anchored = 1U;
    }
  }
  const uint64_t after = sd_route_session_locked(local);
  if (before != after) ++g_sd_route_epoch;
  __set_PRIMASK(mask);
}

volatile uint32_t g_sd_line_drop_count = 0U;
volatile uint32_t g_sd_line_slots_used = 0U;
volatile uint32_t g_sd_line_slots_peak = 0U;
volatile uint32_t g_sd_raw_batch_drop_count = 0U;
volatile uint32_t g_sd_raw_slots_used = 0U;
volatile uint32_t g_sd_raw_slots_peak = 0U;
volatile uint32_t g_sd_max_service_ms = 0U;
volatile uint32_t g_sd_max_read_ms = 0U;
volatile uint32_t g_sd_max_write_ms = 0U;
volatile uint32_t g_sd_max_raw_service_ms = 0U;
volatile uint32_t g_sd_max_telemetry_service_ms = 0U;
volatile uint32_t g_sd_max_flush_ms = 0U;
volatile uint32_t g_sd_write_error_count = 0U;
volatile uint32_t g_sd_flush_count = 0U;
volatile uint32_t g_sd_init_failures = 0U;
volatile uint32_t g_sd_retry_count = 0U;
volatile uint32_t g_sd_warning_publish_count = 0U;
volatile uint32_t g_sd_raw_records_written_count = 0U;
volatile uint32_t g_sd_raw_records_enqueued_count;
volatile uint32_t g_sd_raw_records_dropped_count;
volatile uint32_t g_sd_raw_essential_records_dropped;
volatile uint32_t g_sd_raw_optional_records_dropped;
volatile uint32_t g_sd_optional_telemetry_dropped;
volatile uint32_t g_sd_essential_telemetry_dropped;
volatile uint32_t g_sd_raw_records_coalesced;
volatile uint32_t g_sd_scalar_records_enqueued, g_sd_scalar_records_written, g_sd_scalar_records_coalesced;
volatile uint32_t g_sd_preallocation_attempts;
volatile uint32_t g_sd_preallocation_fallbacks;
volatile uint32_t g_sd_preallocation_errors;
volatile uint32_t g_sd_max_preallocate_ms;
volatile uint32_t g_sd_raw_records_io_lost_count;
volatile uint32_t g_sd_raw_records_flushed_count;
volatile uint32_t g_sd_csv_rows_written_count = 0U;

static uint32_t sd_calibration_snapshot(daq_calibration_t *calibration)
{
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  *calibration = g_sd_calibration;
  const uint32_t generation = g_sd_calibration_generation;
  __set_PRIMASK(saved);
  return generation;
}

/* Only the storage worker calls these adapters. IDMA owns the aligned
 * bounce buffer until completion or confirmed peripheral quiescence. */
static UINT sd_hal_read(UCHAR *destination, ULONG sector, ULONG sector_count)
{
  const uint32_t started = HAL_GetTick();
  while (sector_count != 0U) {
    const ULONG count = sector_count > SD_TRANSFER_SECTORS ? SD_TRANSFER_SECTORS : sector_count;
    if (sd_storage_io_read(g_sd_transfer_buffer, sector, count) != FX_SUCCESS) {
      if (sd_storage_io_requires_reinit()) g_sd_ready = 0U;
      return FX_IO_ERROR;
    }
    memcpy(destination, g_sd_transfer_buffer, (size_t)count * SD_SECTOR_SIZE);
    destination += count * SD_SECTOR_SIZE; sector += count; sector_count -= count;
  }
  const uint32_t elapsed = HAL_GetTick() - started;
  if (elapsed > g_sd_max_read_ms) g_sd_max_read_ms = elapsed;
  return FX_SUCCESS;
}

static UINT sd_hal_write(const UCHAR *source, ULONG sector, ULONG sector_count)
{
  const uint32_t started = HAL_GetTick();
  while (sector_count != 0U) {
    const ULONG count = sector_count > SD_TRANSFER_SECTORS ? SD_TRANSFER_SECTORS : sector_count;
    memcpy(g_sd_transfer_buffer, source, (size_t)count * SD_SECTOR_SIZE);
    if (sd_storage_io_write(g_sd_transfer_buffer, sector, count) != FX_SUCCESS) {
      if (sd_storage_io_requires_reinit()) g_sd_ready = 0U;
      return FX_IO_ERROR;
    }
    const uint32_t saved = __get_PRIMASK(); __disable_irq();
    g_sd_last_sector_write_ms = board_uptime_ticks(); g_sd_write_progress_valid = 1U;
    __set_PRIMASK(saved);
    source += count * SD_SECTOR_SIZE; sector += count; sector_count -= count;
  }
  const uint32_t elapsed = HAL_GetTick() - started;
  if (elapsed > g_sd_max_write_ms) g_sd_max_write_ms = elapsed;
  return FX_SUCCESS;
}

static VOID sd_filex_driver(FX_MEDIA *media)
{
  ULONG sector;
  UINT status = FX_SUCCESS;

  switch (media->fx_media_driver_request)
  {
    case FX_DRIVER_INIT:
      media->fx_media_driver_write_protect = FX_FALSE;
      media->fx_media_driver_free_sector_update = FX_FALSE;
      status = (g_sd_hardware_ready != 0U) ? FX_SUCCESS : FX_IO_ERROR;
      break;

    case FX_DRIVER_UNINIT:
    case FX_DRIVER_FLUSH:
    case FX_DRIVER_ABORT:
    case FX_DRIVER_RELEASE_SECTORS:
      status = FX_SUCCESS;
      break;

    case FX_DRIVER_BOOT_READ:
      status = sd_hal_read(media->fx_media_driver_buffer, 0U,
                           media->fx_media_driver_sectors);
      if (status == FX_SUCCESS)
      {
        ULONG partition_start = 0U;
        ULONG partition_size = 0U;
        if ((_fx_partition_offset_calculate(media->fx_media_driver_buffer, 0U,
                                            &partition_start,
                                            &partition_size) == FX_SUCCESS) &&
            (partition_start != 0U))
        {
          status = sd_hal_read(media->fx_media_driver_buffer, partition_start,
                               media->fx_media_driver_sectors);
        }
      }
      break;

    case FX_DRIVER_BOOT_WRITE:
      status = sd_hal_write(media->fx_media_driver_buffer, media->fx_media_hidden_sectors,
                            media->fx_media_driver_sectors);
      break;

    case FX_DRIVER_READ:
      sector = media->fx_media_driver_logical_sector + media->fx_media_hidden_sectors;
      status = sd_hal_read(media->fx_media_driver_buffer, sector,
                           media->fx_media_driver_sectors);
      break;

    case FX_DRIVER_WRITE:
      sector = media->fx_media_driver_logical_sector + media->fx_media_hidden_sectors;
      status = sd_hal_write(media->fx_media_driver_buffer, sector,
                            media->fx_media_driver_sectors);
      break;

    default:
      status = FX_IO_ERROR;
      break;
  }

  media->fx_media_driver_status = status;
}

/* Reserve FAT chains ahead of writes without extending logical CSV length.
 * FileX allocate() appends clusters; its truncate_release() counterpart must
 * release the unused tail on a normal close. An unclean power loss may retain
 * up to2.25MiB of reserved tail per open log, plus cluster rounding, until
 * filesystem repair/reclamation. */
#define SD_FILE_RESERVE_BYTES (2UL * 1024UL * 1024UL)
#define SD_FILE_RESERVE_LOW_WATER (256UL * 1024UL)
#define SD_OPTIONAL_RAW_BACKLOG_LIMIT 256U
#if DAQ_SD_PREALLOCATE_ENABLE && (defined(FX_UPDATE_FILE_SIZE_ON_ALLOCATE) || defined(FX_ENABLE_FAULT_TOLERANT))
#error "Review SD preallocation: this FileX mode extends logical file size on allocate"
#endif
static uint32_t sd_raw_backlog_records(void)
{
  return g_sd_raw_records_enqueued_count - g_sd_raw_records_written_count -
      g_sd_raw_records_io_lost_count;
}

static uint8_t *sd_file_reserve_disabled(FX_FILE *file)
{
  return file == &g_sd_file ? &g_sd_raw_reserve_disabled : &g_sd_telemetry_reserve_disabled;
}

static UINT sd_prepare_file_space(FX_FILE *file, size_t next_bytes)
{
  if (!DAQ_SD_PREALLOCATE_ENABLE) return FX_SUCCESS;
  uint8_t *disabled = sd_file_reserve_disabled(file);
  if (*disabled) return FX_SUCCESS;
  const uint64_t available = file->fx_file_current_available_size;
  const uint64_t offset = file->fx_file_current_file_offset;
  const uint64_t tail = available > offset ? available - offset : 0U;
  if (tail >= (uint64_t)next_bytes + SD_FILE_RESERVE_LOW_WATER) return FX_SUCCESS;
  /* Top up while the protected queue is nearly empty, rather than adding
   * metadata I/O to a backlog. A write that needs space cannot be deferred. */
  if (tail >= next_bytes && sd_raw_backlog_records() > SD_OPTIONAL_RAW_BACKLOG_LIMIT)
    return FX_SUCCESS;
  if (available >= SD_LOG_PART_LIMIT_BYTES) return FX_SUCCESS;
  const uint64_t room = SD_LOG_PART_LIMIT_BYTES - available;
  const ULONG reserve = (ULONG)(room < SD_FILE_RESERVE_BYTES ? room : SD_FILE_RESERVE_BYTES);
  const uint32_t started = HAL_GetTick();
  ++g_sd_preallocation_attempts;
  UINT status = fx_file_allocate(file, reserve);
  /* Allocate may advance its physical cursor at a cluster boundary even
   * when an earlier reserve still has space. Re-seek through zero so the
   * next write uses the original logical offset in the extended FAT chain. */
  if (status == FX_SUCCESS && offset != 0U) {
    status = fx_file_seek(file, 0U);
    if (status == FX_SUCCESS) status = fx_file_seek(file, (ULONG)offset);
  }
  const uint32_t elapsed = HAL_GetTick() - started;
  if (elapsed > g_sd_max_preallocate_ms) g_sd_max_preallocate_ms = elapsed;
  if (status == FX_NO_MORE_SPACE) {
    /* A fragmented card may not have a consecutive reserve. Preserve ordinary
     * FileX writes; do not retry this performance hint on every4KiB flush. */
    *disabled = 1U; ++g_sd_preallocation_fallbacks;
    return FX_SUCCESS;
  }
  if (status != FX_SUCCESS) ++g_sd_preallocation_errors;
  return status;
}

static UINT sd_release_file_space(FX_FILE *file)
{
  if (file->fx_file_current_available_size <= file->fx_file_current_file_size)
    return FX_SUCCESS;
  /* Ordinary truncate() and close() leave unused allocation attached. */
  return fx_file_truncate_release(file, (ULONG)file->fx_file_current_file_size);
}

static UINT sd_media_flush(void)
{
  const UINT status = fx_media_flush(&g_sd_media);
  if (status == FX_SUCCESS) {
    const uint32_t saved = __get_PRIMASK(); __disable_irq();
    g_sd_last_media_flush_ms = board_uptime_ticks(); g_sd_flush_progress_valid = 1U;
    __set_PRIMASK(saved);
  }
  return status;
}

static void sd_update_filesystem_clock(void)
{
  sd_calendar_t now;
  if (sd_calendar_from_unix_ms(daq_log_unix_ms(), &now))
  {
    (void)fx_system_date_set(now.year, now.month, now.day);
    (void)fx_system_time_set(now.hour, now.minute, now.second);
  }
}

static UINT sd_close_log(FX_FILE *file, char *name)
{
  sd_update_filesystem_clock();
  UINT status = sd_release_file_space(file);
  if (status != FX_SUCCESS) return status;
  status = fx_file_close(file);
  sd_calendar_t now;
  if (status == FX_SUCCESS && sd_calendar_from_unix_ms(daq_log_unix_ms(), &now) &&
      fx_file_date_time_set(&g_sd_media, name, now.year, now.month,
                           now.day, now.hour, now.minute, now.second) != FX_SUCCESS)
    g_sd_write_error_count++;
  /* Timestamp failure must not leave a successfully closed handle marked open. */
  return status;
}

static UINT sd_open_log(FX_FILE *file, char *g_sd_filename, size_t filename_size,
                        const char *prefix, const daq_calibration_t *requested)
{
  sd_update_filesystem_clock();
  const uint64_t stamp = daq_log_unix_ms();
  daq_calibration_t calibration;
  (void)sd_calibration_snapshot(&calibration);
  if (requested != NULL) calibration = *requested;

  for (uint32_t suffix = 0U; suffix < 1000U; ++suffix)
  {
    const uint64_t name_stamp = (stamp != 0U) ? stamp : daq_log_monotonic_ms();
    char stamp_text[21];
    sd_format_u64(stamp_text, name_stamp);
    (void)snprintf(g_sd_filename, filename_size,
                   "%s_%s_%03lu.CSV",
                   prefix, stamp_text,
                   (unsigned long)suffix);
    const UINT create_status = fx_file_create(&g_sd_media, g_sd_filename);
    if (create_status == FX_SUCCESS)
    {
      break;
    }
    if (create_status != FX_ALREADY_CREATED)
    {
      return create_status;
    }
    g_sd_filename[0] = '\0';
  }

  if (g_sd_filename[0] == '\0')
  {
    return FX_NO_MORE_SPACE;
  }

  UINT status = fx_file_open(&g_sd_media, file, g_sd_filename,
                             FX_OPEN_FOR_WRITE);
  if (status != FX_SUCCESS)
  {
    return status;
  }

  *sd_file_reserve_disabled(file) = 0U;
  static const char header[] =
      SD_CSV_HEADER;
  status = sd_prepare_file_space(file, sizeof(header) - 1U);
  if (status == FX_SUCCESS)
    status = fx_file_write(file, (VOID *)header, sizeof(header) - 1U);
  if (status == FX_SUCCESS)
  {
    char calibration_line[768];
    char coefficients[17][24];
    sd_format_float(coefficients[0], calibration.kg1000_slope);
    sd_format_float(coefficients[1], calibration.kg1000_intercept);
    sd_format_float(coefficients[2], calibration.iadc_slope);
    sd_format_float(coefficients[3], calibration.iadc_intercept);
    for (unsigned i = 0; i < 7; ++i)
      sd_format_float(coefficients[4 + i], calibration.kg50[i]);
    for (unsigned i = 0; i < 4; ++i)
      sd_format_float(coefficients[11 + i], calibration.thermal[i]);
    sd_format_float(coefficients[15], calibration.filter_tau_ms[0]);
    sd_format_float(coefficients[16], calibration.filter_tau_ms[1]);
    const int len = snprintf(
        calibration_line, sizeof(calibration_line),
        "# calibration,kg1000_slope=%s,kg1000_intercept=%s,iadc_slope=%s,iadc_intercept=%s,kg50_c0=%s,kg50_c1=%s,kg50_c2=%s,kg50_c3=%s,kg50_c4=%s,kg50_x0=%s,kg50_tare=%s,kg1000_temp_ref=%s,kg1000_raw_per_c=%s,kg50_temp_ref=%s,kg50_raw_per_c=%s,kg1000_filter_tau_ms=%s,kg50_filter_tau_ms=%s\r\n",
        coefficients[0], coefficients[1], coefficients[2], coefficients[3],
        coefficients[4], coefficients[5], coefficients[6], coefficients[7],
        coefficients[8], coefficients[9], coefficients[10],
        coefficients[11], coefficients[12], coefficients[13], coefficients[14], coefficients[15], coefficients[16]);
    if (len <= 0 || (size_t)len >= sizeof(calibration_line))
      status = FX_IO_ERROR;
    else
      status = fx_file_write(file, calibration_line, (ULONG)len);
    if (status == FX_SUCCESS) {
      const int pressure_len = snprintf(calibration_line, sizeof(calibration_line),
          "# chamber_pressure,p4_adc_channel=%d,model=OMEGA_PX309-1KG5V,raw_units=connector_V,converted_units=psig,divider_ratio=11,zero_V=0,span_V=5,span_psig=1000\r\n",
          DAQ_CHAMBER_ADC_CHANNEL);
      status = pressure_len > 0 && (size_t)pressure_len < sizeof(calibration_line)
          ? fx_file_write(file, calibration_line, (ULONG)pressure_len) : FX_IO_ERROR;
    }
  }
  if (status == FX_SUCCESS)
  {
    status = sd_media_flush();
  }
  if (status != FX_SUCCESS) {
    (void)sd_release_file_space(file);
    (void)fx_file_close(file);
  }
  return status;
}

static UINT sd_open_timestamped_log(const daq_calibration_t *requested)
{
  daq_calibration_t calibration;
  (void)sd_calibration_snapshot(&calibration);
  if (requested != NULL) calibration = *requested;
  const UINT status = sd_open_log(&g_sd_file, g_sd_filename,
                                  sizeof(g_sd_filename), g_sd_raw_session != 0U ? "DAQ_LAUNCH" : "DAQ", &calibration);
  if (status == FX_SUCCESS)
  {
    g_file_open = 1U;
    g_sd_open_calibration = calibration;
  }
  return status;
}

static UINT sd_flush_telemetry_pending(void)
{
  static ULONG retry_offset;
  static uint8_t retry_active;
  if (g_sd_telemetry_write_buffer_len == 0U) return FX_SUCCESS;
  if (!retry_active) { retry_offset = g_sd_telemetry_file.fx_file_current_file_offset; retry_active = 1U; }
  UINT status = sd_prepare_file_space(&g_sd_telemetry_file, g_sd_telemetry_write_buffer_len);
  if (status == FX_SUCCESS) status = fx_file_seek(&g_sd_telemetry_file, retry_offset);
  if (status == FX_SUCCESS)
    status = fx_file_write(&g_sd_telemetry_file, g_sd_telemetry_write_buffer, (ULONG)g_sd_telemetry_write_buffer_len);
  if (status != FX_SUCCESS) { g_sd_write_error_count++; return status; }
  retry_active = 0U;
  g_sd_telemetry_write_buffer_len = 0U;
  return FX_SUCCESS;
}

static UINT sd_write_telemetry_bytes(const void *data, size_t len)
{
  if (g_telemetry_file_open == 0U || data == NULL || len == 0U || len > sizeof(g_sd_telemetry_write_buffer))
    return FX_PTR_ERROR;
  if (len > sizeof(g_sd_telemetry_write_buffer) - g_sd_telemetry_write_buffer_len && sd_flush_telemetry_pending() != FX_SUCCESS)
    return FX_IO_ERROR;
  memcpy(g_sd_telemetry_write_buffer + g_sd_telemetry_write_buffer_len, data, len);
  g_sd_telemetry_write_buffer_len += len;
  return FX_SUCCESS;
}

/* Delayed telemetry rows rotate only their own file, never the raw log. */
static UINT sd_write_telemetry_row(const sd_line_slot_t *slot)
{
  if (g_telemetry_file_open != 0U &&
      (g_sd_telemetry_session != slot->session ||
       memcmp(&g_sd_telemetry_calibration, &slot->calibration,
             sizeof(slot->calibration)) != 0 ||
       sd_log_part_needs_rotation((uint64_t)g_sd_telemetry_file.fx_file_current_file_size +
           g_sd_telemetry_write_buffer_len, slot->len)))
  {
    if (sd_flush_telemetry_pending() != FX_SUCCESS ||
        sd_media_flush() != FX_SUCCESS) return FX_IO_ERROR;
    if (sd_close_log(&g_sd_telemetry_file, g_sd_telemetry_filename) != FX_SUCCESS) return FX_IO_ERROR;
    g_telemetry_file_open = 0U;
  }
  if (g_telemetry_file_open == 0U)
  {
    const UINT status = sd_open_log(&g_sd_telemetry_file,
        g_sd_telemetry_filename, sizeof(g_sd_telemetry_filename),
        slot->session != 0U ? "DAQ_LAUNCH_TELEMETRY" : "DAQ_TELEMETRY", &slot->calibration);
    if (status != FX_SUCCESS) return status;
    g_telemetry_file_open = 1U;
    g_sd_telemetry_calibration = slot->calibration;
    g_sd_telemetry_session = slot->session;
  }
  return sd_write_telemetry_bytes(slot->line, slot->len);
}

static UINT sd_marker_exists(void)
{
  FX_FILE marker;
  const UINT status = fx_file_open(&g_sd_media, &marker, SD_PROVISION_MARKER,
                                   FX_OPEN_FOR_READ);
  if (status == FX_SUCCESS)
  {
    (void)fx_file_close(&marker);
    return FX_SUCCESS;
  }
  return status;
}

/* FileX formats FAT tables one sector at a time. Coalesce those writes only
 * during provisioning; normal FileX writes retain their synchronous semantics.
 * Copy the buffer because FileX reuses it immediately after each callback. */
static UCHAR g_sd_format_buffer[SD_TRANSFER_SECTORS * SD_SECTOR_SIZE];
static ULONG g_sd_format_start;
static ULONG g_sd_format_count;
static UINT g_sd_format_error;
volatile uint32_t g_sd_format_sectors_written;
volatile uint32_t g_sd_mount_status;
volatile uint32_t g_sd_format_status;

static UINT sd_format_flush(void)
{
  if (g_sd_format_error != FX_SUCCESS) return g_sd_format_error;
  if (g_sd_format_count == 0U) return FX_SUCCESS;
  g_sd_format_error = sd_hal_write(g_sd_format_buffer, g_sd_format_start,
                                  g_sd_format_count);
  if (g_sd_format_error == FX_SUCCESS)
  {
    g_sd_format_sectors_written += g_sd_format_count;
    g_sd_format_count = 0U;
  }
  return g_sd_format_error;
}

static VOID sd_format_driver(FX_MEDIA *media)
{
  if (media->fx_media_driver_request == FX_DRIVER_INIT)
  {
    g_sd_format_count = 0U;
    g_sd_format_error = FX_SUCCESS;
    g_sd_format_sectors_written = 0U;
    sd_filex_driver(media);
    return;
  }
  if (media->fx_media_driver_request == FX_DRIVER_WRITE)
  {
    ULONG sector = media->fx_media_driver_logical_sector + media->fx_media_hidden_sectors;
    const UCHAR *source = media->fx_media_driver_buffer;
    for (ULONG i = 0U; i < media->fx_media_driver_sectors; ++i, ++sector)
    {
      if (g_sd_format_count != 0U &&
          sector != g_sd_format_start + g_sd_format_count)
        (void)sd_format_flush();
      if (g_sd_format_error != FX_SUCCESS) break;
      if (g_sd_format_count == 0U) g_sd_format_start = sector;
      memcpy(g_sd_format_buffer + g_sd_format_count * SD_SECTOR_SIZE,
             source + i * SD_SECTOR_SIZE, SD_SECTOR_SIZE);
      if (++g_sd_format_count == SD_TRANSFER_SECTORS) (void)sd_format_flush();
    }
    media->fx_media_driver_status = g_sd_format_error;
    return;
  }
  /* Boot writes, reads, flush and uninit must observe all buffered FAT writes. */
  media->fx_media_driver_status = sd_format_flush();
  if (media->fx_media_driver_status == FX_SUCCESS) sd_filex_driver(media);
}

static UINT sd_format_and_mark(void)
{
  HAL_SD_CardInfoTypeDef card_info = {0};
  FX_FILE marker;
  static const char marker_contents[] = "SEDS DAQ FAT volume v1\r\n";

  if (HAL_SD_GetCardInfo(&hsd1, &card_info) != HAL_OK)
  {
    return FX_IO_ERROR;
  }

  g_sd_init_stage = 6U;
  UINT status = fx_media_format(&g_sd_media,
                                sd_format_driver,
                                FX_NULL,
                                g_sd_media_cache,
                                sizeof(g_sd_media_cache),
                                "SEDS_DAQ",
                                2U,
                                512U,
                                0U,
                                card_info.LogBlockNbr,
                                SD_SECTOR_SIZE,
                                64U,
                                255U,
                                63U);
  g_sd_format_status = status;
  if (status != FX_SUCCESS)
  {
    return status;
  }
  g_sd_init_stage = 3U;
  status = fx_media_open(&g_sd_media, "DAQ SD", sd_filex_driver, FX_NULL,
                         g_sd_media_cache, sizeof(g_sd_media_cache));
  if (status != FX_SUCCESS)
  {
    return status;
  }
  /* Format supplies the boot-record label; also create the directory label
   * used by desktop volume browsers before marking provisioning complete. */
  status = fx_media_volume_set(&g_sd_media, "SEDS_DAQ");
  if (status != FX_SUCCESS)
  {
    (void)fx_media_close(&g_sd_media);
    return status;
  }
  status = fx_file_create(&g_sd_media, SD_PROVISION_MARKER);
  if ((status != FX_SUCCESS) && (status != FX_ALREADY_CREATED))
  {
    return status;
  }
  status = fx_file_open(&g_sd_media, &marker, SD_PROVISION_MARKER,
                        FX_OPEN_FOR_WRITE);
  if (status == FX_SUCCESS)
  {
    status = fx_file_write(&marker, (VOID *)marker_contents,
                           sizeof(marker_contents) - 1U);
    (void)fx_file_close(&marker);
  }
  if (status == FX_SUCCESS)
  {
    status = sd_media_flush();
  }
  return status;
}

static UINT sd_mount_or_provision(void)
{
  UINT status = fx_media_open(&g_sd_media, "DAQ SD", sd_filex_driver, FX_NULL,
                              g_sd_media_cache, sizeof(g_sd_media_cache));
  g_sd_mount_status = status;
  /* A transport fault is not an unformatted card. Never turn a failed DMA
   * read into filesystem formatting. The worker retries initialization. */
  /* FileX wraps transport failures as FX_BOOT_ERROR/FX_FAT_READ_ERROR.
   * Only a successfully read but invalid filesystem permits provisioning. */
  if (status != FX_SUCCESS && status != FX_MEDIA_INVALID) return status;
  const UINT marker_status = status == FX_SUCCESS ? sd_marker_exists() : FX_NOT_FOUND;
  if (status == FX_SUCCESS && marker_status != FX_SUCCESS && marker_status != FX_NOT_FOUND) {
    (void)fx_media_close(&g_sd_media);
    return marker_status;
  }
  if ((status == FX_SUCCESS) && (marker_status == FX_SUCCESS))
  {
    /* Upgrade cards provisioned before the directory label was added,
     * preserving their files and provisioning marker. */
    status = fx_media_volume_set(&g_sd_media, "SEDS_DAQ");
    if (status == FX_SUCCESS) status = sd_media_flush();
    if (status != FX_SUCCESS) (void)fx_media_close(&g_sd_media);
    return status;
  }
  if (status == FX_SUCCESS) (void)fx_media_close(&g_sd_media);
  /* A remote retry never grants provisioning permission, including later
   * automatic retries in this boot. Existing logs/unknown media stay intact. */
  if (g_sd_manual_no_format) return status == FX_SUCCESS ? FX_NOT_FOUND : status;
  return sd_format_and_mark();
}

/* Transport recovery keeps the original FileX state and pending records. */
static UINT sd_resume_card_media(void)
{
  if (g_sd_media_mounted) {
    if (memcmp(g_sd_mounted_cid, hsd1.CID, sizeof(g_sd_mounted_cid)) != 0 ||
        g_sd_mounted_blocks != hsd1.SdCard.LogBlockNbr) {
      ++g_sd_card_identity_mismatches;
      return FX_IO_ERROR;
    }
    return FX_SUCCESS;
  }
  const UINT status = sd_mount_or_provision();
  if (status == FX_SUCCESS) {
    memcpy(g_sd_mounted_cid, hsd1.CID, sizeof(g_sd_mounted_cid));
    g_sd_mounted_blocks = hsd1.SdCard.LogBlockNbr;
    g_sd_media_mounted = 1U;
  }
  return status;
}

static UINT sd_flush_pending(void)
{
  static ULONG retry_offset;
  static uint8_t retry_active;
  if (g_sd_write_buffer_len == 0U) return FX_SUCCESS;
  if (!retry_active) { retry_offset = g_sd_file.fx_file_current_file_offset; retry_active = 1U; }
  UINT status = sd_prepare_file_space(&g_sd_file, g_sd_write_buffer_len);
  if (status == FX_SUCCESS) status = fx_file_seek(&g_sd_file, retry_offset);
  if (status == FX_SUCCESS)
    status = fx_file_write(&g_sd_file, g_sd_write_buffer, (ULONG)g_sd_write_buffer_len);
  if (status != FX_SUCCESS) { g_sd_write_error_count++; return status; }
  retry_active = 0U;
  g_sd_write_buffer_len = 0U;
  return FX_SUCCESS;
}

static UINT sd_write_bytes(const void *data, size_t len)
{
  if (g_file_open == 0U || data == NULL || len == 0U || len > sizeof(g_sd_write_buffer))
    return FX_PTR_ERROR;
  if (len > sizeof(g_sd_write_buffer) - g_sd_write_buffer_len && sd_flush_pending() != FX_SUCCESS)
    return FX_IO_ERROR;
  memcpy(g_sd_write_buffer + g_sd_write_buffer_len, data, len);
  g_sd_write_buffer_len += len;
  return FX_SUCCESS;
}

/* ThreadX block pools may return word-aligned memory. Retain the original
 * allocation before an explicitly 8-byte-aligned record (uint64_t members).
 * Allocation is bounded to the static pools and never waits for free memory. */
/* The writer is the only consumer. FIFO head peeks include a retained failed
 * row/batch; producers stamp and publish the epoch in one short critical
 * section. This prevents normal/launch file ping-pong across the two queues. */
static uint8_t sd_telemetry_epoch_ready(uint32_t epoch, uint8_t pressure)
{
  uint32_t other = 0U;
  uint8_t present = 0U;
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  if (pressure) {
    if (g_sd_line_pending) { other = g_sd_line_pending_epoch; present = 1U; }
    else if (g_sd_queue.tx_queue_enqueued != 0U) {
      const sd_line_slot_t *head = (const sd_line_slot_t *)(uintptr_t)*g_sd_queue.tx_queue_read;
      memcpy(&other, (const char *)head + offsetof(sd_line_slot_t, route_epoch), sizeof(other));
      present = 1U;
    }
  } else {
    if (g_sd_pressure_pending) { other = g_sd_pressure_pending_epoch; present = 1U; }
    else if (g_sd_pressure_queue.tx_queue_enqueued != 0U) {
      memcpy(&other, (const UCHAR *)g_sd_pressure_queue.tx_queue_read +
             offsetof(sd_pressure_message_t, route_epoch), sizeof(other));
      present = 1U;
    }
  }
  __set_PRIMASK(mask);
  return !present || (int32_t)(epoch - other) <= 0;
}

/* No IRQ masking around allocation or file I/O. Only queue pointer ownership
 * changes under PRIMASK, so acquisition cannot append a writer-owned slot. */
static UINT sd_take_line_message(ULONG *message)
{
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  const UINT result = tx_queue_receive(&g_sd_queue, message, TX_NO_WAIT);
  if (result == TX_SUCCESS && g_sd_scalar_append_tail == (sd_scalar_slot_t *)(uintptr_t)*message)
    g_sd_scalar_append_tail = NULL;
  __set_PRIMASK(mask);
  return result;
}

static UINT sd_take_raw_message(ULONG *message)
{
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  const UINT result = tx_queue_receive(&g_sd_raw_queue, message, TX_NO_WAIT);
  if (result == TX_SUCCESS && g_sd_raw_append_tail == (sd_raw_slot_t *)(uintptr_t)*message)
    g_sd_raw_append_tail = NULL;
  __set_PRIMASK(mask);
  return result;
}

static void *sd_pool_allocate(TX_BLOCK_POOL *pool, size_t bytes)
{
  void *allocation = NULL;
  if (bytes + SD_POOL_ALIGNMENT_RESERVE > pool->tx_block_pool_block_size ||
      tx_block_allocate(pool, &allocation, TX_NO_WAIT) != TX_SUCCESS) return NULL;
  const uintptr_t aligned = ((uintptr_t)allocation + sizeof(void *) + 7U) & ~(uintptr_t)7U;
  ((void **)aligned)[-1] = allocation;
  return (void *)aligned;
}

static void sd_pool_release(void *slot)
{
  (void)tx_block_release(((void **)slot)[-1]);
}

static sd_line_slot_t *sd_alloc_slot(uint16_t len)
{
  sd_line_slot_t *slot = sd_pool_allocate(&g_sd_line_pool, offsetof(sd_line_slot_t, line) + len + 1U);
  if (slot != NULL) {
    const uint32_t saved = __get_PRIMASK(); __disable_irq();
    if (++g_sd_line_slots_used > g_sd_line_slots_peak) g_sd_line_slots_peak = g_sd_line_slots_used;
    __set_PRIMASK(saved);
  }
  return slot;
}

static void sd_free_slot(sd_line_slot_t *slot)
{
  if (slot != NULL) {
    const uint32_t saved = __get_PRIMASK(); __disable_irq(); --g_sd_line_slots_used; __set_PRIMASK(saved);
    sd_pool_release(slot);
  }
}

static sd_raw_slot_t *sd_alloc_raw_slot(uint16_t count)
{
  sd_raw_slot_t *slot = sd_pool_allocate(&g_sd_raw_pool, offsetof(sd_raw_slot_t, samples) +
                                       (size_t)count * sizeof(sd_raw_adc_record_t));
  if (slot != NULL) {
    slot->capacity = count;
    const uint32_t saved = __get_PRIMASK(); __disable_irq();
    if (++g_sd_raw_slots_used > g_sd_raw_slots_peak) g_sd_raw_slots_peak = g_sd_raw_slots_used;
    __set_PRIMASK(saved);
  }
  return slot;
}

static void sd_free_raw_slot(sd_raw_slot_t *slot)
{
  if (slot != NULL) {
    const uint32_t saved = __get_PRIMASK(); __disable_irq(); --g_sd_raw_slots_used; __set_PRIMASK(saved);
    sd_pool_release(slot);
  }
}

/* Every queued batch/row carries its acquisition calibration. A delayed
 * old batch must never be labelled with newly received coefficients. */
static UINT sd_select_calibration(const daq_calibration_t *calibration)
{
  if (g_file_open == 0U) return sd_open_timestamped_log(calibration);
  /* Reserve a worst-case complete batch before admitting its first row. */
  if (memcmp(&g_sd_open_calibration, calibration, sizeof(*calibration)) == 0 &&
      !sd_log_part_needs_rotation((uint64_t)g_sd_file.fx_file_current_file_size +
          g_sd_write_buffer_len, (uint64_t)SD_RAW_BATCH_MAX * 256U))
    return FX_SUCCESS;
  if (sd_flush_pending() != FX_SUCCESS || sd_media_flush() != FX_SUCCESS)
    return FX_IO_ERROR;
  if (sd_close_log(&g_sd_file, g_sd_filename) != FX_SUCCESS) return FX_IO_ERROR;
  g_file_open = 0U;
  return sd_open_timestamped_log(calibration);
}

/* Two 500 Hz channels can enqueue two rows per acquisition batch. Drain a
 * bounded burst rather than one row per raw batch, which steadily falls behind
 * whenever raw formatting/SD writes take most of the writer's time slice. */
static UINT sd_write_telemetry_item(const sd_line_slot_t *pending,
                                    uint16_t *index, uint8_t *complete)
{
  /* Shared prefix reads use bytes: a scalar allocation has a different
   * effective type from the expanded CSV slot under release strict aliasing. */
  uint16_t tag; memcpy(&tag, pending, sizeof(tag));
  if (tag != SD_SCALAR_SLOT_TAG) {
    *complete = 1U;
    return sd_write_telemetry_row(pending);
  }
  const sd_scalar_slot_t *batch = (const sd_scalar_slot_t *)(const void *)pending;
  const sd_scalar_record_t *row = &batch->records[*index];
  union {
    uint64_t alignment;
    UCHAR bytes[offsetof(sd_line_slot_t, line) + SD_LINE_MAX];
  } storage;
  sd_line_slot_t *expanded = (sd_line_slot_t *)(void *)storage.bytes;
  expanded->calibration = batch->calibration; expanded->session = batch->session;
  expanded->route_epoch = batch->route_epoch;
  char value[24]; sd_format_float(value, row->value);
  const int length = sd_format_csv_record(expanded->line, SD_LINE_MAX,
      row->network_ms, board_uptime_project(batch->uptime_reference, row->monotonic_ms),
      g_sd_scalar_sensors[row->sensor], value, batch->sequence_base + row->sequence_delta);
  if (length <= 0 || (size_t)length >= SD_LINE_MAX) return FX_IO_ERROR;
  expanded->len = (uint16_t)length;
  const UINT result = sd_write_telemetry_row(expanded);
  if (result == FX_SUCCESS) {
    *complete = ++*index == batch->count; ++g_sd_scalar_records_written;
  }
  return result;
}

static uint8_t sd_service_telemetry_rows(void)
{
  static sd_line_slot_t *pending;
  static uint16_t scalar_index;
  uint8_t serviced = 0U;
  for (unsigned budget = 0U; budget < 4U; ++budget) {
    if (!pending) {
      ULONG message;
      if (sd_take_line_message(&message) != TX_SUCCESS) break;
      pending = (sd_line_slot_t *)(uintptr_t)message;
      scalar_index = 0U;
      g_sd_line_pending = pending != NULL;
      if (pending) memcpy(&g_sd_line_pending_epoch,
          (const char *)pending + offsetof(sd_line_slot_t, route_epoch), sizeof(g_sd_line_pending_epoch));
    }
    if (pending && !sd_telemetry_epoch_ready(g_sd_line_pending_epoch, 0U)) break;
    serviced = 1U;
    if (!pending) continue;
    uint8_t complete;
    if (sd_write_telemetry_item(pending, &scalar_index, &complete) != FX_SUCCESS) {
      ++g_sd_write_error_count; break;
    }
    g_sd_csv_rows_written_count++;
    if (!complete) continue;
    sd_free_slot(pending); pending = NULL; g_sd_line_pending = 0U;
  }
  return serviced;
}

/* Retain a failed row and its batch until the media accepts it. A bounded
 * four-row turn leaves the existing raw and telemetry services runnable. */
static uint8_t sd_service_pressure_rows(void)
{
  static union {
    sd_pressure_message_t message;
    ULONG words[SD_PRESSURE_MESSAGE_WORDS];
  } pending;
  static union {
    uint64_t alignment;
    UCHAR bytes[offsetof(sd_line_slot_t, line) + SD_LINE_MAX];
  } storage;
  static unsigned index;
  sd_line_slot_t *slot = (sd_line_slot_t *)(void *)storage.bytes;
  uint8_t serviced = 0U;
  for (unsigned budget = 0U; budget < PRESSURE_SD_SAMPLES; ++budget) {
    if (!g_sd_pressure_pending) {
      if (tx_queue_receive(&g_sd_pressure_queue, pending.words, TX_NO_WAIT) != TX_SUCCESS) break;
      g_sd_pressure_pending = 1U; index = 0U; slot->len = 0U;
      g_sd_pressure_pending_epoch = pending.message.route_epoch;
      slot->session = pending.message.session;
      /* Pressure raw/converted values come from VB; DAQ coefficients are only
       * the common file metadata and never transform a pressure record. */
      (void)sd_calibration_snapshot(&slot->calibration);
    }
    if (!sd_telemetry_epoch_ready(pending.message.route_epoch, 1U)) break;
    serviced = 1U;
    const pressure_sd_batch_t *batch = &pending.message.batch;
    if (slot->len == 0U) {
      char raw[12], converted[24];
      snprintf(raw, sizeof(raw), "%u", (unsigned)batch->raw[index]);
      sd_format_float(converted, batch->psi[index]);
      const int n = sd_format_csv_sensor_record(slot->line, SD_LINE_MAX,
          batch->unix_ms ? batch->unix_ms + index * PRESSURE_SD_PERIOD_MS : 0U,
          batch->uptime_ms + index * PRESSURE_SD_PERIOD_MS, "valve_pressure_psi",
          raw, converted, (uint32_t)(batch->first_sequence + index));
      if (n < 0) {
        ++g_sd_pressure_records_dropped; ++g_sd_write_error_count;
        if (++index == PRESSURE_SD_SAMPLES) g_sd_pressure_pending = 0U;
        continue;
      }
      slot->len = (uint16_t)n;
    }
    if (sd_write_telemetry_row(slot) != FX_SUCCESS) { ++g_sd_write_error_count; break; }
    ++g_sd_pressure_records_written; ++g_sd_csv_rows_written_count;
    slot->len = 0U;
    if (++index == PRESSURE_SD_SAMPLES) g_sd_pressure_pending = 0U;
  }
  return serviced;
}

/* Close a drained launch handle exactly once, retrying buffered data,
 * close and final directory/media flush failures without reopening it. */
static void sd_close_finished_launch_files(uint8_t raw_drained, uint8_t telemetry_drained)
{
  (void)sd_launch_finished();
  const uint64_t current_session = sd_run_snapshot();
  {
    if (g_file_open && g_sd_raw_session && g_sd_raw_session != current_session && raw_drained) {
      if (sd_flush_pending() == FX_SUCCESS && sd_close_log(&g_sd_file, g_sd_filename) == FX_SUCCESS) {
        g_file_open = 0U; g_sd_launch_flush_pending = 1U;
      } else ++g_sd_write_error_count;
    }
    if (g_telemetry_file_open && g_sd_telemetry_session && g_sd_telemetry_session != current_session && telemetry_drained) {
      if (sd_flush_telemetry_pending() == FX_SUCCESS &&
          sd_close_log(&g_sd_telemetry_file, g_sd_telemetry_filename) == FX_SUCCESS) {
        g_telemetry_file_open = 0U; g_sd_launch_flush_pending = 1U;
      } else ++g_sd_write_error_count;
    }
  }
  if (g_sd_launch_flush_pending) {
    if (sd_media_flush() == FX_SUCCESS) g_sd_launch_flush_pending = 0U;
    else ++g_sd_write_error_count;
  }
}

static uint32_t sd_take_manual_retry(void)
{
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  const uint32_t token = g_sd_retry_pending_token;
  if (token != 0U) {
    g_sd_retry_pending_token = 0U; g_sd_retry_active_token = token;
    g_sd_manual_no_format = 1U;
  }
  __set_PRIMASK(saved);
  return token;
}

static void sd_complete_manual_retry(void)
{
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  if (g_sd_retry_active_token != 0U) {
    g_sd_retry_completed_token = g_sd_retry_active_token; g_sd_retry_active_token = 0U;
  }
  __set_PRIMASK(saved);
}

void sd_card_writer_thread_entry(ULONG initial_input)
{
  ULONG message = 0U;
  sd_raw_slot_t *pending_raw = NULL;
  uint16_t pending_raw_index = 0U;
  sd_raw_format_cache_t raw_format_cache = {0};
  ULONG last_flush = tx_time_get();
  /* Attempt the first mount immediately. Subsequent missing-card retries stay
   * at one second, but a healthy card should never delay acquisition logging
   * merely because the board has just booted. */
  ULONG last_retry = (ULONG)(0U - TX_TIMER_TICKS_PER_SECOND);
  uint32_t failed_retries = 0U;
  (void)initial_input;

  for (;;)
  {
    ULONG flags = 0U;
    uint8_t serviced_work = 0U;
    /* This boundary owns no in-flight IDMA buffer or FileX operation. Keep
     * pending records, dirty cache, handles and monotonic launch deadlines. */
    if (sd_take_manual_retry() != 0U) {
      g_sd_ready = 0U;
      last_retry = tx_time_get() - TX_TIMER_TICKS_PER_SECOND;
    }

    if (g_sd_ready == 0U)
    {
      const ULONG now = tx_time_get();
      if ((now - last_retry) >= TX_TIMER_TICKS_PER_SECOND)
      {
        last_retry = now;
        g_sd_retry_count++;
        g_sd_init_stage = 1U;
        /* Init clears ErrorCode only after success, while wide-bus setup
         * checks it earlier. Reset stale errors and controller state first. */
        (void)HAL_SD_DeInit(&hsd1);
        /* Clock gating in MSP deinit does not reset a stuck data-path state
         * machine. Reset the peripheral before the next card initialization. */
        __HAL_RCC_SDMMC1_FORCE_RESET();
        __HAL_RCC_SDMMC1_RELEASE_RESET();
        if ((HAL_SD_Init(&hsd1) == HAL_OK) &&
            ((g_sd_init_stage = 2U) != 0U) &&
            (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) == HAL_OK))
        {
          g_sd_hardware_ready = 1U;
          g_sd_init_stage = 3U;
          const UINT media_status = sd_resume_card_media();
          if (media_status == FX_SUCCESS) {
            g_sd_ready = 1U;
            g_sd_init_stage = 5U;
            failed_retries = 0U;
          }
        }

        sd_complete_manual_retry();
        if (g_sd_ready == 0U)
        {
          g_sd_hardware_ready = 0U;
          g_sd_init_failures++;
          if ((failed_retries == 0U) || ((failed_retries % 60U) == 0U))
          {
#ifdef TELEMETRY_ENABLED
            if (log_telemetry_string_asynchronous(
                    SEDS_DT_WARNING,
                    "DAQ SD card unavailable; acquisition continues without logging") == SEDS_OK)
            {
              g_sd_warning_publish_count++;
            }
#endif
          }
          failed_retries++;
        }
      }
      tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10U);
      continue;
    }

    /* Rotate only when a record with new coefficients arrives. Rotating from
     * the global snapshot while idle creates header-only files and can switch
     * forward then backward across an in-flight old acquisition batch. */
    const uint32_t service_start_ms = HAL_GetTick();

    /* Bound raw work so a continuously replenished ADC queue cannot starve
     * live/replay rows, calibration rotation, or flush requests. */
    if (pending_raw == NULL && sd_take_raw_message(&message) == TX_SUCCESS) {
      pending_raw = (sd_raw_slot_t *)(uintptr_t)message;
      pending_raw_index = 0U;
    }
    if (pending_raw != NULL)
    {
      serviced_work = 1U;
      sd_raw_slot_t *slot = pending_raw;
      char line[256];
      if (slot != NULL)
      {
        if (slot->session != g_sd_raw_session) {
          if (g_file_open != 0U) {
            if (sd_flush_pending() != FX_SUCCESS || sd_close_log(&g_sd_file, g_sd_filename) != FX_SUCCESS) {
              g_sd_write_error_count++; tx_thread_sleep(1U); continue;
            }
            g_file_open = 0U;
          }
          g_sd_raw_session = slot->session;
        }
        if (sd_select_calibration(&slot->calibration) != FX_SUCCESS)
        {
          g_sd_write_error_count++;
          tx_thread_sleep(1U);
          continue;
        }
        /* Larger allocations improve stall capacity without enlarging the
         * writer turn: telemetry/PT queues retain their original service share. */
        const uint16_t turn_end = slot->count - pending_raw_index > SD_RAW_BATCH_MAX ?
            pending_raw_index + SD_RAW_BATCH_MAX : slot->count;
        for (uint16_t i = pending_raw_index; i < turn_end; ++i)
        {
          const sd_raw_adc_record_t *sample = &slot->samples[i];
          const uint64_t captured_uptime = board_uptime_project(slot->uptime_reference, sample->monotonic_ms);
          const char *sensor = daq_chamber_is_channel(sample->channel) ? "chamber_pressure_raw_volts_psi" : sample->channel < 8U ? (const char *const[]){
              "mcp3564r_raw", "kg50_raw", "mcp3564r_ch2_raw", "mcp3564r_ch3_raw",
              "mcp3564r_ch4_raw", "mcp3564r_ch5_raw", "mcp3564r_ch6_raw", "mcp3564r_ch7_raw"
          }[sample->channel] : sample->channel == 9U ? "mcp3564r_ch2_ch3_gain16_raw" : "invalid_adc_channel";
          const int len = sd_format_csv_raw_record(line, sizeof(line),
              sample->network_unix_ms, captured_uptime, sensor, sample->raw_adc_code,
              sample->raw_value, sample->calibrated_value, sample->adc_temperature_c,
              sample->adc_temperature_code, sample->record_sequence, &raw_format_cache);
          if (len <= 0 || (size_t)len >= sizeof(line)) {
            ++g_sd_write_error_count; ++g_sd_raw_records_io_lost_count;
            pending_raw_index = i + 1U;
            continue; /* Count an unformattable record; do not wedge the entire logger. */
          }
          if (sd_write_bytes(line, (size_t)len) != FX_SUCCESS) {
            ++g_sd_write_error_count;
            tx_thread_sleep(1U);
            break; /* Retain this exact record and batch for the next service turn. */
          }
          g_sd_raw_records_written_count++;
          pending_raw_index = i + 1U;
        }
        if (pending_raw_index == slot->count) { sd_free_raw_slot(slot); pending_raw = NULL; }
      }
    }

    const uint32_t raw_ms = HAL_GetTick() - service_start_ms;
    if (raw_ms > g_sd_max_raw_service_ms) g_sd_max_raw_service_ms = raw_ms;
    const uint32_t telemetry_start_ms = HAL_GetTick();
    serviced_work |= sd_service_telemetry_rows();
    serviced_work |= sd_service_pressure_rows();
    const uint32_t telemetry_ms = HAL_GetTick() - telemetry_start_ms;
    if (telemetry_ms > g_sd_max_telemetry_service_ms) g_sd_max_telemetry_service_ms = telemetry_ms;

    if (tx_event_flags_get(&g_sd_flags,
                           SD_FLAG_FLUSH_REQUEST | SD_FLAG_POWER_LOSS,
                           TX_OR_CLEAR, &flags, TX_NO_WAIT) == TX_SUCCESS)
    {
      /* A brownout warning requests an immediate flush, not a permanent
       * recording shutdown. A startup ADC transient must not stop this run. */
    }

    if ((flags != 0U) ||
        ((tx_time_get() - last_flush) >= TX_TIMER_TICKS_PER_SECOND))
    {
      const uint32_t flush_start_ms = HAL_GetTick();
      sd_update_filesystem_clock();
      if ((sd_flush_pending() == FX_SUCCESS) &&
          (sd_flush_telemetry_pending() == FX_SUCCESS) &&
          (sd_media_flush() == FX_SUCCESS))
      {
        g_sd_launch_flush_pending = 0U;
        g_sd_flush_count++;
        g_sd_raw_records_flushed_count = g_sd_raw_records_written_count;
        g_sd_pressure_records_flushed = g_sd_pressure_records_written;
      }
      else
      {
        g_sd_write_error_count++;
      }
      last_flush = tx_time_get();
      const uint32_t flush_ms = HAL_GetTick() - flush_start_ms;
      if (flush_ms > g_sd_max_flush_ms) g_sd_max_flush_ms = flush_ms;
    }

    /* Only launch handles close at the deadline; ordinary SD work continues. */
    sd_close_finished_launch_files(
        pending_raw == NULL && g_sd_raw_slots_used == 0U && g_sd_raw_queue.tx_queue_enqueued == 0U,
        g_sd_line_slots_used == 0U && g_sd_queue.tx_queue_enqueued == 0U &&
            !g_sd_pressure_pending && g_sd_pressure_queue.tx_queue_enqueued == 0U);

    const uint32_t service_ms = HAL_GetTick() - service_start_ms;
    if (service_ms > g_sd_max_service_ms) g_sd_max_service_ms = service_ms;
    if (serviced_work == 0U)
    {
      tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 1000U);
    }
  }
}

UINT sd_card_init(TX_BYTE_POOL *byte_pool)
{
  (void)byte_pool;
  if (g_sd_services_initialized != 0U)
  {
    return TX_SUCCESS;
  }
  if (tx_block_pool_create(&g_sd_line_pool, "sd_lines", SD_LINE_BLOCK_PAYLOAD + SD_POOL_ALIGNMENT_RESERVE,
                           g_sd_line_pool_storage, sizeof(g_sd_line_pool_storage)) != TX_SUCCESS ||
      tx_block_pool_create(&g_sd_raw_pool, "sd_raw", SD_RAW_BLOCK_PAYLOAD + SD_POOL_ALIGNMENT_RESERVE,
                           g_sd_raw_pool_storage, sizeof(g_sd_raw_pool_storage)) != TX_SUCCESS)
    return TX_POOL_ERROR;
  const UINT io_status = sd_storage_io_init();
  if (io_status != TX_SUCCESS) return io_status;
  if (tx_queue_create(&g_sd_queue, "sd_queue", TX_1_ULONG,
                      g_sd_queue_storage, sizeof(g_sd_queue_storage)) != TX_SUCCESS)
  {
    return TX_QUEUE_ERROR;
  }
  if (tx_queue_create(&g_sd_raw_queue, "sd_raw_queue", TX_1_ULONG,
                      g_sd_raw_queue_storage,
                      sizeof(g_sd_raw_queue_storage)) != TX_SUCCESS)
  {
    return TX_QUEUE_ERROR;
  }
  if (tx_queue_create(&g_sd_pressure_queue, "sd_pressure", SD_PRESSURE_MESSAGE_WORDS,
                      g_sd_pressure_queue_storage, sizeof(g_sd_pressure_queue_storage)) != TX_SUCCESS)
    return TX_QUEUE_ERROR;
  if (tx_event_flags_create(&g_sd_flags, "sd_flags") != TX_SUCCESS)
  {
    return TX_GROUP_ERROR;
  }

  fx_system_initialize();
  /* Unknown time is explicitly the FAT epoch, never FileX's 2017 default. */
  (void)fx_system_date_set(1980U, 1U, 1U);
  (void)fx_system_time_set(0U, 0U, 0U);
  g_sd_services_initialized = 1U;

  /* All card/media operations belong to the lower-priority writer. */
  return TX_SUCCESS;
}

void sd_card_set_hardware_ready(uint8_t ready)
{
  g_sd_hardware_ready = (ready != 0U) ? 1U : 0U;
}

static sd_card_status_t sd_enqueue_line(sd_line_slot_t *slot)
{
  const ULONG message = (ULONG)(uintptr_t)slot;
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  slot->session = sd_run_snapshot(); slot->route_epoch = g_sd_route_epoch;
  g_sd_scalar_append_tail = NULL;
  const UINT result = tx_queue_send(&g_sd_queue, (VOID *)&message, TX_NO_WAIT);
  __set_PRIMASK(mask);
  if (result != TX_SUCCESS)
  {
    g_sd_line_drop_count++;
    sd_free_slot(slot);
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  return SD_CARD_STATUS_OK;
}

static uint8_t sd_raw_is_essential(const sd_raw_adc_record_t *sample)
{
  return sample->channel == 0U || sample->channel == 1U || sample->channel == 9U ||
      daq_chamber_is_channel(sample->channel);
}

static void sd_raw_count_dropped(const sd_raw_adc_record_t *samples, uint16_t count,
                                  uint8_t only_optional)
{
  for (uint16_t i = 0U; i < count; ++i) {
    if (sd_raw_is_essential(&samples[i])) {
      if (only_optional) continue;
      ++g_sd_raw_essential_records_dropped;
    } else ++g_sd_raw_optional_records_dropped;
    ++g_sd_raw_records_dropped_count;
  }
}

/* These atomic word reads are an early-shedding hint, never allocation
 * authorization. Required loadcells and pressure do not use this gate. */
static uint8_t sd_raw_optional_admitted(void)
{
  return sd_raw_backlog_records() <= SD_OPTIONAL_RAW_BACKLOG_LIMIT &&
      g_sd_raw_pool.tx_block_pool_available >= (g_sd_raw_pool.tx_block_pool_total + 3U) / 4U;
}

static uint8_t sd_optional_telemetry_admitted(void)
{
  return sd_raw_optional_admitted() &&
      g_sd_line_pool.tx_block_pool_available >= (g_sd_line_pool.tx_block_pool_total * 3U + 3U) / 4U;
}

static uint8_t sd_required_sensor_name(const char *name)
{
  return strcmp(name, "kg1000_network") == 0 || strcmp(name, "kg50_network") == 0 ||
      strcmp(name, "kg50_p6_network_volts") == 0 ||
      strcmp(name, "chamber_pressure_raw_volts_psi") == 0 || strcmp(name, "valve_pressure_psi") == 0;
}

static uint64_t sd_row_sequence;
static uint64_t sd_next_row_sequence(void) {
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  const uint64_t value = ++sd_row_sequence; __set_PRIMASK(saved); return value;
}
static int sd_scalar_sensor_index(const char *name)
{
  for (unsigned i = 0U; i < sizeof(g_sd_scalar_sensors) / sizeof(g_sd_scalar_sensors[0]); ++i)
    if (strcmp(name, g_sd_scalar_sensors[i]) == 0) return (int)i;
  return -1;
}

static sd_card_status_t sd_enqueue_scalar(uint8_t sensor, uint64_t network_ms,
    uint64_t uptime, float value, const daq_calibration_t *calibration)
{
  const uint64_t sequence = sd_next_row_sequence();
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  const uint64_t session = sd_run_snapshot();
  const uint32_t epoch = g_sd_route_epoch;
  sd_scalar_slot_t *tail = g_sd_scalar_append_tail;
  if (tail && tail->count < tail->capacity && tail->session == session &&
      tail->route_epoch == epoch && sequence - tail->sequence_base <= UINT32_MAX &&
      memcmp(&tail->calibration, calibration, sizeof(*calibration)) == 0) {
    tail->records[tail->count++] = (sd_scalar_record_t){network_ms, (uint32_t)uptime,
        (uint32_t)(sequence - tail->sequence_base), value, sensor};
    if (uptime > tail->uptime_reference) tail->uptime_reference = uptime;
    ++g_sd_scalar_records_enqueued; ++g_sd_scalar_records_coalesced;
    __set_PRIMASK(mask); return SD_CARD_STATUS_OK;
  }
  __set_PRIMASK(mask);
  /* Existing allocator owns slot counters and8-byte alignment. Its len is a
   * byte payload size here; the tag is assigned only after allocation. */
  const size_t bytes = offsetof(sd_scalar_slot_t, records) - offsetof(sd_line_slot_t, line) +
      SD_SCALAR_BATCH_MAX * sizeof(sd_scalar_record_t);
  sd_scalar_slot_t *slot = (sd_scalar_slot_t *)(void *)sd_alloc_slot((uint16_t)bytes);
  if (!slot) { ++g_sd_line_drop_count; ++g_sd_essential_telemetry_dropped; return SD_CARD_STATUS_BACKPRESSURE; }
  slot->len = SD_SCALAR_SLOT_TAG; slot->count = 1U; slot->capacity = SD_SCALAR_BATCH_MAX;
  slot->session = session; slot->route_epoch = epoch; slot->calibration = *calibration;
  slot->sequence_base = sequence; slot->uptime_reference = uptime;
  slot->records[0] = (sd_scalar_record_t){network_ms, (uint32_t)uptime, 0U, value, sensor};
  const ULONG message = (ULONG)(uintptr_t)slot;
  const uint32_t queued_mask = __get_PRIMASK(); __disable_irq();
  const UINT result = tx_queue_send(&g_sd_queue, (VOID *)&message, TX_NO_WAIT);
  if (result == TX_SUCCESS) {
    g_sd_scalar_append_tail = slot; ++g_sd_scalar_records_enqueued;
  }
  __set_PRIMASK(queued_mask);
  if (result != TX_SUCCESS) {
    ++g_sd_line_drop_count; ++g_sd_essential_telemetry_dropped;
    sd_free_slot((sd_line_slot_t *)(void *)slot); return SD_CARD_STATUS_BACKPRESSURE;
  }
  return SD_CARD_STATUS_OK;
}

#ifdef TELEMETRY_ENABLED
sd_card_status_t sd_card_log_packet(const SedsPacketView *pkt)
{
  if (!g_sd_ready || !pkt) return SD_CARD_STATUS_BUSY;
  if (!sd_optional_telemetry_admitted()) {
    ++g_sd_optional_telemetry_dropped; ++g_sd_line_drop_count;
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  const int32_t want = seds_pkt_to_string_len(pkt);
  char value[SD_LINE_MAX], line[SD_LINE_MAX];
  if (want < 0 || (size_t)want >= sizeof(value) ||
      seds_pkt_to_string(pkt, value, (size_t)want + 1U) != SEDS_OK) return SD_CARD_STATUS_IO_ERROR;
  const uint64_t uptime = board_uptime_ticks(), utc = daq_log_unix_ms();
  const int len = sd_format_csv_record(line, sizeof(line), utc, uptime, "seds_packet", value, sd_next_row_sequence());
  if (len < 0) return SD_CARD_STATUS_IO_ERROR;
  sd_line_slot_t *slot = sd_alloc_slot((uint16_t)len);
  if (!slot) { g_sd_line_drop_count++; return SD_CARD_STATUS_BACKPRESSURE; }
  (void)sd_calibration_snapshot(&slot->calibration);
  slot->session = sd_run_snapshot(); slot->len = (uint16_t)len;
  memcpy(slot->line, line, (size_t)len + 1U);
  return sd_enqueue_line(slot);
}

#endif

sd_card_status_t sd_card_log_pressure_batch(const pressure_sd_batch_t *batch)
{
  if (!batch) return SD_CARD_STATUS_IO_ERROR;
  /* The writer updates the closed latch at its monotonic launch deadline.
   * Never take router-clock or SD-pool locks from the receive callback. */
  if (!g_sd_services_initialized) {
    g_sd_pressure_records_dropped += PRESSURE_SD_SAMPLES;
    return SD_CARD_STATUS_BUSY;
  }
  union {
    sd_pressure_message_t message;
    ULONG words[SD_PRESSURE_MESSAGE_WORDS];
  } queued = {0};
  queued.message.batch = *batch;
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  queued.message.session = sd_run_snapshot(); queued.message.route_epoch = g_sd_route_epoch;
  const UINT result = tx_queue_send(&g_sd_pressure_queue, queued.words, TX_NO_WAIT);
  __set_PRIMASK(mask);
  if (result != TX_SUCCESS) {
    g_sd_pressure_records_dropped += PRESSURE_SD_SAMPLES;
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  g_sd_pressure_records_enqueued += PRESSURE_SD_SAMPLES;
  return SD_CARD_STATUS_OK;
}

sd_card_status_t sd_card_enqueue_csv_row(const char *sensor_name,
                                         uint64_t timestamp_ms,
                                         float value,
                                         const daq_calibration_t *calibration)
{
  /* The bounded row queue can accept startup records while the writer mounts
   * the card. Do not discard the first network row merely because mounting
   * runs in another task. A missing/slow card still produces backpressure
   * when the fixed block pool fills; the general-purpose heap is not used. */
  if ((g_sd_services_initialized == 0U) || (sensor_name == NULL))
  {
    return SD_CARD_STATUS_BUSY;
  }
  if (!sd_required_sensor_name(sensor_name) && !sd_optional_telemetry_admitted()) {
    ++g_sd_optional_telemetry_dropped; ++g_sd_line_drop_count;
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  char line[SD_LINE_MAX];
  daq_calibration_t captured_calibration;
  if (calibration != NULL) captured_calibration = *calibration;
  else (void)sd_calibration_snapshot(&captured_calibration);
  const uint64_t session = sd_run_snapshot();
  char value_text[24];
  const uint64_t age = (uint32_t)(daq_log_monotonic_ms() - timestamp_ms);
  const uint64_t now = board_uptime_ticks();
  const uint64_t uptime = now >= age ? now - age : 0U;
  const uint64_t network_ms = daq_sample_network_ms(daq_log_unix_ms(), now, uptime);
  const int scalar_sensor = sd_scalar_sensor_index(sensor_name);
  if (scalar_sensor >= 0)
    return sd_enqueue_scalar((uint8_t)scalar_sensor, network_ms, uptime, value, &captured_calibration);
  sd_format_float(value_text, value);
  const int len = sd_format_csv_record(line, sizeof(line), network_ms, uptime,
                                       sensor_name, value_text, sd_next_row_sequence());
  if ((len <= 0) || ((size_t)len >= sizeof(line)))
  {
    return SD_CARD_STATUS_IO_ERROR;
  }
  sd_line_slot_t *slot = sd_alloc_slot((uint16_t)len);
  if (slot == NULL) {
    ++g_sd_line_drop_count;
    if (sd_required_sensor_name(sensor_name)) ++g_sd_essential_telemetry_dropped;
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  slot->calibration = captured_calibration;
  slot->session = session;
  memcpy(slot->line, line, (size_t)len + 1U);
  slot->len = (uint16_t)len;
  const sd_card_status_t result = sd_enqueue_line(slot);
  if (result != SD_CARD_STATUS_OK && sd_required_sensor_name(sensor_name))
    ++g_sd_essential_telemetry_dropped;
  return result;
}

sd_card_status_t sd_card_enqueue_raw_adc_samples(const sd_raw_adc_record_t *samples,
                                                 uint16_t count,
                                                 const daq_calibration_t *calibration)
{
  if ((g_sd_services_initialized == 0U) || !samples || !calibration || count == 0U || count > SD_RAW_BATCH_MAX) {
    if (samples && count && count <= SD_RAW_BATCH_MAX) sd_raw_count_dropped(samples, count, 0U);
    return SD_CARD_STATUS_BUSY;
  }
  const uint8_t optional = sd_raw_optional_admitted();
  uint16_t retained = 0U;
  for (uint16_t i = 0U; i < count; ++i)
    if (optional || sd_raw_is_essential(&samples[i])) ++retained;
  if (!optional) sd_raw_count_dropped(samples, count, 1U);
  if (retained == 0U) return SD_CARD_STATUS_BACKPRESSURE;

  uint16_t index = 0U, appended = 0U;
  const uint64_t uptime = board_uptime_ticks();
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  const uint64_t session = sd_run_snapshot();
  const uint32_t epoch = g_sd_route_epoch;
  sd_raw_slot_t *tail = g_sd_raw_append_tail;
  if (tail && tail->session == session && tail->route_epoch == epoch &&
      memcmp(&tail->calibration, calibration, sizeof(*calibration)) == 0) {
    while (index < count && tail->count < tail->capacity) {
      if (optional || sd_raw_is_essential(&samples[index])) {
        tail->samples[tail->count++] = samples[index]; ++appended;
      }
      ++index;
    }
    /* Existing records project backwards from the newest appended uptime,
     * including a32-bit HAL tick wrap. Every sample keeps its original time. */
    if (appended) tail->uptime_reference = uptime;
    g_sd_raw_records_coalesced += appended;
    g_sd_raw_records_enqueued_count += appended;
  }
  __set_PRIMASK(mask);
  if (appended == retained) return retained == count ? SD_CARD_STATUS_OK : SD_CARD_STATUS_BACKPRESSURE;

  /* Reserve a full batch only when a new queued tail is needed. Under a
   * flush backlog, later acquisition calls fill it without repeating the
   * calibration/session/pool header or allocating another tiny slot. */
  sd_raw_slot_t *slot = sd_alloc_raw_slot(SD_RAW_SLOT_CAPACITY);
  if (!slot) {
    ++g_sd_raw_batch_drop_count;
    for (uint16_t i = index; i < count; ++i)
      if (optional || sd_raw_is_essential(&samples[i])) sd_raw_count_dropped(&samples[i], 1U, 0U);
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  slot->uptime_reference = uptime;
  slot->calibration = *calibration;
  slot->session = session; slot->route_epoch = epoch; slot->count = 0U;
  for (; index < count; ++index)
    if (optional || sd_raw_is_essential(&samples[index])) slot->samples[slot->count++] = samples[index];
  const ULONG message = (ULONG)(uintptr_t)slot;
  const uint32_t queued_mask = __get_PRIMASK(); __disable_irq();
  const UINT result = tx_queue_send(&g_sd_raw_queue, (VOID *)&message, TX_NO_WAIT);
  if (result == TX_SUCCESS) {
    g_sd_raw_append_tail = slot;
    g_sd_raw_records_enqueued_count += slot->count;
  }
  __set_PRIMASK(queued_mask);
  if (result != TX_SUCCESS) {
    ++g_sd_raw_batch_drop_count;
    sd_raw_count_dropped(slot->samples, slot->count, 0U);
    sd_free_raw_slot(slot);
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  return retained == count ? SD_CARD_STATUS_OK : SD_CARD_STATUS_BACKPRESSURE;
}

sd_card_status_t sd_card_request_retry(uint32_t token)
{
  if (token == 0U) return SD_CARD_STATUS_IO_ERROR;
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  for (unsigned i = 0U; i < 8U; ++i) if (g_sd_retry_recent[i] == token) {
    __set_PRIMASK(saved); return SD_CARD_STATUS_OK;
  }
  if (g_sd_retry_pending_token || g_sd_retry_active_token) {
    __set_PRIMASK(saved); return SD_CARD_STATUS_BUSY;
  }
  g_sd_retry_recent[g_sd_retry_recent_next++ % 8U] = token;
  g_sd_retry_accepted_token = token; g_sd_retry_pending_token = token;
  __set_PRIMASK(saved);
  return SD_CARD_STATUS_OK;
}

static uint32_t sd_progress_age(uint64_t now, uint64_t last, uint8_t valid)
{
  if (!valid) return UINT32_MAX;
  const uint64_t age = now >= last ? now - last : 0U;
  return age >= UINT32_MAX ? UINT32_MAX - 1U : (uint32_t)age;
}

void sd_card_health_snapshot(uint32_t words[8])
{
  if (!words) return;
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  const uint64_t now = board_uptime_ticks();
  words[0] = (g_sd_services_initialized ? 1U : 0U) | (g_sd_ready ? 2U : 0U) |
      (g_sd_launch_closed ? 4U : 0U) |
      ((g_sd_retry_pending_token || g_sd_retry_active_token) ? 8U : 0U);
  words[1] = sd_progress_age(now, g_sd_last_sector_write_ms, g_sd_write_progress_valid);
  words[2] = sd_progress_age(now, g_sd_last_media_flush_ms, g_sd_flush_progress_valid);
  words[3] = g_sd_init_stage; words[4] = g_sd_retry_count;
  words[5] = g_sd_retry_accepted_token; words[6] = g_sd_retry_completed_token;
  words[7] = g_sd_init_failures;
  __set_PRIMASK(saved);
}

UINT sd_card_request_flush(void)
{
  return tx_event_flags_set(&g_sd_flags, SD_FLAG_FLUSH_REQUEST, TX_OR);
}

UINT sd_card_notify_power_loss(void)
{
  return tx_event_flags_set(&g_sd_flags, SD_FLAG_POWER_LOSS, TX_OR);
}

UINT sd_card_is_ready(void)
{
  return (UINT)g_sd_ready;
}

void sd_card_set_calibration(const daq_calibration_t *calibration)
{
  if (calibration == NULL) return;
  const uint8_t finished = sd_launch_finished();
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  const uint8_t changed = memcmp(&g_sd_calibration, calibration, sizeof(*calibration)) != 0;
  if (changed) { g_sd_calibration = *calibration; ++g_sd_calibration_generation; }
  __set_PRIMASK(saved);
  if (changed && finished) sd_card_set_launch_clock(0U, 0U);
}
