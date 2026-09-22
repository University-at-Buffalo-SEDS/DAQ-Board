#include "sd_card.h"
#include "daq_rates.h"

#include "fx_api.h"
#include "main.h"
#include "telemetry.h"
#include "sd_float_format.h"
#include "daq_timestamp.h"
#include "sd_calendar.h"

#include <stdio.h>
#include <string.h>

extern SD_HandleTypeDef hsd1;
UINT _fx_partition_offset_calculate(void *partition_sector, UINT partition,
                                    ULONG *partition_start,
                                    ULONG *partition_size);

#define SD_QUEUE_DEPTH 24U
#define SD_RAW_QUEUE_DEPTH DAQ_SD_RAW_QUEUE_DEPTH
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
  uint8_t in_use;
  daq_calibration_t calibration;
  uint64_t session;
  char line[SD_LINE_MAX];
} sd_line_slot_t;

typedef struct
{
  uint16_t count;
  uint8_t in_use;
  daq_calibration_t calibration;
  uint64_t session;
  sd_raw_adc_record_t samples[SD_RAW_BATCH_MAX];
} sd_raw_slot_t;

static TX_QUEUE g_sd_queue;
static ULONG g_sd_queue_storage[SD_QUEUE_DEPTH];
static TX_QUEUE g_sd_raw_queue;
static ULONG g_sd_raw_queue_storage[SD_RAW_QUEUE_DEPTH];
static TX_MUTEX g_sd_pool_mutex;
static TX_EVENT_FLAGS_GROUP g_sd_flags;
static sd_line_slot_t g_sd_slots[SD_QUEUE_DEPTH];
static sd_raw_slot_t g_sd_raw_slots[SD_RAW_QUEUE_DEPTH];

static FX_MEDIA g_sd_media;
static FX_FILE g_sd_file;
static FX_FILE g_sd_telemetry_file;
static uint8_t g_telemetry_file_open;
static daq_calibration_t g_sd_telemetry_calibration;
static char g_sd_telemetry_filename[64];
static UCHAR g_sd_media_cache[SD_MEDIA_CACHE_SIZE] SD_ALIGN_32;
static UCHAR g_sd_transfer_buffer[SD_TRANSFER_SECTORS * SD_SECTOR_SIZE] SD_ALIGN_32;
static UCHAR g_sd_write_buffer[4096U];
static size_t g_sd_write_buffer_len = 0U;
static char g_sd_filename[64];
volatile uint32_t g_sd_ready = 0U;
/* Debugger/simulator progress: 1 card init, 2 bus setup, 3 mount,
 * 4 open log, 5 ready. Preserve the failing stage for diagnosis. */
volatile uint32_t g_sd_init_stage = 0U;
static uint8_t g_sd_hardware_ready = 0U;
static uint8_t g_file_open = 0U;
static daq_calibration_t g_sd_calibration = {1.0f, 0.0f, 1.0f, 0.0f, {0.0f, 1.0f}};
static volatile uint32_t g_sd_calibration_generation = 1U;
static daq_calibration_t g_sd_open_calibration;
static uint8_t g_sd_services_initialized = 0U;
static uint64_t g_sd_session, g_sd_last_launch_session, g_sd_close_unix_ms;
static uint64_t g_sd_raw_session, g_sd_telemetry_session;
static uint64_t g_sd_close_local_ms;
static uint8_t g_sd_launch_closed;

static uint64_t sd_run_snapshot(void)
{
  uint64_t result;
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  result = g_sd_session;
  if (mask == 0U) __enable_irq();
  return result;
}

static uint8_t sd_launch_finished(void)
{
  /* Convert once to a monotonic deadline: losing network time must not leave
   * the launch file open indefinitely, or reopen it on a backwards correction. */
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  const uint64_t deadline = g_sd_close_unix_ms;
  const uint8_t finished_before = g_sd_launch_closed;
  const uint8_t needs_utc = g_sd_close_local_ms == 0U;
  if (mask == 0U) __enable_irq();
  if (deadline == 0U || finished_before) return finished_before;
  /* Never acquire the router clock lock for every raw/report row. Once mapped,
   * the local deadline is sufficient until a new managed clock arrives. */
  const uint64_t utc = needs_utc ? telemetry_unix_ms() : 0U;
  const uint64_t local = telemetry_now_ms();
  mask = __get_PRIMASK();
  __disable_irq();
  if (g_sd_close_unix_ms != 0U && g_sd_close_local_ms == 0U && utc != 0U)
    g_sd_close_local_ms = local + (utc < g_sd_close_unix_ms ? g_sd_close_unix_ms - utc : 0U);
  if (g_sd_close_unix_ms != 0U &&
      ((g_sd_close_local_ms != 0U && local >= g_sd_close_local_ms) ||
       (utc != 0U && utc >= g_sd_close_unix_ms))) g_sd_launch_closed = 1U;
  const uint8_t finished = g_sd_launch_closed;
  if (mask == 0U) __enable_irq();
  return finished;
}

void sd_card_set_launch_clock(uint64_t session, uint64_t close_unix_ms)
{
  const uint64_t now = session != 0U ? telemetry_unix_ms() : 0U;
  const uint64_t local = session != 0U ? telemetry_now_ms() : 0U;
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  if (session == 0U) {
    g_sd_session = 0U; g_sd_close_unix_ms = 0U;
    g_sd_close_local_ms = 0U; g_sd_launch_closed = 0U;
  }
  else if (session > g_sd_last_launch_session)
  {
    g_sd_last_launch_session = session;
    /* A retained clock from a finished launch must not restart it on reboot. */
    if (now == 0U || now < close_unix_ms) {
      g_sd_session = session;
      g_sd_close_unix_ms = close_unix_ms;
      g_sd_close_local_ms = now != 0U ? local + close_unix_ms - now : 0U;
      g_sd_launch_closed = 0U;
    }
  }
  else if (g_sd_session == session && !g_sd_launch_closed &&
           g_sd_close_unix_ms != close_unix_ms) {
    g_sd_close_unix_ms = close_unix_ms;
    if (now != 0U)
      g_sd_close_local_ms = local + (now < close_unix_ms ? close_unix_ms - now : 0U);
  }
  if (mask == 0U) __enable_irq();
}

volatile uint32_t g_sd_line_drop_count = 0U;
volatile uint32_t g_sd_raw_batch_drop_count = 0U;
volatile uint32_t g_sd_write_error_count = 0U;
volatile uint32_t g_sd_flush_count = 0U;
volatile uint32_t g_sd_init_failures = 0U;
volatile uint32_t g_sd_retry_count = 0U;
volatile uint32_t g_sd_warning_publish_count = 0U;
volatile uint32_t g_sd_raw_records_written_count = 0U;
volatile uint32_t g_sd_csv_rows_written_count = 0U;

static uint32_t sd_calibration_snapshot(daq_calibration_t *calibration)
{
  uint32_t generation;
  if ((g_sd_services_initialized != 0U) &&
      (tx_mutex_get(&g_sd_pool_mutex, TX_WAIT_FOREVER) == TX_SUCCESS))
  {
    *calibration = g_sd_calibration;
    generation = g_sd_calibration_generation;
    (void)tx_mutex_put(&g_sd_pool_mutex);
    return generation;
  }
  *calibration = g_sd_calibration;
  return g_sd_calibration_generation;
}

static UINT sd_wait_ready(void)
{
  const uint32_t started = HAL_GetTick();
  while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER)
  {
    if ((uint32_t)(HAL_GetTick() - started) >= 2000U) return FX_IO_ERROR;
    /* Card programming is asynchronous even for HAL polling writes. Yield
     * while it completes so logging cannot monopolize the networking CPU. */
    tx_thread_sleep(1U);
  }
  return FX_SUCCESS;
}

static UINT sd_hal_read(UCHAR *destination, ULONG sector, ULONG sector_count)
{
  while (sector_count != 0U)
  {
    const ULONG count = (sector_count > SD_TRANSFER_SECTORS)
                            ? SD_TRANSFER_SECTORS
                            : sector_count;
    const size_t bytes = (size_t)count * SD_SECTOR_SIZE;

    if ((sd_wait_ready() != FX_SUCCESS) ||
        (HAL_SD_ReadBlocks(&hsd1, g_sd_transfer_buffer, sector, count, 2000U) != HAL_OK))
    {
      return FX_IO_ERROR;
    }
    /* Polling HAL reads the FIFO with CPU stores, not DMA. Invalidating
     * here could discard those fresh stores before FileX sees them. */
    memcpy(destination, g_sd_transfer_buffer, bytes);
    destination += bytes;
    sector += count;
    sector_count -= count;
  }
  return FX_SUCCESS;
}

static UINT sd_hal_write(const UCHAR *source, ULONG sector, ULONG sector_count)
{
  while (sector_count != 0U)
  {
    const ULONG count = (sector_count > SD_TRANSFER_SECTORS)
                            ? SD_TRANSFER_SECTORS
                            : sector_count;
    const size_t bytes = (size_t)count * SD_SECTOR_SIZE;

    memcpy(g_sd_transfer_buffer, source, bytes);
    /* Polling HAL consumes this buffer with CPU loads; no DMA handoff. */
    if ((sd_wait_ready() != FX_SUCCESS) ||
        (HAL_SD_WriteBlocks(&hsd1, g_sd_transfer_buffer, sector, count, 2000U) != HAL_OK) ||
        (sd_wait_ready() != FX_SUCCESS))
    {
      return FX_IO_ERROR;
    }
    source += bytes;
    sector += count;
    sector_count -= count;
  }
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

static void sd_update_filesystem_clock(void)
{
  sd_calendar_t now;
  if (sd_calendar_from_unix_ms(telemetry_unix_ms(), &now))
  {
    (void)fx_system_date_set(now.year, now.month, now.day);
    (void)fx_system_time_set(now.hour, now.minute, now.second);
  }
}

static UINT sd_close_log(FX_FILE *file, char *name)
{
  sd_update_filesystem_clock();
  UINT status = fx_file_close(file);
  sd_calendar_t now;
  if (status == FX_SUCCESS && sd_calendar_from_unix_ms(telemetry_unix_ms(), &now) &&
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
  const uint64_t stamp = telemetry_unix_ms();
  daq_calibration_t calibration;
  (void)sd_calibration_snapshot(&calibration);
  if (requested != NULL) calibration = *requested;

  for (uint32_t suffix = 0U; suffix < 1000U; ++suffix)
  {
    const uint64_t name_stamp = (stamp != 0U) ? stamp : telemetry_now_ms();
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

  static const char header[] =
      "timestamp_ms,monotonic_ms,sensor,value,raw_adc_code,raw_value,calibrated_value,time_source,adc_temperature_c,adc_temperature_code\r\n";
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
  }
  if (status == FX_SUCCESS)
  {
    status = fx_media_flush(&g_sd_media);
  }
  if (status != FX_SUCCESS) (void)fx_file_close(file);
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

/* Delayed telemetry rows rotate only their own file, never the raw log. */
static UINT sd_write_telemetry_row(const sd_line_slot_t *slot)
{
  if (g_telemetry_file_open != 0U &&
      (g_sd_telemetry_session != slot->session ||
       memcmp(&g_sd_telemetry_calibration, &slot->calibration,
             sizeof(slot->calibration)) != 0))
  {
    if (fx_media_flush(&g_sd_media) != FX_SUCCESS) return FX_IO_ERROR;
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
  return fx_file_write(&g_sd_telemetry_file, (VOID *)slot->line, slot->len);
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

static UINT sd_format_and_mark(void)
{
  HAL_SD_CardInfoTypeDef card_info = {0};
  FX_FILE marker;
  static const char marker_contents[] = "SEDS DAQ FAT volume v1\r\n";

  if (HAL_SD_GetCardInfo(&hsd1, &card_info) != HAL_OK)
  {
    return FX_IO_ERROR;
  }

  UINT status = fx_media_format(&g_sd_media,
                                sd_filex_driver,
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
  if (status != FX_SUCCESS)
  {
    return status;
  }
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
    status = fx_media_flush(&g_sd_media);
  }
  return status;
}

static UINT sd_mount_or_provision(void)
{
  UINT status = fx_media_open(&g_sd_media, "DAQ SD", sd_filex_driver, FX_NULL,
                              g_sd_media_cache, sizeof(g_sd_media_cache));
  if ((status == FX_SUCCESS) && (sd_marker_exists() == FX_SUCCESS))
  {
    /* Upgrade cards provisioned before the directory label was added,
     * preserving their files and provisioning marker. */
    status = fx_media_volume_set(&g_sd_media, "SEDS_DAQ");
    if (status == FX_SUCCESS) status = fx_media_flush(&g_sd_media);
    if (status != FX_SUCCESS) (void)fx_media_close(&g_sd_media);
    return status;
  }
  if (status == FX_SUCCESS)
  {
    (void)fx_media_close(&g_sd_media);
  }
  return sd_format_and_mark();
}

static UINT sd_flush_pending(void)
{
  if (g_sd_write_buffer_len == 0U)
  {
    return FX_SUCCESS;
  }
  const UINT status = fx_file_write(&g_sd_file, g_sd_write_buffer,
                                    (ULONG)g_sd_write_buffer_len);
  if (status != FX_SUCCESS)
  {
    g_sd_write_error_count++;
    return status;
  }
  g_sd_write_buffer_len = 0U;
  return FX_SUCCESS;
}

static UINT sd_write_bytes(const void *data, size_t len)
{
  const UCHAR *cursor = (const UCHAR *)data;
  if ((g_file_open == 0U) || (data == NULL) || (len == 0U))
  {
    return FX_PTR_ERROR;
  }

  while (len != 0U)
  {
    const size_t available = sizeof(g_sd_write_buffer) - g_sd_write_buffer_len;
    const size_t copy_len = (len < available) ? len : available;
    memcpy(&g_sd_write_buffer[g_sd_write_buffer_len], cursor, copy_len);
    g_sd_write_buffer_len += copy_len;
    cursor += copy_len;
    len -= copy_len;
    if ((g_sd_write_buffer_len == sizeof(g_sd_write_buffer)) &&
        (sd_flush_pending() != FX_SUCCESS))
    {
      return FX_IO_ERROR;
    }
  }
  return FX_SUCCESS;
}

static sd_line_slot_t *sd_alloc_slot(void)
{
  sd_line_slot_t *slot = NULL;
  if (tx_mutex_get(&g_sd_pool_mutex, TX_WAIT_FOREVER) != TX_SUCCESS)
  {
    return NULL;
  }
  for (uint32_t i = 0U; i < SD_QUEUE_DEPTH; ++i)
  {
    if (g_sd_slots[i].in_use == 0U)
    {
      g_sd_slots[i].in_use = 1U;
      slot = &g_sd_slots[i];
      break;
    }
  }
  (void)tx_mutex_put(&g_sd_pool_mutex);
  return slot;
}

static void sd_free_slot(sd_line_slot_t *slot)
{
  if ((slot != NULL) &&
      (tx_mutex_get(&g_sd_pool_mutex, TX_WAIT_FOREVER) == TX_SUCCESS))
  {
    slot->in_use = 0U;
    slot->len = 0U;
    (void)tx_mutex_put(&g_sd_pool_mutex);
  }
}

static sd_raw_slot_t *sd_alloc_raw_slot(void)
{
  sd_raw_slot_t *slot = NULL;
  if (tx_mutex_get(&g_sd_pool_mutex, TX_WAIT_FOREVER) != TX_SUCCESS)
  {
    return NULL;
  }
  for (uint32_t i = 0U; i < SD_RAW_QUEUE_DEPTH; ++i)
  {
    if (g_sd_raw_slots[i].in_use == 0U)
    {
      g_sd_raw_slots[i].in_use = 1U;
      slot = &g_sd_raw_slots[i];
      break;
    }
  }
  (void)tx_mutex_put(&g_sd_pool_mutex);
  return slot;
}

static void sd_free_raw_slot(sd_raw_slot_t *slot)
{
  if ((slot != NULL) &&
      (tx_mutex_get(&g_sd_pool_mutex, TX_WAIT_FOREVER) == TX_SUCCESS))
  {
    slot->in_use = 0U;
    slot->count = 0U;
    (void)tx_mutex_put(&g_sd_pool_mutex);
  }
}

/* Every queued batch/row carries its acquisition calibration. A delayed
 * old batch must never be labelled with newly received coefficients. */
static UINT sd_select_calibration(const daq_calibration_t *calibration)
{
  if (g_file_open == 0U) return sd_open_timestamped_log(calibration);
  if (memcmp(&g_sd_open_calibration, calibration, sizeof(*calibration)) == 0)
    return FX_SUCCESS;
  if (sd_flush_pending() != FX_SUCCESS || fx_media_flush(&g_sd_media) != FX_SUCCESS)
    return FX_IO_ERROR;
  if (sd_close_log(&g_sd_file, g_sd_filename) != FX_SUCCESS) return FX_IO_ERROR;
  g_file_open = 0U;
  return sd_open_timestamped_log(calibration);
}

/* Two 500 Hz channels can enqueue two rows per acquisition batch. Drain a
 * bounded burst rather than one row per raw batch, which steadily falls behind
 * whenever raw formatting/SD writes take most of the writer's time slice. */
static uint8_t sd_service_telemetry_rows(void)
{
  uint8_t serviced = 0U;
  ULONG message;
  for (unsigned budget = 0U; budget < 4U; ++budget)
  {
    if (tx_queue_receive(&g_sd_queue, &message, TX_NO_WAIT) != TX_SUCCESS) break;
    serviced = 1U;
    sd_line_slot_t *slot = (sd_line_slot_t *)(uintptr_t)message;
    if (slot == NULL) continue;
    if (sd_write_telemetry_row(slot) == FX_SUCCESS) g_sd_csv_rows_written_count++;
    else g_sd_write_error_count++;
    sd_free_slot(slot);
  }
  return serviced;
}

void sd_card_writer_thread_entry(ULONG initial_input)
{
  ULONG message = 0U;
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
          if (sd_mount_or_provision() == FX_SUCCESS)
          {
            g_sd_ready = 1U;
            g_sd_init_stage = 5U;
            failed_retries = 0U;
          }
        }

        if (g_sd_ready == 0U)
        {
          g_sd_hardware_ready = 0U;
          g_sd_init_failures++;
          if ((failed_retries == 0U) || ((failed_retries % 60U) == 0U))
          {
            if (log_telemetry_string_asynchronous(
                    SEDS_DT_WARNING,
                    "DAQ SD card unavailable; acquisition continues without logging") == SEDS_OK)
            {
              g_sd_warning_publish_count++;
            }
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

    /* Bound raw work so a continuously replenished ADC queue cannot starve
     * live/replay rows, calibration rotation, or flush requests. */
    if (tx_queue_receive(&g_sd_raw_queue, &message, TX_NO_WAIT) == TX_SUCCESS)
    {
      serviced_work = 1U;
      sd_raw_slot_t *slot = (sd_raw_slot_t *)(uintptr_t)message;
      char line[192];
      if (slot != NULL)
      {
        if (slot->session != g_sd_raw_session) {
          if (g_file_open != 0U) {
            if (sd_flush_pending() != FX_SUCCESS || sd_close_log(&g_sd_file, g_sd_filename) != FX_SUCCESS) {
              g_sd_write_error_count++; sd_free_raw_slot(slot); continue;
            }
            g_file_open = 0U;
          }
          g_sd_raw_session = slot->session;
        }
        if (sd_select_calibration(&slot->calibration) != FX_SUCCESS)
        {
          g_sd_ready = 0U;
          g_sd_write_error_count++;
          sd_free_raw_slot(slot);
          continue;
        }
        for (uint16_t i = 0U; i < slot->count; ++i)
        {
          const sd_raw_adc_record_t *sample = &slot->samples[i];
          char raw_text[24], calibrated_text[24], temperature_text[24];
          sd_format_float(temperature_text, sample->adc_temperature_c);
          sd_format_float(raw_text, sample->raw_value);
          sd_format_float(calibrated_text, sample->calibrated_value);
          char stamp_text[21];
          sd_format_u64(stamp_text, daq_timestamp_ms(sample->network_unix_ms, sample->monotonic_ms));
          const int len = snprintf(line, sizeof(line), "%s,%lu,%s,,%ld,%s,%s,%s,%s,%ld\r\n",
                                   stamp_text,
                                   (unsigned long)sample->monotonic_ms,
                                   sample->channel == 1U ? "kg50_raw" : "mcp3564r_raw",
                                   (long)sample->raw_adc_code,
                                   raw_text, calibrated_text,
                                   sample->network_unix_ms != 0U ? "network" : "local",
                                   temperature_text, (long)sample->adc_temperature_code);
          if ((len <= 0) || ((size_t)len >= sizeof(line)) ||
              (sd_write_bytes(line, (size_t)len) != FX_SUCCESS))
          {
            g_sd_write_error_count++;
            break;
          }
          g_sd_raw_records_written_count++;
        }
        sd_free_raw_slot(slot);
      }
    }

    serviced_work |= sd_service_telemetry_rows();

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
      sd_update_filesystem_clock();
      if ((sd_flush_pending() == FX_SUCCESS) &&
          (fx_media_flush(&g_sd_media) == FX_SUCCESS))
      {
        g_sd_flush_count++;
      }
      else
      {
        g_sd_write_error_count++;
      }
      last_flush = tx_time_get();
    }

    /* Producers stop enqueueing at T+120 s; drain their already queued data,
     * close both streams once, and leave them closed until reset/new launch. */
    if ((g_file_open != 0U || g_telemetry_file_open != 0U) &&
        sd_launch_finished() && g_sd_raw_queue.tx_queue_enqueued == 0U &&
        g_sd_queue.tx_queue_enqueued == 0U)
    {
      if (g_file_open != 0U && sd_flush_pending() == FX_SUCCESS &&
          sd_close_log(&g_sd_file, g_sd_filename) == FX_SUCCESS) g_file_open = 0U;
      if (g_telemetry_file_open != 0U &&
          sd_close_log(&g_sd_telemetry_file, g_sd_telemetry_filename) == FX_SUCCESS)
        g_telemetry_file_open = 0U;
      (void)fx_media_flush(&g_sd_media);
    }

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
  if (tx_mutex_create(&g_sd_pool_mutex, "sd_pool", TX_NO_INHERIT) != TX_SUCCESS)
  {
    return TX_MUTEX_ERROR;
  }
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
  if (tx_event_flags_create(&g_sd_flags, "sd_flags") != TX_SUCCESS)
  {
    return TX_GROUP_ERROR;
  }

  fx_system_initialize();
  /* Unknown time is explicitly the FAT epoch, never FileX's 2017 default. */
  (void)fx_system_date_set(1980U, 1U, 1U);
  (void)fx_system_time_set(0U, 0U, 0U);
  g_sd_services_initialized = 1U;

  /* A missing card is not a board-startup failure. The writer task retries
   * insertion and reports the condition after SEDSNet starts. */
  if ((g_sd_hardware_ready != 0U) &&
      (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) == HAL_OK) &&
      (sd_mount_or_provision() == FX_SUCCESS))
  {
    g_sd_ready = 1U;
  }
  return TX_SUCCESS;
}

void sd_card_set_hardware_ready(uint8_t ready)
{
  g_sd_hardware_ready = (ready != 0U) ? 1U : 0U;
}

static sd_card_status_t sd_enqueue_line(sd_line_slot_t *slot)
{
  const ULONG message = (ULONG)(uintptr_t)slot;
  if (tx_queue_send(&g_sd_queue, (VOID *)&message, TX_NO_WAIT) != TX_SUCCESS)
  {
    g_sd_line_drop_count++;
    sd_free_slot(slot);
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  return SD_CARD_STATUS_OK;
}

sd_card_status_t sd_card_log_packet(const SedsPacketView *pkt)
{
  if ((g_sd_ready == 0U) || (pkt == NULL) || sd_launch_finished())
  {
    return SD_CARD_STATUS_BUSY;
  }
  sd_line_slot_t *slot = sd_alloc_slot();
  if (slot == NULL)
  {
    g_sd_line_drop_count++;
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  char unix_text[21], monotonic_text[21];
  const uint64_t local_ms = telemetry_now_ms();
  const uint64_t network_ms = telemetry_unix_ms();
  sd_format_u64(unix_text, daq_timestamp_ms(network_ms, local_ms));
  sd_format_u64(monotonic_text, local_ms);
  (void)sd_calibration_snapshot(&slot->calibration);
  slot->session = sd_run_snapshot();
  const int prefix = snprintf(slot->line, sizeof(slot->line), "%s,%s,seds_packet,",
                              unix_text, monotonic_text);
  if ((prefix <= 0) || ((size_t)prefix >= sizeof(slot->line) - 16U))
  {
    sd_free_slot(slot);
    return SD_CARD_STATUS_IO_ERROR;
  }
  int32_t want = seds_pkt_to_string_len(pkt);
  if (want < 0)
  {
    sd_free_slot(slot);
    return SD_CARD_STATUS_IO_ERROR;
  }
  const size_t available = sizeof(slot->line) - (size_t)prefix - 16U;
  if ((size_t)want > available)
  {
    want = (int32_t)available;
  }
  if (seds_pkt_to_string(pkt, &slot->line[prefix], (size_t)want + 1U) != SEDS_OK)
  {
    sd_free_slot(slot);
    return SD_CARD_STATUS_IO_ERROR;
  }
  const int suffix = snprintf(&slot->line[prefix + want],
      sizeof(slot->line) - (size_t)(prefix + want), ",,,,%s,,",
      network_ms != 0U ? "network" : "local");
  if (suffix <= 0 || (size_t)suffix + 2U >= sizeof(slot->line) - (size_t)(prefix + want))
  {
    sd_free_slot(slot);
    return SD_CARD_STATUS_IO_ERROR;
  }
  want += suffix;
  slot->line[prefix + want] = '\r';
  slot->line[prefix + want + 1] = '\n';
  slot->len = (uint16_t)(prefix + want + 2);
  return sd_enqueue_line(slot);
}

sd_card_status_t sd_card_enqueue_csv_row(const char *sensor_name,
                                         uint64_t timestamp_ms,
                                         float value,
                                         const daq_calibration_t *calibration)
{
  /* The bounded row queue can accept startup records while the writer mounts
   * the card. Do not discard the first network row merely because mounting
   * runs in another task. A missing/slow card still produces backpressure
   * when the fixed pool fills; no memory is allocated dynamically. */
  if ((g_sd_services_initialized == 0U) || (sensor_name == NULL) || sd_launch_finished())
  {
    return SD_CARD_STATUS_BUSY;
  }
  sd_line_slot_t *slot = sd_alloc_slot();
  if (slot == NULL)
  {
    g_sd_line_drop_count++;
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  if (calibration != NULL) slot->calibration = *calibration;
  else (void)sd_calibration_snapshot(&slot->calibration);
  slot->session = sd_run_snapshot();
  char value_text[24];
  sd_format_float(value_text, value);
  char unix_text[21], monotonic_text[21];
  const uint64_t network_ms = daq_sample_network_ms(
      telemetry_unix_ms(), telemetry_now_ms(), timestamp_ms);
  sd_format_u64(unix_text, daq_timestamp_ms(network_ms, timestamp_ms));
  sd_format_u64(monotonic_text, timestamp_ms);
  const int len = snprintf(slot->line, sizeof(slot->line), "%s,%s,%s,%s,,,,%s,,\r\n",
                           unix_text, monotonic_text,
                           sensor_name, value_text, network_ms != 0U ? "network" : "local");
  if ((len <= 0) || ((size_t)len >= sizeof(slot->line)))
  {
    sd_free_slot(slot);
    return SD_CARD_STATUS_IO_ERROR;
  }
  slot->len = (uint16_t)len;
  return sd_enqueue_line(slot);
}

sd_card_status_t sd_card_enqueue_raw_adc_samples(const sd_raw_adc_record_t *samples,
                                                 uint16_t count,
                                                 const daq_calibration_t *calibration)
{
  if ((g_sd_ready == 0U) || (samples == NULL) || (calibration == NULL) || (count == 0U) ||
      (count > SD_RAW_BATCH_MAX) || sd_launch_finished())
  {
    return SD_CARD_STATUS_BUSY;
  }
  sd_raw_slot_t *slot = sd_alloc_raw_slot();
  if (slot == NULL)
  {
    g_sd_raw_batch_drop_count++;
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  slot->calibration = *calibration;
  slot->session = sd_run_snapshot();
  memcpy(slot->samples, samples, (size_t)count * sizeof(samples[0]));
  slot->count = count;
  const ULONG message = (ULONG)(uintptr_t)slot;
  if (tx_queue_send(&g_sd_raw_queue, (VOID *)&message, TX_NO_WAIT) != TX_SUCCESS)
  {
    g_sd_raw_batch_drop_count++;
    sd_free_raw_slot(slot);
    return SD_CARD_STATUS_BACKPRESSURE;
  }
  return SD_CARD_STATUS_OK;
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
  if ((g_sd_services_initialized != 0U) &&
      (tx_mutex_get(&g_sd_pool_mutex, TX_WAIT_FOREVER) == TX_SUCCESS))
  {
    if (memcmp(&g_sd_calibration, calibration, sizeof(*calibration)) != 0)
    {
      g_sd_calibration = *calibration;
      g_sd_calibration_generation++;
      if (finished) sd_card_set_launch_clock(0U, 0U);
    }
    (void)tx_mutex_put(&g_sd_pool_mutex);
    return;
  }
  if (memcmp(&g_sd_calibration, calibration, sizeof(*calibration)) != 0)
  {
    g_sd_calibration = *calibration;
    g_sd_calibration_generation++;
    if (finished) sd_card_set_launch_clock(0U, 0U);
  }
}
