#include "sd_card.h"
#include "daq_rates.h"

#include "fx_api.h"
#include "main.h"
#include "telemetry.h"
#include "sd_float_format.h"

#include <stdio.h>
#include <string.h>

extern SD_HandleTypeDef hsd1;
extern DCACHE_HandleTypeDef hdcache1;
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
  char line[SD_LINE_MAX];
} sd_line_slot_t;

typedef struct
{
  uint16_t count;
  uint8_t in_use;
  daq_calibration_t calibration;
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
static uint8_t g_power_loss_mode = 0U;
static uint8_t g_file_open = 0U;
static daq_calibration_t g_sd_calibration = {1.0f, 0.0f, 1.0f, 0.0f};
static volatile uint32_t g_sd_calibration_generation = 1U;
static daq_calibration_t g_sd_open_calibration;
static uint8_t g_sd_services_initialized = 0U;

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

static UINT sd_hal_read(UCHAR *destination, ULONG sector, ULONG sector_count)
{
  while (sector_count != 0U)
  {
    const ULONG count = (sector_count > SD_TRANSFER_SECTORS)
                            ? SD_TRANSFER_SECTORS
                            : sector_count;
    const size_t bytes = (size_t)count * SD_SECTOR_SIZE;

    if (HAL_SD_ReadBlocks(&hsd1, g_sd_transfer_buffer, sector, count, 2000U) != HAL_OK)
    {
      return FX_IO_ERROR;
    }
    (void)HAL_DCACHE_InvalidateByAddr(&hdcache1,
                                      (const uint32_t *)g_sd_transfer_buffer,
                                      (uint32_t)bytes);
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
    (void)HAL_DCACHE_CleanByAddr(&hdcache1,
                                 (const uint32_t *)g_sd_transfer_buffer,
                                 (uint32_t)bytes);
    if (HAL_SD_WriteBlocks(&hsd1, g_sd_transfer_buffer, sector, count, 2000U) != HAL_OK)
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
      status = sd_hal_write(media->fx_media_driver_buffer, 0U,
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

static UINT sd_open_timestamped_log(const daq_calibration_t *requested)
{
  const uint64_t stamp = telemetry_unix_ms();
  daq_calibration_t calibration;
  (void)sd_calibration_snapshot(&calibration);
  if (requested != NULL) calibration = *requested;

  for (uint32_t suffix = 0U; suffix < 1000U; ++suffix)
  {
    const uint64_t name_stamp = (stamp != 0U) ? stamp : telemetry_now_ms();
    char stamp_text[21];
    sd_format_u64(stamp_text, name_stamp);
    (void)snprintf(g_sd_filename, sizeof(g_sd_filename),
                   "DAQ_%s_%03lu.CSV",
                   stamp_text,
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

  UINT status = fx_file_open(&g_sd_media, &g_sd_file, g_sd_filename,
                             FX_OPEN_FOR_WRITE);
  if (status != FX_SUCCESS)
  {
    return status;
  }
  g_file_open = 1U;

  static const char header[] =
      "network_unix_ms,monotonic_ms,sensor,value,raw_adc_code,raw_value,calibrated_value\r\n";
  status = fx_file_write(&g_sd_file, (VOID *)header, sizeof(header) - 1U);
  if (status == FX_SUCCESS)
  {
    char calibration_line[192];
    char coefficients[4][24];
    sd_format_float(coefficients[0], calibration.kg1000_slope);
    sd_format_float(coefficients[1], calibration.kg1000_intercept);
    sd_format_float(coefficients[2], calibration.iadc_slope);
    sd_format_float(coefficients[3], calibration.iadc_intercept);
    const int len = snprintf(
        calibration_line, sizeof(calibration_line),
        "# calibration,kg1000_slope=%s,kg1000_intercept=%s,iadc_slope=%s,iadc_intercept=%s\r\n",
        coefficients[0], coefficients[1], coefficients[2], coefficients[3]);
    if (len <= 0 || (size_t)len >= sizeof(calibration_line)) return FX_IO_ERROR;
    status = fx_file_write(&g_sd_file, calibration_line, (ULONG)len);
  }
  if (status == FX_SUCCESS)
  {
    status = fx_media_flush(&g_sd_media);
    g_sd_open_calibration = calibration;
  }
  return status;
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
    return FX_SUCCESS;
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
  if (memcmp(&g_sd_open_calibration, calibration, sizeof(*calibration)) == 0)
    return FX_SUCCESS;
  if (sd_flush_pending() != FX_SUCCESS || fx_media_flush(&g_sd_media) != FX_SUCCESS)
    return FX_IO_ERROR;
  if (fx_file_close(&g_sd_file) != FX_SUCCESS) return FX_IO_ERROR;
  g_file_open = 0U;
  return sd_open_timestamped_log(calibration);
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
        if ((HAL_SD_Init(&hsd1) == HAL_OK) &&
            ((g_sd_init_stage = 2U) != 0U) &&
            (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) == HAL_OK))
        {
          g_sd_hardware_ready = 1U;
          g_sd_init_stage = 3U;
          if ((sd_mount_or_provision() == FX_SUCCESS) &&
              ((g_sd_init_stage = 4U) != 0U) &&
              (sd_open_timestamped_log(NULL) == FX_SUCCESS))
          {
            g_sd_ready = 1U;
            g_sd_init_stage = 5U;
            failed_retries = 0U;
            (void)log_telemetry_string_asynchronous(
                SEDS_DT_WARNING, "DAQ SD card is available; logging started");
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

    /* When idle, rotate even if the new calibration has not produced a
     * sample yet. Busy queues are handled per record using their own snapshot. */
    if (g_sd_queue.tx_queue_enqueued == 0U && g_sd_raw_queue.tx_queue_enqueued == 0U)
    {
      daq_calibration_t calibration;
      (void)sd_calibration_snapshot(&calibration);
      if (sd_select_calibration(&calibration) != FX_SUCCESS)
      {
        g_sd_ready = 0U;
        g_sd_write_error_count++;
        continue;
      }
    }

    /* Bound raw work so a continuously replenished ADC queue cannot starve
     * live/replay rows, calibration rotation, or flush requests. */
    if (tx_queue_receive(&g_sd_raw_queue, &message, TX_NO_WAIT) == TX_SUCCESS)
    {
      serviced_work = 1U;
      sd_raw_slot_t *slot = (sd_raw_slot_t *)(uintptr_t)message;
      char line[128];
      if (slot != NULL)
      {
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
          char raw_text[24], calibrated_text[24];
          sd_format_float(raw_text, sample->raw_value);
          sd_format_float(calibrated_text, sample->calibrated_value);
          char stamp_text[21];
          sd_format_u64(stamp_text, sample->network_unix_ms);
          const int len = snprintf(line, sizeof(line), "%s,%lu,mcp3564r_raw,,%ld,%s,%s\r\n",
                                   stamp_text,
                                   (unsigned long)sample->monotonic_ms,
                                   (long)sample->raw_adc_code,
                                   raw_text, calibrated_text);
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

    if (tx_queue_receive(&g_sd_queue, &message, TX_NO_WAIT) == TX_SUCCESS)
    {
      serviced_work = 1U;
      sd_line_slot_t *slot = (sd_line_slot_t *)(uintptr_t)message;
      if (slot != NULL)
      {
        if (sd_select_calibration(&slot->calibration) == FX_SUCCESS &&
            sd_write_bytes(slot->line, slot->len) == FX_SUCCESS)
        {
          g_sd_csv_rows_written_count++;
        }
        else
        {
          g_sd_write_error_count++;
        }
        sd_free_slot(slot);
      }
    }

    if (tx_event_flags_get(&g_sd_flags,
                           SD_FLAG_FLUSH_REQUEST | SD_FLAG_POWER_LOSS,
                           TX_OR_CLEAR, &flags, TX_NO_WAIT) == TX_SUCCESS)
    {
      if ((flags & SD_FLAG_POWER_LOSS) != 0U)
      {
        g_power_loss_mode = 1U;
      }
    }

    if ((flags != 0U) ||
        ((tx_time_get() - last_flush) >= TX_TIMER_TICKS_PER_SECOND))
    {
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

    if (g_power_loss_mode != 0U)
    {
      tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2U);
    }
    else if (serviced_work == 0U)
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
  g_sd_services_initialized = 1U;

  /* A missing card is not a board-startup failure. The writer task retries
   * insertion and reports the condition after SEDSNet starts. */
  if ((g_sd_hardware_ready != 0U) &&
      (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) == HAL_OK) &&
      (sd_mount_or_provision() == FX_SUCCESS) &&
      (sd_open_timestamped_log(NULL) == FX_SUCCESS))
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
  if ((g_sd_ready == 0U) || (pkt == NULL) || (g_power_loss_mode != 0U))
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
  sd_format_u64(unix_text, telemetry_unix_ms());
  sd_format_u64(monotonic_text, telemetry_now_ms());
  (void)sd_calibration_snapshot(&slot->calibration);
  const int prefix = snprintf(slot->line, sizeof(slot->line), "%s,%s,seds_packet,",
                              unix_text, monotonic_text);
  if ((prefix <= 0) || ((size_t)prefix >= sizeof(slot->line) - 3U))
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
  const size_t available = sizeof(slot->line) - (size_t)prefix - 3U;
  if ((size_t)want > available)
  {
    want = (int32_t)available;
  }
  if (seds_pkt_to_string(pkt, &slot->line[prefix], (size_t)want + 1U) != SEDS_OK)
  {
    sd_free_slot(slot);
    return SD_CARD_STATUS_IO_ERROR;
  }
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
  if ((g_sd_ready == 0U) || (sensor_name == NULL) || (g_power_loss_mode != 0U))
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
  char value_text[24];
  sd_format_float(value_text, value);
  char unix_text[21], monotonic_text[21];
  sd_format_u64(unix_text, telemetry_unix_ms());
  sd_format_u64(monotonic_text, timestamp_ms);
  const int len = snprintf(slot->line, sizeof(slot->line), "%s,%s,%s,%s,,,\r\n",
                           unix_text, monotonic_text,
                           sensor_name, value_text);
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
      (count > SD_RAW_BATCH_MAX) || (g_power_loss_mode != 0U))
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
  if ((g_sd_services_initialized != 0U) &&
      (tx_mutex_get(&g_sd_pool_mutex, TX_WAIT_FOREVER) == TX_SUCCESS))
  {
    if (memcmp(&g_sd_calibration, calibration, sizeof(*calibration)) != 0)
    {
      g_sd_calibration = *calibration;
      g_sd_calibration_generation++;
    }
    (void)tx_mutex_put(&g_sd_pool_mutex);
    return;
  }
  if (memcmp(&g_sd_calibration, calibration, sizeof(*calibration)) != 0)
  {
    g_sd_calibration = *calibration;
    g_sd_calibration_generation++;
  }
}
