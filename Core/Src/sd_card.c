#include "sd_card.h"

#include "main.h"
#include "telemetry.h"

#include <stdio.h>
#include <string.h>

extern SD_HandleTypeDef hsd1;

#define SD_WRITER_STACK_SIZE (8U * 1024U)
#define SD_QUEUE_DEPTH 24U
#define SD_LINE_MAX 384U

#define SD_CHUNK_MAGIC 0x44414A51UL
#define SD_CHUNK_VERSION 1U
#define SD_CHUNK_SECTOR_COUNT 8U
#define SD_SECTOR_SIZE 512U
#define SD_CHUNK_SIZE_BYTES (SD_CHUNK_SECTOR_COUNT * SD_SECTOR_SIZE)
#define SD_LOG_START_BLOCK 4096U
#define SD_LOG_SLOT_COUNT 256U

#define SD_FLAG_FLUSH_REQUEST 0x00000001UL
#define SD_FLAG_POWER_LOSS 0x00000002UL

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t flags;
  uint32_t sequence;
  uint32_t payload_len;
  uint32_t payload_crc;
  uint64_t unix_ms;
  uint64_t monotonic_ms;
} sd_chunk_header_t;

typedef struct
{
  uint16_t len;
  uint8_t in_use;
  char line[SD_LINE_MAX];
} sd_line_slot_t;

static TX_QUEUE g_sd_queue;
static ULONG g_sd_queue_storage[SD_QUEUE_DEPTH];
static TX_MUTEX g_sd_pool_mutex;
static TX_EVENT_FLAGS_GROUP g_sd_flags;
static TX_SEMAPHORE g_sd_dma_semaphore;
static sd_line_slot_t g_sd_slots[SD_QUEUE_DEPTH];

static uint8_t g_sd_ready = 0U;
static uint8_t g_sd_hardware_ready = 0U;
static uint8_t g_power_loss_mode = 0U;
static uint8_t g_sd_dma_error = 0U;
static uint32_t g_next_sequence = 1U;
static uint32_t g_write_slot_index = 0U;
static HAL_SD_CardInfoTypeDef g_card_info = {0};

static UCHAR g_chunk_buffer[SD_CHUNK_SIZE_BYTES];
static UCHAR g_scan_buffer[SD_CHUNK_SIZE_BYTES];
static UINT g_chunk_payload_len = 0U;

static uint32_t sd_crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
  crc = ~crc;
  for (size_t i = 0; i < len; ++i)
  {
    crc ^= data[i];
    for (uint32_t bit = 0; bit < 8U; ++bit)
    {
      const uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
      crc = (crc >> 1) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

static uint32_t sd_chunk_payload_capacity(void)
{
  return SD_CHUNK_SIZE_BYTES - (uint32_t)sizeof(sd_chunk_header_t);
}

static sd_line_slot_t *sd_alloc_slot(void)
{
  sd_line_slot_t *slot = NULL;

  if (tx_mutex_get(&g_sd_pool_mutex, TX_WAIT_FOREVER) != TX_SUCCESS)
  {
    return NULL;
  }

  for (uint32_t i = 0; i < SD_QUEUE_DEPTH; ++i)
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
  if (slot == NULL)
  {
    return;
  }

  if (tx_mutex_get(&g_sd_pool_mutex, TX_WAIT_FOREVER) == TX_SUCCESS)
  {
    slot->in_use = 0U;
    slot->len = 0U;
    (void)tx_mutex_put(&g_sd_pool_mutex);
  }
}

static UINT sd_wait_for_transfer_complete(void)
{
  if (tx_semaphore_get(&g_sd_dma_semaphore, TX_TIMER_TICKS_PER_SECOND) != TX_SUCCESS)
  {
    return TX_NOT_DONE;
  }

  if (g_sd_dma_error != 0U)
  {
    g_sd_dma_error = 0U;
    return TX_NOT_DONE;
  }

  for (uint32_t retry = 0U; retry < 1000U; ++retry)
  {
    if (HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER)
    {
      return TX_SUCCESS;
    }
    tx_thread_sleep(1U);
  }

  return TX_NOT_DONE;
}

static UINT sd_read_slot(uint32_t slot_index, UCHAR *dst)
{
  const uint32_t block = SD_LOG_START_BLOCK + (slot_index * SD_CHUNK_SECTOR_COUNT);
  const HAL_StatusTypeDef st = HAL_SD_ReadBlocks(&hsd1,
                                                 dst,
                                                 block,
                                                 SD_CHUNK_SECTOR_COUNT,
                                                 1000U);

  return (st == HAL_OK) ? TX_SUCCESS : TX_NOT_DONE;
}

static UINT sd_write_slot_dma(uint32_t slot_index, const UCHAR *src)
{
  const uint32_t block = SD_LOG_START_BLOCK + (slot_index * SD_CHUNK_SECTOR_COUNT);
  const HAL_StatusTypeDef st = HAL_SD_WriteBlocks_DMA(&hsd1,
                                                      src,
                                                      block,
                                                      SD_CHUNK_SECTOR_COUNT);

  if (st != HAL_OK)
  {
    return TX_NOT_DONE;
  }

  return sd_wait_for_transfer_complete();
}

static UINT sd_prepare_media(void)
{
  if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK)
  {
    return TX_NOT_DONE;
  }

  if (HAL_SD_GetCardInfo(&hsd1, &g_card_info) != HAL_OK)
  {
    return TX_NOT_DONE;
  }

  if ((SD_LOG_START_BLOCK + (SD_LOG_SLOT_COUNT * SD_CHUNK_SECTOR_COUNT)) > g_card_info.LogBlockNbr)
  {
    return TX_SIZE_ERROR;
  }

  return TX_SUCCESS;
}

static UINT sd_slot_is_valid(const UCHAR *slot_bytes, const sd_chunk_header_t **header_out)
{
  const sd_chunk_header_t *header = (const sd_chunk_header_t *)slot_bytes;
  const uint8_t *payload = slot_bytes + sizeof(sd_chunk_header_t);
  const uint32_t capacity = sd_chunk_payload_capacity();

  if ((header->magic != SD_CHUNK_MAGIC) || (header->version != SD_CHUNK_VERSION))
  {
    return TX_NOT_DONE;
  }

  if (header->payload_len > capacity)
  {
    return TX_NOT_DONE;
  }

  if (sd_crc32_update(0U, payload, header->payload_len) != header->payload_crc)
  {
    return TX_NOT_DONE;
  }

  *header_out = header;
  return TX_SUCCESS;
}

static UINT sd_scan_log_area(void)
{
  uint8_t found_free = 0U;
  uint32_t highest_seq = 0U;
  uint32_t highest_index = 0U;

  for (uint32_t index = 0U; index < SD_LOG_SLOT_COUNT; ++index)
  {
    const sd_chunk_header_t *header = NULL;

    if (sd_read_slot(index, g_scan_buffer) != TX_SUCCESS)
    {
      return TX_NOT_DONE;
    }

    if (sd_slot_is_valid(g_scan_buffer, &header) != TX_SUCCESS)
    {
      g_write_slot_index = index;
      found_free = 1U;
      break;
    }

    if (header->sequence >= highest_seq)
    {
      highest_seq = header->sequence;
      highest_index = index;
    }
  }

  if (found_free == 0U)
  {
    g_write_slot_index = (highest_index + 1U) % SD_LOG_SLOT_COUNT;
  }

  g_next_sequence = highest_seq + 1U;
  return TX_SUCCESS;
}

static void sd_reset_chunk(void)
{
  memset(g_chunk_buffer, 0xFF, sizeof(g_chunk_buffer));
  g_chunk_payload_len = 0U;
}

static UINT sd_commit_chunk(uint16_t flags)
{
  sd_chunk_header_t *header;
  uint8_t *payload;

  if (g_chunk_payload_len == 0U)
  {
    return TX_SUCCESS;
  }

  header = (sd_chunk_header_t *)g_chunk_buffer;
  payload = g_chunk_buffer + sizeof(sd_chunk_header_t);

  header->magic = SD_CHUNK_MAGIC;
  header->version = SD_CHUNK_VERSION;
  header->flags = flags;
  header->sequence = g_next_sequence++;
  header->payload_len = g_chunk_payload_len;
  header->payload_crc = sd_crc32_update(0U, payload, g_chunk_payload_len);
  header->unix_ms = telemetry_unix_ms();
  header->monotonic_ms = telemetry_now_ms();

  if (sd_write_slot_dma(g_write_slot_index, g_chunk_buffer) != TX_SUCCESS)
  {
    return TX_NOT_DONE;
  }

  g_write_slot_index = (g_write_slot_index + 1U) % SD_LOG_SLOT_COUNT;
  sd_reset_chunk();
  return TX_SUCCESS;
}

static UINT sd_append_bytes(const uint8_t *bytes, uint16_t len)
{
  uint16_t remaining = len;
  const uint8_t *cursor = bytes;
  const uint32_t capacity = sd_chunk_payload_capacity();

  while (remaining > 0U)
  {
    const uint32_t space = capacity - g_chunk_payload_len;
    const uint16_t copy_len = (remaining < space) ? remaining : (uint16_t)space;

    memcpy(g_chunk_buffer + sizeof(sd_chunk_header_t) + g_chunk_payload_len, cursor, copy_len);
    g_chunk_payload_len += copy_len;
    cursor += copy_len;
    remaining = (uint16_t)(remaining - copy_len);

    if (g_chunk_payload_len >= capacity)
    {
      if (sd_commit_chunk(0U) != TX_SUCCESS)
      {
        return TX_NOT_DONE;
      }
    }
  }

  return TX_SUCCESS;
}

void sd_card_writer_thread_entry(ULONG initial_input)
{
  ULONG msg = 0U;

  (void)initial_input;

  sd_reset_chunk();

  for (;;)
  {
    ULONG actual_flags = 0U;

    if (tx_queue_receive(&g_sd_queue, &msg, TX_TIMER_TICKS_PER_SECOND / 4U) == TX_SUCCESS)
    {
      sd_line_slot_t *slot = (sd_line_slot_t *)(uintptr_t)msg;

      if (slot != NULL)
      {
        if (sd_append_bytes((const uint8_t *)slot->line, slot->len) != TX_SUCCESS)
        {
          g_sd_ready = 0U;
        }
        sd_free_slot(slot);
      }
    }

    if (tx_event_flags_get(&g_sd_flags,
                           SD_FLAG_FLUSH_REQUEST | SD_FLAG_POWER_LOSS,
                           TX_OR_CLEAR,
                           &actual_flags,
                           TX_NO_WAIT) == TX_SUCCESS)
    {
      uint16_t flags = 0U;

      if ((actual_flags & SD_FLAG_POWER_LOSS) != 0U)
      {
        g_power_loss_mode = 1U;
        flags |= (uint16_t)SD_FLAG_POWER_LOSS;
      }

      if ((actual_flags & (SD_FLAG_FLUSH_REQUEST | SD_FLAG_POWER_LOSS)) != 0U)
      {
        flags |= (uint16_t)SD_FLAG_FLUSH_REQUEST;
        (void)sd_commit_chunk(flags);
      }
    }

    if (g_power_loss_mode != 0U)
    {
      tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2U);
    }
  }
}

UINT sd_card_init(TX_BYTE_POOL *byte_pool)
{
  (void)byte_pool;

  if (g_sd_hardware_ready == 0U)
  {
    return TX_NOT_DONE;
  }

  if (g_sd_ready != 0U)
  {
    return TX_SUCCESS;
  }

  if (tx_mutex_create(&g_sd_pool_mutex, "sd_pool", TX_NO_INHERIT) != TX_SUCCESS)
  {
    return TX_MUTEX_ERROR;
  }

  if (tx_queue_create(&g_sd_queue, "sd_queue", TX_1_ULONG, g_sd_queue_storage, sizeof(g_sd_queue_storage)) != TX_SUCCESS)
  {
    return TX_QUEUE_ERROR;
  }

  if (tx_event_flags_create(&g_sd_flags, "sd_flags") != TX_SUCCESS)
  {
    return TX_GROUP_ERROR;
  }

  if (tx_semaphore_create(&g_sd_dma_semaphore, "sd_dma", 0U) != TX_SUCCESS)
  {
    return TX_SEMAPHORE_ERROR;
  }

  if (sd_prepare_media() != TX_SUCCESS)
  {
    return TX_NOT_DONE;
  }

  if (sd_scan_log_area() != TX_SUCCESS)
  {
    return TX_NOT_DONE;
  }

  g_sd_ready = 1U;
  return TX_SUCCESS;
}

void sd_card_set_hardware_ready(uint8_t ready)
{
  g_sd_hardware_ready = (ready != 0U) ? 1U : 0U;
}

sd_card_status_t sd_card_log_packet(const SedsPacketView *pkt)
{
  sd_line_slot_t *slot;
  int32_t want;
  int32_t rc;
  ULONG msg;

  if ((g_sd_ready == 0U) || (pkt == NULL) || (g_power_loss_mode != 0U))
  {
    return SD_CARD_STATUS_BUSY;
  }

  slot = sd_alloc_slot();
  if (slot == NULL)
  {
    return SD_CARD_STATUS_BACKPRESSURE;
  }

  want = seds_pkt_to_string_len(pkt);
  if (want < 0)
  {
    sd_free_slot(slot);
    return SD_CARD_STATUS_IO_ERROR;
  }

  if ((uint32_t)want >= (SD_LINE_MAX - 2U))
  {
    want = (int32_t)(SD_LINE_MAX - 3U);
  }

  rc = seds_pkt_to_string(pkt, slot->line, (size_t)(want + 1));
  if (rc != SEDS_OK)
  {
    sd_free_slot(slot);
    return SD_CARD_STATUS_IO_ERROR;
  }

  slot->line[want] = '\n';
  slot->len = (uint16_t)(want + 1);

  msg = (ULONG)(uintptr_t)slot;
  if (tx_queue_send(&g_sd_queue, &msg, TX_NO_WAIT) != TX_SUCCESS)
  {
    sd_free_slot(slot);
    return SD_CARD_STATUS_BACKPRESSURE;
  }

  return SD_CARD_STATUS_OK;
}

sd_card_status_t sd_card_enqueue_csv_row(const char *sensor_name,
                                         uint64_t timestamp_ms,
                                         float value)
{
  sd_line_slot_t *slot;
  ULONG msg;
  int len;

  if ((g_sd_ready == 0U) || (sensor_name == NULL) || (g_power_loss_mode != 0U))
  {
    return SD_CARD_STATUS_BUSY;
  }

  slot = sd_alloc_slot();
  if (slot == NULL)
  {
    return SD_CARD_STATUS_BACKPRESSURE;
  }

  len = snprintf(slot->line,
                 sizeof(slot->line),
                 "%llu,%s,%.6f\n",
                 (unsigned long long)timestamp_ms,
                 sensor_name,
                 (double)value);
  if ((len <= 0) || (len >= (int)sizeof(slot->line)))
  {
    sd_free_slot(slot);
    return SD_CARD_STATUS_IO_ERROR;
  }

  slot->len = (uint16_t)len;
  msg = (ULONG)(uintptr_t)slot;
  if (tx_queue_send(&g_sd_queue, &msg, TX_NO_WAIT) != TX_SUCCESS)
  {
    sd_free_slot(slot);
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

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
  if (hsd == &hsd1)
  {
    g_sd_dma_error = 0U;
    (void)tx_semaphore_put(&g_sd_dma_semaphore);
  }
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
  if (hsd == &hsd1)
  {
    g_sd_dma_error = 0U;
    (void)tx_semaphore_put(&g_sd_dma_semaphore);
  }
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
  if (hsd == &hsd1)
  {
    g_sd_dma_error = 1U;
    (void)tx_semaphore_put(&g_sd_dma_semaphore);
  }
}
