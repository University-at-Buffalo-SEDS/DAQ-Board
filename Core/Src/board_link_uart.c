#include "board_link_uart.h"

#include "tx_api.h"
#include <string.h>

#define BOARD_LINK_UART_RX_BUF_SIZE 256U
#define BOARD_LINK_UART_RX_RING_DEPTH 32U
#define BOARD_LINK_UART_TX_QUEUE_DEPTH 16U
#define BOARD_LINK_UART_MAX_SUBSCRIBERS 4U
#define BOARD_LINK_UART_TX_FRAMES_PER_SERVICE 2U
#define BOARD_LINK_UART_FRAME_SYNC_0 0xA5U
#define BOARD_LINK_UART_FRAME_SYNC_1 0x5AU
#define BOARD_LINK_UART_FRAME_HEADER_SIZE 4U
#define BOARD_LINK_UART_MAX_PAYLOAD_SIZE 256U
#define BOARD_LINK_UART_FRAME_BUF_SIZE \
  (BOARD_LINK_UART_FRAME_HEADER_SIZE + BOARD_LINK_UART_MAX_PAYLOAD_SIZE)
#define BOARD_LINK_UART_TX_TIMEOUT_FLOOR_MS 20U
#define BOARD_LINK_UART_TX_TIMEOUT_MARGIN_MS 20U

typedef struct {
  board_link_uart_rx_cb_t cb;
  void *user;
} BoardLinkUartSub;

typedef struct {
  uint16_t len;
  uint8_t data[BOARD_LINK_UART_RX_BUF_SIZE];
} BoardLinkUartRxItem;

typedef struct {
  uint16_t len;
  uint8_t data[BOARD_LINK_UART_FRAME_BUF_SIZE];
} BoardLinkUartTxItem;

static UART_HandleTypeDef *g_huart = NULL;
extern DCACHE_HandleTypeDef hdcache1;
static uint8_t g_rx_dma_buf[BOARD_LINK_UART_RX_BUF_SIZE];
static uint8_t g_frame_buf[BOARD_LINK_UART_FRAME_BUF_SIZE];
static size_t g_frame_len = 0U;
static BoardLinkUartSub g_subs[BOARD_LINK_UART_MAX_SUBSCRIBERS];

static volatile uint32_t g_rx_head = 0U;
static volatile uint32_t g_rx_tail = 0U;
static volatile uint32_t g_rx_count = 0U;
static BoardLinkUartRxItem g_rx_ring[BOARD_LINK_UART_RX_RING_DEPTH];

static volatile uint32_t g_rx_isr_drops = 0U;
static volatile uint32_t g_rx_frames_ok = 0U;
static volatile uint32_t g_rx_sync_loss = 0U;
static volatile uint32_t g_rx_bad_len = 0U;
static volatile uint32_t g_rx_restart_errors = 0U;
static volatile uint32_t g_rx_errors = 0U;
static volatile uint32_t g_tx_ok = 0U;
static volatile uint32_t g_tx_errors = 0U;
static volatile uint32_t g_tx_busy = 0U;
static volatile uint32_t g_tx_timeouts = 0U;
static volatile uint32_t g_tx_drops = 0U;

static BoardLinkUartTxItem g_tx_queue[BOARD_LINK_UART_TX_QUEUE_DEPTH];
static uint32_t g_tx_head = 0U;
static uint32_t g_tx_tail = 0U;
static uint32_t g_tx_count = 0U;

static uint32_t board_link_irq_save(void) {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void board_link_irq_restore(uint32_t primask) {
  if (primask == 0U) {
    __enable_irq();
  }
}

static void board_link_dcache_invalidate(const void *data, size_t len) {
  if (data == NULL || len == 0U) {
    return;
  }

  (void)HAL_DCACHE_InvalidateByAddr(&hdcache1, (const uint32_t *)data, (uint32_t)len);
}

static void board_link_dcache_clean(const void *data, size_t len) {
  if (data == NULL || len == 0U) {
    return;
  }

  (void)HAL_DCACHE_CleanByAddr(&hdcache1, (const uint32_t *)data, (uint32_t)len);
}

static void board_link_dcache_clean_invalidate(const void *data, size_t len) {
  if (data == NULL || len == 0U) {
    return;
  }

  (void)HAL_DCACHE_CleanInvalidByAddr(&hdcache1, (const uint32_t *)data, (uint32_t)len);
}

static uint32_t board_link_tx_timeout_ms(uint16_t len) {
  uint32_t baud = (g_huart != NULL) ? g_huart->Init.BaudRate : 0U;
  if (baud == 0U) {
    baud = 9600U;
  }

  uint64_t wire_ms = (((uint64_t)len * 10ULL * 1000ULL) + (uint64_t)baud - 1ULL) /
                     (uint64_t)baud;
  uint64_t timeout_ms = wire_ms + (uint64_t)BOARD_LINK_UART_TX_TIMEOUT_MARGIN_MS;

  if (timeout_ms < (uint64_t)BOARD_LINK_UART_TX_TIMEOUT_FLOOR_MS) {
    timeout_ms = (uint64_t)BOARD_LINK_UART_TX_TIMEOUT_FLOOR_MS;
  }

  return (timeout_ms > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)timeout_ms;
}

static HAL_StatusTypeDef board_link_transmit_dma_blocking(const uint8_t *data, uint16_t len) {
  const uint32_t start = tx_time_get();
  const uint32_t timeout_ticks =
      ((board_link_tx_timeout_ms(len) * TX_TIMER_TICKS_PER_SECOND) + 999U) / 1000U;
  HAL_StatusTypeDef status;

  if (g_huart == NULL || data == NULL || len == 0U) {
    return HAL_ERROR;
  }

  board_link_dcache_clean(data, len);
  status = HAL_UART_Transmit_DMA(g_huart, data, len);
  if (status != HAL_OK) {
    return status;
  }

  while (HAL_UART_GetState(g_huart) != HAL_UART_STATE_READY &&
         HAL_UART_GetState(g_huart) != HAL_UART_STATE_BUSY_RX) {
    if ((uint32_t)(tx_time_get() - start) > timeout_ticks) {
      (void)HAL_UART_AbortTransmit(g_huart);
      return HAL_TIMEOUT;
    }
    tx_thread_sleep(1U);
  }

  return HAL_OK;
}

static void board_link_notify_rx(const uint8_t *data, size_t len) {
  for (uint32_t i = 0U; i < BOARD_LINK_UART_MAX_SUBSCRIBERS; ++i) {
    if (g_subs[i].cb != NULL) {
      g_subs[i].cb(data, len, g_subs[i].user);
    }
  }
}

static void board_link_frame_buf_consume(size_t count) {
  if (count >= g_frame_len) {
    g_frame_len = 0U;
    return;
  }

  memmove(g_frame_buf, &g_frame_buf[count], g_frame_len - count);
  g_frame_len -= count;
}

static void board_link_frame_buf_append(const uint8_t *data, size_t len) {
  if (data == NULL || len == 0U) {
    return;
  }

  if (len >= BOARD_LINK_UART_FRAME_BUF_SIZE) {
    data += len - BOARD_LINK_UART_FRAME_BUF_SIZE;
    len = BOARD_LINK_UART_FRAME_BUF_SIZE;
    g_frame_len = 0U;
  } else if (g_frame_len + len > BOARD_LINK_UART_FRAME_BUF_SIZE) {
    board_link_frame_buf_consume(g_frame_len + len - BOARD_LINK_UART_FRAME_BUF_SIZE);
  }

  memcpy(&g_frame_buf[g_frame_len], data, len);
  g_frame_len += len;
}

static void board_link_process_framed_bytes(const uint8_t *data, size_t len) {
  board_link_frame_buf_append(data, len);

  while (g_frame_len > 0U) {
    size_t sync_pos = 0U;
    uint8_t found_sync = 0U;

    while ((sync_pos + 1U) < g_frame_len) {
      if (g_frame_buf[sync_pos] == BOARD_LINK_UART_FRAME_SYNC_0 &&
          g_frame_buf[sync_pos + 1U] == BOARD_LINK_UART_FRAME_SYNC_1) {
        found_sync = 1U;
        break;
      }
      ++sync_pos;
    }

    if (!found_sync) {
      g_rx_sync_loss++;
      if (g_frame_buf[g_frame_len - 1U] == BOARD_LINK_UART_FRAME_SYNC_0) {
        g_frame_buf[0] = BOARD_LINK_UART_FRAME_SYNC_0;
        g_frame_len = 1U;
      } else {
        g_frame_len = 0U;
      }
      return;
    }

    if (sync_pos > 0U) {
      board_link_frame_buf_consume(sync_pos);
    }

    if (g_frame_len < BOARD_LINK_UART_FRAME_HEADER_SIZE) {
      return;
    }

    const size_t payload_len =
        (size_t)g_frame_buf[2] | ((size_t)g_frame_buf[3] << 8U);
    if (payload_len == 0U || payload_len > BOARD_LINK_UART_MAX_PAYLOAD_SIZE) {
      g_rx_bad_len++;
      board_link_frame_buf_consume(1U);
      continue;
    }

    if (g_frame_len < (BOARD_LINK_UART_FRAME_HEADER_SIZE + payload_len)) {
      return;
    }

    board_link_notify_rx(&g_frame_buf[BOARD_LINK_UART_FRAME_HEADER_SIZE], payload_len);
    g_rx_frames_ok++;
    board_link_frame_buf_consume(BOARD_LINK_UART_FRAME_HEADER_SIZE + payload_len);
  }
}

static void board_link_rx_ring_push_isr(const uint8_t *data, uint16_t len) {
  if (data == NULL || len == 0U) {
    return;
  }
  if (len > BOARD_LINK_UART_RX_BUF_SIZE) {
    len = BOARD_LINK_UART_RX_BUF_SIZE;
  }

  if (g_rx_count >= BOARD_LINK_UART_RX_RING_DEPTH) {
    g_rx_isr_drops++;
    return;
  }

  g_rx_ring[g_rx_tail].len = len;
  memcpy(g_rx_ring[g_rx_tail].data, data, len);
  g_rx_tail = (g_rx_tail + 1U) % BOARD_LINK_UART_RX_RING_DEPTH;
  g_rx_count++;
}

static uint8_t board_link_rx_ring_pop(BoardLinkUartRxItem *out) {
  uint8_t have = 0U;
  const uint32_t primask = board_link_irq_save();

  if (out != NULL && g_rx_count > 0U) {
    *out = g_rx_ring[g_rx_head];
    g_rx_head = (g_rx_head + 1U) % BOARD_LINK_UART_RX_RING_DEPTH;
    g_rx_count--;
    have = 1U;
  }

  board_link_irq_restore(primask);
  return have;
}

static HAL_StatusTypeDef board_link_enqueue_frame(const uint8_t *payload, size_t len) {
  uint8_t slot;
  const uint32_t primask = board_link_irq_save();

  if (payload == NULL || len == 0U || len > BOARD_LINK_UART_MAX_PAYLOAD_SIZE) {
    board_link_irq_restore(primask);
    return HAL_ERROR;
  }

  if (g_tx_count >= BOARD_LINK_UART_TX_QUEUE_DEPTH) {
    g_tx_head = (g_tx_head + 1U) % BOARD_LINK_UART_TX_QUEUE_DEPTH;
    g_tx_count--;
    g_tx_drops++;
  }

  slot = (uint8_t)g_tx_tail;
  g_tx_queue[slot].len = (uint16_t)(BOARD_LINK_UART_FRAME_HEADER_SIZE + len);
  g_tx_queue[slot].data[0] = BOARD_LINK_UART_FRAME_SYNC_0;
  g_tx_queue[slot].data[1] = BOARD_LINK_UART_FRAME_SYNC_1;
  g_tx_queue[slot].data[2] = (uint8_t)(len & 0xFFU);
  g_tx_queue[slot].data[3] = (uint8_t)((len >> 8U) & 0xFFU);
  memcpy(&g_tx_queue[slot].data[BOARD_LINK_UART_FRAME_HEADER_SIZE], payload, len);

  g_tx_tail = (g_tx_tail + 1U) % BOARD_LINK_UART_TX_QUEUE_DEPTH;
  g_tx_count++;

  board_link_irq_restore(primask);
  return HAL_OK;
}

static uint8_t board_link_dequeue_frame(BoardLinkUartTxItem *out) {
  uint8_t have = 0U;
  const uint32_t primask = board_link_irq_save();

  if (out != NULL && g_tx_count > 0U) {
    *out = g_tx_queue[g_tx_head];
    g_tx_head = (g_tx_head + 1U) % BOARD_LINK_UART_TX_QUEUE_DEPTH;
    g_tx_count--;
    have = 1U;
  }

  board_link_irq_restore(primask);
  return have;
}

void board_link_uart_init(UART_HandleTypeDef *huart) {
  g_huart = huart;
  g_frame_len = 0U;
  g_rx_head = 0U;
  g_rx_tail = 0U;
  g_rx_count = 0U;
  g_tx_head = 0U;
  g_tx_tail = 0U;
  g_tx_count = 0U;
  memset(g_subs, 0, sizeof(g_subs));
}

HAL_StatusTypeDef board_link_uart_start_rx(void) {
  HAL_StatusTypeDef status;

  if (g_huart == NULL) {
    return HAL_ERROR;
  }

  board_link_dcache_clean_invalidate(g_rx_dma_buf, sizeof(g_rx_dma_buf));
  status = HAL_UARTEx_ReceiveToIdle_DMA(g_huart, g_rx_dma_buf, sizeof(g_rx_dma_buf));
  if (status == HAL_OK && g_huart->hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(g_huart->hdmarx, DMA_IT_HT);
  }
  return status;
}

void board_link_uart_process(void) {
  BoardLinkUartRxItem rx_item;
  BoardLinkUartTxItem tx_item;
  uint32_t sent = 0U;

  while (board_link_rx_ring_pop(&rx_item)) {
    board_link_process_framed_bytes(rx_item.data, (size_t)rx_item.len);
  }

  while (sent < BOARD_LINK_UART_TX_FRAMES_PER_SERVICE && board_link_dequeue_frame(&tx_item)) {
    HAL_StatusTypeDef status = board_link_transmit_dma_blocking(tx_item.data, tx_item.len);
    if (status != HAL_OK) {
      g_tx_errors++;
      if (status == HAL_BUSY) {
        g_tx_busy++;
      } else if (status == HAL_TIMEOUT) {
        g_tx_timeouts++;
      }
      break;
    }
    g_tx_ok++;
    sent++;
  }
}

SedsResult board_link_uart_tx_send(const uint8_t *bytes, size_t len, void *user) {
  (void)user;

  if (bytes == NULL || len == 0U) {
    return SEDS_BAD_ARG;
  }

  return (board_link_enqueue_frame(bytes, len) == HAL_OK) ? SEDS_OK : SEDS_IO;
}

HAL_StatusTypeDef board_link_uart_subscribe_rx(board_link_uart_rx_cb_t cb, void *user) {
  if (cb == NULL) {
    return HAL_ERROR;
  }

  for (uint32_t i = 0U; i < BOARD_LINK_UART_MAX_SUBSCRIBERS; ++i) {
    if (g_subs[i].cb == cb && g_subs[i].user == user) {
      return HAL_ERROR;
    }
  }

  for (uint32_t i = 0U; i < BOARD_LINK_UART_MAX_SUBSCRIBERS; ++i) {
    if (g_subs[i].cb == NULL) {
      g_subs[i].cb = cb;
      g_subs[i].user = user;
      return HAL_OK;
    }
  }

  return HAL_ERROR;
}

HAL_StatusTypeDef board_link_uart_unsubscribe_rx(board_link_uart_rx_cb_t cb, void *user) {
  for (uint32_t i = 0U; i < BOARD_LINK_UART_MAX_SUBSCRIBERS; ++i) {
    if (g_subs[i].cb == cb && g_subs[i].user == user) {
      g_subs[i].cb = NULL;
      g_subs[i].user = NULL;
      return HAL_OK;
    }
  }

  return HAL_ERROR;
}

void board_link_uart_handle_rx_event(UART_HandleTypeDef *huart, uint16_t size) {
  if (g_huart == NULL || huart == NULL || huart->Instance != g_huart->Instance) {
    return;
  }

  board_link_dcache_invalidate(g_rx_dma_buf, size);
  board_link_rx_ring_push_isr(g_rx_dma_buf, size);
  if (board_link_uart_start_rx() != HAL_OK) {
    g_rx_restart_errors++;
  }
}

void board_link_uart_handle_error(UART_HandleTypeDef *huart) {
  if (g_huart == NULL || huart == NULL || huart->Instance != g_huart->Instance) {
    return;
  }

  g_rx_errors++;
  (void)HAL_UART_AbortReceive(huart);
  if (board_link_uart_start_rx() != HAL_OK) {
    g_rx_restart_errors++;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  board_link_uart_handle_rx_event(huart, Size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  board_link_uart_handle_error(huart);
}
