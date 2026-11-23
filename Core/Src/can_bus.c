// can_bus.c
//
// CAN / CAN-FD helper with:
//  - Subscriber fanout
//  - Correct HAL DLC handling (DataLength is a DLC code, not byte count)
//  - Optional fragmentation/reassembly layer so you can send >64B buffers
//  - ISR does *minimal work*: drains HW FIFO into a lock-free ring buffer
//  - Thread/main-loop calls can_bus_process_rx() to reassemble + notify
//    subscribers
//
// Notes / Assumptions:
//  - Uses CAN FD frames for fragmentation (default payload 64 bytes).
//  - Fragment frames are distinguished by a small "magic" header in the
//  payload.
//  - Reassembly is bounded (no malloc). New RX frames are dropped on ring
//  overflow.
//  - One producer (ISR) and one consumer (thread calling can_bus_process_rx()).
//  - You can call can_bus_process_rx() from a ThreadX thread, or main
//  superloop.
//
// IMPORTANT CONCURRENCY NOTE:
//  `volatile` head/tail alone does NOT guarantee publish/consume ordering for
//  the ring slot contents by the C language rules. We add `__DMB()` barriers to
//  ensure the slot is fully written before publishing `head` (release), and
//  ensure the consumer sees the slot contents after observing `head` (acquire).

#include "board_clock_us.h"
#include "can_bus.h"
#include "can_tx_queue.h"
#include "main.h"
#include <stdint.h>
#include <string.h>

// CMSIS barrier intrinsics (for __DMB()).
// On STM32 this is typically available via core_cm*.h brought in by
// stm32xx_hal.h, but including CMSIS directly is safest if your build allows
// it.
#if defined(__ARMCC_VERSION) || defined(__GNUC__) || defined(__ICCARM__)
#include "cmsis_compiler.h"
#endif

#ifndef CAN_BUS_MAX_SUBSCRIBERS
#define CAN_BUS_MAX_SUBSCRIBERS 8
#endif

#ifndef CAN_BUS_POLLING
#define CAN_BUS_POLLING 0
#endif


// =========================
// FD DLC helpers
// =========================

static size_t can_bus_dlc_to_len(uint32_t dlc) {
  static const uint8_t map[16] = {0, 1,  2,  3,  4,  5,  6,  7,
                                  8, 12, 16, 20, 24, 32, 48, 64};
  dlc &= 0xF;
  return map[dlc];
}

static uint32_t can_bus_len_to_dlc(size_t len) {
  switch (len) {
  case 0:
    return FDCAN_DLC_BYTES_0;
  case 1:
    return FDCAN_DLC_BYTES_1;
  case 2:
    return FDCAN_DLC_BYTES_2;
  case 3:
    return FDCAN_DLC_BYTES_3;
  case 4:
    return FDCAN_DLC_BYTES_4;
  case 5:
    return FDCAN_DLC_BYTES_5;
  case 6:
    return FDCAN_DLC_BYTES_6;
  case 7:
    return FDCAN_DLC_BYTES_7;
  case 8:
    return FDCAN_DLC_BYTES_8;
  case 12:
    return FDCAN_DLC_BYTES_12;
  case 16:
    return FDCAN_DLC_BYTES_16;
  case 20:
    return FDCAN_DLC_BYTES_20;
  case 24:
    return FDCAN_DLC_BYTES_24;
  case 32:
    return FDCAN_DLC_BYTES_32;
  case 48:
    return FDCAN_DLC_BYTES_48;
  case 64:
    return FDCAN_DLC_BYTES_64;
  default:
    return 0xFFFFFFFFu;
  }
}

static size_t can_bus_round_up_fd_len(size_t len) {
  if (len <= 8)
    return len;
  if (len <= 12)
    return 12;
  if (len <= 16)
    return 16;
  if (len <= 20)
    return 20;
  if (len <= 24)
    return 24;
  if (len <= 32)
    return 32;
  if (len <= 48)
    return 48;
  if (len <= 64)
    return 64;
  return 64;
}

// =========================
// Subscriber fanout
// =========================

typedef struct {
  can_bus_rx_cb_t cb;
  void *user;
} can_bus_sub_t;

static FDCAN_HandleTypeDef *g_hfdcan = NULL;
static can_bus_sub_t g_subs[CAN_BUS_MAX_SUBSCRIBERS];

/* Receive time of the frame being delivered (0 outside delivery). */
static uint64_t g_can_rx_us;
uint64_t can_bus_current_rx_us(void) { return g_can_rx_us; }

uint32_t can_bus_latest_tx_buffer(void) {
  return g_hfdcan ? HAL_FDCAN_GetLatestTxFifoQRequestBuffer(g_hfdcan) : 0U;
}

static inline void can_bus_notify_rx(const uint8_t *data, size_t len) {
  for (unsigned i = 0; i < CAN_BUS_MAX_SUBSCRIBERS; i++) {
    can_bus_rx_cb_t cb = g_subs[i].cb;
    if (cb)
      cb(data, len, g_subs[i].user);
  }
}

// =========================
// Fragmentation protocol
// =========================
//
// We mark fragment frames by a magic header at the start of payload.
// You can use a dedicated CAN ID range too, but magic is simplest.

#include "can_link.h"
#define CAN_BUS_REASM_TIMEOUT_MS 1000u // tolerate interleaved multi-sender packets

// =========================
// RX ring buffer (ISR -> thread)
// =========================
//
// ISR drains FDCAN FIFO into this ring. Consumer calls can_bus_process_rx()
// from a thread/main loop.

#ifndef CAN_BUS_RX_RING_DEPTH
#define CAN_BUS_RX_RING_DEPTH 64
#endif

typedef struct {
  uint32_t std_id; // 11-bit ID in lower bits (we only handle standard here)
  uint8_t len;     // payload bytes (0..64)
  uint8_t ts[3];   // receive ISR time, low 24 bits of board_now_us() (struct padding)
  uint8_t data[64];
} can_bus_rx_frame_t;

/* Full board_now_us() of a ring frame from its 24-bit stamp (frames are
 * delivered within 16.7 s of receipt). */
static uint64_t can_bus_frame_rx_us(const can_bus_rx_frame_t *f) {
  const uint64_t now = board_now_us();
  const uint32_t ts = (uint32_t)f->ts[0] | ((uint32_t)f->ts[1] << 8) | ((uint32_t)f->ts[2] << 16);
  const uint32_t age = ((uint32_t)now - ts) & 0xFFFFFFU;
  return now - age;
}


static volatile uint16_t g_rx_head = 0;
static volatile uint16_t g_rx_tail = 0;
volatile uint32_t g_fdcan_bus_off_count = 0;
volatile uint32_t g_fdcan_recovery_count = 0;
static volatile uint8_t g_can_recovering;
volatile uint32_t g_fdcan_rx_count = 0;
volatile uint32_t g_fdcan_rx_hw_overflow_count = 0;
volatile uint32_t g_fdcan_rx_ring_drop_count = 0;
volatile uint32_t g_fdcan_tx_ok_count = 0;
volatile uint32_t g_fdcan_tx_fail_count = 0;

/* Recovery runs only in the foreground. A failed HAL Stop can leave ERROR,
 * and PSR.BusOff is not evidence that the HAL/controller is running. */
#define CAN_BUS_RECOVERY_RETRY_MS 100U
static uint8_t g_can_restart_pending;
static uint8_t g_can_recovery_attempted;
static uint32_t g_can_last_recovery_attempt_ms;
volatile uint32_t g_fdcan_recovery_attempt_count;
volatile uint32_t g_fdcan_recovery_fail_count;
static HAL_StatusTypeDef can_bus_configure_filters(FDCAN_HandleTypeDef *hfdcan);

static HAL_StatusTypeDef can_bus_recover_if_bus_off(void) {
  FDCAN_ProtocolStatusTypeDef protocol_status;
  if (g_hfdcan == NULL) return HAL_ERROR;
  if (HAL_FDCAN_GetProtocolStatus(g_hfdcan, &protocol_status) != HAL_OK) {
    g_can_restart_pending = 1U;
    g_can_recovering = 1U;
    return HAL_ERROR;
  }
  if (protocol_status.BusOff == 0U && !g_can_restart_pending &&
      g_hfdcan->State == HAL_FDCAN_STATE_BUSY) return HAL_OK;

  const uint32_t now = HAL_GetTick();
  if (g_can_recovery_attempted &&
      (uint32_t)(now - g_can_last_recovery_attempt_ms) < CAN_BUS_RECOVERY_RETRY_MS)
    return HAL_BUSY;
  g_can_recovery_attempted = 1U;
  g_can_last_recovery_attempt_ms = now;
  g_can_restart_pending = 1U;
  g_can_recovering = 1U;
  ++g_fdcan_recovery_attempt_count;
  if (protocol_status.BusOff != 0U) ++g_fdcan_bus_off_count;

  if (g_hfdcan->State == HAL_FDCAN_STATE_BUSY) {
    (void)HAL_FDCAN_AbortTxRequest(
        g_hfdcan, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);
    if (HAL_FDCAN_Stop(g_hfdcan) != HAL_OK) goto failed;
  }
  /* Stop timeout leaves HAL ERROR; Start accepts READY only. Reinitialize the
   * existing controller configuration before retrying, without resetting SDK. */
  if (g_hfdcan->State != HAL_FDCAN_STATE_READY &&
      HAL_FDCAN_Init(g_hfdcan) != HAL_OK) goto failed;
  if (can_bus_configure_filters(g_hfdcan) != HAL_OK) goto failed;
  can_tx_queue_reset();
  if (HAL_FDCAN_Start(g_hfdcan) != HAL_OK) goto failed;
#if !CAN_BUS_POLLING
  if (HAL_FDCAN_ActivateNotification(
          g_hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
              FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_RX_FIFO1_MESSAGE_LOST |
              FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_TX_COMPLETE,
          FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) != HAL_OK)
    goto failed;
#endif
  g_can_restart_pending = 0U;
  g_can_recovering = 0U;
  ++g_fdcan_recovery_count;
  return HAL_OK;
failed:
  ++g_fdcan_recovery_fail_count;
  /* Keep ISR TX pumping/admission gated, but let the next foreground retry run. */
  return HAL_ERROR;
}

static can_bus_rx_frame_t g_rx_ring[CAN_BUS_RX_RING_DEPTH];

static inline uint16_t rb_next(uint16_t v) {
  v++;
  if (v >= CAN_BUS_RX_RING_DEPTH)
    v = 0;
  return v;
}

static inline int __attribute__((unused)) rb_is_empty(void) {
  return g_rx_head == g_rx_tail;
}

static inline int rb_is_full(void) { return rb_next(g_rx_head) == g_rx_tail; }

// Only the consumer writes tail. Drop-newest on overflow so an interrupt
// cannot overwrite the slot the consumer is copying or race its tail update.
//
// Memory ordering:
//  - We must ensure slot writes are visible before publishing head.
//  - `__DMB()` acts as a release barrier here.
static inline void rb_push_drop_newest(uint32_t std_id, const uint8_t *data,
                                       uint8_t len) {
  if (len > 64)
    len = 64;

  if (rb_is_full()) {
    g_fdcan_rx_ring_drop_count++;
    return;
  }

  uint16_t h = g_rx_head;

  g_rx_ring[h].std_id = std_id;
  g_rx_ring[h].len = len;
  {
    const uint32_t us = (uint32_t)board_now_us();
    g_rx_ring[h].ts[0] = (uint8_t)us; g_rx_ring[h].ts[1] = (uint8_t)(us >> 8);
    g_rx_ring[h].ts[2] = (uint8_t)(us >> 16);
  }
  memcpy(g_rx_ring[h].data, data, len);

  __DMB(); // publish slot before updating head (release)
  g_rx_head = rb_next(h);
}

// Pop frame in thread context
//
// Memory ordering:
//  - After observing head != tail, we must ensure subsequent reads of the slot
//    see the writes that happened-before the producer published head.
//  - `__DMB()` acts as an acquire barrier here.
static inline int rb_pop(can_bus_rx_frame_t *out) {
  uint16_t t = g_rx_tail;
  uint16_t h = g_rx_head;

  if (h == t)
    return 0;

  __DMB(); // ensure slot contents are visible after seeing head advance
           // (acquire)

  *out = g_rx_ring[t];

  __DMB(); // ensure slot read completes before we advance tail (conservative)
  g_rx_tail = rb_next(t);
  return 1;
}

// =========================
// Reassembly state
// =========================

#ifndef CAN_BUS_REASM_SLOTS
#define CAN_BUS_REASM_SLOTS 4
#endif

/* Link packets are at most CAN_LINK_MAX_PACKET bytes (two frames); larger
 * messages travel as SDK side-transport chunks. */
#if defined(CAN_BUS_REASM_MAX_BYTES) && CAN_BUS_REASM_MAX_BYTES < CAN_LINK_MAX_PACKET
#error "CAN reassembly must hold a whole link packet"
#endif

static can_link_slot_t g_reasm[CAN_BUS_REASM_SLOTS];

static void reasm_reset(can_link_slot_t *s)
{
  s->active = 0U;
  s->len = 0U;
}

static void reasm_expire_old(uint32_t now_ms)
{
  const unsigned expired =
      can_link_expire(g_reasm, CAN_BUS_REASM_SLOTS, now_ms, CAN_BUS_REASM_TIMEOUT_MS);
  (void)expired;
}

static HAL_StatusTypeDef can_bus_configure_filters(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan == NULL) {
    return HAL_ERROR;
  }

  return HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                      FDCAN_ACCEPT_IN_RX_FIFO0,
                                      FDCAN_ACCEPT_IN_RX_FIFO0,
                                      FDCAN_REJECT_REMOTE,
                                      FDCAN_REJECT_REMOTE);
}

static void can_bus_drain_rx_fifo(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo) {
  FDCAN_RxHeaderTypeDef hdr;
  uint8_t data[64];

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, rx_fifo) > 0U) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, rx_fifo, &hdr, data) != HAL_OK) {
      break;
    }
    g_fdcan_rx_count++;

    rb_push_drop_newest(hdr.Identifier & 0x7FFu, data,
                        (uint8_t)can_bus_dlc_to_len(hdr.DataLength));
  }
}

// Handle one RX frame (thread context)
static void handle_rx_frame(const can_bus_rx_frame_t *f, uint32_t now_ms)
{
  /* Link format v2: the identifier carries the frame kind; see can_link.h. */
  const uint8_t *packet = NULL;
  size_t len = 0U;
  const int result = can_link_rx(g_reasm, CAN_BUS_REASM_SLOTS, f->std_id, f->data,
                                 f->len, now_ms, &packet, &len);
  if (result != CAN_LINK_RX_COMPLETE) return;
  g_can_rx_us = can_bus_frame_rx_us(f);
  can_bus_notify_rx(packet, len);
  g_can_rx_us = 0U;
}

// =========================
// Public API
// =========================

void can_bus_init(FDCAN_HandleTypeDef *hfdcan) {
  g_can_recovering = 1U;
  g_can_restart_pending = 1U;
  g_can_recovery_attempted = 0U;
  can_tx_queue_reset();
  g_hfdcan = hfdcan;
  g_rx_head = 0;
  g_rx_tail = 0;
  for (unsigned i = 0; i < CAN_BUS_REASM_SLOTS; i++) {
    reasm_reset(&g_reasm[i]);
  }
  if (hfdcan != NULL) {
    if (hfdcan->State == HAL_FDCAN_STATE_BUSY &&
        HAL_FDCAN_Stop(hfdcan) != HAL_OK) return;
    if (hfdcan->State != HAL_FDCAN_STATE_READY &&
        HAL_FDCAN_Init(hfdcan) != HAL_OK) return;
    if (can_bus_configure_filters(hfdcan) != HAL_OK) return;
    /* Retained RX interrupts must not run before HAL enters BUSY state. */
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
      return;
    }
#if !CAN_BUS_POLLING
    if (HAL_FDCAN_ActivateNotification(
        hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
                FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_RX_FIFO1_MESSAGE_LOST |
                FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_TX_COMPLETE,
        FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) != HAL_OK) {
      g_can_recovering = 1U;
      return;
    }
#endif
  }
  if (hfdcan != NULL) {
    g_can_restart_pending = 0U;
    g_can_recovering = 0U;
  }
}

HAL_StatusTypeDef can_bus_subscribe_rx(can_bus_rx_cb_t cb, void *user) {
  if (!cb)
    return HAL_ERROR;

  for (unsigned i = 0; i < CAN_BUS_MAX_SUBSCRIBERS; i++) {
    if (g_subs[i].cb == cb && g_subs[i].user == user)
      return HAL_ERROR;
  }
  for (unsigned i = 0; i < CAN_BUS_MAX_SUBSCRIBERS; i++) {
    if (g_subs[i].cb == NULL) {
      g_subs[i].cb = cb;
      g_subs[i].user = user;
      return HAL_OK;
    }
  }
  return HAL_ERROR;
}

HAL_StatusTypeDef can_bus_unsubscribe_rx(can_bus_rx_cb_t cb, void *user) {
  if (!cb)
    return HAL_ERROR;

  for (unsigned i = 0; i < CAN_BUS_MAX_SUBSCRIBERS; i++) {
    if (g_subs[i].cb == cb && g_subs[i].user == user) {
      g_subs[i].cb = NULL;
      g_subs[i].user = NULL;
      return HAL_OK;
    }
  }
  return HAL_ERROR;
}

// Send a single CAN/CAN-FD payload up to 64 bytes.
// If len is not an exact FD size, it rounds up and zero-pads.
HAL_StatusTypeDef can_bus_send_bytes(const uint8_t *bytes, size_t len,
                                     uint32_t std_id) {
  if (!g_hfdcan)
    return HAL_ERROR;
  if (!bytes || len == 0)
    return HAL_ERROR;

  if (g_can_recovering) return HAL_ERROR;
  if (HAL_FDCAN_GetTxFifoFreeLevel(g_hfdcan) == 0U) return HAL_BUSY;

  if (len > 64)
    len = 64;

  size_t wire_len = can_bus_round_up_fd_len(len);
  uint32_t dlc = can_bus_len_to_dlc(wire_len);
  if (dlc == 0xFFFFFFFFu)
    return HAL_ERROR;

  FDCAN_TxHeaderTypeDef txHeader;
  memset(&txHeader, 0, sizeof(txHeader));

  txHeader.Identifier = std_id & 0x7FFu;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;

  txHeader.DataLength = dlc; // DLC code (HAL expects this)
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = (wire_len <= 8U) ? FDCAN_CLASSIC_CAN : FDCAN_FD_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  uint8_t txData[64] = {0};
  memcpy(txData, bytes, len);

  const HAL_StatusTypeDef status =
      HAL_FDCAN_AddMessageToTxFifoQ(g_hfdcan, &txHeader, txData);
  if (status == HAL_OK) {
    g_fdcan_tx_ok_count++;
  } else {
    g_fdcan_tx_fail_count++;
  }
  return status;
}

// Retain a complete 1..128-byte side frame before asynchronous fragmentation.
// Multi-fragment packets use 64-byte frames and a small header in each frame.
HAL_StatusTypeDef can_bus_send_large(const uint8_t *bytes, size_t len,
                                     uint32_t std_id) {
  if (!g_hfdcan || g_can_recovering) return HAL_ERROR;
  return can_tx_queue_submit(bytes, len, std_id);
}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan == g_hfdcan && !g_can_recovering) can_tx_queue_pump();
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan,
                                      uint32_t buffers) {
  if (hfdcan == g_hfdcan) can_tx_queue_note_complete(buffers);
  if (buffers != 0U && hfdcan == g_hfdcan && !g_can_recovering)
    can_tx_queue_pump();
}

// Call this periodically from thread/main-loop context.
// It drains the ISR ring buffer, expires old partial reassembly slots,
// reassembles fragmented messages, and notifies subscribers.
void can_bus_process_rx(void) {
  uint32_t now = HAL_GetTick();
  reasm_expire_old(now);
  if (g_hfdcan != NULL && can_bus_recover_if_bus_off() == HAL_OK && !g_can_recovering)
    can_tx_queue_service();

#if CAN_BUS_POLLING
  if (g_hfdcan != NULL) {
    can_bus_drain_rx_fifo(g_hfdcan, FDCAN_RX_FIFO0);
    can_bus_drain_rx_fifo(g_hfdcan, FDCAN_RX_FIFO1);
  }
#endif

  can_bus_rx_frame_t f;
  while (rb_pop(&f)) {
    handle_rx_frame(&f, now);
  }
}

// =========================
// HAL ISR callback
// =========================
//
// IMPORTANT: ensure only one definition exists in the entire link.
//
// This ISR does minimal work: drains RX FIFOs into our ring buffer.
// Reassembly and subscriber callbacks happen in can_bus_process_rx().

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  if (hfdcan != g_hfdcan) {
    return;
  }

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
    g_fdcan_rx_hw_overflow_count++;

  /* Overflow may be the only remaining event after NEW_MESSAGE was cleared.
   * Drain retained frames then too, or a full FIFO can stay wedged forever. */
  if ((RxFifo0ITs & (FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                    FDCAN_IT_RX_FIFO0_MESSAGE_LOST)) == 0U) {
    return;
  }

  can_bus_drain_rx_fifo(hfdcan, FDCAN_RX_FIFO0);
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo1ITs) {
  if (hfdcan != g_hfdcan) {
    return;
  }

  if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) != 0U)
    g_fdcan_rx_hw_overflow_count++;

  if ((RxFifo1ITs & (FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
                    FDCAN_IT_RX_FIFO1_MESSAGE_LOST)) == 0U) {
    return;
  }

  can_bus_drain_rx_fifo(hfdcan, FDCAN_RX_FIFO1);
}

uint32_t board_health_can_rx_drops(void) { return g_fdcan_rx_ring_drop_count; }
