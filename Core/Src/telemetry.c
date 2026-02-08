// telemetry.c
#include "telemetry.h"

#include "app_threadx.h" // should bring in tx_api.h; if not, include tx_api.h directly
#include "can_bus.h"
#include "sedsprintf.h"
#include "stm32u5xx_hal.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifndef TELEMETRY_ENABLED
static void print_data_no_telem(void *data, size_t len) {
  (void)data;
  (void)len;
  // Optional: dump bytes for debug
}
#endif

#if defined(__GNUC__) || defined(__clang__)
#define UNUSED_FUNCTION __attribute__((unused))
#else
#define UNUSED_FUNCTION
#endif

static uint8_t g_can_rx_subscribed = 0;
static int32_t g_can_side_id = -1; // side ID returned by seds_router_add_side_serialized

/* ---------------- ThreadX clock helpers (32->64 extender) ----------------
 *
 * ThreadX "clock" is tx_time_get()/tx_time_set() (ULONG ticks).
 * We extend 32-bit wrap and expose ms.
 *
 * This file previously used HAL_GetTick(); updated to use ThreadX time so
 * time sync can adjust the ThreadX clock and your telemetry timestamps follow.
 */
#ifndef TX_TIMER_TICKS_PER_SECOND
#error "TX_TIMER_TICKS_PER_SECOND must be defined by ThreadX."
#endif

static uint64_t tx_raw_now_ms(void *user) {
  (void)user;

  // Extend 32-bit ULONG tick wrap
  static uint32_t last_ticks32 = 0;
  static uint64_t high = 0;

  uint32_t cur32 = (uint32_t)tx_time_get();
  if (cur32 < last_ticks32) {
    high += (1ULL << 32);
  }
  last_ticks32 = cur32;

  uint64_t ticks64 = high | (uint64_t)cur32;

  // Convert ticks -> ms (works even when not divisible)
  return (ticks64 * 1000ULL) / (uint64_t)TX_TIMER_TICKS_PER_SECOND;
}

static uint64_t node_now_since_ms(void *user);

/* ---------------- Time sync (NTP-style offset/delay) ---------------- */
static void compute_offset_delay(uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4,
                                 int64_t *offset_ms, uint64_t *delay_ms) {
  const int64_t o = ((int64_t)(t2 - t1) + (int64_t)(t3 - t4)) / 2;
  const int64_t d = (int64_t)(t4 - t1) - (int64_t)(t3 - t2);
  *offset_ms = o;
  *delay_ms = (d < 0) ? 0 : (uint64_t)d;
}

static void threadx_apply_offset_ms(int64_t offset_ms) {
  // Optional sanity clamp: ignore insane corrections
  if (offset_ms > 30 * 1000 || offset_ms < -(30 * 1000)) {
    return;
  }

  // Convert ms -> ticks
  const int64_t tps = (int64_t)TX_TIMER_TICKS_PER_SECOND;
  int64_t delta_ticks = (offset_ms * tps) / 1000;

  ULONG cur = tx_time_get();
  int64_t new_ticks = (int64_t)(uint32_t)cur + delta_ticks;
  if (new_ticks < 0) new_ticks = 0;

  tx_time_set((ULONG)(uint32_t)new_ticks);
}

/* Local endpoint handler: consume TIME_SYNC_RESPONSE and adjust ThreadX clock.
 *
 * Payload (u64, little-endian on STM32):
 *   resp[0]=seq, resp[1]=t1, resp[2]=t2, resp[3]=t3
 * and t4 is captured locally on receipt.
 */
static SedsResult on_timesync(const SedsPacketView *pkt, void *user) {
  (void)user;
  if (!pkt || !pkt->payload) return SEDS_ERR;

  if (pkt->ty == SEDS_DT_TIME_SYNC_RESPONSE && pkt->payload_len >= 32) {
    uint64_t seq = 0, t1 = 0, t2 = 0, t3 = 0;
    memcpy(&seq, pkt->payload + 0, 8);
    memcpy(&t1,  pkt->payload + 8, 8);
    memcpy(&t2,  pkt->payload + 16, 8);
    memcpy(&t3,  pkt->payload + 24, 8);

    const uint64_t t4 = tx_raw_now_ms(NULL);

    int64_t offset_ms = 0;
    uint64_t delay_ms = 0;
    compute_offset_delay(t1, t2, t3, t4, &offset_ms, &delay_ms);

    threadx_apply_offset_ms(offset_ms);

    // Optional debug:
    // printf("timesync seq=%llu offset_ms=%lld delay_ms=%llu\r\n",
    //        (unsigned long long)seq, (long long)offset_ms, (unsigned long long)delay_ms);
  }

  return SEDS_OK;
}

/* ---------------- Router timebase ---------------- */
static uint64_t node_now_since_ms(void *user) {
  (void)user;
  const uint64_t now = tx_raw_now_ms(NULL);
  const RouterState s = g_router; /* snapshot */
  return s.r ? (now - s.start_time) : 0;
}

/* ---------------- Global router state ---------------- */
RouterState g_router = {.r = NULL, .created = 0, .start_time = 0};

/* ---------------- TX helpers ---------------- */
SedsResult tx_send(const uint8_t *bytes, size_t len, void *user) {
  (void)user;

  if (!bytes || len == 0) {
    return SEDS_BAD_ARG;
  }

  // Only CAN TX in this build
  return (can_bus_send_large(bytes, len, 0x03) == HAL_OK) ? SEDS_OK : SEDS_IO;
}

/* ---------------- Local endpoint handler(s) ----------------
 * SD endpoint packets terminate here (Sink mode).
 * If you don't have SD logging wired yet, just accept.
 */
SedsResult on_sd_packet(const SedsPacketView *pkt, void *user) {
  (void)user;
  (void)pkt;

  // TODO: write serialized form or payload to SD card.
  return SEDS_OK;
}

/* ---------------- RX helpers ---------------- */
static void telemetry_can_rx(const uint8_t *data, size_t len, void *user) {
  (void)user;
  rx_asynchronous(data, len);
}

void rx_asynchronous(const uint8_t *bytes, size_t len) {
#ifndef TELEMETRY_ENABLED
  (void)bytes;
  (void)len;
  return;
#else
  if (!bytes || len == 0) {
    return;
  }

  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) {
      return;
    }
  }

  // If we have a registered CAN side, tag RX as "from that side" so the router
  // can do side-aware behavior (e.g., avoid reflexively re-sending to origin).
  if (g_can_side_id >= 0) {
    (void)seds_router_rx_serialized_packet_to_queue_from_side(
        g_router.r, (uint32_t)g_can_side_id, bytes, len);
  } else {
    (void)seds_router_rx_serialized_packet_to_queue(g_router.r, bytes, len);
  }
#endif
}

/* Optional synchronous RX (not in telemetry.h, but sometimes handy internally) */
static UNUSED_FUNCTION void rx_synchronous(const uint8_t *bytes, size_t len) {
#ifndef TELEMETRY_ENABLED
  (void)bytes;
  (void)len;
  return;
#else
  if (!bytes || len == 0) {
    return;
  }

  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) {
      return;
    }
  }

  if (g_can_side_id >= 0) {
    (void)seds_router_receive_serialized_from_side(
        g_router.r, (uint32_t)g_can_side_id, bytes, len);
  } else {
    (void)seds_router_receive_serialized(g_router.r, bytes, len);
  }
#endif
}


// --- Time sync request (client-side) ---
// Called by telemetry thread periodically to request a resync with the master.
//
// Packet format matches your earlier example:
//   req[0]=seq, req[1]=t1
// where t1 is local send timestamp in ms (same timebase used for NTP math).
//
// The master should reply with TIME_SYNC_RESPONSE containing:
//   [seq, t1, t2, t3]
//
// This node's on_timesync() will capture t4 and compute/apply offset.
static uint64_t g_timesync_seq = 1;

SedsResult telemetry_timesync_request(void)
{
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }

  const uint64_t t1 = tx_raw_now_ms(NULL);   // use your local base time
  const uint64_t req[2] = { g_timesync_seq++, t1 };

  // IMPORTANT:
  // - This logs a TIME_SYNC_REQUEST packet to the router so it can be routed to the master.
  // - timestamp is explicitly t1 so the receiver can use it directly.
  return seds_router_log_ts(g_router.r, SEDS_DT_TIME_SYNC_REQUEST, t1, req, 2);
#endif
}


/* ---------------- Router init (idempotent) ---------------- */
SedsResult init_telemetry_router(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (g_router.created && g_router.r) {
    return SEDS_OK;
  }

  /* Subscribe exactly once */
  if (!g_can_rx_subscribed) {
    if (can_bus_subscribe_rx(telemetry_can_rx, NULL) == HAL_OK) {
      g_can_rx_subscribed = 1;
    } else {
      printf("Error: can_bus_subscribe_rx failed\r\n");
      // Not fatal: you can still TX/log, but you'll miss CAN RX
    }
  }

  // Local endpoint handlers:
  // - SD terminates here
  // - TIME_SYNC adjusts ThreadX clock
  const SedsLocalEndpointDesc locals[] = {
      {
          .endpoint = (uint32_t)SEDS_EP_SD_CARD,
          .packet_handler = on_sd_packet,
          .serialized_handler = NULL,
          .user = NULL,
      },
      {
          .endpoint = (uint32_t)SEDS_EP_TIME_SYNC,
          .packet_handler = on_timesync,
          .serialized_handler = NULL,
          .user = NULL,
      },
  };

  // Correct API: seds_router_new(mode, now_ms_cb, user, handlers, n_handlers)
  SedsRouter *r = seds_router_new(Seds_RM_Sink, node_now_since_ms, NULL, locals,
                                  (size_t)(sizeof(locals) / sizeof(locals[0])));

  if (!r) {
    printf("Error: failed to create router\r\n");
    g_router.r = NULL;
    g_router.created = 0;
    g_can_side_id = -1;
    return SEDS_ERR;
  }

  // Add a CAN "side" so the router can be side-aware for RX/TX.
  // TX callback signature: SedsTransmitFn(const uint8_t*, size_t, void*)
  g_can_side_id = seds_router_add_side_serialized(
      r, "can", 3, tx_send, NULL,
      /*reliable_enabled=*/false);

  if (g_can_side_id < 0) {
    // Side registration failure is meaningful; router still exists though.
    printf("Error: failed to add CAN side: %ld\r\n", (long)g_can_side_id);
    // Keep router alive but clear side id so RX falls back to non-side calls.
    g_can_side_id = -1;
  }

  g_router.r = r;
  g_router.created = 1;
  g_router.start_time = tx_raw_now_ms(NULL);

  return SEDS_OK;
#endif
}

/* ---------------- Logging APIs ---------------- */
static inline SedsElemKind guess_kind_from_elem_size(size_t elem_size) {
  // Heuristic: most of your schema looks float32-heavy; treat 4/8 as float.
  // If you need exact kinds per datatype, we can add a switch(data_type).
  if (elem_size == 4 || elem_size == 8) {
    return SEDS_EK_FLOAT;
  }
  return SEDS_EK_UNSIGNED;
}

SedsResult log_telemetry_synchronous(SedsDataType data_type, const void *data,
                                     size_t element_count,
                                     size_t element_size) {
#ifdef TELEMETRY_ENABLED
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) {
      return SEDS_ERR;
    }
  }
  if (!data || element_count == 0 || element_size == 0) {
    return SEDS_BAD_ARG;
  }

  const SedsElemKind kind = guess_kind_from_elem_size(element_size);

  // Correct API: seds_router_log_typed_ex expects (count, elem_size) not total bytes.
  return seds_router_log_typed_ex(g_router.r, data_type, data, element_count,
                                  element_size, kind,
                                  /*timestamp*/ NULL,
                                  /*queue*/ 0);
#else
  (void)data_type;
  print_data_no_telem((void *)data, element_count * element_size);
  return SEDS_OK;
#endif
}

SedsResult log_telemetry_asynchronous(SedsDataType data_type, const void *data,
                                      size_t element_count,
                                      size_t element_size) {
#ifdef TELEMETRY_ENABLED
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) {
      return SEDS_ERR;
    }
  }
  if (!data || element_count == 0 || element_size == 0) {
    return SEDS_BAD_ARG;
  }

  const SedsElemKind kind = guess_kind_from_elem_size(element_size);

  return seds_router_log_typed_ex(g_router.r, data_type, data, element_count,
                                  element_size, kind,
                                  /*timestamp*/ NULL,
                                  /*queue*/ 1);
#else
  (void)data_type;
  print_data_no_telem((void *)data, element_count * element_size);
  return SEDS_OK;
#endif
}

/* ---------------- Queue processing ---------------- */
SedsResult dispatch_tx_queue(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) {
      return SEDS_ERR;
    }
  }
  return seds_router_process_tx_queue(g_router.r);
#endif
}

SedsResult process_rx_queue(void) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) {
      return SEDS_ERR;
    }
  }
  return seds_router_process_rx_queue(g_router.r);
#endif
}

SedsResult dispatch_tx_queue_timeout(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) {
      return SEDS_ERR;
    }
  }
  return seds_router_process_tx_queue_with_timeout(g_router.r, timeout_ms);
#endif
}

SedsResult process_rx_queue_timeout(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) {
      return SEDS_ERR;
    }
  }
  return seds_router_process_rx_queue_with_timeout(g_router.r, timeout_ms);
#endif
}

SedsResult process_all_queues_timeout(uint32_t timeout_ms) {
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) {
      return SEDS_ERR;
    }
  }
  return seds_router_process_all_queues_with_timeout(g_router.r, timeout_ms);
#endif
}

/* ---------------- Error logging ----------------
 * Use the string-aware API so fixed-size schema string types don't explode with
 * SEDS_SIZE_MISMATCH and the router can pad/truncate.
 */
SedsResult log_error_asyncronous(const char *fmt, ...) {
#ifndef TELEMETRY_ENABLED
  (void)fmt;
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) {
      return SEDS_ERR;
    }
  }

  va_list args;
  va_start(args, fmt);

  va_list args_copy;
  va_copy(args_copy, args);
  int len = vsnprintf(NULL, 0, fmt, args_copy);
  va_end(args_copy);

  if (len < 0) {
    va_end(args);
    const char *empty = "";
    return seds_router_log_string_ex(g_router.r, SEDS_DT_GENERIC_ERROR, empty, 0,
                                    NULL, 1);
  }

  if (len > 512) {
    len = 512;
  }

  char buf[(size_t)len + 1];
  int written = vsnprintf(buf, (size_t)len + 1, fmt, args);
  va_end(args);

  if (written < 0) {
    const char *empty = "";
    return seds_router_log_string_ex(g_router.r, SEDS_DT_GENERIC_ERROR, empty, 0,
                                    NULL, 1);
  }

  return seds_router_log_string_ex(g_router.r, SEDS_DT_GENERIC_ERROR, buf,
                                   (size_t)written, NULL, 1);
#endif
}

SedsResult log_error_syncronous(const char *fmt, ...) {
#ifndef TELEMETRY_ENABLED
  (void)fmt;
  return SEDS_OK;
#else
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) {
      return SEDS_ERR;
    }
  }

  va_list args;
  va_start(args, fmt);

  va_list args_copy;
  va_copy(args_copy, args);
  int len = vsnprintf(NULL, 0, fmt, args_copy);
  va_end(args_copy);

  if (len < 0) {
    va_end(args);
    const char *empty = "";
    return seds_router_log_string_ex(g_router.r, SEDS_DT_GENERIC_ERROR, empty, 0,
                                    NULL, 0);
  }

  if (len > 512) {
    len = 512;
  }

  char buf[(size_t)len + 1];
  int written = vsnprintf(buf, (size_t)len + 1, fmt, args);
  va_end(args);

  if (written < 0) {
    const char *empty = "";
    return seds_router_log_string_ex(g_router.r, SEDS_DT_GENERIC_ERROR, empty, 0,
                                    NULL, 0);
  }

  return seds_router_log_string_ex(g_router.r, SEDS_DT_GENERIC_ERROR, buf,
                                   (size_t)written, NULL, 0);
#endif
}

/* ---------------- Error printing ---------------- */
SedsResult print_telemetry_error(const int32_t error_code) {
#ifndef TELEMETRY_ENABLED
  (void)error_code;
  return SEDS_OK;
#else
  const int32_t need = seds_error_to_string_len(error_code);
  if (need <= 0) {
    return (SedsResult)need;
  }

  // VLA is OK here; need is typically small.
  char buf[(size_t)need];
  SedsResult res = seds_error_to_string(error_code, buf, sizeof(buf));
  if (res == SEDS_OK) {
    printf("Error: %s\r\n", buf);
  } else {
    (void)log_error_asyncronous("Error: seds_error_to_string failed: %d\r\n",
                                (int)res);
  }
  return res;
#endif
}

/* ---------------- Fatal helper ---------------- */
void die(const char *fmt, ...) {
  char buf[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  while (1) {
    printf("FATAL: %s\r\n", buf);
    HAL_Delay(1000);
  }
}
