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

/* ---------------- Time helpers: 32->64 extender ---------------- */
static uint64_t stm_now_ms(void *user) {
  (void)user;
  static uint32_t last32 = 0;
  static uint64_t high = 0;

  uint32_t cur32 = HAL_GetTick();
  if (cur32 < last32) {
    high += (1ULL << 32); /* 32-bit wrap (~49.7 days) */
  }
  last32 = cur32;
  return high | (uint64_t)cur32;
}

uint64_t node_now_since_ms(void *user) {
  (void)user;
  const uint64_t now = stm_now_ms(NULL);
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

  // Local endpoint handlers (SD terminates here)
  const SedsLocalEndpointDesc locals[] = {
      {
          .endpoint = (uint32_t)SEDS_EP_SD_CARD,
          .packet_handler = on_sd_packet,
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
  g_router.start_time = stm_now_ms(NULL);

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
