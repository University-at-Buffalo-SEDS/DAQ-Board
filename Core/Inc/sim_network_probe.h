#ifndef SIM_NETWORK_PROBE_H
#define SIM_NETWORK_PROBE_H

#include <stddef.h>
#include <stdint.h>

extern volatile uint32_t g_telemetry_peer_mask;
extern volatile uint32_t g_sim_heartbeat_attempts;
extern volatile uint32_t g_sim_heartbeat_ok;
extern volatile uint32_t g_sim_heartbeat_fail;
extern volatile uint32_t g_sim_heartbeat_wire_tx;

static inline uint32_t sim_probe_peer_bit_sender(const char *sender, size_t len) {
#ifdef SEDS_FIRMWARE_SIM_TEST
  static const struct { const char *name; size_t len; uint32_t bit; } peers[] = {
      {"RF",2U,1U},{"PB",2U,2U},{"FC",2U,4U},{"GB",2U,8U},
      {"AB",2U,16U},{"VB",2U,32U},{"DAQ",3U,64U}};
  for (size_t i=0U; sender != NULL && i<sizeof(peers)/sizeof(peers[0]); ++i) {
    size_t j=0U; if (len != peers[i].len) continue;
    while (j<len && sender[j]==peers[i].name[j]) j++;
    if (j==len) return peers[i].bit;
  }
#else
  (void)sender; (void)len;
#endif
  return 0U;
}
static inline SedsResult sim_probe_heartbeat_handler(const SedsPacketView *pkt, void *user) {
  (void)user;
#ifdef SEDS_FIRMWARE_SIM_TEST
  if (pkt != NULL && pkt->ty == (uint32_t)SEDS_DT_HEARTBEAT)
    g_telemetry_peer_mask |= sim_probe_peer_bit_sender(pkt->sender, pkt->sender_len);
#else
  (void)pkt;
#endif
  return SEDS_OK;
}

/* Decode only 32-bit wire IDs; skipped timestamps may still use 64 bits. */
static inline uint8_t sim_probe_read_id(const uint8_t *data, size_t len,
                                      size_t *offset, uint32_t *value) {
  uint32_t out = 0U;
  for (uint32_t shift = 0U; shift < 35U && *offset < len; shift += 7U) {
    const uint8_t byte = data[(*offset)++];
    if (shift == 28U && byte > 0x0FU) return 0U;
    out |= ((uint32_t)(byte & 0x7FU)) << shift;
    if ((byte & 0x80U) == 0U) { *value = out; return 1U; }
  }
  return 0U;
}
static inline uint8_t sim_probe_skip_uleb(const uint8_t *data, size_t len,
                                        size_t *offset) {
  for (uint32_t i = 0U; i < 10U && *offset < len; ++i) {
    const uint8_t byte = data[(*offset)++];
    if (i == 9U && byte > 1U) return 0U;
    if ((byte & 0x80U) == 0U) return 1U;
  }
  return 0U;
}

static inline uint32_t sim_probe_packed_data_type(const uint8_t *data,
                                                  size_t len) {
  size_t offset = 2U;
  uint32_t data_type = 0U;
  if (data != NULL && len >= 3U &&
      sim_probe_read_id(data, len, &offset, &data_type)) {
    return data_type;
  }
  return UINT32_MAX;
}

static inline void sim_probe_observe_can_tx(const uint8_t *data, size_t len) {
#ifdef SEDS_FIRMWARE_SIM_TEST
  if (sim_probe_packed_data_type(data, len) == (uint32_t)SEDS_DT_HEARTBEAT) {
    g_sim_heartbeat_wire_tx++;
  }
#else
  (void)data;
  (void)len;
#endif
}

/* Allocation-free observation of SEDSNet v4's packed source-address field. */
static inline void sim_probe_observe_packed(const uint8_t *data, size_t len) {
#ifdef SEDS_FIRMWARE_SIM_TEST
  /* Stable SEDSNet source addresses for RF, PB, FC, GB, AB, VB, DAQ.
   * Tests compare these with the wire encoder; never recompute seven
   * 64-bit sender hashes for each observed packet. */
  static const uint32_t peer_addresses[] = {
      UINT32_C(0xEE4B3E5B), UINT32_C(0x0CC12D6E), UINT32_C(0x3C3A3E96), UINT32_C(0x1283C6CE), UINT32_C(0x1CD0258F), UINT32_C(0xE59A119E), UINT32_C(0x00D8168E)};
  size_t offset = 0U;
  uint32_t source_address = 0U;
  if (data == NULL || len < 3U) return;
  const uint8_t flags = data[offset++];
  offset++; /* selected endpoint count */
  if (!sim_probe_skip_uleb(data, len, &offset) ||
      !sim_probe_skip_uleb(data, len, &offset) ||
      !sim_probe_skip_uleb(data, len, &offset)) return;
  if ((flags & 0x08U) != 0U &&
      !sim_probe_skip_uleb(data, len, &offset)) return;
  if (!sim_probe_read_id(data, len, &offset, &source_address)) return;
  for (size_t i = 0U; i < sizeof(peer_addresses) / sizeof(peer_addresses[0]); ++i) {
    if (source_address == peer_addresses[i]) {
      g_telemetry_peer_mask |= UINT32_C(1) << i;
      return;
    }
  }
#else
  (void)data;
  (void)len;
#endif
}

/* Emit real end-to-end traffic for linked-bay validation. Unlike aggregated
 * topology advertisements, this packet retains the originating board ID
 * through every relay, so receivers can certify multi-hop delivery. */
static inline void sim_probe_emit_heartbeat(SedsRouter *router,
                                            uint64_t now_ms) {
#ifdef SEDS_FIRMWARE_SIM_TEST
  static uint64_t next_emit_ms = 2300U;
  static uint8_t emit_count = 0U;
  static uint32_t service_cycles = 0U;
  static uint64_t last_observed_ms = 0U;
  static const uint8_t empty_payload = 0U;
  service_cycles++;
  /* Keep test-origin traffic alive across host restarts; this is not sensor
   * telemetry and is compiled out of ordinary firmware. */
  if (now_ms < last_observed_ms) next_emit_ms = now_ms;
  last_observed_ms = now_ms;
  if (router != NULL &&
      (now_ms >= next_emit_ms ||
       (emit_count < 4U && service_cycles >= (60U + ((uint32_t)emit_count * 75U))))) {
    g_sim_heartbeat_attempts++;
    if (seds_router_log_bytes_ex(router, SEDS_DT_HEARTBEAT, &empty_payload,
                                 0U, NULL, 1) == SEDS_OK) {
      if (emit_count < 4U) emit_count++;
      next_emit_ms = now_ms + (emit_count < 4U ? 100U : 1000U);
      g_sim_heartbeat_ok++;
    } else {
      next_emit_ms = now_ms + 50U;
      g_sim_heartbeat_fail++;
    }
  }
#else
  (void)router;
  (void)now_ms;
#endif
}
#endif
