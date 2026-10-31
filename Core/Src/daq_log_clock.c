#include "daq_log_clock.h"
#include "sd_card.h"
#include "main.h"
#include "sd_calendar.h"
#include <stdbool.h>
static bool clock_seen;
static uint32_t last_request;
/* Survive router reinitialization within this boot. Updated GS persists a
 * strictly increasing value revision in Packet.timestamp; UTC is in payload. */
static uint64_t clock_session, clock_revision, clock_high_water, clock_deadline, clock_value_session;
static bool revision_seen, reset_ambiguous;
volatile uint32_t g_daq_clock_stale_updates, g_daq_clock_ambiguous_resets;
extern volatile uint32_t g_telemetry_discovery_seen;
static uint64_t read_u64(const uint8_t *p)
{
  uint64_t value = 0;
  for (unsigned i=0; i<8; ++i) value |= (uint64_t)p[i] << (8U*i);
  return value;
}
static SedsResult clock_update(const SedsPacketView *packet, void *context)
{
  (void)context;
  if (!packet || packet->ty != SEDS_DT_DAQ_LOG_CLOCK || !packet->payload || packet->payload_len != 16U)
    return SEDS_HANDLER_ERROR;
  const uint64_t session = read_u64(packet->payload), deadline = read_u64(packet->payload+8);
  sd_calendar_t date;
  if ((session == 0U) != (deadline == 0U) ||
      (deadline != 0U && !sd_calendar_from_unix_ms(deadline, &date))) return SEDS_HANDLER_ERROR;
  clock_seen = true;
  const uint64_t stamp = packet->timestamp;
  const uint64_t high_before = clock_high_water;
  if (stamp > clock_high_water) clock_high_water = stamp;
  if (session != 0U && session < clock_session) {
    if (stamp > clock_revision) reset_ambiguous = true;
    ++g_daq_clock_stale_updates; return SEDS_OK;
  }
  if (revision_seen && session <= clock_session) {
    if (session == 0U && clock_session != 0U && reset_ambiguous) {
      /* A legacy boot-clock regression makes reset's session unknowable:
       * never guess that an unscoped tombstone belongs to this newer run. */
      ++g_daq_clock_ambiguous_resets; return SEDS_OK;
    }
    if (stamp <= clock_revision) {
      /* Equal identical values are harmless retries; equal conflicting ones
       * cannot prove a T0 correction or reset. Do not create a retry storm. */
      if (session != clock_value_session || deadline != clock_deadline) ++g_daq_clock_stale_updates;
      return SEDS_OK;
    }
  }
  if (session > clock_session) {
    clock_session = session;
    reset_ambiguous = stamp < high_before || stamp < UINT64_C(315532800000);
  }
  clock_deadline = deadline; clock_value_session = session; revision_seen = true;
  sd_card_set_launch_clock(session, deadline);
  /* Publish receipt only after storage has considered this revision. A late
   * correction may be accepted by the receiver but rejected by the closed
   * storage session; the status reporter therefore also sends its actual pair. */
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  clock_revision = stamp;
  __set_PRIMASK(saved);
  return SEDS_OK;
}
SedsResult daq_log_clock_init(SedsRouter *router)
{
  clock_seen = false;
  last_request = HAL_GetTick();
  SedsResult result = seds_router_enable_network_variable(router, SEDS_DT_DAQ_LOG_CLOCK, true, false);
  if (result == SEDS_OK) result = seds_router_on_network_variable_update(router, SEDS_DT_DAQ_LOG_CLOCK, clock_update, NULL);
  return result;
}
SedsResult daq_log_clock_poll(SedsRouter *router)
{
  const uint32_t now = HAL_GetTick();
  if (clock_seen || !g_telemetry_discovery_seen || (uint32_t)(now-last_request) < 500U) return SEDS_OK;
  last_request = now;
  return seds_router_request_managed_variable(router, SEDS_DT_DAQ_LOG_CLOCK);
}

uint64_t daq_log_clock_revision(void)
{
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  const uint64_t revision = clock_revision;
  __set_PRIMASK(saved);
  return revision;
}
