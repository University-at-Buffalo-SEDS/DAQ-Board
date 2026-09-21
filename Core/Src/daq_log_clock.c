#include "daq_log_clock.h"
#include "sd_card.h"
#include "main.h"
#include "sd_calendar.h"
#include <stdbool.h>
static bool clock_seen;
static uint32_t last_request;
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
  sd_card_set_launch_clock(session, deadline);
  clock_seen = true;
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
