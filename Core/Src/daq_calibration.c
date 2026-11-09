#include "daq_calibration.h"

#include "main.h"
#include "mcp3564r.h"
#include "persistent_store.h"
#include "sd_card.h"
#include "sedsnet_config.h"
#ifdef TELEMETRY_ENABLED
#include "board_recovery_transport.h"
#endif

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

extern volatile uint32_t g_telemetry_discovery_seen;

#define DAQ_CALIBRATION_PERSIST_KEY 0x44414341u
#define DAQ_KG50_INPUT_PERSIST_KEY 0x4441494Eu
#define DAQ_CALIBRATION_RETRY_MS 500U

static daq_calibration_t g_calibration = {1.0f, 0.0f, 1.0f, 0.0f, {0.0f, 1.0f}, {0}, {0}};
static bool g_restore_attempted;
static bool g_persist_ready;
#ifdef TELEMETRY_ENABLED
static uint8_t g_kg50_input_stored;
static bool g_network_value_seen;
static bool g_kg50_network_value_seen;
static bool g_thermal_network_value_seen;
static bool g_filter_network_value_seen;
static bool g_input_network_value_seen;
static uint32_t g_last_refresh_ms;
#endif

volatile uint32_t g_daq_calibration_updates __attribute__((used, externally_visible));
volatile uint32_t g_daq_calibration_restores __attribute__((used, externally_visible));
volatile uint32_t g_daq_calibration_persist_writes __attribute__((used, externally_visible));
volatile uint32_t g_daq_calibration_errors __attribute__((used, externally_visible));
volatile uint32_t g_daq_kg50_input_restores __attribute__((used, externally_visible));
volatile uint32_t g_daq_kg50_input_persist_writes __attribute__((used, externally_visible));

static bool calibration_valid(const daq_calibration_t *value)
{
  if (value == NULL) return false;
  for (unsigned i = 0; i < 2; ++i)
    if (!isfinite(value->filter_tau_ms[i]) || value->filter_tau_ms[i] < 0.0f || value->filter_tau_ms[i] > 2000.0f) return false;
  for (unsigned i = 0; i < 4; ++i)
    if (!isfinite(value->thermal[i])) return false;
  for (unsigned i = 0; i < 7; ++i)
    if (!isfinite(value->kg50[i])) return false;
  if (value->thermal[0] < -40.0f || value->thermal[0] > 125.0f ||
      value->thermal[2] < -40.0f || value->thermal[2] > 125.0f) return false;
  return isfinite(value->kg1000_slope) &&
         isfinite(value->kg1000_intercept) && isfinite(value->iadc_slope) &&
         isfinite(value->iadc_intercept);
}

/* Local copy of the GroundStation's selection, so a reboot without the network
 * keeps scanning the same physical input. GroundStation's value still wins as
 * soon as it arrives; this board never publishes the variable. */
static void kg50_input_restore(void)
{
#ifdef TELEMETRY_ENABLED
  uint8_t stored = 0U;
  size_t size = sizeof(stored);
  const launchcore_persist_status_t status = persistent_store_get(
      DAQ_KG50_INPUT_PERSIST_KEY, &stored, &size);
  if (status == LAUNCHCORE_PERSIST_NOT_FOUND) return;
  if (status != LAUNCHCORE_PERSIST_OK || size != sizeof(stored) || stored > 1U)
  {
    g_daq_calibration_errors++;
    return;
  }
  g_kg50_input_stored = stored;
  mcp3564r_request_kg50_input(stored);
  g_daq_kg50_input_restores++;
#endif
}

void daq_calibration_restore(void)
{
  if (g_restore_attempted) return;
  g_restore_attempted = true;
  if (persistent_store_init() != LAUNCHCORE_PERSIST_OK)
  {
    g_daq_calibration_errors++;
    return;
  }
  g_persist_ready = true;
  kg50_input_restore();
  daq_calibration_t stored = g_calibration;
  size_t size = sizeof(stored);
  const launchcore_persist_status_t status = persistent_store_get(
      DAQ_CALIBRATION_PERSIST_KEY, &stored, &size);
  if (status == LAUNCHCORE_PERSIST_NOT_FOUND) return;
  if (status != LAUNCHCORE_PERSIST_OK || (size != sizeof(stored) && size != 4U * sizeof(float) && size != 11U * sizeof(float) && size != 15U * sizeof(float)) ||
      !calibration_valid(&stored))
  {
    g_daq_calibration_errors++;
    return;
  }
  g_calibration = stored;
  g_daq_calibration_restores++;
}

#ifdef TELEMETRY_ENABLED
/* A value this board refuses can never succeed on retransmit. Retire the SDK's
 * cached observation (user is the router) and acknowledge it without effect. */
static SedsResult refuse_calibration(const SedsPacketView *packet, void *user)
{
  if (packet != NULL)
    (void)seds_router_invalidate_network_variable_cache((SedsRouter *)user, packet->ty, NULL);
  g_daq_calibration_errors++;
  ++g_reliable_refusal_count;
  return SEDS_OK;
}

static SedsResult apply_calibration(const SedsPacketView *packet, void *user)
{
  const bool filter = packet != NULL && packet->ty == SEDS_DT_DAQ_FILTER_CALIBRATION;
  const bool thermal = packet != NULL && packet->ty == SEDS_DT_DAQ_THERMAL_CALIBRATION;
  const bool kg50 = packet != NULL && packet->ty == SEDS_DT_DAQ_KG50_CALIBRATION;
  if (packet == NULL || (!filter && !thermal && !kg50 && packet->ty != SEDS_DT_DAQ_LOADCELL_CALIBRATION) ||
      packet->payload == NULL || packet->payload_len != (filter ? 2U : kg50 ? 7U : 4U) * sizeof(float))
  {
    return refuse_calibration(packet, user);
  }
  daq_calibration_t next = daq_calibration_current();
  memcpy(filter ? (void *)next.filter_tau_ms : thermal ? (void *)next.thermal : kg50 ? (void *)next.kg50 : (void *)&next,
         packet->payload, packet->payload_len);
  if (!calibration_valid(&next)) return refuse_calibration(packet, user);
  const bool changed = memcmp(&next, &g_calibration, sizeof(next)) != 0;
  if (changed)
  {
    if (!g_persist_ready ||
        persistent_store_set(DAQ_CALIBRATION_PERSIST_KEY, &next, sizeof(next)) !=
            LAUNCHCORE_PERSIST_OK)
    {
      /* Transient: retire the cached observation so the identical retry
       * runs this callback again instead of being accepted unpersisted. */
      (void)seds_router_invalidate_network_variable_cache((SedsRouter *)user, packet->ty, NULL);
      g_daq_calibration_errors++;
      return SEDS_HANDLER_ERROR;
    }
    g_daq_calibration_persist_writes++;
  }
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  g_calibration = next;
  if (primask == 0U) __enable_irq();
  if (filter) g_filter_network_value_seen = true;
  else if (thermal) g_thermal_network_value_seen = true;
  else if (kg50) g_kg50_network_value_seen = true;
  else g_network_value_seen = true;
  g_daq_calibration_updates++;
  if (changed) sd_card_set_calibration(&next);
  return SEDS_OK;
}

static SedsResult apply_kg50_input(const SedsPacketView *packet, void *user)
{
  if (packet == NULL || packet->payload == NULL || packet->payload_len != 1U || ((const uint8_t *)packet->payload)[0] > 1U)
    return refuse_calibration(packet, user);
  const uint8_t input = ((const uint8_t *)packet->payload)[0];
  /* A failed write must not hold back the selection: only the next reboot
   * without GroundStation would see the older value. */
  if (input != g_kg50_input_stored)
  {
    if (g_persist_ready &&
        persistent_store_set(DAQ_KG50_INPUT_PERSIST_KEY, &input, sizeof(input)) == LAUNCHCORE_PERSIST_OK)
    {
      g_kg50_input_stored = input;
      g_daq_kg50_input_persist_writes++;
    }
    else
    {
      g_daq_calibration_errors++;
    }
  }
  mcp3564r_request_kg50_input(input);
  g_input_network_value_seen = true;
  return SEDS_OK;
}

SedsResult daq_calibration_init(SedsRouter *router)
{
  if (router == NULL) return SEDS_BAD_ARG;
  daq_calibration_restore();
  sd_card_set_calibration(&g_calibration);
  SedsResult result = seds_router_enable_network_variable(
      router, SEDS_DT_DAQ_LOADCELL_CALIBRATION, true, false);
  if (result != SEDS_OK) return result;
  result = seds_router_on_network_variable_update(
      router, SEDS_DT_DAQ_LOADCELL_CALIBRATION, apply_calibration, router);
  if (result != SEDS_OK) return result;
  result = seds_router_enable_network_variable(router, SEDS_DT_DAQ_KG50_CALIBRATION, true, false);
  if (result != SEDS_OK) return result;
  result = seds_router_on_network_variable_update(router, SEDS_DT_DAQ_KG50_CALIBRATION,
                                                  apply_calibration, router);
  if (result != SEDS_OK) return result;
  result = seds_router_enable_network_variable(router, SEDS_DT_DAQ_THERMAL_CALIBRATION, true, false);
  if (result != SEDS_OK) return result;
  result = seds_router_on_network_variable_update(router, SEDS_DT_DAQ_THERMAL_CALIBRATION,
                                                  apply_calibration, router);
  if (result != SEDS_OK) return result;
  result = seds_router_enable_network_variable(router, SEDS_DT_DAQ_FILTER_CALIBRATION, true, false);
  if (result != SEDS_OK) return result;
  result = seds_router_on_network_variable_update(router, SEDS_DT_DAQ_FILTER_CALIBRATION, apply_calibration, router);
  if (result != SEDS_OK) return result;
  result = seds_router_enable_network_variable(router, SEDS_DT_DAQ_KG50_INPUT, true, false);
  if (result != SEDS_OK) return result;
  result = seds_router_on_network_variable_update(router, SEDS_DT_DAQ_KG50_INPUT, apply_kg50_input, router);
  g_last_refresh_ms = HAL_GetTick();
  return result;
}

SedsResult daq_calibration_poll(SedsRouter *router)
{
  if (router == NULL) return SEDS_BAD_ARG;
  if ((g_network_value_seen && g_kg50_network_value_seen && g_thermal_network_value_seen && g_filter_network_value_seen && g_input_network_value_seen) || g_telemetry_discovery_seen == 0U) return SEDS_OK;
  const uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - g_last_refresh_ms) < DAQ_CALIBRATION_RETRY_MS) return SEDS_OK;
  g_last_refresh_ms = now;
  SedsResult result = SEDS_OK;
  if (!g_network_value_seen)
    result = seds_router_request_managed_variable(router, SEDS_DT_DAQ_LOADCELL_CALIBRATION);
  if (result == SEDS_OK && !g_kg50_network_value_seen)
    result = seds_router_request_managed_variable(router, SEDS_DT_DAQ_KG50_CALIBRATION);
  if (result == SEDS_OK && !g_thermal_network_value_seen)
    result = seds_router_request_managed_variable(router, SEDS_DT_DAQ_THERMAL_CALIBRATION);
  if (result == SEDS_OK && !g_filter_network_value_seen)
    result = seds_router_request_managed_variable(router, SEDS_DT_DAQ_FILTER_CALIBRATION);
  if (result == SEDS_OK && !g_input_network_value_seen)
    result = seds_router_request_managed_variable(router, SEDS_DT_DAQ_KG50_INPUT);
  return result;
}

#endif

daq_calibration_t daq_calibration_current(void)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const daq_calibration_t result = g_calibration;
  if (primask == 0U) __enable_irq();
  return result;
}

float daq_calibration_apply_kg1000(float raw_value)
{
  return g_calibration.kg1000_slope * raw_value + g_calibration.kg1000_intercept;
}

float daq_calibration_apply_kg50(const daq_calibration_t *calibration, float raw_value)
{
  const float *c = calibration->kg50;
  const float x = raw_value - c[5];
  return ((((c[4] * x + c[3]) * x + c[2]) * x + c[1]) * x + c[0]) - c[6];
}

float daq_calibration_temperature_raw(const daq_calibration_t *calibration,
                                      unsigned channel, float raw, float temperature_c)
{
  if (channel > 1U) return NAN;
  const float *t = &calibration->thermal[channel * 2U];
  if (t[1] == 0.0f) return raw;
  if (!isfinite(temperature_c) || temperature_c < -40.0f || temperature_c > 125.0f) return NAN;
  return raw - t[1] * (temperature_c - t[0]);
}
