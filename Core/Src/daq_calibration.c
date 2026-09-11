#include "daq_calibration.h"

#include "main.h"
#include "persistent_store.h"
#include "sd_card.h"
#include "sedsnet_config.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

extern volatile uint32_t g_telemetry_discovery_seen;

#define DAQ_CALIBRATION_PERSIST_KEY 0x44414341u
#define DAQ_CALIBRATION_RETRY_MS 500U

static daq_calibration_t g_calibration = {1.0f, 0.0f, 1.0f, 0.0f};
static bool g_restore_attempted;
static bool g_persist_ready;
static bool g_network_value_seen;
static uint32_t g_last_refresh_ms;

volatile uint32_t g_daq_calibration_updates __attribute__((used, externally_visible));
volatile uint32_t g_daq_calibration_restores __attribute__((used, externally_visible));
volatile uint32_t g_daq_calibration_persist_writes __attribute__((used, externally_visible));
volatile uint32_t g_daq_calibration_errors __attribute__((used, externally_visible));

static bool calibration_valid(const daq_calibration_t *value)
{
  return value != NULL && isfinite(value->kg1000_slope) &&
         isfinite(value->kg1000_intercept) && isfinite(value->iadc_slope) &&
         isfinite(value->iadc_intercept);
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
  daq_calibration_t stored;
  size_t size = sizeof(stored);
  const launchcore_persist_status_t status = persistent_store_get(
      DAQ_CALIBRATION_PERSIST_KEY, &stored, &size);
  if (status == LAUNCHCORE_PERSIST_NOT_FOUND) return;
  if (status != LAUNCHCORE_PERSIST_OK || size != sizeof(stored) ||
      !calibration_valid(&stored))
  {
    g_daq_calibration_errors++;
    return;
  }
  g_calibration = stored;
  g_daq_calibration_restores++;
}

static SedsResult apply_calibration(const SedsPacketView *packet, void *user)
{
  (void)user;
  if (packet == NULL || packet->ty != SEDS_DT_DAQ_LOADCELL_CALIBRATION ||
      packet->payload == NULL || packet->payload_len != sizeof(daq_calibration_t))
  {
    return SEDS_HANDLER_ERROR;
  }
  daq_calibration_t next;
  memcpy(&next, packet->payload, sizeof(next));
  if (!calibration_valid(&next))
  {
    g_daq_calibration_errors++;
    return SEDS_HANDLER_ERROR;
  }
  const bool changed = memcmp(&next, &g_calibration, sizeof(next)) != 0;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  g_calibration = next;
  if (primask == 0U) __enable_irq();
  g_network_value_seen = true;
  g_daq_calibration_updates++;
  if (changed)
  {
    if (!g_persist_ready ||
        persistent_store_set(DAQ_CALIBRATION_PERSIST_KEY, &next, sizeof(next)) !=
            LAUNCHCORE_PERSIST_OK)
    {
      g_daq_calibration_errors++;
      return SEDS_HANDLER_ERROR;
    }
    g_daq_calibration_persist_writes++;
    sd_card_set_calibration(&next);
  }
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
      router, SEDS_DT_DAQ_LOADCELL_CALIBRATION, apply_calibration, NULL);
  g_last_refresh_ms = HAL_GetTick();
  return result;
}

SedsResult daq_calibration_poll(SedsRouter *router)
{
  if (router == NULL) return SEDS_BAD_ARG;
  if (g_network_value_seen || g_telemetry_discovery_seen == 0U) return SEDS_OK;
  const uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - g_last_refresh_ms) < DAQ_CALIBRATION_RETRY_MS) return SEDS_OK;
  g_last_refresh_ms = now;
  return seds_router_request_managed_variable(router,
                                               SEDS_DT_DAQ_LOADCELL_CALIBRATION);
}

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
