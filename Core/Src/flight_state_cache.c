#include "flight_state_cache.h"
#include "main.h"
#include "sedsnet_config.h"
#ifdef TELEMETRY_ENABLED
#include "board_recovery_transport.h"
#include "ota_stream.h"
#endif

#include "persistent_store.h"

#include <stddef.h>
#include <string.h>

extern volatile uint32_t g_telemetry_discovery_seen;

#define FLIGHT_STATE_PERSIST_KEY 0x46535445u
/* InFlight (16), the highest value in contract set (a). */
#define FLIGHT_STATE_MAX_VALUE 16U
#define FLIGHT_STATE_ABORTED 15U
#define FLIGHT_STATE_UNSYNCED_RETRY_MS 500U
#define FLIGHT_STATE_PERSIST_RETRY_MS 1000U

volatile uint32_t g_flight_state_cache_value __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_flight_state_cache_restores __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_flight_state_cache_writes __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_flight_state_cache_updates __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_flight_state_cache_errors __attribute__((used, externally_visible)) = 0U;

static bool g_restore_attempted;
static bool g_persist_ready;
static bool g_has_value;
static bool g_persist_pending;
static uint8_t g_persisted_state = UINT8_MAX;
#ifdef TELEMETRY_ENABLED
static bool g_network_value_seen;
/* GET cadence while unseen; once a value is applied (and only then can a
 * persist be pending) it paces the durability retry instead. */
static uint32_t g_last_refresh_ms;
static SedsRouter *g_cache_router;
static SedsResult persist_update(const SedsPacketView *packet, void *user);
/* SDK 4.3.6 caches inbound data before its callback. Retire only this
 * observation; keep read-only permissions, callbacks and advertisements. */
static SedsResult clear_managed_cache(void)
{
    if (g_cache_router == NULL) return SEDS_BAD_ARG;
    return seds_router_invalidate_network_variable_cache(
        g_cache_router, SEDS_DT_FLIGHT_STATE, NULL);
}
#endif

#ifdef TELEMETRY_ENABLED
__attribute__((weak)) SedsResult
flight_state_cache_apply_network_update(const SedsPacketView *packet)
{
    (void)packet;
    return SEDS_OK;
}
/* board_recovery_transport.c owns the definition. */
__attribute__((weak)) SedsResult
board_recovery_push_abort_status(SedsRouter *router)
{
    (void)router;
    return SEDS_OK;
}

#endif

/* Retired FLIGHT_STATE values received (contract violations, one per
 * delivery), and the one-time log of the first: its raw value. */
volatile uint32_t g_flight_state_contract_violations __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_flight_state_contract_logged __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_flight_state_contract_first_raw __attribute__((used, externally_visible)) = 0U;

uint8_t flight_state_cache_from_wire(uint8_t raw)
{
    uint8_t retired = 0U;
    const uint8_t state = flight_state_from_wire(raw, &retired);
    if (retired != 0U) {
        ++g_flight_state_contract_violations;
        if (g_flight_state_contract_logged == 0U) {
            g_flight_state_contract_logged = 1U;
            g_flight_state_contract_first_raw = raw;
        }
    }
    return state;
}

void flight_state_cache_restore(void)
{
    if (g_restore_attempted) return;
    g_restore_attempted = true;

    if (persistent_store_init() != LAUNCHCORE_PERSIST_OK)
    {
        g_flight_state_cache_errors++;
        return;
    }
    g_persist_ready = true;

    uint8_t state = 0U;
    size_t size = sizeof(state);
    const launchcore_persist_status_t status = persistent_store_get(
        FLIGHT_STATE_PERSIST_KEY, &state, &size);
    if (status == LAUNCHCORE_PERSIST_NOT_FOUND) return;
    if (status != LAUNCHCORE_PERSIST_OK || size != sizeof(state) ||
        (state = flight_state_from_wire(state, NULL)) == FLIGHT_STATE_NET_INVALID)
    {
        g_flight_state_cache_errors++;
        return;
    }

    g_has_value = true;
    g_flight_state_cache_value = state;
    g_persisted_state = state;
#ifdef TELEMETRY_ENABLED
    /* A restored unsafe phase is evidence against automatic OTA, not proof
     * that this board physically launched or owns any outputs. */
    if (ota_stream_state_begin(state)) ota_stream_state_end();
#endif
    g_flight_state_cache_restores++;
}

bool flight_state_cache_has_value(void)
{
    return g_has_value;
}

uint8_t flight_state_cache_value(void)
{
    return (uint8_t)g_flight_state_cache_value;
}

bool flight_state_cache_is_durable(void)
{
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    const bool durable = persistent_store_recovery_ready() && g_has_value && !g_persist_pending &&
        g_persisted_state != UINT8_MAX && g_persisted_state == g_flight_state_cache_value;
    __set_PRIMASK(mask); return durable;
}

/* Existing SDK/cache owner only. The physical owner remains held through
 * write/readback and persisted-byte bookkeeping. No delayed work reapplies a
 * state or changes its packet origin. Busy never probes/writes storage. */
static int32_t persist_state_nowait(uint8_t value, bool retry_initialization)
{
    const persistent_store_owner_t owner = persistent_store_try_enter();
    if (owner == PERSISTENT_STORE_OWNER_BUSY) return -14;
    if (owner != PERSISTENT_STORE_OWNER_ACQUIRED) return -9;
    /* A different checked metadata failure may have arrived between admission
     * and acquisition. Explicit existing requalification must win first. */
    if (!persistent_store_recovery_ready()) {
        (void)persistent_store_leave(); return -9;
    }
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    g_persisted_state = UINT8_MAX; /* An uncertain physical mutation is not OLD. */
    __set_PRIMASK(mask);
    uint8_t verified = UINT8_MAX; size_t size = sizeof(verified);
    const bool valid = (!retry_initialization || persistent_store_init() == LAUNCHCORE_PERSIST_OK) &&
        persistent_store_set(FLIGHT_STATE_PERSIST_KEY, &value, sizeof(value)) == LAUNCHCORE_PERSIST_OK &&
        persistent_store_get(FLIGHT_STATE_PERSIST_KEY, &verified, &size) == LAUNCHCORE_PERSIST_OK &&
        size == sizeof(verified) && verified == value;
    mask = __get_PRIMASK(); __disable_irq();
    if (valid) g_persisted_state = value;
    __set_PRIMASK(mask);
    const bool released = persistent_store_leave();
    if (!valid || !released) {
        if (!valid) persistent_store_note_failure();
        mask = __get_PRIMASK(); __disable_irq(); g_persisted_state = UINT8_MAX; __set_PRIMASK(mask);
        return -9;
    }
    return 0;
}

int32_t flight_state_cache_reset_startup(void)
{
    const uint8_t startup = 0U;
    uint32_t mask = __get_PRIMASK(); __disable_irq(); g_persist_pending = true; __set_PRIMASK(mask);
    /* An explicit manual retry may inspect storage; it must first win the
     * same owner without waiting. Do not convert normal contention to IO. */
    const int32_t stored = persist_state_nowait(startup, true);
    if (stored == -14) return -14;
    if (stored != 0) { ++g_flight_state_cache_errors; return -9; }
    g_persist_ready = true; ++g_flight_state_cache_writes;
#ifdef TELEMETRY_ENABLED
    if (clear_managed_cache() != SEDS_OK) {
        mask = __get_PRIMASK(); __disable_irq(); g_persisted_state = UINT8_MAX; __set_PRIMASK(mask);
        ++g_flight_state_cache_errors; return -9;
    }
    g_network_value_seen = false;
    g_last_refresh_ms = HAL_GetTick();
#endif
    mask = __get_PRIMASK(); __disable_irq();
    g_flight_state_cache_value = startup; g_has_value = true; g_persist_pending = false;
    __set_PRIMASK(mask);
    return 0;
}

#ifdef TELEMETRY_ENABLED
/* Caller holds the OTA owner. Busy is ordinary contention, not a fault. */
static void persist_pending(void)
{
    const int32_t stored =
        persist_state_nowait((uint8_t)g_flight_state_cache_value, !g_persist_ready);
    if (stored != 0) {
        if (stored != -14) ++g_flight_state_cache_errors;
        return;
    }
    g_persist_ready = true;
    ++g_flight_state_cache_writes;
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    g_persist_pending = false;
    __set_PRIMASK(mask);
}

static SedsResult persist_update(const SedsPacketView *packet, void *user)
{
    (void)user;
    if (packet == NULL || packet->ty != SEDS_DT_FLIGHT_STATE ||
        packet->payload == NULL || packet->payload_len != 1U ||
        flight_state_from_wire(packet->payload[0], NULL) == FLIGHT_STATE_NET_INVALID)
    {
        /* Public SDK435 caches managed data before invoking this callback.
         * A malformed FSTATE must not poison future ordering/cache reads. */
        if (packet != NULL && packet->ty == SEDS_DT_FLIGHT_STATE)
            (void)clear_managed_cache();
        g_flight_state_cache_errors++;
        /* Retransmitting a malformed value cannot succeed: acknowledge it. */
        ++g_reliable_refusal_count;
        return SEDS_OK;
    }

    if (!board_recovery_managed_update_allowed(packet)) {
        /* Stale, unready or not the station: never applied or cached, but
         * acknowledged. An error here aborted RX before its ACK, so the
         * station retransmitted the same refused FLIGHT_STATE forever. */
        (void)clear_managed_cache();
        ++g_reliable_refusal_count;
        return SEDS_OK;
    }
    /* A retired value maps onto set (a); it is never refused into a
     * retransmit loop. */
    const uint8_t state = flight_state_cache_from_wire(packet->payload[0]);
    /* An applied change to Aborted pushes one status to the station. */
    const bool aborting = state == FLIGHT_STATE_ABORTED &&
        !(g_has_value && g_flight_state_cache_value == FLIGHT_STATE_ABORTED);
    /* Taking the OTA owner applies its inhibition even when busy; busy only
     * means flash is not ours now, so durability waits for the poll. */
    const bool owner = ota_stream_state_begin(state);
    const SedsResult apply_result = flight_state_cache_apply_network_update(packet);
    if (apply_result != SEDS_OK) {
        if (owner) ota_stream_state_end();
        (void)clear_managed_cache();
        g_flight_state_cache_errors++;
        return apply_result;
    }
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    const bool changed = g_persist_pending || state != g_persisted_state;
    g_has_value = true; g_network_value_seen = true;
    g_flight_state_cache_value = state;
    if (changed) g_persist_pending = true;
    __set_PRIMASK(mask);
    g_flight_state_cache_updates++;
    if (aborting) (void)board_recovery_push_abort_status(g_cache_router);
    if (changed) {
        /* Applied: acknowledge it whatever flash does. An error withholds
         * the receipt and stalls the station's ordered FLIGHT_STATE stream
         * (Aborted included) behind this write. */
        g_last_refresh_ms = HAL_GetTick();
        if (owner) persist_pending();
    }
    if (owner) ota_stream_state_end();
    return SEDS_OK;
}

SedsResult flight_state_cache_init(SedsRouter *router)
{
    if (router == NULL) return SEDS_BAD_ARG;
    /* A valid reinitialization must establish both services before cache
     * retirement may report success. Invalid NULL input preserves readiness. */
    g_cache_router = NULL;
    flight_state_cache_restore();
    SedsResult result = seds_router_enable_network_variable(
        router, SEDS_DT_FLIGHT_STATE, true, false);
    if (result != SEDS_OK) { ++g_flight_state_cache_errors; return result; }
    result = seds_router_on_network_variable_update(
        router, SEDS_DT_FLIGHT_STATE, persist_update, NULL);
    if (result != SEDS_OK) { ++g_flight_state_cache_errors; return result; }
    g_cache_router = router;
    g_last_refresh_ms = HAL_GetTick();
    return SEDS_OK;
}

SedsResult flight_state_cache_poll(SedsRouter *router)
{
    if (router == NULL) return SEDS_BAD_ARG;
    const uint32_t now_ms = HAL_GetTick();
    if (g_persist_pending &&
        (uint32_t)(now_ms - g_last_refresh_ms) >= FLIGHT_STATE_PERSIST_RETRY_MS) {
        g_last_refresh_ms = now_ms;
        if (ota_stream_state_begin((uint8_t)g_flight_state_cache_value)) {
            persist_pending();
            ota_stream_state_end();
        }
    }
    if (g_network_value_seen) return SEDS_OK;
    if (g_telemetry_discovery_seen == 0U) return SEDS_OK;
    if ((uint32_t)(now_ms - g_last_refresh_ms) <
        FLIGHT_STATE_UNSYNCED_RETRY_MS) return SEDS_OK;
    g_last_refresh_ms = now_ms;
    return seds_router_request_managed_variable(router, SEDS_DT_FLIGHT_STATE);
}

#endif /* TELEMETRY_ENABLED */
