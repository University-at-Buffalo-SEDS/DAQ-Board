#ifndef BOARD_RECOVERY_TRANSPORT_H
#define BOARD_RECOVERY_TRANSPORT_H
#include "sedsnet.h"
#include <stdbool.h>
#include "sedsnet_config.h"
#include "board_stamp.h"
void board_recovery_transport_set_router(SedsRouter *router);
/* Delivered reliable packets refused (not applied) but still acknowledged. */
extern volatile uint32_t g_reliable_refusal_count;
/* Caller holds the router: one nonce-0 Abort-latched BOARD_STATE_STATUS. */
SedsResult board_recovery_push_abort_status(SedsRouter *router);
SedsResult board_recovery_handle(SedsRouter *router,const SedsPacketView *packet,
                                uint32_t local_endpoint,uint32_t uptime_ms);
/* Application replay floor; SDK stays generic. No current-time/RNG claim.
 * Every stamp is (time_epoch, timestamp) as a board_stamp key. */
uint8_t board_recovery_command_is_stale(const SedsPacketView *packet);
uint8_t board_recovery_managed_update_allowed(const SedsPacketView *packet);
uint8_t board_recovery_automatic_is_stale(const SedsPacketView *packet);
/* A station request's (epoch, timestamp) key; 0 when it has none. */
uint64_t board_recovery_packet_stamp(const SedsPacketView *packet);
/* Station request validity window, judged by seds_router_network_stamp_fresh. */
#define BOARD_RECOVERY_FRESH_MAX_AGE_MS 15000U
#define BOARD_RECOVERY_FRESH_MAX_AHEAD_MS 1000U
uint8_t board_recovery_stamp_fresh(SedsRouter *router,uint64_t stamp);
/* Router owner, once the router exists at boot: raise the network time epoch
 * to the persisted recovery floor's, so the floor can never be above every
 * new station stamp after a whole-stack power cycle. */
SedsResult board_recovery_restore_time_epoch(SedsRouter *router);
#endif
