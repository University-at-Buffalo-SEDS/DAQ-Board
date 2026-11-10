#include "persistent_store_health.h"
#include "board_commissioning.h"
#include "tx_api.h"
#include "board_recovery_transport.h"
#include "board_recovery_wire.h"
#include "board_recovery_boot.h"
#include "board_recovery_floor.h"
#include "board_recovery.h"
#include <stdatomic.h>
#include <string.h>
static SedsRouter *recovery_router;
/* Delivered reliable packets deliberately not applied, yet acknowledged so the
 * sender stops retransmitting a deterministic refusal. */
volatile uint32_t g_reliable_refusal_count __attribute__((used));
void board_recovery_transport_set_router(SedsRouter *router) { recovery_router=router; }
static uint8_t authorized_primary(SedsRouter *router,const SedsPacketView *packet)
{
 return packet!=NULL && seds_router_sender_matches_hostname(router,
     packet->sender,packet->sender_len,"GS",2U);
}
/* Station clock: the newest GS-issued timestamp, advanced by the board tick
 * since its receipt. It is a lower bound of GS time (transit only delays it),
 * so it never makes an old request look fresher than the GS clock would. It
 * only moves forward (a network clock resync restarts it from the next GS
 * packet) and is used beside, never instead of, a valid SEDSnet network
 * clock: an unsynced or wrongly anchored network clock cannot make a fresh
 * station request stale, and an absent one no longer refuses everything.
 * One writer (the router owner); readers use the sequence count. */
static volatile uint64_t station_utc;static volatile ULONG station_tick;static _Atomic uint32_t station_seq;
static uint64_t station_estimate(void){
 for(unsigned i=0U;i<4U;i++){
  uint32_t seq=atomic_load_explicit(&station_seq,memory_order_acquire);
  uint64_t utc=station_utc;ULONG tick=station_tick;
  atomic_thread_fence(memory_order_acquire);
  if(!(seq&1U)&&seq==atomic_load_explicit(&station_seq,memory_order_relaxed))
#if TX_TIMER_TICKS_PER_SECOND == 1000
   return utc?utc+(ULONG)(tx_time_get()-tick):0U;
#else
   return utc?utc+((uint64_t)(ULONG)(tx_time_get()-tick)*1000U)/TX_TIMER_TICKS_PER_SECOND:0U;
#endif
 }
 return 0U;
}
static void station_store(uint64_t utc){
 uint32_t seq=atomic_load_explicit(&station_seq,memory_order_relaxed);
 atomic_store_explicit(&station_seq,seq+1U,memory_order_relaxed);atomic_thread_fence(memory_order_release);
 station_utc=utc;station_tick=tx_time_get();
 atomic_store_explicit(&station_seq,seq+2U,memory_order_release);
}
void board_recovery_observe_station_time(SedsRouter*router,const SedsPacketView*packet){
 if(!router)router=recovery_router;
 if(router&&authorized_primary(router,packet)&&packet->timestamp!=UINT64_MAX&&packet->timestamp>station_estimate())station_store(packet->timestamp);
}
/* A relayed consumer cannot complete a verified SEDSnet time exchange, so a
 * first anchor from a source without real time (an RF board without a GPS fix
 * announces its uptime) is only slewed at 5% and never corrected. A valid
 * network clock more than 60 s from the station clock needs a timesync restart
 * so the next announce anchors it afresh. The station clock also restarts from
 * the next GS packet, so a GS clock step cannot make this repeat every poll. */
bool board_recovery_network_clock_resync_needed(uint64_t network_now){
 uint64_t station=station_estimate();
 if(!station||network_now-station+60000U<=120000U)return false;
 station_store(0U);return true;
}
/* Freshness clock: max(valid network time, station clock); 0 when neither. */
uint64_t board_recovery_station_now(SedsRouter*router){
 uint64_t now=0U;if(!router)router=recovery_router;
 if(!router||seds_router_get_network_time_ms(router,&now)!=SEDS_OK)now=0U;
 uint64_t station=station_estimate();return station>now?station:now;
}
uint8_t board_recovery_command_is_stale(const SedsPacketView *packet)
{
 if (packet==0 || !board_recovery_command_time_allowed(packet->timestamp)) return 1U;
 if (board_recovery_floor_active()) {
  uint64_t now=recovery_router?board_recovery_station_now(recovery_router):0U;
  if (packet->timestamp==0U || packet->timestamp==UINT64_MAX || now==0U || now==UINT64_MAX || (now>=packet->timestamp ? now-packet->timestamp>15000U : packet->timestamp-now>1000U)) return 1U;
 }
 return 0U;
}
uint8_t board_recovery_managed_update_allowed(const SedsPacketView *packet)
{
 if (!persistent_store_controls_ready() || !board_recovery_floor_is_healthy()) return 0U;
 if (!board_recovery_floor_active()) return packet!=NULL && packet->ty==SEDS_DT_FLIGHT_STATE ? recovery_router!=NULL && authorized_primary(recovery_router,packet) : 1U;
 return recovery_router!=0 && authorized_primary(recovery_router,packet) &&
        !board_recovery_command_is_stale(packet);
}
uint8_t board_recovery_automatic_is_stale(const SedsPacketView *packet)
{
 return board_recovery_command_is_stale(packet);
}

static SedsResult commissioning_status(SedsRouter*router,uint8_t result,const BoardRecoveryRequest*request,uint32_t uptime,uint8_t extra_flags){
 BoardRecoverySnapshot snapshot={.actual_state=255};
 if(board_recovery_snapshot(&snapshot)!=0)result=BOARD_RECOVERY_RESULT_IO;
 uint64_t generation=board_commissioning_query_generation();
 if(!generation&&!board_commissioning_uncertain_first())snapshot.flags&=(uint8_t)~BOARD_RECOVERY_SUPPORTED;
 if(request->operation==BOARD_RECOVERY_RECOVER&&result==BOARD_RECOVERY_RESULT_OK&&(snapshot.flags&BOARD_RECOVERY_ABORT_LATCHED)&&!board_commissioning_pending())result=BOARD_RECOVERY_RESULT_STALE;
 uint8_t wire[BOARD_RECOVERY_STATUS_SIZE];
 board_recovery_encode_status(wire,result,snapshot.actual_state,(uint8_t)(snapshot.flags|extra_flags|BOARD_RECOVERY_VERIFIED_IMPLEMENTATION),request,generation,snapshot.abort_epoch,uptime,snapshot.actual_outputs_mask);
 return seds_router_log_queue_typed(router,SEDS_DT_BOARD_STATE_STATUS,wire,sizeof wire,1,SEDS_EK_UNSIGNED);
}
static uint64_t commissioning_now(void*context){return board_recovery_station_now((SedsRouter*)context);}
static bool commissioning_reply(void*context,const BoardRecoveryRequest*request,uint8_t result){uint32_t uptime=(uint32_t)(((uint64_t)tx_time_get()*1000U)/TX_TIMER_TICKS_PER_SECOND);return commissioning_status((SedsRouter*)context,result,request,uptime,0U)==SEDS_OK;}
/* Issuer of the last valid station QUERY, echoed by the Abort push. */
static uint64_t last_query_issuer;
/* Caller holds the router (FLIGHT_STATE callback): one unsolicited status for
 * an applied change to Aborted, so the station sees it even while its ordered
 * queries wait. Nonce 0, the last valid QUERY issuer, the QUERY generation. */
SedsResult board_recovery_push_abort_status(SedsRouter*router){
 if(!router)return SEDS_BAD_ARG;
 BoardRecoveryRequest request={0};request.issuer_session=last_query_issuer;
 return commissioning_status(router,BOARD_RECOVERY_RESULT_OK,&request,
  (uint32_t)(((uint64_t)tx_time_get()*1000U)/TX_TIMER_TICKS_PER_SECOND),BOARD_RECOVERY_ABORT_LATCHED);
}
SedsResult board_recovery_handle(SedsRouter *router,const SedsPacketView *packet,
                                uint32_t local_endpoint,uint32_t uptime_ms)
{
 if (router==0 || packet==0) return SEDS_BAD_ARG;
 BoardRecoveryRequest request;
 /* A delivered request this board refuses (wrong type/endpoint, not the
  * station, malformed) can never succeed on retransmit: acknowledge it
  * without effect instead of making the station retry it forever. */
 if (packet->ty!=SEDS_DT_BOARD_RECOVERY_REQUEST || packet->num_endpoints!=1U ||
     packet->endpoints==0 || packet->endpoints[0]!=local_endpoint ||
     authorized_primary(router,packet)==0U ||
     board_recovery_decode_request(packet->payload,packet->payload_len,&request)==0U) {
  ++g_reliable_refusal_count; return SEDS_OK;
 }
 BoardRecoverySnapshot snapshot={.actual_state=255U};
 uint64_t generation=board_commissioning_query_generation();
 /* Stored MAX is exhaustion; only strict eligible cache may advertiseMAX. */
 /* Eligible MAX is supplied only by the strict commissioning boot cache. */
 uint8_t result=BOARD_RECOVERY_RESULT_OK;
 if (board_recovery_snapshot(&snapshot)!=0) result=BOARD_RECOVERY_RESULT_IO;
 if ((generation==0U&&!board_commissioning_uncertain_first()) || (snapshot.flags&BOARD_RECOVERY_SUPPORTED)==0U) {
  snapshot.flags&=(uint8_t)~BOARD_RECOVERY_SUPPORTED;
  result=BOARD_RECOVERY_RESULT_UNSUPPORTED;
 }
 /* These four domains own only cache Startup, not Idle/output recovery.
  * Reject unsupported intent before loading/committing a floor or calling owner. */
 if (request.operation==BOARD_RECOVERY_RECOVER && request.desired_state!=0U)
  result=BOARD_RECOVERY_RESULT_INVALID;
 /* Commissioning has its own explicit two-leg path. QUERY only reads boot
  * caches/registers and never provisions or refreshes storage. */
 if(request.operation==BOARD_RECOVERY_QUERY){
  last_query_issuer=request.issuer_session;
  /* The station's own QUERY time may advance the station clock: the station
   * acts only on a reply to its nonce within its query deadline. */
  board_recovery_observe_station_time(router,packet);
  if(result==BOARD_RECOVERY_RESULT_OK){
   result=board_commissioning_query_result();
   if(board_commissioning_pending()&&!board_commissioning_note_query(&request,packet->timestamp,commissioning_now(router),snapshot.abort_epoch))result=BOARD_RECOVERY_RESULT_IO;
  }
  
  return commissioning_status(router,result,&request,uptime_ms,0U);
 }
 if(result==BOARD_RECOVERY_RESULT_OK){
  int32_t staged=board_commissioning_stage(&request,packet->timestamp,commissioning_now(router),snapshot.abort_epoch,router,commissioning_reply,commissioning_now);
  if(staged!=-2){if(staged==-1)return SEDS_OK;return commissioning_status(router,(uint8_t)staged,&request,uptime_ms,0U);}
 }
 /* No ordinary fallback may bypass the durable transaction service. Strict
  * unclassified/tombstoned/failed metadata remains observable and inhibited. */
 return commissioning_status(router,result==BOARD_RECOVERY_RESULT_OK?BOARD_RECOVERY_RESULT_IO:result,&request,uptime_ms,0U);
}
