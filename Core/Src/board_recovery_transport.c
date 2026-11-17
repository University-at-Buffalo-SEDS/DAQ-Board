#include "persistent_store_health.h"
#include "board_commissioning.h"
#include "tx_api.h"
#include "board_recovery_transport.h"
#include "board_recovery_wire.h"
#include "board_recovery_boot.h"
#include "board_recovery_floor.h"
#include "board_recovery.h"
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
uint64_t board_recovery_packet_stamp(const SedsPacketView *packet)
{
 return packet==NULL?0U:board_stamp_key(packet->time_epoch,packet->timestamp);
}
/* The station request validity window (-15 s/+1 s), judged by SEDSnet against
 * this board's network time across the time steps it remembers. No network
 * time, or an older epoch it cannot translate, is never fresh. */
uint8_t board_recovery_stamp_fresh(SedsRouter *router,uint64_t stamp)
{
 bool fresh=false;
 if(router==NULL)router=recovery_router;
 return router!=NULL&&stamp!=0U&&stamp!=UINT64_MAX&&
        seds_router_network_stamp_fresh(router,board_stamp_epoch(stamp),board_stamp_ms(stamp),
            BOARD_RECOVERY_FRESH_MAX_AGE_MS,BOARD_RECOVERY_FRESH_MAX_AHEAD_MS,&fresh)==SEDS_OK&&fresh;
}
uint8_t board_recovery_command_is_stale(const SedsPacketView *packet)
{
 const uint64_t stamp=board_recovery_packet_stamp(packet);
 if (packet==0 || !board_recovery_command_time_allowed(stamp)) return 1U;
 if (board_recovery_floor_active() && !board_recovery_stamp_fresh(recovery_router,stamp)) return 1U;
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
SedsResult board_recovery_restore_time_epoch(SedsRouter *router)
{
 uint64_t mark=0U;
 if(router==NULL)return SEDS_BAD_ARG;
 if(!board_recovery_floor_cached_cutoff(&mark)||board_stamp_epoch(mark)==0U)return SEDS_OK;
 return seds_router_raise_time_epoch(router,board_stamp_epoch(mark),NULL);
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
static bool commissioning_fresh(void*context,uint64_t stamp){return board_recovery_stamp_fresh((SedsRouter*)context,stamp)!=0U;}
static bool commissioning_reply(void*context,const BoardRecoveryRequest*request,uint8_t result){uint32_t uptime=(uint32_t)(((uint64_t)tx_time_get()*1000U)/TX_TIMER_TICKS_PER_SECOND);return commissioning_status((SedsRouter*)context,result,request,uptime,0U)==SEDS_OK;}
/* BOARD_STATE_STATUS is an ordered reliable stream: a status goes on air only
 * once every older one has its station receipt. The Abort push carries the
 * live snapshot, so it supersedes them; without this it waits behind them, for
 * seconds when their receipts are lost on air. Rows are read one at a time (no
 * scratch); a completed row shifts the rest down, so that index is read again.
 * The SDK never completes ABORT or another urgent type this way. */
static void retire_superseded_status(SedsRouter*router){
 SedsReliablePendingRow row;
 for(size_t index=0,guard=0;guard<64U;++guard){
  if(seds_router_copy_reliable_pending(router,index,&row,1)<=(int32_t)index)return;
  if(row.data_type==(uint32_t)SEDS_DT_BOARD_STATE_STATUS&&
     seds_router_complete_reliable(router,row.packet_id)==SEDS_OK)continue;
  ++index;
 }
}
/* Issuer of the last valid station QUERY, echoed by the Abort push. */
static uint64_t last_query_issuer;
/* Caller holds the router (FLIGHT_STATE callback): one unsolicited status for
 * an applied change to Aborted, so the station sees it even while its ordered
 * queries wait. Nonce 0, the last valid QUERY issuer, the QUERY generation. */
SedsResult board_recovery_push_abort_status(SedsRouter*router){
 if(!router)return SEDS_BAD_ARG;
 retire_superseded_status(router);
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
  if(result==BOARD_RECOVERY_RESULT_OK){
   result=board_commissioning_query_result();
   if(board_commissioning_pending()&&!board_commissioning_note_query(&request,board_recovery_packet_stamp(packet),snapshot.abort_epoch,router,commissioning_fresh))result=BOARD_RECOVERY_RESULT_IO;
  }
  
  return commissioning_status(router,result,&request,uptime_ms,0U);
 }
 if(result==BOARD_RECOVERY_RESULT_OK){
  int32_t staged=board_commissioning_stage(&request,board_recovery_packet_stamp(packet),snapshot.abort_epoch,router,commissioning_reply,commissioning_fresh);
  if(staged!=-2){if(staged==-1)return SEDS_OK;return commissioning_status(router,(uint8_t)staged,&request,uptime_ms,0U);}
 }
 /* No ordinary fallback may bypass the durable transaction service. Strict
  * unclassified/tombstoned/failed metadata remains observable and inhibited. */
 return commissioning_status(router,result==BOARD_RECOVERY_RESULT_OK?BOARD_RECOVERY_RESULT_IO:result,&request,uptime_ms,0U);
}
