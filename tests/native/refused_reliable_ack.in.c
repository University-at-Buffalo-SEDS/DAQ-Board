/* Real SDK router -> actual DAQ receive callbacks. A delivered reliable packet
 * that the DAQ deliberately refuses must still be acknowledged (no endless
 * retransmit) and must not be applied; a transient refusal keeps its retry. */
#include "sedsnet.h"
#include "sedsnet_config.h"
#include "daq_calibration.h"
#include "sd_calendar.h"
#include "board_recovery_wire.h"
#include "board_recovery.h"
#include "board_commissioning.h"
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static struct { SedsRouter *r; } g_router;
volatile uint32_t g_reliable_refusal_count;
static unsigned irq;
static uint32_t __get_PRIMASK(void){return irq;}
static void __disable_irq(void){irq=1;}
static void __enable_irq(void){irq=0;}
static void __set_PRIMASK(uint32_t n){irq=n;}
#define LAUNCHCORE_PERSIST_OK 0
static int fail_persist;
static int persistent_store_set(uint32_t key,const void*v,size_t n){(void)key;(void)v;(void)n;return fail_persist;}

/* SD retry request. */
typedef enum{SD_CARD_STATUS_OK=0,SD_CARD_STATUS_BUSY=1,SD_CARD_STATUS_BACKPRESSURE=2,SD_CARD_STATUS_IO_ERROR=3}sd_card_status_t;
static unsigned retry_calls,retry_busy;static uint32_t retry_token;
static sd_card_status_t sd_card_request_retry(uint32_t token){++retry_calls;if(!token)return SD_CARD_STATUS_IO_ERROR;if(retry_busy)return SD_CARD_STATUS_BUSY;retry_token=token;return SD_CARD_STATUS_OK;}
/* SOURCE_FUNCTION:sd_retry_receive */

/* Calibration network variables. */
static daq_calibration_t g_calibration={1.0f,0.0f,1.0f,0.0f,{0},{0},{0.0f,1.0f}};
static bool g_persist_ready=true,g_network_value_seen,g_kg50_network_value_seen,g_thermal_network_value_seen,g_filter_network_value_seen;
static uint32_t g_daq_calibration_errors,g_daq_calibration_persist_writes,g_daq_calibration_updates;
#define DAQ_CALIBRATION_PERSIST_KEY 1U
static unsigned calibration_sets;static void sd_card_set_calibration(const daq_calibration_t*c){(void)c;++calibration_sets;}
/* SOURCE_FUNCTION:calibration_valid */
/* SOURCE_FUNCTION:daq_calibration_current */
/* SOURCE_FUNCTION:refuse_calibration */
/* SOURCE_FUNCTION:apply_calibration */

/* Launch log clock network variable. */
static bool clock_seen;
static uint64_t clock_session,clock_revision,clock_high_water,clock_deadline,clock_value_session;
static bool revision_seen,reset_ambiguous;
static volatile uint32_t g_daq_clock_stale_updates,g_daq_clock_ambiguous_resets;
static unsigned clock_sets;static void sd_card_set_launch_clock(uint64_t s,uint64_t d){(void)s;(void)d;++clock_sets;}
/* SOURCE_FUNCTION:read_u64 */
/* SOURCE_FUNCTION:clock_update */

/* FLIGHT_STATE managed variable. */
#define FLIGHT_STATE_MAX_VALUE 15U
static volatile uint32_t g_flight_state_cache_value,g_flight_state_cache_writes,g_flight_state_cache_updates,g_flight_state_cache_errors;
static bool g_has_value,g_persist_pending;static uint8_t g_persisted_state=UINT8_MAX;
static SedsRouter*g_cache_router;
static uint8_t managed_allowed;static unsigned applies,ota_open;
static uint8_t board_recovery_managed_update_allowed(const SedsPacketView*p){(void)p;return managed_allowed;}
static bool ota_stream_state_begin(uint8_t s){(void)s;++ota_open;return true;}
static void ota_stream_state_end(void){--ota_open;}
static SedsResult flight_state_cache_apply_network_update(const SedsPacketView*p){(void)p;++applies;return SEDS_OK;}
static int32_t persist_state_nowait(uint8_t s,bool f){(void)s;(void)f;return 0;}
static uint32_t g_last_refresh_ms;
static uint32_t HAL_GetTick(void){return 0U;}
/* SOURCE_FUNCTION:clear_managed_cache */
/* SOURCE_FUNCTION:persist_pending */
/* SOURCE_FUNCTION:persist_update */

/* BOARD_RECOVERY_REQUEST: only the refusal paths are reachable here. */
uint8_t board_recovery_decode_request(const uint8_t*w,size_t n,BoardRecoveryRequest*r){(void)r;return w&&n==32U&&w[0]==1U;}
int32_t board_recovery_snapshot(BoardRecoverySnapshot*o){(void)o;abort();}
uint64_t board_commissioning_query_generation(void){abort();}
bool board_commissioning_uncertain_first(void){abort();}
uint8_t board_commissioning_query_result(void){abort();}
bool board_commissioning_pending(void){abort();}
bool board_commissioning_note_query(const BoardRecoveryRequest*r,uint64_t a,uint64_t b,uint32_t c){(void)r;(void)a;(void)b;(void)c;abort();}
int32_t board_commissioning_stage(const BoardRecoveryRequest*r,uint64_t a,uint64_t b,uint32_t c,void*x,CommissionReply y,CommissionClock z){(void)r;(void)a;(void)b;(void)c;(void)x;(void)y;(void)z;abort();}
static void board_recovery_observe_station_time(SedsRouter*r,const SedsPacketView*p){(void)r;(void)p;abort();}
static SedsResult commissioning_status(SedsRouter*r,uint8_t x,const BoardRecoveryRequest*q,uint32_t u){(void)r;(void)x;(void)q;(void)u;abort();}
static uint64_t commissioning_now(void*c){(void)c;abort();}
static bool commissioning_reply(void*c,const BoardRecoveryRequest*q,uint8_t x){(void)c;(void)q;(void)x;abort();}
/* SOURCE_FUNCTION:authorized_primary */
/* SOURCE_FUNCTION:board_recovery_handle */
static SedsResult recovery_handler(const SedsPacketView*p,void*u){(void)u;return board_recovery_handle(g_router.r,p,SEDS_EP_BOARD_RECOVERY,0U);}

static uint64_t now(void*u){(void)u;return 1767225601000ULL;}
static unsigned acks;
static SedsResult side(const SedsPacketView*p,void*u){(void)u;if(p->sender_len>=7&&!memcmp(p->sender,"E2EACK:",7))acks++;return SEDS_OK;}
static SedsRouter*router(const SedsLocalEndpointDesc*l,size_t n,int32_t*sid){
 SedsRouter*r=seds_router_new(now,NULL,l,n);assert(r);assert(seds_router_set_sender_id(r,"DAQ",3)==SEDS_OK);g_router.r=r;
 *sid=seds_router_add_side_packet(r,"link",4,side,NULL,false);assert(*sid>=0);return r;
}
static uint64_t stamp=1767225600000ULL;
/* Deliver one new reliable packet; report the receive result and ACK delta. */
static SedsResult deliver(SedsRouter*r,int32_t sid,uint32_t ty,uint32_t ep,const char*sender,const void*payload,size_t n,unsigned*acked){
 SedsPacketView p={.ty=ty,.data_size=n,.sender=sender,.sender_len=strlen(sender),.endpoints=&ep,.num_endpoints=1,.timestamp=++stamp,.payload=payload,.payload_len=n};
 unsigned before=acks;SedsResult res=seds_router_receive_from_side(r,(uint32_t)sid,&p);(void)seds_router_process_all_queues(r);*acked=acks-before;
 printf("deliver ty=%u sender=%s receive=%d acks=%u refusals=%u\n",ty,sender,res,*acked,g_reliable_refusal_count);return res;
}
static void put_u64(uint8_t*p,uint64_t v){for(unsigned i=0;i<8;i++)p[i]=(uint8_t)(v>>(8U*i));}
int main(int argc,char**argv){
 setbuf(stdout,NULL);assert(argc==3);assert(seds_schema_register_json_file(argv[1],strlen(argv[1]))==SEDS_OK);
 const char*mode=argv[2];int32_t sid;unsigned acked;SedsRouter*r;
 if(!strcmp(mode,"sd_retry")){
  SedsLocalEndpointDesc l={.endpoint=SEDS_EP_SD_CARD,.packet_handler=sd_retry_receive};r=router(&l,1,&sid);
  uint8_t token[4]={5,0,0,0},zero[4]={0};
  /* Unauthorized requester: refused, never applied, but acknowledged. */
  assert(deliver(r,sid,SEDS_DT_DAQ_SD_RETRY,SEDS_EP_SD_CARD,"VB",token,4,&acked)==SEDS_OK&&acked==1&&retry_calls==0&&g_reliable_refusal_count==1);
  /* Invalid token from the station: deterministic refusal, acknowledged. */
  assert(deliver(r,sid,SEDS_DT_DAQ_SD_RETRY,SEDS_EP_SD_CARD,"GS",zero,4,&acked)==SEDS_OK&&acked==1&&retry_token==0&&g_reliable_refusal_count==2);
  /* Busy writer is transient: no ACK, so the station retries. */
  retry_busy=1;assert(deliver(r,sid,SEDS_DT_DAQ_SD_RETRY,SEDS_EP_SD_CARD,"GS",token,4,&acked)!=SEDS_OK&&acked==0&&retry_token==0&&g_reliable_refusal_count==2);
  retry_busy=0;assert(deliver(r,sid,SEDS_DT_DAQ_SD_RETRY,SEDS_EP_SD_CARD,"GS",token,4,&acked)==SEDS_OK&&acked==1&&retry_token==5);
  puts("PASS refused SD retry requests are acknowledged and not applied; busy keeps its retry");
 }else if(!strcmp(mode,"calibration")){
  r=router(NULL,0,&sid);
  assert(seds_router_enable_network_variable(r,SEDS_DT_DAQ_FILTER_CALIBRATION,true,false)==SEDS_OK);
  assert(seds_router_on_network_variable_update(r,SEDS_DT_DAQ_FILTER_CALIBRATION,apply_calibration,r)==SEDS_OK);
  float bad[2]={-1.0f,1.0f},good[2]={5.0f,6.0f},other[2]={7.0f,8.0f};
  /* Out-of-range value: refused, not applied, not left cached, acknowledged. */
  assert(deliver(r,sid,SEDS_DT_DAQ_FILTER_CALIBRATION,SEDS_EP_SD_CARD,"GS",bad,sizeof bad,&acked)==SEDS_OK&&acked==1);
  assert(g_calibration.filter_tau_ms[0]==0.0f&&calibration_sets==0&&g_reliable_refusal_count==1);
  assert(seds_router_cached_managed_variable_packed_len(r,SEDS_DT_DAQ_FILTER_CALIBRATION)==0);
  assert(deliver(r,sid,SEDS_DT_DAQ_FILTER_CALIBRATION,SEDS_EP_SD_CARD,"GS",good,sizeof good,&acked)==SEDS_OK&&acked==1&&g_calibration.filter_tau_ms[0]==5.0f);
  /* A failed storage write is transient: no ACK and the observation is
   * retired, so the identical retry runs the callback again. */
  fail_persist=1;assert(deliver(r,sid,SEDS_DT_DAQ_FILTER_CALIBRATION,SEDS_EP_SD_CARD,"GS",other,sizeof other,&acked)!=SEDS_OK&&acked==0&&g_calibration.filter_tau_ms[0]==5.0f);
  assert(seds_router_cached_managed_variable_packed_len(r,SEDS_DT_DAQ_FILTER_CALIBRATION)==0);
  puts("PASS refused calibration is acknowledged, not applied and not cached; storage failure keeps its retry");
 }else if(!strcmp(mode,"clock")){
  r=router(NULL,0,&sid);
  assert(seds_router_enable_network_variable(r,SEDS_DT_DAQ_LOG_CLOCK,true,false)==SEDS_OK);
  assert(seds_router_on_network_variable_update(r,SEDS_DT_DAQ_LOG_CLOCK,clock_update,r)==SEDS_OK);
  uint8_t v[16];put_u64(v,7);put_u64(v+8,0); /* session without deadline */
  assert(deliver(r,sid,SEDS_DT_DAQ_LOG_CLOCK,SEDS_EP_SD_CARD,"GS",v,16,&acked)==SEDS_OK&&acked==1&&clock_sets==0&&g_reliable_refusal_count==1);
  assert(seds_router_cached_managed_variable_packed_len(r,SEDS_DT_DAQ_LOG_CLOCK)==0);
  put_u64(v+8,1767225620000ULL);
  assert(deliver(r,sid,SEDS_DT_DAQ_LOG_CLOCK,SEDS_EP_SD_CARD,"GS",v,16,&acked)==SEDS_OK&&acked==1&&clock_sets==1);
  puts("PASS refused log clock is acknowledged, not applied and not cached");
 }else if(!strcmp(mode,"fstate")){
  r=router(NULL,0,&sid);g_cache_router=r;
  assert(seds_router_enable_network_variable(r,SEDS_DT_FLIGHT_STATE,true,false)==SEDS_OK);
  assert(seds_router_on_network_variable_update(r,SEDS_DT_FLIGHT_STATE,persist_update,NULL)==SEDS_OK);
  uint8_t state=3,bad=99;
  /* Not allowed now (stale, unready or not the station): acknowledged only. */
  assert(deliver(r,sid,SEDS_DT_FLIGHT_STATE,SEDS_EP_FLIGHT_STATE,"GS",&state,1,&acked)==SEDS_OK&&acked==1);
  assert(applies==0&&g_flight_state_cache_value==0&&!g_has_value&&ota_open==0&&g_reliable_refusal_count==1);
  assert(seds_router_cached_managed_variable_packed_len(r,SEDS_DT_FLIGHT_STATE)==0);
  managed_allowed=1;
  assert(deliver(r,sid,SEDS_DT_FLIGHT_STATE,SEDS_EP_FLIGHT_STATE,"GS",&bad,1,&acked)==SEDS_OK&&acked==1&&applies==0&&g_reliable_refusal_count==2);
  assert(deliver(r,sid,SEDS_DT_FLIGHT_STATE,SEDS_EP_FLIGHT_STATE,"GS",&state,1,&acked)==SEDS_OK&&acked==1&&applies==1&&g_flight_state_cache_value==3);
  puts("PASS refused FLIGHT_STATE is acknowledged, not applied and not cached");
 }else if(!strcmp(mode,"recovery")){
  SedsLocalEndpointDesc l={.endpoint=SEDS_EP_BOARD_RECOVERY,.packet_handler=recovery_handler};r=router(&l,1,&sid);
  uint8_t body[32]={1};uint8_t bad[32]={9};
  assert(deliver(r,sid,SEDS_DT_BOARD_RECOVERY_REQUEST,SEDS_EP_BOARD_RECOVERY,"VB",body,32,&acked)==SEDS_OK&&acked==1&&g_reliable_refusal_count==1);
  assert(deliver(r,sid,SEDS_DT_BOARD_RECOVERY_REQUEST,SEDS_EP_BOARD_RECOVERY,"GS",bad,32,&acked)==SEDS_OK&&acked==1&&g_reliable_refusal_count==2);
  puts("PASS refused recovery requests are acknowledged and never reach the recovery owner");
 }else return 2;
 seds_router_free(r);return 0;
}
