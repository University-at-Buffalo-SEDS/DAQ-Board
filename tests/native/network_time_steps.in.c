/* Real SDK GS and DAQ routers -> actual DAQ station freshness/floor checks.
 * The GS leads network time; its clock steps 4 h back and 3.8 h forward. The
 * DAQ follows each step (a new time-base epoch), and the board's actual
 * board_recovery_command_is_stale() judges the delivered views: a fresh
 * request is admitted at once after either step, though its timestamp is
 * hours below the persisted floor's after a step back; a copy sent before a
 * step and delivered late is refused, as is a replay at the floor; a request
 * in a newer epoch is fresh while the DAQ is still re-anchoring. */
#include "sedsnet.h"
#include "sedsnet_config.h"
#include "board_stamp.h"
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define BOARD_RECOVERY_FRESH_MAX_AGE_MS 15000U
#define BOARD_RECOVERY_FRESH_MAX_AHEAD_MS 1000U
static SedsRouter *recovery_router;
/* Persisted recovery floor: the last Reset's (epoch, timestamp). */
static uint64_t floor_key;static uint8_t floor_on;
static uint8_t board_recovery_floor_active(void){return floor_on;}
static uint8_t board_recovery_command_time_allowed(uint64_t stamp){return !floor_on||stamp>floor_key;}
/* SOURCE_FUNCTION:board_recovery_packet_stamp */
/* SOURCE_FUNCTION:board_recovery_stamp_fresh */
/* SOURCE_FUNCTION:board_recovery_command_is_stale */

static uint64_t fake_ms=1000U;
static uint64_t now_ms(void*u){(void)u;return fake_ms;}
static uint64_t now_us(void){return fake_ms*1000U;}
static int64_t days_from_civil(int64_t y,unsigned m,unsigned d){y-=m<=2;const int64_t era=(y>=0?y:y-399)/400;const unsigned yoe=(unsigned)(y-era*400);const unsigned doy=(153*(m+(m>2?-3:9))+2)/5+d-1;const unsigned doe=yoe*365+yoe/4-yoe/100+doy;return era*146097+(int64_t)doe-719468;}
static void set_utc(SedsRouter*r,uint64_t unix_ms){
 int64_t z=(int64_t)(unix_ms/86400000U)+719468;const int64_t era=(z>=0?z:z-146096)/146097;const unsigned doe=(unsigned)(z-era*146097);
 const unsigned yoe=(doe-doe/1460+doe/36524-doe/146096)/365;const unsigned doy=doe-(365*yoe+yoe/4-yoe/100);const unsigned mp=(5*doy+2)/153;
 const unsigned d=doy-(153*mp+2)/5+1,m=mp<10?mp+3:mp-9;const int64_t y=(int64_t)yoe+era*400+(m<=2);
 const uint64_t ms=unix_ms%86400000U;
 assert(days_from_civil(y,m,d)*86400000+(int64_t)ms==(int64_t)unix_ms);
 assert(seds_router_set_local_network_datetime_millis(r,(int32_t)y,(uint8_t)m,(uint8_t)d,(uint8_t)(ms/3600000U),(uint8_t)(ms/60000U%60U),(uint8_t)(ms/1000U%60U),(uint16_t)(ms%1000U))==SEDS_OK);
}

/* In-process link: frames queue per direction; GS->DAQ frames may be held.
 * The link has no MTU, so the DAQ's discovery schema response arrives as one
 * frame: about 4.1 KiB with 58 types, growing with each added type. */
typedef struct {uint8_t bytes[8192];size_t len;} Frame;
typedef struct {Frame f[512];size_t n;} Queue;
static Queue to_daq,to_gs,held;static bool holding;
static SedsRouter *gs,*daq;static int32_t gs_side,daq_side;
static SedsResult tx_to_daq(const uint8_t*b,size_t n,void*u){(void)u;Queue*q=holding?&held:&to_daq;assert(n<=sizeof q->f[0].bytes&&q->n<512);memcpy(q->f[q->n].bytes,b,n);q->f[q->n++].len=n;return SEDS_OK;}
static SedsResult tx_to_gs(const uint8_t*b,size_t n,void*u){(void)u;assert(n<=sizeof to_gs.f[0].bytes&&to_gs.n<512);memcpy(to_gs.f[to_gs.n].bytes,b,n);to_gs.f[to_gs.n++].len=n;return SEDS_OK;}
static void drain(Queue*q,SedsRouter*r,int32_t side){
 static Queue copy;copy=*q;q->n=0;
 for(size_t i=0;i<copy.n;i++)(void)seds_router_receive_packed_from_side(r,(uint32_t)side,copy.f[i].bytes,copy.f[i].len);
}
static void pump(unsigned ms){
 for(unsigned t=0;t<ms;t+=5){
  fake_ms+=5;
  (void)seds_router_poll_timesync(gs,NULL);(void)seds_router_poll_timesync(daq,NULL);
  (void)seds_router_process_all_queues(gs);drain(&to_daq,daq,daq_side);(void)seds_router_process_all_queues(daq);
  drain(&to_gs,gs,gs_side);(void)seds_router_process_all_queues(gs);
 }
}

/* DAQ heartbeat endpoint: the board judges each delivered view. */
static unsigned seen;static bool last_admitted;static SedsPacketView last;static uint16_t epoch_at_receipt;
static SedsResult heartbeat(const SedsPacketView*p,void*u){
 (void)u;if(!p||p->sender_len!=2||memcmp(p->sender,"GS",2))return SEDS_OK;
 SedsTimeSyncStatus st;assert(seds_router_get_timesync_status(daq,&st)==SEDS_OK);epoch_at_receipt=st.epoch;
 seen++;last=*p;last_admitted=!board_recovery_command_is_stale(p);return SEDS_OK;
}
static void send_heartbeat(void){
 static const uint8_t none=0;
 assert(seds_router_log_bytes_ex(gs,SEDS_DT_HEARTBEAT,&none,0U,NULL,1)==SEDS_OK);
 (void)seds_router_process_all_queues(gs);
}
/* A fresh heartbeat from the GS, delivered at once. */
static bool fresh_request(void){unsigned before=seen;send_heartbeat();pump(20);assert(seen>before);return last_admitted;}
static void deliver_held(void){holding=false;unsigned before=seen;drain(&held,daq,daq_side);(void)seds_router_process_all_queues(daq);assert(seen>before);}
static uint16_t epoch_of(SedsRouter*r){SedsTimeSyncStatus st;assert(seds_router_get_timesync_status(r,&st)==SEDS_OK);return st.epoch;}
static uint64_t gs_now(void){uint64_t t=0;assert(seds_router_get_network_time_ms(gs,&t)==SEDS_OK);return t;}

int main(int argc,char**argv){
 setbuf(stdout,NULL);assert(argc==2);assert(seds_schema_register_json_file(argv[1],strlen(argv[1]))==SEDS_OK);
 seds_set_clock_us(now_us);
 static const SedsLocalEndpointDesc locals[]={{.endpoint=SEDS_EP_HEART_BEAT,.packet_handler=heartbeat,.packed_handler=NULL,.user=NULL}};
 gs=seds_router_new(now_ms,NULL,NULL,0);daq=seds_router_new(now_ms,NULL,locals,1);assert(gs&&daq);
 assert(seds_router_set_sender_id(gs,"GS",2)==SEDS_OK&&seds_router_set_sender_id(daq,"DAQ",3)==SEDS_OK);
 gs_side=seds_router_add_side_packed(gs,"can",3,tx_to_daq,NULL,false);daq_side=seds_router_add_side_packed(daq,"can",3,tx_to_gs,NULL,false);
 assert(gs_side>=0&&daq_side>=0);
 assert(seds_router_configure_timesync(gs,true,1U,50U,5000U,200U,200U)==SEDS_OK);
 assert(seds_router_configure_timesync(daq,true,0U,100U,5000U,200U,200U)==SEDS_OK);
 const uint64_t T0=1794183600000ULL;set_utc(gs,T0);
 recovery_router=daq;
 pump(5000);
 uint64_t daq_now=0;assert(seds_router_get_network_time_ms(daq,&daq_now)==SEDS_OK);
 printf("synced: gs %llu daq %llu epochs %u/%u\n",(unsigned long long)gs_now(),(unsigned long long)daq_now,epoch_of(gs),epoch_of(daq));
 assert(fresh_request()&&last.time_epoch==epoch_of(gs));
 /* A Reset with this request: the floor is its (epoch, timestamp). */
 floor_key=board_recovery_packet_stamp(&last);floor_on=1U;const SedsPacketView reset=last;
 assert(board_recovery_command_is_stale(&reset));
 pump(500);assert(fresh_request());

 /* --- 4 h step back. A copy sent just before it is delivered late. */
 holding=true;send_heartbeat();holding=false;
 const uint16_t before_back=epoch_of(gs);
 set_utc(gs,gs_now()-14400000ULL);
 assert(epoch_of(gs)>before_back);
 pump(1000);
 printf("after step back: gs epoch %u daq epoch %u\n",epoch_of(gs),epoch_of(daq));
 assert(epoch_of(daq)==epoch_of(gs));
 assert(fresh_request());
 assert(board_stamp_ms(board_recovery_packet_stamp(&last))<board_stamp_ms(floor_key));
 pump(16000);
 deliver_held();assert(last.time_epoch==before_back&&!last_admitted);
 /* A replay at the floor (the Reset's own stamp) never passes. */
 assert(board_recovery_command_is_stale(&reset));
 /* Reset after the step back: its stamp becomes the floor. */
 assert(fresh_request());floor_key=board_recovery_packet_stamp(&last);
 pump(500);assert(fresh_request());

 /* --- 3.8 h step forward, again with a late copy from before it. */
 holding=true;send_heartbeat();holding=false;
 const uint16_t before_forward=epoch_of(gs);
 set_utc(gs,gs_now()+13680000ULL);
 assert(epoch_of(gs)>before_forward);
 pump(1000);assert(epoch_of(daq)==epoch_of(gs));
 assert(fresh_request());
 pump(16000);deliver_held();assert(!last_admitted);
 assert(fresh_request());

 /* --- Newer epoch while the DAQ re-anchors: everything the GS sends after
  * its step is held; the heartbeat (sent last) is delivered first. */
 holding=true;const uint16_t daq_before=epoch_of(daq);
 set_utc(gs,gs_now()-14400000ULL);pump(5);send_heartbeat();
 assert(held.n>0);Frame newest=held.f[held.n-1];held.n--;holding=false;
 unsigned before=seen;
 (void)seds_router_receive_packed_from_side(daq,(uint32_t)daq_side,newest.bytes,newest.len);(void)seds_router_process_all_queues(daq);
 assert(seen==before+1);
 printf("newer epoch: view epoch %u daq epoch at receipt %u admitted %d\n",last.time_epoch,epoch_at_receipt,last_admitted);
 assert(last.time_epoch>daq_before&&epoch_at_receipt==daq_before&&last_admitted);
 held.n=0;pump(1000);assert(epoch_of(daq)==epoch_of(gs));assert(fresh_request());

 /* No floor: freshness is not required (ordering is the SDK cache's). */
 floor_on=0U;assert(!board_recovery_command_is_stale(&reset));
 puts("PASS station requests across 4 h and 3.8 h network time steps");
 seds_router_free(daq);seds_router_free(gs);return 0;
}
