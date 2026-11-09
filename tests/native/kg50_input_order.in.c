/* Real SDK router -> actual DAQ_KG50_INPUT callback. The station owns this
 * selection and re-sends it while the DAQ reports another input, so the DAQ
 * must apply each delivered selection. The SDK drops a managed value whose
 * timestamp is older than the one cached from the same sender, and the
 * station's revision stamps are not comparable across its calibration files
 * (one per station mode) or across a station clock step. */
#include "sedsnet.h"
#include "sedsnet_config.h"
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
static struct { SedsRouter *r; } g_router;
volatile uint32_t g_reliable_refusal_count;
#define LAUNCHCORE_PERSIST_OK 0
#define DAQ_KG50_INPUT_PERSIST_KEY 2U
static int persistent_store_set(uint32_t key,const void*v,size_t n){(void)key;(void)v;(void)n;return LAUNCHCORE_PERSIST_OK;}
static bool g_persist_ready=true,g_input_network_value_seen;
static uint8_t g_kg50_input_stored;
static uint32_t g_daq_calibration_errors,g_daq_kg50_input_persist_writes;
static uint8_t requested=UINT8_MAX;static unsigned applies;
static void mcp3564r_request_kg50_input(uint8_t input){requested=input;++applies;}
/* SOURCE_FUNCTION:refuse_calibration */
/* SOURCE_FUNCTION:apply_kg50_input */

static uint64_t now(void*u){(void)u;return 1794224371000ULL;}
static unsigned acks;
static SedsResult side(const SedsPacketView*p,void*u){(void)u;if(p->sender_len>=7&&!memcmp(p->sender,"E2EACK:",7))acks++;return SEDS_OK;}
static void deliver(SedsRouter*r,int32_t sid,uint8_t input,uint64_t stamp){
 uint32_t ep=SEDS_EP_SD_CARD;
 SedsPacketView p={.ty=SEDS_DT_DAQ_KG50_INPUT,.data_size=1,.sender="GS",.sender_len=2,.endpoints=&ep,.num_endpoints=1,.timestamp=stamp,.payload=&input,.payload_len=1};
 unsigned before=acks;SedsResult res=seds_router_receive_from_side(r,(uint32_t)sid,&p);(void)seds_router_process_all_queues(r);
 printf("deliver input=%u stamp=%llu receive=%d acks=%u requested=%u\n",input,(unsigned long long)stamp,res,acks-before,requested);
 assert(res==SEDS_OK&&acks-before==1);
}
int main(int argc,char**argv){
 setbuf(stdout,NULL);assert(argc==2);assert(seds_schema_register_json_file(argv[1],strlen(argv[1]))==SEDS_OK);
 SedsRouter*r=seds_router_new(now,NULL,NULL,0);assert(r);assert(seds_router_set_sender_id(r,"DAQ",3)==SEDS_OK);g_router.r=r;
 int32_t sid=seds_router_add_side_packet(r,"link",4,side,NULL,false);assert(sid>=0);
 assert(seds_router_enable_network_variable(r,SEDS_DT_DAQ_KG50_INPUT,true,false)==SEDS_OK);
 assert(seds_router_on_network_variable_update(r,SEDS_DT_DAQ_KG50_INPUT,apply_kg50_input,r)==SEDS_OK);
 /* AMP2 from a calibration file whose revision is newer (another mode). */
 const uint64_t normal=1794153668825ULL,other=1794220000000ULL;
 deliver(r,sid,0,other);assert(requested==0&&applies==1);
 /* The operator's P6 selection carries its own file's older revision. */
 deliver(r,sid,1,normal);assert(requested==1&&applies==2&&g_kg50_input_stored==1);
 /* Each re-send of the same selection still reaches the board, idempotently. */
 deliver(r,sid,1,normal);assert(requested==1&&applies==3&&g_daq_kg50_input_persist_writes==1);
 /* After a station clock step backwards, the next selection still applies. */
 deliver(r,sid,0,normal-60000ULL);assert(requested==0&&applies==4);
 assert(g_reliable_refusal_count==0);
 puts("PASS every delivered station selection is applied regardless of its revision stamp");
 seds_router_free(r);return 0;
}
