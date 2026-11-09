
#include "sedsnet.h"
#include "sedsnet_config.h"
#include "pressure_sd_wire.h"
#include "daq_calibration.h"
#include "sd_calendar.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static struct { SedsRouter*r; } g_router;
typedef unsigned UINT;typedef uint32_t ULONG;typedef struct{unsigned tx_queue_enqueued;ULONG*tx_queue_read;}TX_QUEUE;
typedef enum{SD_CARD_STATUS_OK,SD_CARD_STATUS_IO_ERROR,SD_CARD_STATUS_BUSY,SD_CARD_STATUS_BACKPRESSURE}sd_card_status_t;
static uint32_t g_sd_pressure_records_enqueued,g_sd_pressure_records_dropped;
static uint32_t g_pressure_sd_batches_received,g_pressure_sd_invalid_batches,g_reliable_refusal_count;
static unsigned g_sd_services_initialized=1;static uint64_t g_sd_close_unix_ms=1767225620000ULL;
static uint32_t g_sd_route_epoch=3;
#define SD_LAUNCH_WINDOW_MS 20000U
static unsigned irq;static uint32_t __get_PRIMASK(void){return irq;}static void __disable_irq(void){irq=1;}static void __set_PRIMASK(uint32_t n){irq=n;}
/* Inside the launch window: route key and launch key are both2. */
static uint64_t sd_run_snapshot(void){assert(irq);return 2;}
static uint64_t sd_launch_key_locked(void){assert(irq);return 2;}
#define TX_SUCCESS 0U
#define TX_NO_WAIT 0U
typedef struct {
  pressure_sd_batch_t batch;
  uint32_t launch_key, route_key;
  uint32_t route_epoch;
  /* With SD_PRESSURE_ROUTE_CLASSIFIED, bits0-3 mark rows captured inside the
   * launch window and bits8-11 rows captured after it; other rows precede it.
   * Bit4 explicitly records an unknown capture clock; never synthesize UTC. */
  uint32_t capture_route;
} sd_pressure_message_t;
#define SD_PRESSURE_ROUTE_UNKNOWN_UTC 16U
#define SD_PRESSURE_ROUTE_CLASSIFIED 32U
#define SD_PRESSURE_MESSAGE_WORDS ((sizeof(sd_pressure_message_t) + sizeof(ULONG) - 1U) / sizeof(ULONG))
_Static_assert(SD_PRESSURE_MESSAGE_WORDS <= 16U, "ThreadX pressure message bounds");
static TX_QUEUE g_sd_pressure_queue;
/* 800 pressure samples cover3.2s of SD-card programming/flush latency. */
#define SD_PRESSURE_QUEUE_BATCHES 200U

static ULONG messages[200][SD_PRESSURE_MESSAGE_WORDS];static unsigned handoffs;
static UINT tx_queue_send(TX_QUEUE*q,void*p,ULONG wait){assert(irq&&q==&g_sd_pressure_queue&&wait==TX_NO_WAIT);if(handoffs==200)return 1;memcpy(messages[handoffs++],p,sizeof(messages[0]));return 0;}
/* SOURCE_FUNCTION:sd_pressure_capture_route */
/* SOURCE_FUNCTION:sd_cohort */
/* SOURCE_FUNCTION:sd_card_log_pressure_batch */
/* SOURCE_FUNCTION:pressure_sd_receive */
static FILE*discovery;
static SedsResult discovery_wire(const uint8_t*b,size_t n,void*u){(void)u;if(!discovery)return SEDS_OK;assert(n<=UINT32_MAX);uint32_t size=(uint32_t)n;assert(fwrite(&size,4,1,discovery)==1&&fwrite(b,1,n,discovery)==n);return SEDS_OK;}
static unsigned callbacks;static SedsResult last_result;static char last_sender[80];
static uint64_t now(void*u){(void)u;return 1767225601000ULL;}
static SedsResult observe(const SedsPacketView*p,void*u){callbacks++;snprintf(last_sender,sizeof last_sender,"%.*s",(int)p->sender_len,p->sender);last_result=pressure_sd_receive(p,u);printf("callback sender=%s result=%d handoffs=%u\n",last_sender,last_result,handoffs);return last_result;}
static unsigned acks;static SedsResult side(const SedsPacketView*p,void*u){(void)u;if(p->sender_len>=7&&!memcmp(p->sender,"E2EACK:",7))acks++;return SEDS_OK;}
typedef struct {SedsRouter *receiver;uint32_t ingress;} DiscoveryTarget;
static SedsResult learn_discovery(const SedsPacketView*p,void*u){DiscoveryTarget*t=u;return seds_router_receive_from_side(t->receiver,t->ingress,p);}
static void learn_identity(SedsRouter*receiver,uint32_t ingress,const char*name,uint32_t address){
 SedsRouter*producer=seds_router_new(now,NULL,NULL,0);assert(producer);assert(seds_router_set_sender_id(producer,name,strlen(name))==SEDS_OK);assert(seds_router_configure_address(producer,2,address)==SEDS_OK);
 DiscoveryTarget target={receiver,ingress};assert(seds_router_add_side_packet(producer,"discovery",9,learn_discovery,&target,false)>=0);bool queued=false;assert(seds_router_poll_discovery(producer,&queued)==SEDS_OK);assert(seds_router_process_all_queues(producer)==SEDS_OK);assert(seds_router_process_all_queues(receiver)==SEDS_OK);seds_router_free(producer);
}
int main(int argc,char**argv){
 setbuf(stdout,NULL);
 assert(argc==4);char discovery_path[4096];assert(snprintf(discovery_path,sizeof discovery_path,"%s.discovery",argv[3])<(int)sizeof discovery_path);assert(seds_schema_register_json_file(argv[1],strlen(argv[1]))==SEDS_OK);
 SedsLocalEndpointDesc ep={.endpoint=SEDS_EP_VALVE_PRESSURE_LOG,.packet_handler=observe};
 SedsRouter*r=seds_router_new(now,NULL,&ep,1);assert(r);assert(seds_router_set_sender_id(r,"DAQ",3)==SEDS_OK);g_router.r=r;
 int32_t sid=seds_router_add_side_packet(r,"VB-link",7,side,NULL,false);assert(sid>=0);
 pressure_sd_batch_t b={.uptime_ms=100,.unix_ms=1767225600100ULL,.first_sequence=42,.raw={0,1,2048,4095},.psi={0,1,100,2000}};
 uint8_t payload[44];assert(pressure_sd_encode(payload,sizeof payload,&b)==44);uint32_t endpoint=SEDS_EP_VALVE_PRESSURE_LOG;
 SedsPacketView p={.ty=SEDS_DT_VALVE_PRESSURE_SD_V1,.data_size=44,.sender="VB",.sender_len=2,.endpoints=&endpoint,.num_endpoints=1,.timestamp=1767225600100ULL,.payload=payload,.payload_len=sizeof payload};
 if(!strcmp(argv[2],"collision")){
  const char*a="identity-collision-51666",*bname="identity-collision-117548",*alias="@addr:2711038533";
  learn_identity(r,(uint32_t)sid,a,101);assert(seds_router_sender_matches_hostname(r,alias,strlen(alias),a,strlen(a)));
  learn_identity(r,(uint32_t)sid,bname,102);
  assert(!seds_router_sender_matches_hostname(r,alias,strlen(alias),a,strlen(a)));assert(!seds_router_sender_matches_hostname(r,alias,strlen(alias),bname,strlen(bname)));
  assert(seds_router_sender_matches_hostname(r,"@addr:101",9,a,strlen(a)));assert(seds_router_sender_matches_hostname(r,"@addr:102",9,bname,strlen(bname)));
  assert(!seds_router_sender_matches_hostname(r,"@addr:101",9,bname,strlen(bname)));
  unsigned before=acks;p.sender=alias;p.sender_len=strlen(alias);assert(seds_router_receive_from_side(r,(uint32_t)sid,&p)!=SEDS_OK);(void)seds_router_process_all_queues(r);assert(handoffs==0&&acks==before&&g_sd_pressure_records_enqueued==0);
  puts("PASS ambiguous compact collision rejected by actual CAPI and actual DAQ receiver; assigned owners remain distinct");seds_router_free(r);return 0;
 }
 if(!strcmp(argv[2],"receipt")){
  assert(seds_router_receive_from_side(r,(uint32_t)sid,&p)==SEDS_OK);assert(seds_router_process_all_queues(r)==SEDS_OK);assert(handoffs==1&&acks==1&&g_sd_pressure_records_enqueued==4);
  p.sender="RF";p.timestamp++;assert(seds_router_receive_from_side(r,(uint32_t)sid,&p)!=SEDS_OK);(void)seds_router_process_all_queues(r);assert(handoffs==1&&acks==1);
  p.sender="VB";p.timestamp++;handoffs=200;assert(seds_router_receive_from_side(r,(uint32_t)sid,&p)!=SEDS_OK && last_result==SEDS_IO);(void)seds_router_process_all_queues(r);assert(handoffs==200&&acks==1&&g_sd_pressure_records_enqueued==4);
  handoffs=1;assert(seds_router_receive_from_side(r,(uint32_t)sid,&p)==SEDS_OK);assert(seds_router_process_all_queues(r)==SEDS_OK);assert(handoffs==2&&acks==2&&g_sd_pressure_records_enqueued==8);
  puts("PASS SDK receipts only after atomic SD admission; wrong owner and full queue withhold ACK; exact packet retry succeeds after drain");seds_router_free(r);return 0;
 }
 /* Actual named callback positive, immutable atomic SD handoff. */
 assert(observe(&p,NULL)==SEDS_OK&&handoffs==1);sd_pressure_message_t*q=(void*)messages[0];assert(q->batch.first_sequence==42&&q->batch.uptime_ms==100&&q->batch.unix_ms==1767225600100ULL&&q->batch.raw[0]==0&&q->batch.psi[3]==2000&&q->launch_key==2&&q->route_key==2&&q->route_epoch==3&&q->capture_route==(SD_PRESSURE_ROUTE_CLASSIFIED|15U));
 uint8_t wire[512];int32_t n;
 if(!strncmp(argv[2],"produce",7)){seds_router_free(r);
 SedsLocalEndpointDesc vb_locals[]={{.endpoint=2001U,.packet_handler=side},{.endpoint=SEDS_EP_VALVE_BOARD,.packet_handler=side},{.endpoint=SEDS_EP_ABORT,.packet_handler=side},{.endpoint=SEDS_EP_FLIGHT_STATE,.packet_handler=side},{.endpoint=SEDS_EP_HEART_BEAT,.packet_handler=side}};
 r=seds_router_new(now,NULL,vb_locals,5);assert(r);assert(seds_router_set_sender_id(r,"VB",2)==SEDS_OK);
 discovery=fopen(discovery_path,"wb");assert(discovery);assert(seds_router_add_side_packed(r,"wire",4,discovery_wire,NULL,false)>=0);
 bool queued=false;assert(seds_router_poll_discovery(r,&queued)==SEDS_OK);assert(seds_router_process_all_queues(r)==SEDS_OK);fclose(discovery);discovery=NULL;
 char assigned_sender[24];if(!strcmp(argv[2],"produce-assigned")){uint32_t assigned=0;assert(seds_router_current_address(r,&assigned)==SEDS_OK);snprintf(assigned_sender,sizeof assigned_sender,"@addr:%u",assigned);p.sender=assigned_sender;p.sender_len=strlen(assigned_sender);printf("assigned producer=%s\n",assigned_sender);}
 n=seds_pkt_pack(&p,wire,sizeof wire);assert(n>0);FILE*f=fopen(argv[3],"wb");assert(f);assert(fwrite(wire,1,(size_t)n,f)==(size_t)n);fclose(f);seds_router_free(r);return 0;}
 if(!strcmp(argv[2],"resolved")){
  FILE*df=fopen(discovery_path,"rb");assert(df);uint32_t size;
  while(fread(&size,4,1,df)==1){assert(size<65536);uint8_t buf[65536];assert(fread(buf,1,size,df)==size);SedsResult a=seds_router_receive_packed_from_side(r,(uint32_t)sid,buf,size);printf("discovery ingress=%d bytes=%u\n",a,size);(void)seds_router_process_all_queues(r);}fclose(df);
  uint32_t address=0;SedsResult resolved=seds_router_resolve_hostname_address(r,"VB",2,&address);printf("SDKresolveVB=%d address=%u\n",resolved,address);assert(resolved==SEDS_OK);
 }
 FILE*f=fopen(argv[3],"rb");assert(f);n=(int32_t)fread(wire,1,sizeof wire,f);fclose(f);assert(n>0);

 unsigned before_acks=acks;SedsResult received=seds_router_receive_packed_from_side(r,(uint32_t)sid,wire,(size_t)n);SedsResult processed=seds_router_process_all_queues(r);
 printf("packed VB receive=%d process=%d callbacks=%u last=%d sender=%s handoffs=%u\n",received,processed,callbacks,last_result,last_sender,handoffs);
 assert(callbacks>1);if(!strcmp(argv[2],"unknown")){assert(last_result==SEDS_HANDLER_ERROR&&handoffs==1&&acks==before_acks);}else{assert(last_result==SEDS_OK&&handoffs==2&&acks==before_acks+1);}
 seds_router_free(r);return 0;
}
