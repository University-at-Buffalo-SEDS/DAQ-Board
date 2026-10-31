"""Production compact telemetry: exact CSV, retry ownership and3s card stalls."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from test_sd_raw_queue import function
ROOT=Path(__file__).resolve().parents[1]

class ScalarQueueTests(unittest.TestCase):
    def test_compact_rows_retain_payload_clocks_epochs_and_retry(self):
        source=(ROOT/'Core/Src/sd_card.c').read_text()
        types=source[source.index('typedef struct'):source.index('static TX_QUEUE')]
        sensors=source[source.index('static const char *const g_sd_scalar_sensors'):source.index('/* The receive callback')]
        code=r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "daq_calibration.h"
#include "board_uptime.h"
#include "sd_float_format.h"
#include "sd_record_format.h"
typedef uintptr_t ULONG; typedef unsigned UINT; typedef void VOID; typedef unsigned char UCHAR;
typedef int sd_card_status_t;
enum{TX_SUCCESS=0,TX_NO_WAIT=0,TX_WAIT_FOREVER=1,FX_SUCCESS=0,FX_IO_ERROR=2,
SD_CARD_STATUS_OK=0,SD_CARD_STATUS_BUSY=1,SD_CARD_STATUS_BACKPRESSURE=2};
#define SD_RAW_BATCH_MAX 16U
#define SD_LINE_MAX 384U
typedef struct{uint64_t timestamp;uint32_t words[8];} sd_raw_adc_record_t;
typedef struct{size_t tx_byte_pool_size,tx_byte_pool_available;} TX_BYTE_POOL;
typedef struct{ULONG data[1024];unsigned head,tail,count;} TX_QUEUE;
static TX_BYTE_POOL g_sd_line_pool={62U*1024U,62U*1024U};
static TX_QUEUE g_sd_queue;
static int g_sd_pool_mutex;
static unsigned irq_mask,g_sd_route_epoch,g_sd_line_slots_used,g_sd_line_slots_peak;
static unsigned g_sd_line_drop_count,g_sd_essential_telemetry_dropped;
static unsigned g_sd_scalar_records_enqueued,g_sd_scalar_records_written,g_sd_scalar_records_coalesced;
static uint64_t sd_row_sequence;
static uint64_t clock_now=4294967000ULL,session=5;
static uint32_t __get_PRIMASK(void){return irq_mask;}
static void __disable_irq(void){irq_mask=1;}
static void __set_PRIMASK(uint32_t m){irq_mask=m;}
static uint64_t sd_run_snapshot(void){assert(irq_mask);return session;}
static int tx_mutex_get(int *m,unsigned w){(void)m;(void)w;assert(!irq_mask);return 0;}
static int tx_mutex_put(int *m){(void)m;return 0;}
static unsigned force_allocation,force_send,fail_write;
typedef struct{void *base,*returned;size_t bytes;} allocation;
static allocation allocations[1024];
static unsigned tx_byte_allocate(TX_BYTE_POOL *p,void **out,size_t size,unsigned w){
 assert(!irq_mask&&!w&&p==&g_sd_line_pool);size_t charged=(size+31U)&~7U;
 if(force_allocation||charged>p->tx_byte_pool_available)return 1;
 for(unsigned i=0;i<1024;i++)if(!allocations[i].base){void *m=malloc(size+16);assert(m);*out=(char*)m+4;
 allocations[i]=(allocation){m,*out,charged};p->tx_byte_pool_available-=charged;return 0;}
 abort();
}
static unsigned tx_byte_release(void *p){assert(!irq_mask);for(unsigned i=0;i<1024;i++)if(allocations[i].returned==p){
 g_sd_line_pool.tx_byte_pool_available+=allocations[i].bytes;free(allocations[i].base);allocations[i]=(allocation){0};return 0;}
 abort();}
static unsigned tx_queue_send(TX_QUEUE *q,void *m,unsigned w){assert(irq_mask&&!w&&q==&g_sd_queue);
 if(force_send||q->count==1024)return 1;q->data[q->tail]=*(ULONG*)m;q->tail=(q->tail+1)%1024;q->count++;return 0;}
static unsigned tx_queue_receive(TX_QUEUE *q,void *m,unsigned w){assert(irq_mask&&!w&&q==&g_sd_queue);
 if(!q->count)return 1;*(ULONG*)m=q->data[q->head];q->head=(q->head+1)%1024;q->count--;return 0;}
'''+types+'\nstatic sd_scalar_slot_t *g_sd_scalar_append_tail;\n'+sensors+r'''
static char expected[20000][384];static uint64_t expected_sessions[20000];static uint32_t expected_epochs[20000];
static float expected_cal[20000];static unsigned seen[20000],row_count,rows_written;
static UINT sd_write_telemetry_row(const sd_line_slot_t *slot){
 assert(!irq_mask);if(fail_write){--fail_write;return FX_IO_ERROR;}
 unsigned i=rows_written++;assert(i<row_count);assert(!strcmp(slot->line,expected[i]));
 assert(slot->len==strlen(expected[i]));assert(slot->session==expected_sessions[i]&&slot->route_epoch==expected_epochs[i]);
 assert(slot->calibration.kg1000_slope==expected_cal[i]);assert(!seen[i]++);return FX_SUCCESS;
}
'''
        for name in ['sd_pool_allocate','sd_pool_release','sd_alloc_slot','sd_free_slot','sd_take_line_message',
                     'sd_next_row_sequence','sd_enqueue_scalar','sd_write_telemetry_item']:
            code+=function(source,name)+'\n'
        code+=r'''
static sd_line_slot_t *take(void){ULONG m;assert(!sd_take_line_message(&m));return(sd_line_slot_t*)(uintptr_t)m;}
static void enqueue(unsigned i,daq_calibration_t *cal){
 const unsigned sensor=i%5;const float edge[]={NAN,INFINITY,-INFINITY,-0.0f,FLT_MAX,FLT_MIN};
 const float value=i%11<6?edge[i%11]:(float)i/7;
 uint64_t network=i%11?1700000000000ULL+i:0;
 char text[24];sd_format_float(text,value);
 sd_format_csv_record(expected[row_count],384,network,clock_now,g_sd_scalar_sensors[sensor],text,i+1);
 expected_sessions[row_count]=session;expected_epochs[row_count]=g_sd_route_epoch;expected_cal[row_count]=cal->kg1000_slope;row_count++;
 assert(sd_enqueue_scalar(sensor,network,clock_now,value,cal)==0);
}
int main(void){
 daq_calibration_t calibration={.kg1000_slope=2};sd_line_slot_t *pending=NULL;uint16_t index=0;
 unsigned phase_credit=0,peak=0;
 /* 500 protected CSV rows/s. Producer continues throughout repeated3s card
  * stalls12s apart; available writer budget is1000rows/s. Raw queue has its
  * independent2500/s production harness with the same measured stall model. */
 for(unsigned ms=0;ms<26000;ms++){
  clock_now=4294967000ULL+ms;
  if(ms==8000){session=0;g_sd_route_epoch++;}
  if(ms==10000){calibration.kg1000_slope=3;g_sd_route_epoch++;}
  if(!(ms%2))enqueue(ms/2,&calibration);
  unsigned phase=ms%12000;
  if(phase>=2000&&phase<5000)phase_credit=0;
  else{phase_credit+=1000;while(phase_credit>=1000){phase_credit-=1000;
   if(!pending&&g_sd_queue.count){pending=take();index=0;}
   if(!pending)break;
   assert((void*)pending!=(void*)g_sd_scalar_append_tail);
   uint8_t done=0;if(ms%37==0)fail_write=1;
   UINT result=sd_write_telemetry_item(pending,&index,&done);
   if(result!=FX_SUCCESS){assert(result==FX_IO_ERROR);continue;}
   if(done){sd_free_slot(pending);pending=NULL;}
  }}
  unsigned used=g_sd_line_pool.tx_byte_pool_size-g_sd_line_pool.tx_byte_pool_available;if(used>peak)peak=used;
 }
 while(pending||g_sd_queue.count){if(!pending){pending=take();index=0;}uint8_t done;
 assert(!sd_write_telemetry_item(pending,&index,&done));if(done){sd_free_slot(pending);pending=NULL;}}
 assert(rows_written==13000&&g_sd_scalar_records_enqueued==13000&&g_sd_scalar_records_written==13000);
 assert(!g_sd_line_drop_count&&!g_sd_essential_telemetry_dropped&&g_sd_scalar_records_coalesced>0);
 assert(!g_sd_scalar_append_tail&&g_sd_line_pool.tx_byte_pool_available==g_sd_line_pool.tx_byte_pool_size);
 /* Allocation and send failures are explicit, bounded losses, never leaks. */
 force_allocation=1;assert(sd_enqueue_scalar(0,0,clock_now,1,&calibration)==SD_CARD_STATUS_BACKPRESSURE);force_allocation=0;
 force_send=1;assert(sd_enqueue_scalar(0,0,clock_now,1,&calibration)==SD_CARD_STATUS_BACKPRESSURE);force_send=0;
 assert(g_sd_essential_telemetry_dropped==2&&!g_sd_scalar_append_tail);
 assert(g_sd_line_pool.tx_byte_pool_available==g_sd_line_pool.tx_byte_pool_size);
 printf("compact scalar:13000rows saved; peak%u/%zu bytes;3s stalls; exactCSV/clocks/epochs/retries verified\n",peak,g_sd_line_pool.tx_byte_pool_size);
}
'''
        with tempfile.TemporaryDirectory() as d:
            exe=Path(d)/'scalar'
            build=subprocess.run(['cc','-std=c11','-O1','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
                '-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(exe)],input=code,text=True,capture_output=True)
            self.assertEqual(build.returncode,0,build.stderr)
            subprocess.run([str(exe)],check=True)
