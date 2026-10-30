"""Exercise production raw batching/admission with bounded ThreadX memory/queue mocks."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]

def function(source, name):
    start=source.index(name+'(');start=source.rfind('\n',0,start)+1
    brace=source.index('{',start);depth=1;end=brace+1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]

class RawQueueTest(unittest.TestCase):
    def test_coalescing_ownership_epochs_wrap_pressure_and_backpressure(self):
        source=(ROOT/'Core/Src/sd_card.c').read_text()
        types=source[source.index('typedef struct'):source.index('static TX_QUEUE')]
        record=(ROOT/'Core/Inc/sd_card.h').read_text().split('typedef struct',1)[1].split('sd_raw_adc_record_t;',1)[0]
        program=r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "daq_rates.h"
#include "board_uptime.h"
typedef uintptr_t ULONG;
typedef void VOID;
typedef unsigned UINT;
typedef int sd_card_status_t;
enum {TX_SUCCESS=0,TX_NO_WAIT=0,TX_WAIT_FOREVER=1,SD_CARD_STATUS_OK=0,
 SD_CARD_STATUS_BUSY=1,SD_CARD_STATUS_BACKPRESSURE=2};
#define SD_RAW_BATCH_MAX DAQ_RAW_BATCH_CAPACITY
typedef struct {float slope,coefficients[16];} daq_calibration_t;
typedef struct {size_t tx_byte_pool_size,tx_byte_pool_available;} TX_BYTE_POOL;
typedef struct {ULONG messages[1024];unsigned head,tail,count;} TX_QUEUE;
static unsigned irq_mask;
static uint32_t __get_PRIMASK(void){return irq_mask;}
static void __disable_irq(void){irq_mask=1;}
static void __set_PRIMASK(uint32_t mask){irq_mask=mask;}
static unsigned g_sd_services_initialized=1,g_sd_route_epoch;
static unsigned g_sd_raw_records_dropped_count,g_sd_raw_records_enqueued_count;
static unsigned g_sd_raw_essential_records_dropped,g_sd_raw_optional_records_dropped,g_sd_raw_records_coalesced;
static unsigned g_sd_raw_batch_drop_count,g_sd_raw_slots_used,g_sd_raw_slots_peak;
static TX_QUEUE g_sd_raw_queue;
static int g_sd_pool_mutex;
static TX_BYTE_POOL g_sd_raw_pool={230976,230976};
static uint64_t now=456,session=123;
uint64_t board_uptime_ticks(void){return now;}
static uint64_t sd_run_snapshot(void){assert(irq_mask);return session;}
static unsigned force_full,force_send_failure,allocations_count;
typedef struct {void *base,*returned;size_t bytes;} Allocation;
static Allocation allocations[1024];
static int tx_mutex_get(int *mutex,unsigned wait){(void)mutex;(void)wait;return 0;}
static int tx_mutex_put(int *mutex){(void)mutex;return 0;}
static int tx_byte_allocate(TX_BYTE_POOL *pool,void **out,size_t bytes,unsigned wait){
 assert(!irq_mask && pool==&g_sd_raw_pool && wait==TX_NO_WAIT);
 const size_t reserved=bytes+16;
 if(force_full||reserved>pool->tx_byte_pool_available)return 1;
 for(unsigned i=0;i<1024;i++)if(!allocations[i].base){
  void *base=malloc(bytes+16);assert(base);*out=(char*)base+4;
  allocations[i]=(Allocation){base,*out,reserved};pool->tx_byte_pool_available-=reserved;allocations_count++;return 0;
 }
 abort();
}
static int tx_byte_release(void *memory){
 assert(!irq_mask);
 for(unsigned i=0;i<1024;i++)if(allocations[i].returned==memory){
  g_sd_raw_pool.tx_byte_pool_available+=allocations[i].bytes;free(allocations[i].base);allocations[i]=(Allocation){0};return 0;
 }
 abort();
}
static int tx_queue_send(TX_QUEUE *queue,void *message,unsigned wait){
 assert(irq_mask && queue==&g_sd_raw_queue && wait==TX_NO_WAIT);
 if(force_send_failure||queue->count==1024)return 1;
 queue->messages[queue->tail]=*(ULONG*)message;queue->tail=(queue->tail+1)%1024;queue->count++;return 0;
}
static int tx_queue_receive(TX_QUEUE *queue,void *message,unsigned wait){
 assert(irq_mask && queue==&g_sd_raw_queue && wait==TX_NO_WAIT);
 if(!queue->count)return 1;
 *(ULONG*)message=queue->messages[queue->head];queue->head=(queue->head+1)%1024;queue->count--;return 0;
}
'''+'typedef struct'+record+'sd_raw_adc_record_t;\n'+types+r'''
static sd_raw_slot_t *g_sd_raw_append_tail;
'''
        for name in ['sd_pool_allocate','sd_pool_release','sd_alloc_raw_slot','sd_free_raw_slot',
                     'sd_take_raw_message','sd_raw_optional_admitted','sd_raw_is_essential',
                     'sd_raw_count_dropped','sd_card_enqueue_raw_adc_samples']:
            program+=function(source,name)+'\n'
        program+=r'''
static sd_raw_slot_t *take(void){ULONG message;assert(sd_take_raw_message(&message)==0);return(sd_raw_slot_t*)(uintptr_t)message;}
static sd_raw_adc_record_t make(uint32_t sequence,unsigned channel){
 return(sd_raw_adc_record_t){.network_unix_ms=1700000000000ULL+sequence,.monotonic_ms=(uint32_t)now,
  .record_sequence=sequence,.channel=channel,.raw_adc_code=(int32_t)sequence-50000,
  .raw_value=(float)sequence/3,.calibrated_value=(float)sequence/2,.adc_temperature_c=26.125f,.adc_temperature_code=123};
}
static void basic(void){
 daq_calibration_t calibration={.slope=2};sd_raw_adc_record_t samples[]={make(1,6),make(2,0)};
 g_sd_services_initialized=0;assert(sd_card_enqueue_raw_adc_samples(samples,2,&calibration)==SD_CARD_STATUS_BUSY);
 assert(g_sd_raw_essential_records_dropped==2);g_sd_services_initialized=1;
 assert(sd_card_enqueue_raw_adc_samples(samples,2,&calibration)==0);
 sd_raw_slot_t *tail=g_sd_raw_append_tail;assert(tail->count==2&&tail->capacity==SD_RAW_BATCH_MAX&&tail->session==123);
 now=458;samples[0]=make(3,9);samples[1]=make(4,0);
 assert(sd_card_enqueue_raw_adc_samples(samples,2,&calibration)==0&&g_sd_raw_queue.count==1);
 assert(tail->count==4&&tail->uptime_reference==458&&g_sd_raw_records_coalesced==2);
 sd_raw_slot_t *owned=take();assert(owned==tail&&!g_sd_raw_append_tail);
 now=460;samples[0]=make(5,6);samples[1]=make(6,0);
 assert(sd_card_enqueue_raw_adc_samples(samples,2,&calibration)==0&&g_sd_raw_append_tail!=owned&&owned->count==4);
 sd_free_raw_slot(owned);tail=g_sd_raw_append_tail;
 ++g_sd_route_epoch;assert(sd_card_enqueue_raw_adc_samples(samples,2,&calibration)==0&&g_sd_raw_append_tail!=tail);
 tail=g_sd_raw_append_tail;calibration.slope=3;
 assert(sd_card_enqueue_raw_adc_samples(samples,2,&calibration)==0&&g_sd_raw_append_tail!=tail);
 tail=g_sd_raw_append_tail;session=0;
 assert(sd_card_enqueue_raw_adc_samples(samples,2,&calibration)==0&&g_sd_raw_append_tail!=tail);
 while(g_sd_raw_queue.count)sd_free_raw_slot(take());
 assert(g_sd_raw_pool.tx_byte_pool_available==g_sd_raw_pool.tx_byte_pool_size);
 /* Optional input is shed while required records retain the same payload. */
 g_sd_raw_pool.tx_byte_pool_available=10000;
 samples[0]=make(20,2);samples[1]=make(21,6);
 assert(sd_card_enqueue_raw_adc_samples(samples,2,&calibration)==SD_CARD_STATUS_BACKPRESSURE);
 owned=take();assert(owned->count==1&&owned->samples[0].record_sequence==21&&g_sd_raw_optional_records_dropped==1);
 sd_free_raw_slot(owned);g_sd_raw_pool.tx_byte_pool_available=g_sd_raw_pool.tx_byte_pool_size;
 force_full=1;assert(sd_card_enqueue_raw_adc_samples(samples,2,&calibration)==SD_CARD_STATUS_BACKPRESSURE);
 assert(g_sd_raw_essential_records_dropped==3&&g_sd_raw_optional_records_dropped==2);force_full=0;
 force_send_failure=1;assert(sd_card_enqueue_raw_adc_samples(samples,2,&calibration)==SD_CARD_STATUS_BACKPRESSURE);
 assert(g_sd_raw_essential_records_dropped==4&&g_sd_raw_optional_records_dropped==3);
 assert(!g_sd_raw_slots_used&&g_sd_raw_pool.tx_byte_pool_available==g_sd_raw_pool.tx_byte_pool_size);
}
static sd_raw_adc_record_t expected[60000];static uint64_t expected_uptime[60000],expected_session[60000];
static unsigned expected_epoch[60000];static unsigned char seen[60000];static float expected_calibration[60000];
static unsigned written,essential_written;
static void verify(sd_raw_slot_t *slot,unsigned index){
 const sd_raw_adc_record_t *sample=&slot->samples[index];unsigned sequence=sample->record_sequence;
 assert(sequence<60000&&!seen[sequence]);seen[sequence]=1;
 assert(!memcmp(sample,&expected[sequence],sizeof(*sample)));
 assert(board_uptime_project(slot->uptime_reference,sample->monotonic_ms)==expected_uptime[sequence]);
 assert(slot->session==expected_session[sequence]&&slot->route_epoch==expected_epoch[sequence]);
 assert(slot->calibration.slope==expected_calibration[sequence]);written++;essential_written+=sd_raw_is_essential(sample);
}
static void stress(unsigned all_protected){
 daq_calibration_t calibration={.slope=2};sd_raw_slot_t *pending=NULL;unsigned at=0,sequence=0,credit=0;
 unsigned expected_essential=0,peak_backlog=0;
 for(unsigned ms=0;ms<20000;ms++){
  now=4294966000ULL+ms;
  if(ms==4000){session=1234;g_sd_route_epoch++;}
  if(ms==5000){session=0;g_sd_route_epoch++;}
  if(ms==8000){calibration.slope=3;g_sd_route_epoch++;}
  if(!(ms%2)){
   sd_raw_adc_record_t samples[5];const unsigned channels[]={0,6,9,2,3};
   for(unsigned i=0;i<5;i++){
    samples[i]=make(++sequence,channels[all_protected ? i%3 : i]);expected[sequence]=samples[i];expected_uptime[sequence]=now;
    expected_session[sequence]=session;expected_epoch[sequence]=g_sd_route_epoch;expected_calibration[sequence]=calibration.slope;
    expected_essential+=all_protected || i<3;
   }
   (void)sd_card_enqueue_raw_adc_samples(samples,5,&calibration);
  }
  /* Real card flush bursts exceed one second. The producer never pauses.
   * When available, a3.5krecords/s consumer drains the accepted sample FIFO. */
  const unsigned phase=ms%6000;
  if(phase>=2000&&phase<3600){credit=0;}
  else {
   credit+=3500;
   while(credit>=1000){credit-=1000;
    if(!pending&&g_sd_raw_queue.count){pending=take();at=0;}
    if(!pending)break;
    assert(pending!=g_sd_raw_append_tail);verify(pending,at++);
    if(at==pending->count){sd_free_raw_slot(pending);pending=NULL;}
   }
  }
  unsigned backlog=g_sd_raw_records_enqueued_count-written;if(backlog>peak_backlog)peak_backlog=backlog;
 }
 if(pending){for(;at<pending->count;at++)verify(pending,at);sd_free_raw_slot(pending);}
 while(g_sd_raw_queue.count){pending=take();for(unsigned i=0;i<pending->count;i++)verify(pending,i);sd_free_raw_slot(pending);}
 assert(g_sd_raw_essential_records_dropped==0&&essential_written==expected_essential);
 if(!all_protected)assert(g_sd_raw_optional_records_dropped>0);
 else assert(g_sd_raw_optional_records_dropped==0);
 assert(written+g_sd_raw_records_dropped_count==sequence&&written==g_sd_raw_records_enqueued_count);
 assert(g_sd_raw_records_coalesced>sequence/2&&g_sd_raw_slots_peak<400);
 assert(g_sd_raw_pool.tx_byte_pool_available==g_sd_raw_pool.tx_byte_pool_size&&!g_sd_raw_append_tail);
 printf("raw stress20s/2.5ksps/1.6s flush stalls: protected%u all saved, optionaldrop%u, coalesced%u, peakbacklog%u slots%u allocations%u\n",
  essential_written,g_sd_raw_optional_records_dropped,g_sd_raw_records_coalesced,peak_backlog,g_sd_raw_slots_peak,allocations_count);
}
int main(int argc,char**argv){assert(argc==2);if(!strcmp(argv[1],"basic"))basic();else stress(!strcmp(argv[1],"protected"));}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary=pathlib.Path(tmp)/'raw_queue'
            subprocess.run(['cc','-std=c11','-O1','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
                '-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(binary)],input=program,text=True,check=True)
            for mode in ('basic','stress','protected'):subprocess.run([str(binary),mode],check=True)
