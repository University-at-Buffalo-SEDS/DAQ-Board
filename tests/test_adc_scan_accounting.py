"""Settled SCAN/DRDY model around the real driver and real SD admission.

Virtual SPI has zero service time: this verifies cadence and ownership, not a
hardware CPU/SD bandwidth guarantee. ADC conversions run independently of the
DRDY interrupt and the 2 ms acquisition worker, including AZ_MUX and aux/temp.
"""
import json
import pathlib
import subprocess
import tempfile
import unittest

from test_mcp3564r_driver import HAL, HARNESS, ROOT

MODEL = r'''
#include <stdio.h>
#include <stdlib.h>
extern uint32_t board_health_adc_overruns(void);
typedef uintptr_t ULONG;
typedef void VOID;
typedef unsigned UINT;
typedef int sd_card_status_t;
#define TX_NO_WAIT 0
#define SD_CARD_STATUS_OK 0
#define SD_CARD_STATUS_BUSY 1
#define SD_CARD_STATUS_BACKPRESSURE 2
#define SD_RAW_BATCH_MAX DAQ_RAW_BATCH_CAPACITY
static void __set_PRIMASK(uint32_t n) {(void)n;}
typedef struct {float slope;} daq_calibration_t;
typedef struct {uint64_t network_unix_ms; uint32_t monotonic_ms, record_sequence;
 int32_t raw_adc_code; float raw_value,calibrated_value,adc_temperature_c;
 int32_t adc_temperature_code; uint8_t channel;} sd_raw_adc_record_t;
typedef struct {daq_calibration_t calibration; uint64_t uptime_reference,session;
 sd_raw_adc_record_t samples[SD_RAW_BATCH_MAX]; unsigned count;} sd_raw_slot_t;
static sd_raw_slot_t slots[DAQ_SD_RAW_QUEUE_DEPTH];
static unsigned head,tail,queued,slot_taken,stored,per_channel[16],last_sequence,last_raw;
static unsigned g_sd_services_initialized=1,g_sd_raw_batch_drop_count;
static unsigned g_sd_raw_records_dropped_count,g_sd_raw_records_enqueued_count;
static int g_sd_raw_queue;
static uint64_t board_uptime_ticks(void) {return tick;}
static uint64_t sd_run_snapshot(void) {return 123;}
static sd_raw_slot_t *sd_alloc_raw_slot(uint16_t n) {
 assert(n<=SD_RAW_BATCH_MAX); if(queued==DAQ_SD_RAW_QUEUE_DEPTH)return NULL;
 assert(!slot_taken);slot_taken=1;return &slots[head];
}
static void sd_free_raw_slot(sd_raw_slot_t *s) {assert(s==&slots[head]);slot_taken=0;}
static int tx_queue_send(int *q,VOID *msg,int wait) {
 assert(q==&g_sd_raw_queue && wait==TX_NO_WAIT && slot_taken);
 assert(*(ULONG*)msg==(ULONG)(uintptr_t)&slots[head]);
 head=(head+1)%DAQ_SD_RAW_QUEUE_DEPTH;queued++;slot_taken=0;return TX_SUCCESS;
}
static void write_owned_sd(void) {
 while(queued) {
  const sd_raw_slot_t *slot=&slots[tail];
  for(unsigned i=0;i<slot->count;i++) {
   const sd_raw_adc_record_t *s=&slot->samples[i];
   assert(s->record_sequence==++last_sequence);
   assert(s->network_unix_ms==1700000000000ULL+s->monotonic_ms);
   assert(s->raw_adc_code>(int32_t)last_raw && s->monotonic_ms<=tick);
   last_raw=(unsigned)s->raw_adc_code;
   per_channel[s->channel]++;stored++;
  }
  tail=(tail+1)%DAQ_SD_RAW_QUEUE_DEPTH;queued--;
 }
}
'''

SIM = r'''
static unsigned conversion_us(void) {
 const unsigned bits=(registers[2]>>2)&15U;
 const unsigned osr=32U<<bits;
 const unsigned clocks=osr<=512?3*osr:osr+1024;
 const unsigned pre=1U<<(registers[2]>>6);
 const unsigned az=(registers[3]&4U)?2:1;
 return clocks*pre*az/4; /* 16MHz MCLK /4 -> 4MHz DMCLK. */
}
int main(int argc,char **argv) {
 assert(argc==2);const unsigned stall=(unsigned)atoi(argv[1]);
 assert(mcp3564r_init(&spi)==TX_SUCCESS);
 model_us=(uint64_t)tick*1000;
 uint64_t next_adc=model_us+conversion_us();
 const uint64_t end=model_us+10000000ULL,start=model_us;
 unsigned scan=registers[7],cfg=registers[2],starts=start_commands,ch=15;
 unsigned sequence=0,drained=0,dma_raw=0,physical_overwrite=0,peak=0,temps=0,aux=0;
 daq_calibration_t calibration={2};
 for(;model_us<end;model_us++) {
  tick=(uint32_t)(model_us/1000);
  if(model_us==next_adc) {
   do {ch=ch?ch-1:15;} while(!(scan&(1U<<ch)));
   if(ready && (scan&(3U|DAQ_CHAMBER_SCAN_MASK)))physical_overwrite++;
   wire_word=(ch<<28)|(++sequence&0xfffffU);ready=1;
   if(ch==12){wire_word=(12U<<28)|305656U;temps++;}
   else if(!(scan&(3U|DAQ_CHAMBER_SCAN_MASK)))aux++;
   next_adc+=conversion_us();
   /* DRDY falls per conversion; its auto-clear interval precedes the next
    * conversion even when an old result was not read. */
   in_isr=1;mcp3564r_data_ready_callback(ADC_IRQ_Pin);in_isr=0;
  }
  if(dma_pending) {
   const unsigned rawch=wire_word>>28;
   dma_pending=0;in_isr=1;HAL_SPI_TxRxCpltCallback(&spi);in_isr=0;
   if(rawch!=12)dma_raw++;
   const unsigned pending=mcp3564r_pending_samples();if(pending>peak)peak=pending;
  }
  const uint64_t elapsed=model_us-start;
  if(elapsed%2000==0 && !(stall && elapsed>=1000000 && elapsed<1250000)) {
   sd_raw_adc_record_t batch[SD_RAW_BATCH_MAX];unsigned n=0;
   while(n<SD_RAW_BATCH_MAX && mcp3564r_pending_samples()) {
    mcp3564r_sample_t s;assert(mcp3564r_get_sample(&s)==TX_SUCCESS && s.sample_valid);
    batch[n++]=(sd_raw_adc_record_t){.network_unix_ms=1700000000000ULL+s.monotonic_ms,
      .monotonic_ms=s.monotonic_ms,.record_sequence=++drained,.raw_adc_code=s.code,.channel=s.channel};
   }
   if(n){assert(sd_card_enqueue_raw_adc_samples(batch,n,&calibration)==SD_CARD_STATUS_OK);
    memset(batch,0,sizeof(batch));} /* Writer must own immutable copies. */
   const HAL_StatusTypeDef status=mcp3564r_start_dma();
   assert(status==HAL_OK || status==HAL_BUSY);
   if(registers[7]!=scan || registers[2]!=cfg || start_commands!=starts) {
    scan=registers[7];cfg=registers[2];starts=start_commands;ch=15;ready=0;
    next_adc=model_us+conversion_us();
   }
  }
  /* A 600ms SD stall is within the existing800ms queue reserve. */
  if(elapsed%100000==0 && !(elapsed>=3000000 && elapsed<3600000))write_owned_sd();
 }
 while(mcp3564r_pending_samples()) {
  sd_raw_adc_record_t batch[SD_RAW_BATCH_MAX];unsigned n=0;
  while(n<SD_RAW_BATCH_MAX && mcp3564r_pending_samples()) {
   mcp3564r_sample_t s;assert(mcp3564r_get_sample(&s)==TX_SUCCESS && s.sample_valid);
   batch[n++]=(sd_raw_adc_record_t){.network_unix_ms=1700000000000ULL+s.monotonic_ms,
    .monotonic_ms=s.monotonic_ms,.record_sequence=++drained,.raw_adc_code=s.code,.channel=s.channel};
  }
  assert(sd_card_enqueue_raw_adc_samples(batch,n,&calibration)==SD_CARD_STATUS_OK);
 }
 write_owned_sd();
 const unsigned lost=board_health_adc_overruns();
 assert(stored==drained && stored==g_sd_raw_records_enqueued_count);
 assert(!g_sd_raw_batch_drop_count && !g_sd_raw_records_dropped_count);
 assert(dma_raw==drained+lost && temps>50 && aux>100);
 if(physical_overwrite || (stall?lost==0:lost!=0))fprintf(stderr,"overwrite%u lost%u stall%u\n",physical_overwrite,lost,stall);
 assert(!physical_overwrite && (stall?lost>0:lost==0));
 printf("{\"kg1000\":%u,\"kg50\":%u,\"pressure\":%u,\"stored\":%u,\"overruns\":%u,\"peak\":%u,\"temp\":%u,\"aux\":%u}\n",
 per_channel[0],per_channel[1],per_channel[6],stored,lost,peak,temps,aux);
 return 0;
}
'''


class ScanAccountingTests(unittest.TestCase):
    def test_fresh_density_owned_sd_and_explicit_stall_losses(self):
        enqueue = (ROOT/'Core/Src/sd_card.c').read_text().split(
            'sd_card_status_t sd_card_enqueue_raw_adc_samples(', 1)[1].split(
            'UINT sd_card_request_flush(void)', 1)[0]
        enqueue = 'sd_card_status_t sd_card_enqueue_raw_adc_samples('+enqueue
        with tempfile.TemporaryDirectory() as tmp:
            p=pathlib.Path(tmp)
            (p/'stm32u5xx_hal.h').write_text(HAL)
            (p/'tx_api.h').write_text('typedef unsigned UINT;\n#define TX_SUCCESS 0\n#define TX_PTR_ERROR 1\n#define TX_NOT_DONE 2\n')
            (p/'main.h').write_text('#include "stm32u5xx_hal.h"\n#define EN_12V_GPIO_Port ((void*)1)\n#define EN_12V_Pin 1\n#define ADC_NCS_GPIO_Port ((void*)2)\n#define ADC_NCS_Pin 2\n#define ADC_IRQ_GPIO_Port ((void*)3)\n#define ADC_IRQ_Pin 1024\n')
            h=HARNESS[:HARNESS.index('int main(int argc, char **argv) {')]
            h=h.replace('static uint32_t tick, wire_word;', 'static uint32_t tick, wire_word;\nstatic uint64_t model_us;')
            h=h.replace('ready = 0;', 'if(command==0x41) ready = 0;')
            h=h.replace('static void test_', 'static __attribute__((unused)) void test_')
            (p/'model.c').write_text(h+MODEL+enqueue+SIM)
            results={}
            for name,defs in [('legacy',['-DDAQ_CHAMBER_ADC_CHANNEL=-1']),('slow_three',['-DDAQ_ADC_OSR=512']),('pressure',[])]:
                binary=p/name
                subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror',*defs,'-I',str(p),'-I',str(ROOT/'Core/Inc'),str(ROOT/'Core/Src/mcp3564r.c'),str(p/'model.c'),'-lm','-o',str(binary)],check=True,text=True)
                for stall in (0,1):
                    result=subprocess.run([str(binary),str(stall)],check=True,text=True,stdout=subprocess.PIPE)
                    results[name,stall]=json.loads(result.stdout)
            for channel in ('kg1000','kg50'):
                self.assertLess(results['slow_three',0][channel], results['legacy',0][channel])
                self.assertGreaterEqual(results['pressure',0][channel], results['legacy',0][channel])
                self.assertGreater(results['pressure',0][channel],4000)
            self.assertGreater(results['pressure',0]['pressure'],4000)
            print('ADC settled scan accounting:', results)
