import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class RawQueueTest(unittest.TestCase):
    def test_startup_copy_and_backpressure(self):
        source = (ROOT / 'Core/Src/sd_card.c').read_text()
        enqueue = source[source.index('sd_card_status_t sd_card_enqueue_raw_adc_samples('):
                         source.index('UINT sd_card_request_flush(void)')]
        program = r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
typedef uintptr_t ULONG;
typedef void VOID;
typedef int sd_card_status_t;
typedef unsigned UINT;
static uint32_t __get_PRIMASK(void){return 0;}
static void __disable_irq(void){}
static void __set_PRIMASK(uint32_t value){(void)value;}
enum {TX_SUCCESS=0, TX_NO_WAIT=0, SD_CARD_STATUS_OK=0,
      SD_CARD_STATUS_BUSY=1, SD_CARD_STATUS_BACKPRESSURE=2};
#define SD_RAW_BATCH_MAX 12
typedef struct {float slope;} daq_calibration_t;
typedef struct
{
  uint64_t network_unix_ms;
  uint32_t monotonic_ms;
  uint32_t record_sequence;
  int32_t raw_adc_code;
  float raw_value;
  float calibrated_value;
  float adc_temperature_c;
  int32_t adc_temperature_code;
  uint8_t channel;
} sd_raw_adc_record_t;
typedef struct {
  daq_calibration_t calibration;
  uint64_t uptime_reference;
  uint64_t session;
  sd_raw_adc_record_t samples[SD_RAW_BATCH_MAX];
  unsigned count;
} sd_raw_slot_t;
static unsigned g_sd_services_initialized, g_sd_raw_batch_drop_count;
static unsigned g_sd_raw_records_dropped_count, g_sd_raw_records_enqueued_count;
static unsigned full, closed, send_failure, freed, sent;
static int g_sd_raw_queue;
static sd_raw_slot_t slot;
static uint64_t board_uptime_ticks(void) {return 456;}
static uint64_t sd_run_snapshot(void) {return closed ? 0 : 123;}
static sd_raw_slot_t *sd_alloc_raw_slot(uint16_t count) {assert(count<=12); return full ? 0 : &slot;}
static void sd_free_raw_slot(sd_raw_slot_t *s) {assert(s==&slot); ++freed;}
static int tx_queue_send(int *q, VOID *message, int wait) {
  assert(q==&g_sd_raw_queue && wait==TX_NO_WAIT);
  assert(*(ULONG *)message==(ULONG)(uintptr_t)&slot);
  ++sent; return send_failure;
}
''' + enqueue + r'''
int main(void) {
  sd_raw_adc_record_t samples[2]={{.raw_adc_code=17,.channel=6,.raw_value=2.5f,.calibrated_value=500,.network_unix_ms=1700000000000ULL,.monotonic_ms=456,.record_sequence=21},{.raw_adc_code=23}};
  daq_calibration_t calibration={2};
  assert(sd_card_enqueue_raw_adc_samples(samples,2,&calibration)==SD_CARD_STATUS_BUSY);
  g_sd_services_initialized=1; /* Card not mounted yet: preserve first samples. */
  assert(sd_card_enqueue_raw_adc_samples(samples,2,&calibration)==SD_CARD_STATUS_OK);
  samples[0].raw_adc_code=0;samples[0].network_unix_ms=0; calibration.slope=0;
  assert(slot.count==2 && slot.samples[0].raw_adc_code==17 && slot.samples[1].raw_adc_code==23);
  assert(slot.calibration.slope==2 && slot.session==123);
  assert(slot.samples[0].channel==6 && slot.samples[0].raw_value==2.5f && slot.samples[0].calibrated_value==500);
  assert(slot.samples[0].network_unix_ms==1700000000000ULL && slot.samples[0].monotonic_ms==456 && slot.samples[0].record_sequence==21);
  assert(sd_card_enqueue_raw_adc_samples(samples,13,&calibration)==SD_CARD_STATUS_BUSY);
  full=1;
  assert(sd_card_enqueue_raw_adc_samples(samples,2,&calibration)==SD_CARD_STATUS_BACKPRESSURE);
  assert(g_sd_raw_batch_drop_count==1 && sent==1);
  full=0; send_failure=1;
  assert(sd_card_enqueue_raw_adc_samples(samples,2,&calibration)==SD_CARD_STATUS_BACKPRESSURE);
  assert(g_sd_raw_batch_drop_count==2 && freed==1);
  closed=1;
  send_failure=0;
  assert(sd_card_enqueue_raw_adc_samples(samples,2,&calibration)==SD_CARD_STATUS_OK);
  assert(slot.session==0); /* Launch closed, ordinary raw recording continues. */
  assert(sent==3);
  assert(g_sd_raw_records_enqueued_count==4 && g_sd_raw_records_dropped_count==6);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / 'raw_queue'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-x', 'c', '-', '-o', str(binary)], input=program,
                           text=True, check=True)
            subprocess.run([str(binary)], check=True)
