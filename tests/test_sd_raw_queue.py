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
enum {TX_SUCCESS=0, TX_NO_WAIT=0, SD_CARD_STATUS_OK=0,
      SD_CARD_STATUS_BUSY=1, SD_CARD_STATUS_BACKPRESSURE=2};
#define SD_RAW_BATCH_MAX 12
typedef struct {float slope;} daq_calibration_t;
typedef struct {int code;} sd_raw_adc_record_t;
typedef struct {
  daq_calibration_t calibration;
  uint64_t session;
  sd_raw_adc_record_t samples[SD_RAW_BATCH_MAX];
  unsigned count;
} sd_raw_slot_t;
static unsigned g_sd_services_initialized, g_sd_raw_batch_drop_count;
static unsigned full, closed, send_failure, freed, sent;
static int g_sd_raw_queue;
static sd_raw_slot_t slot;
static int sd_launch_finished(void) {return closed;}
static uint64_t sd_run_snapshot(void) {return 123;}
static sd_raw_slot_t *sd_alloc_raw_slot(uint16_t count) {assert(count<=12); return full ? 0 : &slot;}
static void sd_free_raw_slot(sd_raw_slot_t *s) {assert(s==&slot); ++freed;}
static int tx_queue_send(int *q, VOID *message, int wait) {
  assert(q==&g_sd_raw_queue && wait==TX_NO_WAIT);
  assert(*(ULONG *)message==(ULONG)(uintptr_t)&slot);
  ++sent; return send_failure;
}
''' + enqueue + r'''
int main(void) {
  sd_raw_adc_record_t samples[2]={{17},{23}};
  daq_calibration_t calibration={2};
  assert(sd_card_enqueue_raw_adc_samples(samples,2,&calibration)==SD_CARD_STATUS_BUSY);
  g_sd_services_initialized=1; /* Card not mounted yet: preserve first samples. */
  assert(sd_card_enqueue_raw_adc_samples(samples,2,&calibration)==SD_CARD_STATUS_OK);
  samples[0].code=0; calibration.slope=0;
  assert(slot.count==2 && slot.samples[0].code==17 && slot.samples[1].code==23);
  assert(slot.calibration.slope==2 && slot.session==123);
  assert(sd_card_enqueue_raw_adc_samples(samples,13,&calibration)==SD_CARD_STATUS_BUSY);
  full=1;
  assert(sd_card_enqueue_raw_adc_samples(samples,2,&calibration)==SD_CARD_STATUS_BACKPRESSURE);
  assert(g_sd_raw_batch_drop_count==1 && sent==1);
  full=0; send_failure=1;
  assert(sd_card_enqueue_raw_adc_samples(samples,2,&calibration)==SD_CARD_STATUS_BACKPRESSURE);
  assert(g_sd_raw_batch_drop_count==2 && freed==1);
  closed=1;
  assert(sd_card_enqueue_raw_adc_samples(samples,2,&calibration)==SD_CARD_STATUS_BUSY);
  assert(sent==2);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / 'raw_queue'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-x', 'c', '-', '-o', str(binary)], input=program,
                           text=True, check=True)
            subprocess.run([str(binary)], check=True)
