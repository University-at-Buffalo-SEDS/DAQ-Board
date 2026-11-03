"""Production SD-only snapshot retention and independent network work ownership."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from sd_cohort_fixture import ordinary_owner_fixture
ROOT = Path(__file__).resolve().parents[1]
def production_function(source, name):
    pos = 0
    while True:
        at = source.index(name + '(', pos)
        start = source.rfind('\n', 0, at) + 1
        prefix = source[start:at].strip()
        if re.fullmatch(r'(?:static\s+)?\w+(?:\s*\*)?', prefix):
            brace = source.index('{', at)
            semi = source.find(';', at)
            if semi < 0 or semi > brace:
                break
        pos = at + len(name) + 1
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]
class OptionalRetentionTests(unittest.TestCase):
    def run_c(self, code):
        with tempfile.TemporaryDirectory() as directory:
            exe = Path(directory) / 'retention'
            build = subprocess.run(['cc','-std=c11','-O1','-Wall','-Wextra','-Werror',
                '-Wno-unused-function','-Wno-misleading-indentation','-fsanitize=address,undefined',
                '-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(exe)],
                input=code,text=True,capture_output=True)
            self.assertEqual(build.returncode,0,build.stderr)
            subprocess.run([str(exe)],check=True)
    def owner_fixture(self):
        source=(ROOT/'Core/Src/sd_card.c').read_text()
        code=ordinary_owner_fixture((ROOT/'tests/sd_optional_retention_fixture.c.in').read_text(),source)
        for name in re.findall(r'/\* SOURCE_FUNCTION:(\w+) \*/',code):
            code=code.replace('/* SOURCE_FUNCTION:'+name+' */',production_function(source,name))
        ranges={
            'TYPES':(source.index('typedef struct'),source.index('static TX_QUEUE')),
            'SENSORS':(source.index('static const char *const g_sd_scalar_sensors'),source.index('/* The receive callback')),
            'STORAGE':(source.index('static sd_optional_snapshot_t g_sd_optional_snapshots'),source.index('static FX_MEDIA')),
        }
        for marker,(start,end) in ranges.items():
            code=code.replace('/* SOURCE_'+marker+' */',source[start:end])
        self.assertNotIn('/* SOURCE_',code)
        return code
    def test_actual_retained_owners_preserve_context_and_required_headroom(self):
        self.run_c(self.owner_fixture())
    def test_actual_analog_worker_never_holds_or_republishes_network_work(self):
        source=(ROOT/'Core/Src/daq_thread.c').read_text()
        start=source.index('typedef struct');end=source.index('daq_loadcell_window_t;',start)+len('daq_loadcell_window_t;')
        window=source[start:end]
        header=(ROOT/'Core/Inc/daq_board.h').read_text()
        snapshot_type=header[header.index('typedef struct'):header.index('daq_snapshot_t;')+len('daq_snapshot_t;')]
        start=source.index('typedef struct {\n  daq_snapshot_t snapshot;');end=source.index('static TX_THREAD g_analog_thread;',start)
        work=source[start:end]
        handoff=production_function(source,'daq_store_snapshot_csv')
        worker=production_function(source,'daq_analog_thread_entry')
        code=r"""
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <setjmp.h>
"""+snapshot_type+r"""
#include "daq_calibration.h"
#define DISABLE_SD_CARD 0
#define SD_OPTIONAL_SNAPSHOT_ROWS 24U
#define SD_CARD_STATUS_OK 0U
#define TX_SUCCESS 0U
#define TX_WAIT_FOREVER 1U
typedef uintptr_t ULONG;
static unsigned g_analog_queue,g_daq_sd_snapshot_row_drop_count;
"""+window+work+r"""
static daq_analog_work_t work_items[20];
static unsigned issued,published,offered,released,retained;
static jmp_buf finished;
static unsigned tx_queue_receive(unsigned*q,void*p,unsigned wait){
 assert(q==&g_analog_queue&&wait==TX_WAIT_FOREVER);
 if(issued==20)longjmp(finished,1);
 *(ULONG*)p=(ULONG)(uintptr_t)&work_items[issued++];return TX_SUCCESS;
}
static void daq_publish_analog(const daq_snapshot_t*s,const daq_loadcell_window_t*w){
 assert(s==&work_items[published].snapshot&&w==&work_items[published].window);published++;
}
static void sd_card_capture_fault(sd_capture_t c){assert(c==0);}
static void sd_card_capture_release(sd_capture_t c){assert(c==0);}
static unsigned sd_card_enqueue_optional_snapshot_captured(const float *v,uint64_t captured,const daq_calibration_t*c,sd_capture_t capture){
 assert(capture==0);
 assert(published==offered+1 && captured==22+100*offered && c->kg1000_slope==42+offered);
 assert(v[10]==8+offered&&v[23]==3+offered);offered++;
 if(retained==3)return 2;
 retained++;return SD_CARD_STATUS_OK;
}
static unsigned tx_block_release(void*p){
 assert(p==&work_items[released]&&released+1==published&&offered==published);released++;return TX_SUCCESS;
}
"""+handoff+worker+r"""
int main(void){
 for(unsigned i=0;i<20;i++){
  work_items[i].snapshot.monotonic_ms=22+100*i;work_items[i].snapshot.analog_sample_fresh=1;
  work_items[i].snapshot.analog_inputs_v[7]=8+i;work_items[i].calibration.kg1000_slope=42+i;
  work_items[i].window.count[1]=2;work_items[i].window.voltage_sum[1]=2*(3+i);
 }
 if(!setjmp(finished))daq_analog_thread_entry(0);
 assert(published==20&&offered==20&&released==20&&retained==3);
 assert(g_daq_sd_snapshot_row_drop_count==17*24);
}
"""
        self.run_c(code)

    def test_static_pending_owner_blocks_early_close_and_newer_pressure_epoch(self):
        source=(ROOT/'Core/Src/sd_card.c').read_text()
        code=self.owner_fixture().split('int main(void){',1)[0]
        code=code.replace('unsigned head,tail,count,words;}TX_QUEUE;',
                          'unsigned head,tail,count,words,tx_queue_enqueued;ULONG *tx_queue_read;}TX_QUEUE;')
        code=code.replace('q->count++;return 0;}',
                          'q->count++;q->tx_queue_enqueued=q->count;q->tx_queue_read=q->messages[q->head];return 0;}')
        code=code.replace('q->count--;return 0;}',
                          'q->count--;q->tx_queue_enqueued=q->count;q->tx_queue_read=q->messages[q->head];return 0;}')
        predicate=production_function(source,'sd_telemetry_epoch_ready').replace(
            'sd_telemetry_epoch_ready','actual_epoch_ready',1)
        start=source.index('    sd_close_finished_launch_files(\n')
        end=source.index(');',start)+2
        close_call=source[start:end].replace('sd_close_finished_launch_files','probe_close')
        code+=predicate+r"""
static unsigned raw_drained,telemetry_drained;
static void probe_close(unsigned raw,unsigned telemetry){raw_drained=raw;telemetry_drained=telemetry;}
int main(void){
 g_sd_queue.words=g_sd_raw_queue.words=1;g_sd_pressure_queue.words=SD_PRESSURE_MESSAGE_WORDS;
 mock_pool_init(&g_sd_line_pool,line_pool_storage,62U*1024U,SD_LINE_BLOCK_PAYLOAD+SD_POOL_ALIGNMENT_RESERVE);
 mock_pool_init(&g_sd_raw_pool,raw_pool_storage,332U*1024U,SD_RAW_BLOCK_PAYLOAD+SD_POOL_ALIGNMENT_RESERVE);
 daq_calibration_t cal={.kg1000_slope=2};now=22;snapshot(22,42,0,&cal);
 assert(!actual_epoch_ready(1,1)); /* queued static prefix owns older epoch */
 assert(sd_service_telemetry_rows()&&consumed==5&&g_sd_line_pending);
 assert(!g_sd_line_slots_used&&!g_sd_queue.count);
 g_sd_pressure_pending=1;g_sd_pressure_pending_epoch=1;
 assert(!actual_epoch_ready(1,1)&&actual_epoch_ready(0,0));
 g_sd_pressure_pending=0;
"""+close_call+r"""
 assert(raw_drained&&!telemetry_drained); /* ONLY static pending prevents close */
 drain();
"""+close_call+r"""
 assert(raw_drained&&telemetry_drained);
}
"""
        self.run_c(code)
