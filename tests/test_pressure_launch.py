"""Production VB pressure ingress -> route classification -> file selection.

The real receive callback, enqueue, launch routing state, pressure writer and
route-key file selector run against inert FileX/RTOS boundaries. OX tank PT
rows captured in [T0, T+20 s) land in the single launch file; rows before and
after land in the regular session files, each exactly once.
"""
from pathlib import Path
import csv, io, subprocess, tempfile, unittest
from test_pressure_sd_handoff import function
from sd_cohort_fixture import pressure_cohort, route_epoch_support
ROOT = Path(__file__).resolve().parents[1]

FIXTURE = r'''
#include "pressure_sd_wire.h"
#include "daq_calibration.h"
#include "sd_file_limit.h"
#include "sd_calendar.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
/* Named-only SDK boundary stub; actual compact/assigned discovery and
 * collision behavior is exercised by test_pressure_sd_identity with real SDK. */
static struct { void *r; } g_router = {(void *)1};
static int seds_router_sender_matches_hostname(void *router,const char *sender,size_t n,
                                               const char *expected,size_t expected_n)
{
 assert(router == g_router.r);
 return router != NULL && sender != NULL && n == expected_n && memcmp(sender,expected,n)==0;
}
typedef uint32_t ULONG; typedef unsigned UINT; typedef unsigned char UCHAR; typedef void VOID;
typedef struct { unsigned tx_queue_enqueued; ULONG *tx_queue_read; } TX_QUEUE;
static uint32_t mask;
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t value){mask=value;}
#define TX_SUCCESS 0U
#define TX_NO_WAIT 0U
#define FX_SUCCESS 0U
#define FX_IO_ERROR 99U
#define FX_PTR_ERROR 98U
#define SD_LINE_MAX 384U
typedef enum { SD_CARD_STATUS_OK, SD_CARD_STATUS_IO_ERROR, SD_CARD_STATUS_BUSY, SD_CARD_STATUS_BACKPRESSURE } sd_card_status_t;
typedef struct { uint16_t len; daq_calibration_t calibration; uint64_t session; uint32_t route_epoch; sd_capture_t capture; char line[]; } sd_line_slot_t;
static unsigned g_sd_services_initialized=1;
static unsigned g_sd_write_error_count,g_sd_csv_rows_written_count;
static uint64_t utc=1767225600000ULL, local=100000;
static uint64_t daq_log_unix_ms(void){assert(!mask);return utc;}
static uint64_t daq_log_monotonic_ms(void){return local;}
static unsigned sd_calibration_snapshot(daq_calibration_t *value){memset(value,0,sizeof(*value));value->kg1000_slope=7;return 1;}
#include "sd_record_format.h"
@routing@
typedef struct { uint64_t fx_file_current_file_size, fx_file_current_file_offset; } FX_FILE;
@log_type@
static sd_log_t g_sd_regular_log, g_sd_launch_log;
static uint64_t g_sd_regular_closed_key, g_sd_launch_done_key;
static unsigned regular_opens, launch_opens, fail_write;
static char files[8][65536]; static unsigned file_of[2], next_file;
static UINT sd_media_flush(void){return 0;}
static UINT sd_prepare_file_space(sd_log_t *log,size_t n){(void)log;(void)n;return 0;}
static UINT fx_file_seek(FX_FILE *f,ULONG o){f->fx_file_current_file_offset=o;return 0;}
static UINT fx_file_write(FX_FILE *f,VOID *data,ULONG len){
 if(fail_write){--fail_write;return FX_IO_ERROR;}
 char *file=files[file_of[f==&g_sd_launch_log.file]];
 assert(strlen(file)+len<sizeof files[0]);strncat(file,data,len);
 f->fx_file_current_file_offset+=len;f->fx_file_current_file_size=f->fx_file_current_file_offset;return 0;
}
static UINT sd_close_file(sd_log_t *log){log->open=0;return 0;}
static int sd_format_calibration_line(char *line,size_t size,const daq_calibration_t *c){return snprintf(line,size,"# calibration,%g\r\n",(double)c->kg1000_slope);}
static UINT sd_open_log(sd_log_t *log,uint64_t key,const daq_calibration_t *calibration){
 const int launch=log==&g_sd_launch_log;
 if(launch)++launch_opens;else ++regular_opens;
 assert(next_file<8);file_of[launch]=next_file++;
 log->open=1;log->key=key;log->calibration=*calibration;log->len=0;return 0;
}
@writer@
@declarations@
static ULONG messages[8][SD_PRESSURE_MESSAGE_WORDS];static unsigned count;
static UINT tx_queue_send(TX_QUEUE*q,void*data,ULONG wait){assert(q==&g_sd_pressure_queue&&wait==0&&mask);if(count==8)return 1;memcpy(messages[count++],data,sizeof messages[0]);q->tx_queue_enqueued=count;q->tx_queue_read=messages[0];return 0;}
static UINT tx_queue_receive(TX_QUEUE*q,void*data,ULONG wait){assert(q==&g_sd_pressure_queue&&wait==0);if(!count)return 1;memcpy(data,messages[0],sizeof messages[0]);memmove(messages[0],messages[1],(--count)*sizeof messages[0]);q->tx_queue_enqueued=count;return 0;}
@sd_pressure_capture_route@
@sd_pressure_row_key@
@sd_pressure_oldest_key@
@sd_service_pressure_rows@
@sd_card_log_pressure_batch@
typedef int SedsResult;
#define SEDS_OK 0
#define SEDS_IO -14
#define SEDS_HANDLER_ERROR -7
#define SEDS_DT_VALVE_PRESSURE_SD_V1 157797238U
static unsigned g_pressure_sd_batches_received,g_pressure_sd_invalid_batches,g_reliable_refusal_count;
typedef struct {uint32_t ty;const uint8_t*payload;size_t payload_len;const char*sender;size_t sender_len;}SedsPacketView;
@pressure_sd_receive@
static uint32_t sequence=1000;
static void receive_at(uint64_t captured_utc){
 pressure_sd_batch_t b={.uptime_ms=captured_utc?captured_utc-1767225500000ULL:5,.unix_ms=captured_utc,.first_sequence=sequence,.raw={1,2,3,4},.psi={10,20,30,40}};
 sequence+=4;uint8_t wire[44];assert(pressure_sd_encode(wire,sizeof wire,&b)==44);
 SedsPacketView p={SEDS_DT_VALVE_PRESSURE_SD_V1,wire,sizeof wire,"VB",2};
 assert(pressure_sd_receive(&p,0)==SEDS_OK);memset(wire,0xa5,sizeof wire);
}
static void drain(void){while(sd_service_pressure_rows());assert(!count&&!g_sd_pressure_pending);}
static void advance(uint64_t ms){utc+=ms;local+=ms;}
int main(void){
 _Static_assert(sizeof(sd_pressure_message_t)==64,"ARM-sized message cannot grow");
 const uint64_t T0=1767225700000ULL;
 /* Boot session: regular file (key1). */
 receive_at(utc-40);drain();assert(regular_opens==1&&!launch_opens);
 /* Launch command at T-10 s with the deadline T0+20 s: the countdown stays
  * in the same regular file; nothing opens or rotates. */
 utc=T0-10000;local=200000;sd_card_set_launch_clock(42,T0+20000);
 receive_at(T0-9000);advance(1000);receive_at(T0-8000);drain();
 assert(regular_opens==1&&!launch_opens&&sd_run_snapshot()==1);
 /* T0: a batch straddling T0 splits by VB capture time. Only the window rows
  * open the single launch file. */
 utc=T0;local=210000;assert(sd_run_snapshot()==2);
 /* DAQ rows use the same capture-time classification as VB pressure. */
 mask=1;assert(sd_capture_key_locked(2,T0-1)==1&&sd_capture_key_locked(2,T0)==2&&sd_capture_key_locked(2,T0+20000)==3);mask=0;
 receive_at(T0-8);drain();assert(launch_opens==1&&regular_opens==1);
 /* Pre-T0 batch delivered late, inside the window: still the regular file. */
 receive_at(T0-1000);drain();assert(launch_opens==1&&regular_opens==1);
 /* A failed write retains its exact row; the retry writes it once. */
 advance(5000);receive_at(T0+5000);fail_write=1;
 for(unsigned i=0;i<20;++i)receive_at(0),drain(); /* unknown UTC, receive-time key2 */
 drain();
 /* T+20: a batch crossing the deadline (received just before the local
  * cutoff) opens ONE new regular session file for its post-window rows. */
 utc=T0+19990;local=229990;receive_at(T0+19992);drain();
 assert(launch_opens==1&&regular_opens==2&&sd_run_snapshot()==2);
 utc=T0+20000;local=230000;assert(sd_run_snapshot()==3&&sd_launch_finished());
 /* A late in-window batch still reaches the open launch file. */
 receive_at(T0+19980);drain();assert(launch_opens==1&&regular_opens==2);
 /* After the launch file is finished, late window rows go to the current
  * regular file instead of a second launch file. */
 assert(sd_log_finish(&g_sd_launch_log)==0);
 receive_at(T0+19960);receive_at(T0+21000);drain();
 assert(launch_opens==1&&regular_opens==2);
 /* Foreign sender is rejected before handoff; zeros remain valid VB data. */
 uint8_t wire[44];pressure_sd_batch_t z={0};pressure_sd_encode(wire,sizeof wire,&z);
 SedsPacketView p={SEDS_DT_VALVE_PRESSURE_SD_V1,wire,sizeof wire,"DAQ",3};
 assert(pressure_sd_receive(&p,0)==SEDS_HANDLER_ERROR&&!count);
 assert(sd_log_flush(&g_sd_regular_log)==0&&g_sd_write_error_count>=1);
 assert(g_sd_pressure_records_enqueued==g_sd_pressure_records_written&&!g_sd_pressure_records_dropped);
 for(unsigned i=0;i<next_file;i++)printf("=== %u\n%s",i,files[i]);
 fprintf(stderr,"PASS pressure launch routing: one launch file, crossing/late/unknown rows, retained cursor\n");
}
'''


class PressureLaunchTests(unittest.TestCase):
    def test_actual_ingress_routes_launch_window_pt_into_one_file(self):
        source = (ROOT / 'Core/Src/sd_card.c').read_text()
        telemetry = (ROOT / 'Core/Src/telemetry.c').read_text()
        routing = source[source.index('static uint64_t g_sd_session'):source.index('volatile uint32_t g_sd_line_drop_count')]
        routing = routing.replace('static uint8_t g_sd_launch_flush_pending;', '')
        code = FIXTURE.replace('@routing@', 'static uint8_t g_sd_launch_flush_pending;\n' + routing + route_epoch_support(source))
        code = code.replace('@log_type@', source[source.index('typedef struct {\n  FX_FILE file;'):source.index('} sd_log_t;') + len('} sd_log_t;')])
        code = code.replace('@writer@', source[source.index('static UINT sd_log_flush(sd_log_t *log)'):source.index('static UINT sd_marker_exists(void)')])
        declarations = source[source.index('typedef struct {\n  pressure_sd_batch_t batch;'):source.index('static TX_EVENT_FLAGS_GROUP g_sd_flags;')]
        declarations = declarations.replace('static ULONG g_sd_pressure_queue_storage[SD_PRESSURE_QUEUE_BATCHES * SD_PRESSURE_MESSAGE_WORDS];', '')
        cohort_begin = source.index('static struct { uint32_t current, old, cutoff, flags;')
        cohort_state = source[cohort_begin:source.index('static void sd_route_epoch_advance(', cohort_begin)]
        code = code.replace('@declarations@', declarations + pressure_cohort(source).replace(cohort_state, ''))
        for name in ('sd_pressure_capture_route', 'sd_pressure_row_key', 'sd_pressure_oldest_key',
                     'sd_service_pressure_rows', 'sd_card_log_pressure_batch'):
            code = code.replace('@' + name + '@', function(source, name))
        code = code.replace('@pressure_sd_receive@', function(telemetry, 'pressure_sd_receive'))
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / 'pressure-launch'
            build = subprocess.run(['cc', '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror', '-Wno-unused-variable',
                                    '-fsanitize=address,undefined', '-I', str(ROOT / 'Core/Inc'), '-x', 'c', '-',
                                    '-o', str(executable)], input=code, text=True, capture_output=True)
            self.assertEqual(build.returncode, 0, build.stderr)
            result = subprocess.run([str(executable)], text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
        files, current = [], None
        for line in result.stdout.splitlines():
            if line.startswith('=== '):
                current = []; files.append(current); continue
            if line and not line.startswith('#'):
                current.append(next(csv.reader(io.StringIO(line))))
        self.assertEqual(len(files), 3)  # regular before, launch, regular after
        T0 = 1767225700000
        before, launch, after = files
        self.assertTrue(all(r[2] == 'valve_pressure_psi' for f in files for r in f))
        network = lambda f: [int(r[0]) for r in f if r[7] == 'network']
        self.assertTrue(all(t < T0 for t in network(before)))
        self.assertTrue(launch and all(T0 <= t < T0 + 20000 for t in network(launch)))
        self.assertEqual(min(network(launch)), T0)
        self.assertEqual(max(network(launch)), T0 + 19996)
        # Late in-window rows after the launch file was finished are retained.
        self.assertEqual(sorted(t for t in network(after) if t < T0 + 20000),
                         [T0 + 19960, T0 + 19964, T0 + 19968, T0 + 19972])
        sequences = [int(r[12]) for f in files for r in f]
        self.assertEqual(len(sequences), len(set(sequences)))
        self.assertEqual(sorted(sequences), list(range(1000, 1000 + len(sequences))))
        local = [r for f in files for r in f if r[7] == 'local']
        self.assertEqual(len(local), 80)
        self.assertTrue(all(r in launch for r in local))  # unknown UTC: receive-time key


if __name__ == '__main__':
    unittest.main()
