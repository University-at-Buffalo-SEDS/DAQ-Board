"""Execute production cutoff, queue-order and close retry paths without hardware."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from test_pressure_sd_handoff import function
ROOT = Path(__file__).resolve().parents[1]

class LaunchCutoffTests(unittest.TestCase):
    def execute(self, source):
        with tempfile.TemporaryDirectory() as directory:
            exe = Path(directory)/'cutoff'
            build = subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',
                '-fsanitize=address,undefined','-I',str(ROOT/'Core/Inc'),'-x','c','-',
                '-o',str(exe)],input=source,text=True,capture_output=True)
            self.assertEqual(build.returncode,0,build.stderr)
            subprocess.run([str(exe)],check=True)

    def test_monotonic_boundary_wrap_replay_reset_and_new_launch(self):
        source=(ROOT/'Core/Src/sd_card.c').read_text()
        body=source[source.index('static uint64_t g_sd_session'):source.index('volatile uint32_t g_sd_line_drop_count')]
        body=body.replace('static uint64_t g_sd_raw_session, g_sd_telemetry_session;','').replace('static uint8_t g_sd_launch_flush_pending;','')
        self.execute(r"""
#include <assert.h>
#include <stdint.h>
#define sd_media_flush() fx_media_flush(&g_sd_media)
#include "board_uptime.h"
static uint64_t utc=1800000000000ULL;
static uint32_t raw=UINT32_MAX-5000,mask;
static board_uptime_state uptime;
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t m){mask=m;}
static uint64_t daq_log_unix_ms(void){assert(!mask);return utc;}
static uint64_t daq_log_monotonic_ms(void){return board_uptime_extend(&uptime,raw);}
"""+body+r"""
int main(void){
 const uint64_t start=utc;
 sd_card_set_launch_clock(10,start+20000);
 assert(sd_run_snapshot()==10 && !sd_launch_finished());
 raw+=19999;utc+=19999; // underlying32-bit millisecond clock wraps
 assert(sd_run_snapshot()==10);
 ++raw;utc=0;
 assert(sd_run_snapshot()==0 && sd_launch_finished()); // exact20s boundary, UTC lost
 utc=start-100000;sd_card_set_launch_clock(10,start+120000); // latched closure cannot reopen
 assert(sd_run_snapshot()==0 && sd_launch_finished());
 sd_card_set_launch_clock(0,0);assert(!sd_launch_finished());
 sd_card_set_launch_clock(10,start+999999);assert(sd_run_snapshot()==0); // reset replay
 utc=start+30000;sd_card_set_launch_clock(11,utc+20000);
 assert(sd_run_snapshot()==11);
 raw+=10000;utc+=10000;
 sd_card_set_launch_clock(11,utc+5000); // authoritative earlier T0 shortens
 raw+=4999;assert(sd_run_snapshot()==11);
 ++raw;assert(sd_run_snapshot()==0);
 sd_card_set_launch_clock(0,0);utc=0;
 sd_card_set_launch_clock(12,start+100000);
 assert(sd_run_snapshot()==0); // no invented UTC mapping
 utc=start+90000;assert(!sd_launch_finished() && sd_run_snapshot()==12);
 raw+=10000;utc=0;assert(sd_launch_finished() && sd_run_snapshot()==0);
 utc=start+999999;sd_card_set_launch_clock(13,start+100000);
 assert(sd_launch_finished() && sd_run_snapshot()==0); // expired restart never reopens
 assert(!mask);
}
""")

    def test_packet_revisions_correct_true_t0_and_reject_frozen_tombstones(self):
        source=(ROOT/'Core/Src/sd_card.c').read_text()
        lifecycle=source[source.index('static uint64_t g_sd_session'):source.index('volatile uint32_t g_sd_line_drop_count')]
        lifecycle=lifecycle.replace('static uint64_t g_sd_raw_session, g_sd_telemetry_session;','').replace('static uint8_t g_sd_launch_flush_pending;','')
        receiver=(ROOT/'Core/Src/daq_log_clock.c').read_text()
        receiver='\n'.join(line for line in receiver.splitlines() if not line.startswith('#include'))
        self.execute(r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define sd_media_flush() fx_media_flush(&g_sd_media)
#include <stddef.h>
#include "sd_calendar.h"
typedef int SedsRouter;typedef int SedsResult;
#define SEDS_OK 0
#define SEDS_HANDLER_ERROR 1
#define SEDS_DT_DAQ_LOG_CLOCK 139U
typedef struct {unsigned ty;const uint8_t *payload;size_t payload_len;uint64_t timestamp;} SedsPacketView;
static uint64_t utc=1800000000000ULL, local=1000;
static unsigned mask;
volatile uint32_t g_telemetry_discovery_seen=1;
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t m){mask=m;}
static uint64_t daq_log_unix_ms(void){assert(!mask);return utc;}
static uint64_t daq_log_monotonic_ms(void){return local;}
static uint32_t HAL_GetTick(void){return (uint32_t)local;}
static int seds_router_enable_network_variable(SedsRouter *r,unsigned ty,bool read,bool write){(void)r;(void)ty;(void)read;(void)write;return 0;}
static int seds_router_on_network_variable_update(SedsRouter *r,unsigned ty,SedsResult (*cb)(const SedsPacketView*,void*),void *u){(void)r;(void)ty;(void)cb;(void)u;return 0;}
static int seds_router_request_managed_variable(SedsRouter *r,unsigned ty){(void)r;(void)ty;return 0;}
"""+lifecycle+receiver+r"""
static void publish(uint64_t session,uint64_t deadline,uint64_t stamp){
 uint8_t data[16];for(unsigned i=0;i<8;++i){data[i]=(uint8_t)(session>>(i*8));data[8+i]=(uint8_t)(deadline>>(i*8));}
 SedsPacketView packet={.ty=139,.payload=data,.payload_len=16,.timestamp=stamp};
 assert(clock_update(&packet,NULL)==SEDS_OK);
}
int main(void){
 SedsRouter router=1;assert(!daq_log_clock_init(&router));
 const uint64_t start=utc,revision=utc;
 publish(100,start+20000,revision);assert(sd_run_snapshot()==100);
 local+=5000;utc+=5000;
 publish(100,start+25000,revision+1); // actual Pilot/T0 five seconds later
 assert(g_sd_close_local_ms==26000);
 publish(100,start+20000,revision); // frozen old estimated launch packet
 publish(100,start+90000,revision+1); // equal-stamp conflicting replay
 assert(g_sd_close_local_ms==26000);
 utc=start-999000;publish(100,start+25000,revision+1); // retained same value, UTC rollback
 assert(g_sd_close_local_ms==26000);
 publish(100,start+24000,revision+2); // earlier authoritative correction
 assert(g_sd_close_local_ms==25000);
 local=24999;assert(sd_run_snapshot()==100);
 local=25000;assert(sd_run_snapshot()==0&&sd_launch_finished());
 publish(100,start+90000,revision+3);assert(sd_run_snapshot()==0); // closed stays closed
 assert(g_sd_session==100&&g_sd_close_unix_ms==start+24000&&daq_log_clock_revision()==revision+3);
 /* New receiver receipt MUST NOT pretend the closed storage deadline changed. */
 publish(0,0,revision+4);assert(sd_run_snapshot()==0);
 publish(100,start+90000,revision+5);assert(sd_run_snapshot()==0); // tombstoned ID stays closed
 assert(g_sd_session==0&&g_sd_close_unix_ms==0&&daq_log_clock_revision()==revision+5);
 utc=start+100000;publish(101,utc+20000,revision+6);assert(sd_run_snapshot()==101);
 publish(0,0,revision+4);publish(0,0,revision+6);assert(sd_run_snapshot()==101);
 publish(0,0,revision+7);assert(sd_run_snapshot()==0); // actual current reset
 assert(!daq_log_clock_init(&router)); // new router no longer has its old SDK cache
 utc+=100000;publish(102,utc+20000,10);assert(sd_run_snapshot()==102); // legacy boot fallback
 publish(0,0,revision+1000);assert(sd_run_snapshot()==102&&g_daq_clock_ambiguous_resets==1);
 publish(102,utc+30000,11); // newer current-context T0 correction, despite lower global boot stamp
 const uint64_t boundary=g_sd_close_local_ms;local=boundary-1;assert(sd_run_snapshot()==102);
 ++local;utc=0;assert(sd_run_snapshot()==0); // ambiguous reset never drops ordinary data or prevents cutoff
 utc=start+300000;publish(103,utc+20000,revision+2000);assert(sd_run_snapshot()==103);
 assert(!daq_log_clock_init(&router)); // router recreation preserves revision context
 publish(102,utc+40000,revision+1001);assert(sd_run_snapshot()==103);
 publish(0,0,revision+2001);assert(sd_run_snapshot()==0);
 assert(g_daq_clock_stale_updates>=4&&!mask);
}
""")

    def test_launch_close_waits_for_drain_retries_and_never_closes_ordinary(self):
        source=(ROOT/'Core/Src/sd_card.c').read_text()
        self.execute(r"""
#include <assert.h>
#include <stdint.h>
#define sd_media_flush() fx_media_flush(&g_sd_media)
#define FX_SUCCESS 0U
static unsigned g_file_open=1,g_telemetry_file_open=1,g_sd_launch_flush_pending;
static uint64_t g_sd_raw_session=10,g_sd_telemetry_session=10;
static unsigned g_sd_write_error_count,fail_write,fail_close,fail_media,closes,flushes;
static int g_sd_file,g_sd_telemetry_file,g_sd_media;
static char g_sd_filename[64],g_sd_telemetry_filename[64];
static uint64_t current_session=10;
static uint64_t sd_run_snapshot(void){return current_session;}
static unsigned sd_launch_finished(void){return current_session==0;}
static unsigned sd_flush_pending(void){return fail_write;}
static unsigned sd_flush_telemetry_pending(void){return fail_write;}
static unsigned sd_close_log(int *file,char *name){(void)file;(void)name;if(fail_close)return 1;++closes;return 0;}
static unsigned fx_media_flush(int *media){(void)media;++flushes;return fail_media;}
"""+function(source,'sd_close_finished_launch_files')+r"""
int main(void){
 sd_close_finished_launch_files(1,1);assert(!closes&&!flushes); // active launch remains open
 current_session=0; // cutoff or explicit clock reset
 sd_close_finished_launch_files(0,0);assert(!closes&&!flushes);
 fail_write=1;sd_close_finished_launch_files(1,0);assert(g_file_open&&!closes);
 fail_write=0;fail_close=1;sd_close_finished_launch_files(1,0);assert(g_file_open&&!closes);
 fail_close=0;fail_media=1;sd_close_finished_launch_files(1,0);
 assert(!g_file_open&&closes==1&&g_sd_launch_flush_pending);
 fail_media=0;sd_close_finished_launch_files(1,0);
 assert(closes==1&&!g_sd_launch_flush_pending); // directory/media retry, no reopen
 g_file_open=1;g_sd_raw_session=0; // ordinary raw stream stays open
 sd_close_finished_launch_files(1,1);
 assert(g_file_open&&!g_telemetry_file_open&&closes==2);
 unsigned before=flushes;
 for(unsigned i=0;i<1000;++i)sd_close_finished_launch_files(1,1);
 assert(closes==2&&flushes==before&&g_file_open&&g_sd_write_error_count==3);
}
""")

    def test_independent_csv_and_pressure_queues_preserve_file_generation(self):
        source=(ROOT/'Core/Src/sd_card.c').read_text()
        declarations=source[source.index('typedef struct {\n  pressure_sd_batch_t batch;'):source.index('static TX_EVENT_FLAGS_GROUP g_sd_flags;')]
        declarations=declarations.replace('static ULONG g_sd_pressure_queue_storage[SD_PRESSURE_QUEUE_BATCHES * SD_PRESSURE_MESSAGE_WORDS];','')
        code=r"""
#include "pressure_sd_wire.h"
#include <assert.h>
#include <stdint.h>
#define sd_media_flush() fx_media_flush(&g_sd_media)
#include <stdio.h>
#include <stddef.h>
#include <string.h>
typedef uintptr_t ULONG;typedef unsigned UINT;typedef unsigned char UCHAR;
typedef struct {unsigned tx_queue_enqueued;ULONG *tx_queue_read;} TX_QUEUE;
typedef struct {uint16_t len;uint32_t route_epoch;unsigned calibration;uint64_t session;char line[384];} sd_line_slot_t;
#define TX_SUCCESS 0U
#define TX_NO_WAIT 0U
#define FX_SUCCESS 0U
#define SD_LINE_MAX 384U
static TX_QUEUE g_sd_queue;
static uint32_t mask;
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t m){mask=m;}
static unsigned g_sd_write_error_count,g_sd_csv_rows_written_count,freed,fail;
static uint64_t file_session=UINT64_MAX;
static unsigned writes,opens,closes;
static unsigned sd_calibration_snapshot(unsigned *c){*c=7;return 1;}
static void sd_format_float(char *s,float f){snprintf(s,24,"%.1f",(double)f);}
static int sd_format_csv_sensor_record(char *s,size_t n,uint64_t u,uint64_t t,const char *name,const char *raw,const char *converted,uint32_t seq){
 (void)u;(void)t;(void)name;(void)raw;(void)converted;return snprintf(s,n,"p%u",seq);
}
static unsigned sd_write_telemetry_row(const sd_line_slot_t *row){
 assert(!mask);if(fail){fail=0;return 1;}
 if(file_session!=row->session){if(file_session!=UINT64_MAX)++closes;++opens;file_session=row->session;}
 ++writes;return 0;
}
static void sd_free_slot(sd_line_slot_t *s){(void)s;++freed;}
"""+declarations+r"""
static ULONG csv[16],pressure[16][SD_PRESSURE_MESSAGE_WORDS];
static void push_csv(sd_line_slot_t *s){csv[g_sd_queue.tx_queue_enqueued++]=(ULONG)(uintptr_t)s;g_sd_queue.tx_queue_read=csv;}
static void push_pressure(uint32_t epoch,uint64_t session){
 sd_pressure_message_t p={0};p.route_epoch=epoch;p.session=session;
 memcpy(pressure[g_sd_pressure_queue.tx_queue_enqueued++],&p,sizeof(p));g_sd_pressure_queue.tx_queue_read=pressure[0];
}
static UINT tx_queue_receive(TX_QUEUE *q,void *data,ULONG wait){
 assert(!wait&&!mask);if(!q->tx_queue_enqueued)return 1;
 if(q==&g_sd_queue){memcpy(data,csv,sizeof(ULONG));memmove(csv,csv+1,(q->tx_queue_enqueued-1)*sizeof(ULONG));}
 else{assert(q==&g_sd_pressure_queue);memcpy(data,pressure[0],sizeof(pressure[0]));memmove(pressure[0],pressure[1],(q->tx_queue_enqueued-1)*sizeof(pressure[0]));}
 --q->tx_queue_enqueued;return 0;
}
static unsigned sd_take_line_message(ULONG *message){return tx_queue_receive(&g_sd_queue,message,TX_NO_WAIT);}
static unsigned sd_write_telemetry_item(sd_line_slot_t *s,uint16_t *i,uint8_t *done){(void)i;*done=1;return sd_write_telemetry_row(s);}
"""+function(source,'sd_telemetry_epoch_ready')+'\n'+function(source,'sd_service_telemetry_rows')+'\n'+function(source,'sd_service_pressure_rows')+r"""
int main(void){
 sd_line_slot_t launch={.route_epoch=UINT32_MAX,.session=10,.len=1};
 sd_line_slot_t normal={.route_epoch=0,.session=0,.len=1};
 push_csv(&launch);push_csv(&normal);push_pressure(UINT32_MAX,10);push_pressure(UINT32_MAX,10);
 assert(sd_service_telemetry_rows()&&writes==1&&opens==1&&file_session==10);
 fail=1;assert(sd_service_pressure_rows()&&writes==1&&g_sd_pressure_pending);
 assert(!sd_service_telemetry_rows()); // failed older pressure retains ownership
 assert(sd_service_pressure_rows()&&writes==5);
 assert(!sd_service_telemetry_rows()); // second older batch still in FIFO
 assert(sd_service_pressure_rows()&&writes==9);
 assert(sd_service_telemetry_rows()&&writes==10&&file_session==0&&opens==2&&closes==1);
 sd_line_slot_t next={.route_epoch=2,.session=11,.len=1};
 push_csv(&next);push_pressure(1,0);
 assert(!sd_service_telemetry_rows()); // ordinary pressure precedes next launch
 assert(sd_service_pressure_rows()&&file_session==0&&writes==14);
 assert(sd_service_telemetry_rows()&&file_session==11&&writes==15&&opens==3&&closes==2);
 assert(!sd_service_pressure_rows()&&!sd_service_telemetry_rows());
 assert(freed==3&&g_sd_pressure_records_written==12&&g_sd_csv_rows_written_count==15);
 assert(!mask&&g_sd_write_error_count==1);
}
"""
        self.execute(code)
