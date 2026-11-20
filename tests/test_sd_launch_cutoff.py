"""Execute production cutoff, queue-order and close retry paths without hardware."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from sd_cohort_fixture import route_epoch_support, pressure_cohort
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
/* The network clock is not about to step to a newer epoch here. */
static uint64_t daq_log_unix_ms_settled(void){return daq_log_unix_ms();}
static uint64_t daq_log_monotonic_ms(void){return board_uptime_extend(&uptime,raw);}
"""+body+route_epoch_support(source)+r"""
int main(void){
 const uint64_t start=utc;
 /* Route keys: odd = regular session file, even = launch-window file. */
 assert(sd_run_snapshot()==1);
 sd_card_set_launch_clock(10,start+20000); /* set at T0: the window opens now */
 assert(sd_run_snapshot()==2 && !sd_launch_finished() && sd_launch_key_locked()==2);
 assert(sd_capture_key_locked(2,start-1)==1&&sd_capture_key_locked(2,start)==2&&sd_capture_key_locked(2,start+20000)==3);
 raw+=19999;utc+=19999; // underlying32-bit millisecond clock wraps
 assert(sd_run_snapshot()==2);
 ++raw;utc=0;
 assert(sd_run_snapshot()==3 && sd_launch_finished()); // exact20s boundary, UTC lost
 utc=start-100000;sd_card_set_launch_clock(10,start+120000); // latched closure cannot reopen
 assert(sd_run_snapshot()==3 && sd_launch_finished());
 sd_card_set_launch_clock(0,0);assert(!sd_launch_finished());
 sd_card_set_launch_clock(10,start+999999);assert(sd_run_snapshot()==3); // reset replay
 utc=start+30000;sd_card_set_launch_clock(11,utc+20000);
 assert(sd_run_snapshot()==4);
 raw+=10000;utc+=10000;
 sd_card_set_launch_clock(11,utc+5000); // authoritative earlier T0 shortens
 raw+=4999;assert(sd_run_snapshot()==4);
 ++raw;assert(sd_run_snapshot()==5);
 sd_card_set_launch_clock(0,0);utc=0;
 sd_card_set_launch_clock(12,start+100000);
 assert(sd_run_snapshot()==5); // no invented UTC mapping
 utc=start+90000;assert(!sd_launch_finished() && sd_run_snapshot()==6);
 raw+=10000;utc=0;assert(sd_launch_finished() && sd_run_snapshot()==7);
 utc=start+999999;sd_card_set_launch_clock(13,start+100000);
 assert(sd_launch_finished() && sd_run_snapshot()==7); // expired restart never reopens
 /* The launch file opens at Launch acceptance (here 10 s before T0) and
  * holds the countdown through T+20 s. */
 utc=start+2000000;sd_card_set_launch_clock(14,utc+30000);
 assert(sd_run_snapshot()==8);
 raw+=29999;assert(sd_run_snapshot()==8);++raw;assert(sd_run_snapshot()==9&&sd_launch_finished());
 /* The raw-stream key floor survives replays, not a new or revised window. */
 g_sd_raw_key_floor=99;sd_card_set_launch_clock(14,start+2030000);assert(g_sd_raw_key_floor==99);
 sd_card_set_launch_clock(15,utc+60000);assert(g_sd_raw_key_floor==0);
 /* Accepted 40 s before T0: the file opens SD_LAUNCH_LEAD_MAX_MS before T0. */
 assert(sd_run_snapshot()==9);raw+=24999;assert(sd_run_snapshot()==9);
 ++raw;assert(sd_run_snapshot()==10);
 assert(!mask);
}
""")

    def test_supply_logging_follows_the_launch_window_with_a_short_lead(self):
        source=(ROOT/'Core/Src/sd_card.c').read_text()
        body=source[source.index('static uint64_t g_sd_session'):source.index('volatile uint32_t g_sd_line_drop_count')]
        body=body.replace('static uint64_t g_sd_raw_session, g_sd_telemetry_session;','').replace('static uint8_t g_sd_launch_flush_pending;','')
        self.execute(r"""
#include <assert.h>
#include <stdint.h>
#define sd_media_flush() fx_media_flush(&g_sd_media)
#include "board_uptime.h"
static uint64_t utc=1800000000000ULL;
static uint32_t raw=1000,mask;
static board_uptime_state uptime;
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t m){mask=m;}
static uint64_t daq_log_unix_ms(void){assert(!mask);return utc;}
static uint64_t daq_log_unix_ms_settled(void){return daq_log_unix_ms();}
static uint64_t daq_log_monotonic_ms(void){return board_uptime_extend(&uptime,raw);}
"""+body+route_epoch_support(source)+r"""
int main(void){
 (void)sd_launch_finished;(void)sd_capture_key_locked;
 assert(!sd_card_launch_window_active()); /* no launch session */
 /* T0 = deadline - 20 s = now + 20 s: accepted more than SD_LAUNCH_LEAD_MAX_MS
  * early, so the launch file opens at T0 - 15 s = now + 5 s. */
 sd_card_set_launch_clock(10,utc+40000);
 raw+=4499;assert(!sd_card_launch_window_active());
 raw+=1;assert(sd_card_launch_window_active()); /* SD_LAUNCH_SUPPLY_LEAD_MS before it opens */
 raw+=500;assert(sd_card_launch_window_active()&&sd_run_snapshot()==2); /* launch file open */
 raw+=34999;assert(sd_card_launch_window_active());
 raw+=1;assert(!sd_card_launch_window_active()&&sd_run_snapshot()==3); /* closed at T0+20 s */
 sd_card_set_launch_clock(0,0);assert(!sd_card_launch_window_active());
 assert(!mask);
}
""")

    def test_time_since_t0_is_known_only_inside_the_launch_window(self):
        """The chamber no-signal check (daq_input_faults.h) runs from T0 + 1.5 s
        to T0 + 5 s; T0 is the DAQ_LOG_CLOCK deadline - 20 s."""
        source=(ROOT/'Core/Src/sd_card.c').read_text()
        body=source[source.index('static uint64_t g_sd_session'):source.index('volatile uint32_t g_sd_line_drop_count')]
        body=body.replace('static uint64_t g_sd_raw_session, g_sd_telemetry_session;','').replace('static uint8_t g_sd_launch_flush_pending;','')
        self.execute(r"""
#include <assert.h>
#include <stdint.h>
#define sd_media_flush() fx_media_flush(&g_sd_media)
#include "board_uptime.h"
static uint64_t utc=1800000000000ULL;
static uint32_t raw=1000,mask;
static board_uptime_state uptime;
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t m){mask=m;}
static uint64_t daq_log_unix_ms(void){assert(!mask);return utc;}
static uint64_t daq_log_unix_ms_settled(void){return daq_log_unix_ms();}
static uint64_t daq_log_monotonic_ms(void){return board_uptime_extend(&uptime,raw);}
"""+body+route_epoch_support(source)+r"""
int main(void){
 (void)sd_launch_finished;(void)sd_capture_key_locked;(void)sd_run_snapshot;
 assert(sd_card_launch_since_t0_ms()==UINT32_MAX); /* no launch scheduled */
 sd_card_set_launch_clock(10,utc+30000);         /* T0 = now + 10 s */
 raw+=9999;assert(sd_card_launch_since_t0_ms()==UINT32_MAX);
 raw+=1;assert(sd_card_launch_since_t0_ms()==0);
 raw+=1500;assert(sd_card_launch_since_t0_ms()==1500);
 sd_card_set_launch_clock(10,utc+40000); /* T0 revised 10 s later */
 assert(sd_card_launch_since_t0_ms()==UINT32_MAX);
 raw+=10000;assert(sd_card_launch_since_t0_ms()==1500);
 raw+=18499;assert(sd_card_launch_since_t0_ms()==19999);
 raw+=1;assert(sd_card_launch_since_t0_ms()==UINT32_MAX); /* closed at T0 + 20 s */
 utc+=100000;sd_card_set_launch_clock(11,utc+20000);assert(sd_card_launch_since_t0_ms()==0);
 sd_card_set_launch_clock(0,0);assert(sd_card_launch_since_t0_ms()==UINT32_MAX); /* reset */
 assert(!mask);
}
""")

    def test_network_time_step_inside_the_window_keeps_the_monotonic_close(self):
        """A 4 h step back and a 3.8 h step forward of the network clock inside
        the launch-log window (a follower re-anchoring to a source, or a leader
        step outside Armed..Recovery) leave the close time unchanged on the
        monotonic clock; captures after the step are still routed by the
        window, and a T0 revision on the new clock moves it by its true delta."""
        source=(ROOT/'Core/Src/sd_card.c').read_text()
        body=source[source.index('static uint64_t g_sd_session'):source.index('volatile uint32_t g_sd_line_drop_count')]
        body=body.replace('static uint64_t g_sd_raw_session, g_sd_telemetry_session;','').replace('static uint8_t g_sd_launch_flush_pending;','')
        self.execute(r"""
#include <assert.h>
#include <stdint.h>
#include "pressure_sd_wire.h"
#define SD_PRESSURE_ROUTE_UNKNOWN_UTC 16U
static uint64_t base=1800000000000ULL,local=1000;static int64_t stepped;
static uint32_t mask;
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t m){mask=m;}
static uint64_t utc_now(void){return (uint64_t)((int64_t)(base+local)+stepped);}
static uint64_t daq_log_unix_ms(void){assert(!mask);return utc_now();}
static uint64_t daq_log_unix_ms_settled(void){return daq_log_unix_ms();}
static uint64_t daq_log_monotonic_ms(void){return local;}
"""+body+route_epoch_support(source)+function(source,'sd_pressure_capture_route')+r"""
static void tick(uint64_t ms){local+=ms;sd_card_note_network_time(utc_now(),local);}
static uint32_t pressure_bits(uint64_t at){pressure_sd_batch_t b={.unix_ms=at};mask=1;uint32_t r=sd_pressure_capture_route(&b,sd_launch_key_locked(),g_sd_open_frame_ms,g_sd_close_frame_ms);mask=0;return r;}
static void run(uint64_t session,int64_t step){
 sd_card_set_launch_clock(0,0);stepped=0;tick(1000);
 const uint64_t t0=utc_now(),opened=local;
 const uint64_t key=sd_run_snapshot();
 /* Launch accepted now, T0 10 s from now: the launch file opens now. */
 sd_card_set_launch_clock(session,t0+30000);
 const uint64_t close=g_sd_close_local_ms;assert(close==opened+30000);
 assert(sd_run_snapshot()==key+1);
 tick(15000);const uint64_t launch=sd_run_snapshot();assert(launch==key+1&&sd_launch_key_locked()==launch);
 /* The network clock steps inside the window. */
 stepped=step;tick(5);
 assert(g_sd_close_local_ms==close&&!sd_launch_finished()&&sd_run_snapshot()==launch);
 /* Captures on the stepped clock are routed by the window, as before it. */
 mask=1;
 assert(sd_capture_key_locked(launch,utc_now())==launch);
 assert(sd_capture_key_locked(launch,utc_now()+14994)==launch);
 assert(sd_capture_key_locked(launch,utc_now()+14995)==launch+1);
 assert(sd_capture_key_locked(launch,utc_now()-15006)==launch-1);
 assert(sd_capture_key_locked(launch,utc_now()-15005)==launch);
 mask=0;
 assert(pressure_bits(utc_now())==0xFU);
 assert(pressure_bits(utc_now()+14995)==0xF00U);
 /* A T0 revision sent on the stepped clock, 2 s later than the mapped one. */
 sd_card_set_launch_clock(session,(uint64_t)((int64_t)(t0+32000)+step));
 assert(g_sd_close_local_ms==close+2000);
 sd_card_set_launch_clock(session,(uint64_t)((int64_t)(t0+30000)+step));
 assert(g_sd_close_local_ms==close);
 /* Close exactly at the monotonic deadline. */
 local=close-1;assert(sd_run_snapshot()==launch&&!sd_launch_finished());
 local=close;assert(sd_run_snapshot()==launch+1&&sd_launch_finished());
 assert(g_sd_close_unix_ms==(uint64_t)((int64_t)(t0+30000)+step));
}
int main(void){
 run(20,-INT64_C(14400000));
 run(21,INT64_C(13680000));
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
volatile uint32_t g_telemetry_discovery_seen=1, g_reliable_refusal_count;
static int seds_router_invalidate_network_variable_cache(SedsRouter *r,unsigned ty,bool *out){(void)r;(void)ty;(void)out;return 0;}
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t m){mask=m;}
static uint64_t daq_log_unix_ms(void){assert(!mask);return utc;}
/* The network clock is not about to step to a newer epoch here. */
static uint64_t daq_log_unix_ms_settled(void){return daq_log_unix_ms();}
static uint64_t daq_log_monotonic_ms(void){return local;}
static uint32_t HAL_GetTick(void){return (uint32_t)local;}
static int seds_router_enable_network_variable(SedsRouter *r,unsigned ty,bool read,bool write){(void)r;(void)ty;(void)read;(void)write;return 0;}
static int seds_router_on_network_variable_update(SedsRouter *r,unsigned ty,SedsResult (*cb)(const SedsPacketView*,void*),void *u){(void)r;(void)ty;(void)cb;(void)u;return 0;}
static int seds_router_request_managed_variable(SedsRouter *r,unsigned ty){(void)r;(void)ty;return 0;}
"""+lifecycle+route_epoch_support(source)+receiver+r"""
static void publish(uint64_t session,uint64_t deadline,uint64_t stamp){
 uint8_t data[16];for(unsigned i=0;i<8;++i){data[i]=(uint8_t)(session>>(i*8));data[8+i]=(uint8_t)(deadline>>(i*8));}
 SedsPacketView packet={.ty=139,.payload=data,.payload_len=16,.timestamp=stamp};
 assert(clock_update(&packet,NULL)==SEDS_OK);
}
int main(void){
 SedsRouter router=1;assert(!daq_log_clock_init(&router));
 const uint64_t start=utc,revision=utc;
 publish(100,start+20000,revision);assert(sd_run_snapshot()==2&&sd_launch_key_locked()==2);
 assert(sd_capture_key_locked(2,start-1)==1&&sd_capture_key_locked(2,start+19999)==2);
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
 local=24999;assert(sd_run_snapshot()==2);
 local=25000;assert(sd_run_snapshot()==3&&sd_launch_finished());
 publish(100,start+90000,revision+3);assert(sd_run_snapshot()==3); // closed stays closed
 assert(g_sd_session==100&&g_sd_close_unix_ms==start+24000&&daq_log_clock_revision()==revision+3);
 /* New receiver receipt MUST NOT pretend the closed storage deadline changed. */
 publish(0,0,revision+4);assert(sd_run_snapshot()==3);
 publish(100,start+90000,revision+5);assert(sd_run_snapshot()==3); // tombstoned ID stays closed
 assert(g_sd_session==0&&g_sd_close_unix_ms==0&&daq_log_clock_revision()==revision+5);
 utc=start+100000;publish(101,utc+20000,revision+6);assert(sd_run_snapshot()==4);
 publish(0,0,revision+4);publish(0,0,revision+6);assert(sd_run_snapshot()==4);
 publish(0,0,revision+7);assert(sd_run_snapshot()==5); // actual current reset ends the window once
 assert(!daq_log_clock_init(&router)); // new router no longer has its old SDK cache
 utc+=100000;publish(102,utc+20000,10);assert(sd_run_snapshot()==6); // legacy boot fallback
 publish(0,0,revision+1000);assert(sd_run_snapshot()==6&&g_daq_clock_ambiguous_resets==1);
 publish(102,utc+30000,11); // newer current-context T0 correction, despite lower global boot stamp
 const uint64_t boundary=g_sd_close_local_ms;local=boundary-1;assert(sd_run_snapshot()==6);
 ++local;utc=0;assert(sd_run_snapshot()==7); // ambiguous reset never drops ordinary data or prevents cutoff
 utc=start+300000;publish(103,utc+20000,revision+2000);assert(sd_run_snapshot()==8);
 assert(!daq_log_clock_init(&router)); // router recreation preserves revision context
 publish(102,utc+40000,revision+1001);assert(sd_run_snapshot()==8);
 publish(0,0,revision+2001);assert(sd_run_snapshot()==9);
 assert(g_daq_clock_stale_updates>=4&&!mask);
}
""")

    def test_superseded_file_closes_once_after_grace_and_drain(self):
        source=(ROOT/'Core/Src/sd_card.c').read_text()
        self.execute(r"""
#include <assert.h>
#include <stdint.h>
#define FX_SUCCESS 0U
#define SD_ROUTE_GRACE_MS 1000U
typedef struct { uint8_t open; uint64_t key; } sd_log_t;
static sd_log_t g_sd_regular_log={1,1},g_sd_launch_log={1,2};
static unsigned g_sd_launch_flush_pending,g_sd_write_error_count,fail_finish,fail_media,closes,flushes,mask;
static uint64_t local=5000,route=2,g_sd_route_change_local=5000,oldest=1;
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t m){mask=m;}
static unsigned sd_launch_finished(void){assert(!mask);return 0;}
static uint64_t daq_log_monotonic_ms(void){return local;}
static uint64_t sd_route_session_locked(uint64_t l){assert(mask&&l==local);return route;}
static uint64_t sd_oldest_queued_key(void){assert(!mask);return oldest;}
static unsigned sd_log_finish(sd_log_t *log){assert(!mask);if(fail_finish){--fail_finish;return 1;}log->open=0;++closes;g_sd_launch_flush_pending=1;return 0;}
static unsigned sd_media_flush(void){++flushes;return fail_media;}
"""+function(source,'sd_close_superseded_logs')+r"""
int main(void){
 /* T0: the window opened; the pre-launch regular file stays open for grace. */
 sd_close_superseded_logs();local+=999;sd_close_superseded_logs();
 assert(g_sd_regular_log.open&&!closes&&!flushes);
 /* After grace, a queued pre-window row still holds it open. */
 local+=1;sd_close_superseded_logs();assert(g_sd_regular_log.open&&!closes);
 /* Once drained it closes exactly once; the active launch file never does. */
 oldest=2;fail_finish=1;sd_close_superseded_logs();assert(g_sd_regular_log.open&&g_sd_write_error_count==1);
 fail_media=1;sd_close_superseded_logs();assert(!g_sd_regular_log.open&&closes==1&&g_sd_launch_flush_pending);
 fail_media=0;sd_close_superseded_logs();assert(!g_sd_launch_flush_pending&&g_sd_launch_log.open);
 /* T+20: the new regular file opens on demand; the launch file waits for grace
  * and for late window rows, then closes once and never reopens here. */
 g_sd_regular_log=(sd_log_t){1,3};route=3;g_sd_route_change_local=local;
 sd_close_superseded_logs();assert(g_sd_launch_log.open);
 local+=1000;sd_close_superseded_logs();assert(g_sd_launch_log.open);
 oldest=UINT64_MAX;sd_close_superseded_logs();assert(!g_sd_launch_log.open&&closes==2);
 unsigned before=flushes;
 for(unsigned i=0;i<1000;++i)sd_close_superseded_logs();
 assert(closes==2&&flushes==before&&g_sd_regular_log.open&&g_sd_write_error_count==2&&!mask);
}
""")

    def test_independent_queues_route_each_row_by_key_without_gating(self):
        source=(ROOT/'Core/Src/sd_card.c').read_text()
        declarations=source[source.index('typedef struct {\n  pressure_sd_batch_t batch;'):source.index('static TX_EVENT_FLAGS_GROUP g_sd_flags;')]
        declarations=declarations.replace('static ULONG g_sd_pressure_queue_storage[SD_PRESSURE_QUEUE_BATCHES * SD_PRESSURE_MESSAGE_WORDS];','')
        code=r"""
#include "pressure_sd_wire.h"
#include "daq_calibration.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
typedef uintptr_t ULONG;typedef unsigned UINT;typedef unsigned char UCHAR;
typedef struct {unsigned tx_queue_enqueued;ULONG *tx_queue_read;} TX_QUEUE;
typedef struct {uint16_t len;uint32_t route_epoch;daq_calibration_t calibration;sd_capture_t capture;uint64_t session;char line[384];} sd_line_slot_t;
typedef struct {uint64_t session;} sd_raw_slot_t;
#define TX_SUCCESS 0U
#define TX_NO_WAIT 0U
#define FX_SUCCESS 0U
#define SD_LINE_MAX 384U
static TX_QUEUE g_sd_queue,g_sd_raw_queue;
static uint8_t g_sd_raw_pending;static uint64_t g_sd_raw_pending_key;
static uint32_t mask;
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t m){mask=m;}
static unsigned g_sd_write_error_count,g_sd_csv_rows_written_count,freed,fail;
static uint64_t keys[64];static unsigned writes;
static unsigned sd_calibration_snapshot(daq_calibration_t *c){*c=(daq_calibration_t){.kg1000_slope=7};return 1;}
static void sd_format_float(char *s,float f){snprintf(s,24,"%.1f",(double)f);}
static int sd_format_csv_sensor_record(char *s,size_t n,uint64_t u,uint64_t t,const char *name,const char *raw,const char *converted,uint32_t seq){
 (void)u;(void)t;(void)name;(void)raw;(void)converted;return snprintf(s,n,"p%u",seq);
}
/* The file selector is exercised by test_sd_log_separation; here only the
 * key each row carries matters, and no queue waits for another. */
static unsigned sd_write_telemetry_row(const sd_line_slot_t *row){
 assert(!mask);if(fail){fail=0;return 1;}
 assert(writes<64);keys[writes++]=row->session;return 0;
}
static void sd_free_slot(sd_line_slot_t *s){(void)s;++freed;}
"""+declarations+pressure_cohort(source)+function(source,'sd_card_capture_retain')+r"""
static ULONG csv[16],pressure[16][SD_PRESSURE_MESSAGE_WORDS];
static void push_csv(sd_line_slot_t *s){sd_card_capture_retain(s->capture);csv[g_sd_queue.tx_queue_enqueued++]=(ULONG)(uintptr_t)s;g_sd_queue.tx_queue_read=csv;}
static void push_pressure(uint32_t launch,uint32_t route,uint32_t bits,uint32_t seq){
 sd_pressure_message_t p={0};p.route_epoch=0;p.launch_key=launch;p.route_key=route;p.capture_route=bits;p.batch.first_sequence=seq;
 sd_card_capture_retain(0);
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
#define SD_OPTIONAL_SNAPSHOT_TAG (UINT16_MAX-1U)
static unsigned g_sd_optional_snapshots_completed;
static uint16_t sd_telemetry_tag(const sd_line_slot_t *s){return s->len;}
static uint8_t sd_optional_snapshot_guard(sd_line_slot_t*s,uint16_t i){(void)s;(void)i;assert(0);return 0;}
static void sd_release_telemetry_owner(sd_line_slot_t*s){sd_card_capture_release(s->capture);sd_free_slot(s);}
"""+'\n'.join(function(source,n) for n in ('sd_pressure_row_key','sd_pressure_oldest_key','sd_oldest_queued_key','sd_service_telemetry_rows','sd_service_pressure_rows'))+r"""
int main(void){
 assert(sd_oldest_queued_key()==UINT64_MAX);
 sd_line_slot_t pre={.session=1,.len=1},launch={.session=2,.len=1},post={.session=3,.len=1};
 push_csv(&launch);push_csv(&post);
 /* VB capture classification against launch key2: rows0-1 precede T0, rows2-3
  * are inside the window; then a batch crossing T+20; then receive-time keys. */
 push_pressure(2,1,SD_PRESSURE_ROUTE_CLASSIFIED|0xcU,10);
 push_pressure(2,2,SD_PRESSURE_ROUTE_CLASSIFIED|0x3U|0xc00U,14);
 push_pressure(0,3,0,18);
 push_pressure(0,1,SD_PRESSURE_ROUTE_UNKNOWN_UTC,22);
 assert(sd_oldest_queued_key()==1); /* pre-T0 pressure rows still pending */
 g_sd_raw_pending=1;g_sd_raw_pending_key=0;assert(sd_oldest_queued_key()==0);g_sd_raw_pending=0;
 /* No cross-queue gating: newer CSV keys are written immediately. */
 assert(sd_service_telemetry_rows()&&writes==2&&keys[0]==2&&keys[1]==3);
 fail=1;assert(sd_service_pressure_rows()&&writes==2&&g_sd_pressure_pending&&g_sd_write_error_count==1);
 assert(sd_oldest_queued_key()==1); /* the retained failed row keeps its key visible */
 while(sd_service_pressure_rows());
 static const uint64_t expected[]={2,3, 1,1,2,2, 2,2,3,3, 3,3,3,3, 1,1,1,1};
 assert(writes==18);for(unsigned i=0;i<18;++i)assert(keys[i]==expected[i]);
 push_csv(&pre);assert(sd_oldest_queued_key()==1);assert(sd_service_telemetry_rows()&&keys[18]==1);
 assert(sd_oldest_queued_key()==UINT64_MAX);
 assert(freed==3&&g_sd_pressure_records_written==16&&g_sd_csv_rows_written_count==19&&!mask);
}
"""
        self.execute(code)
