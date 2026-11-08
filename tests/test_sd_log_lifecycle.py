"""Compile production logging lifecycle code with deterministic clocks/FileX."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from sd_cohort_fixture import route_epoch_support, pressure_cohort

ROOT = Path(__file__).resolve().parents[1]


class SdLifecycleTests(unittest.TestCase):
    def test_report_rows_drain_faster_than_two_channel_producer(self):
        source = (ROOT / "Core/Src/sd_card.c").read_text()
        start = source.index("static uint8_t sd_service_telemetry_rows")
        end = source.index("{", start) + 1
        depth = 1
        while depth:
            depth += (source[end] == "{") - (source[end] == "}")
            end += 1
        functions = source[start:end]
        self.run_c(r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
typedef uintptr_t ULONG;
typedef uint32_t sd_capture_t;
typedef struct { uint32_t route_epoch;sd_capture_t capture;uint64_t session; } sd_line_slot_t;
static void sd_card_capture_fault(sd_capture_t capture){assert(capture==0);}
static unsigned g_sd_line_pending;
static uint64_t g_sd_line_pending_key;
#define TX_NO_WAIT 0
#define TX_SUCCESS 0
#define FX_SUCCESS 0
static unsigned g_sd_queue, queued, freed, fail;
static unsigned g_sd_csv_rows_written_count, g_sd_write_error_count;
static sd_line_slot_t slot;
static unsigned tx_queue_receive(unsigned *q, ULONG *message, unsigned wait) {
  (void)q; (void)wait;
  if (!queued) return 1;
  --queued; *message=(ULONG)&slot; return 0;
}
static unsigned sd_write_telemetry_row(sd_line_slot_t *s) { (void)s; return fail; }
static void sd_free_slot(sd_line_slot_t *s) { (void)s; ++freed; }
static unsigned sd_take_line_message(ULONG *message){return tx_queue_receive(&g_sd_queue,message,TX_NO_WAIT);}
static unsigned sd_write_telemetry_item(sd_line_slot_t *s,uint16_t *i,uint8_t *done){(void)i;*done=1;return sd_write_telemetry_row(s);}
#define SD_OPTIONAL_SNAPSHOT_TAG (UINT16_MAX-1U)
static unsigned g_sd_optional_snapshots_completed;
static uint16_t sd_telemetry_tag(const sd_line_slot_t*s){(void)s;return 0;}
static uint8_t sd_optional_snapshot_guard(sd_line_slot_t*s,uint16_t i){(void)s;(void)i;assert(0);return 0;}
static void sd_release_telemetry_owner(sd_line_slot_t*s){sd_free_slot(s);}
''' + functions + r'''
int main(void) {
  for (unsigned i=0;i<300000;i++) {
    queued+=2;
    assert(sd_service_telemetry_rows());
    assert(queued==0);
  }
  assert(freed==600000 && g_sd_csv_rows_written_count==600000);
  queued=24;
  assert(sd_service_telemetry_rows() && queued==19); /* bounded five-row fairness */
  fail=1;
  assert(sd_service_telemetry_rows() && queued==18);
  assert(g_sd_write_error_count==1 && freed==600005); /* Failed row remains owned. */
  assert(sd_service_telemetry_rows() && queued==18 && freed==600005);
  fail=0;
  while (queued) assert(sd_service_telemetry_rows());
  assert(!sd_service_telemetry_rows());
  assert(g_sd_csv_rows_written_count==600024 && freed==600024);
}
''')

    def run_c(self, code):
        with tempfile.TemporaryDirectory() as tmp:
            exe = Path(tmp) / "test"
            result = subprocess.run(
                ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                 "-I", str(ROOT / "Core/Inc"), "-x", "c", "-", "-o", str(exe)],
                input=code, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(exe)], check=True)

    def test_calendar_and_close_timestamp_failure(self):
        source = (ROOT / "Core/Src/sd_card.c").read_text()
        functions = source[source.index("static void sd_update_filesystem_clock"):
                           source.index("static int sd_format_calibration_line")]
        self.run_c(r'''
#include <assert.h>
#include "sd_calendar.h"
typedef unsigned UINT;
typedef unsigned FX_FILE;
typedef struct { FX_FILE file; char name[64]; unsigned char open; } sd_log_t;
#define FX_SUCCESS 0U
static unsigned g_sd_media, g_sd_write_error_count, closes, year, day;
static uint64_t utc=1790035200000ULL;
static uint64_t daq_log_unix_ms(void) { return utc; }
static UINT fx_system_date_set(unsigned y,unsigned m,unsigned d) {
  year=y; day=d; (void)m; return 0;
}
static UINT fx_system_time_set(unsigned h,unsigned m,unsigned s) {
  (void)h; (void)m; (void)s; return 0;
}
static UINT sd_release_file_space(FX_FILE *f) {(void)f;return 0;}
static UINT fx_file_close(FX_FILE *f) { (void)f; ++closes; return 0; }
static UINT fx_file_date_time_set(unsigned *media,char *name,unsigned y,unsigned m,
                                unsigned d,unsigned h,unsigned min,unsigned sec) {
  (void)media; (void)name; (void)m; (void)h; (void)min; (void)sec;
  assert(y==2026 && d==22); return 1; /* Timestamp IO failed AFTER close. */
}
''' + functions + r'''
int main(void) {
  sd_calendar_t c;
  assert(!sd_calendar_from_unix_ms(6830679,&c));
  assert(!sd_calendar_from_unix_ms(4354819200000ULL,&c));
  assert(sd_calendar_from_unix_ms(951782400000ULL,&c));
  assert(c.year==2000 && c.month==2 && c.day==29);
  assert(sd_calendar_from_unix_ms(4107542400000ULL,&c));
  assert(c.year==2100 && c.month==3 && c.day==1);
  sd_log_t log={0,"DAQ.CSV",1};
  assert(sd_close_file(&log)==FX_SUCCESS);
  assert(closes==1 && g_sd_write_error_count==1 && year==2026 && day==22);
  assert(!log.open); /* A closed handle is never left marked open. */
}
''')

    def test_reconnects_do_not_restart_launch_and_time_loss_still_closes(self):
        source = (ROOT / "Core/Src/sd_card.c").read_text()
        lifecycle = source[source.index("static uint64_t g_sd_session"):
                           source.index("volatile uint32_t g_sd_line_drop_count")]
        # File-specific session state is exercised by test_sd_log_separation.
        lifecycle = lifecycle.replace(
            "static uint64_t g_sd_raw_session, g_sd_telemetry_session;", "").replace(
                "static uint8_t g_sd_launch_flush_pending;", "")
        self.run_c(r'''
#include <assert.h>
#include <stdint.h>
static uint64_t utc=1800000000000ULL, local=1000;
static unsigned utc_reads;
static uint32_t __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __set_PRIMASK(uint32_t value) {(void)value;}
static uint64_t daq_log_unix_ms(void) { ++utc_reads; return utc; }
static uint64_t daq_log_monotonic_ms(void) { return local; }
''' + lifecycle + route_epoch_support(source) + r'''
int main(void) {
  const uint64_t start=utc;
  for (unsigned i=0;i<10000;i++) assert(!sd_launch_finished());
  assert(utc_reads==0); /* Idle acquisition must not contend for the router. */
  assert(sd_run_snapshot()==1 && !sd_launch_key_locked()); /* Boot session file. */
  sd_card_set_launch_clock(100,start+30000); /* T-10 -> T+20 */
  /* The countdown stays in the regular session file; the window is T0..T+20. */
  assert(sd_run_snapshot()==1 && sd_launch_key_locked()==2 && !sd_launch_finished());
  assert(utc_reads==1); /* Active local deadline also avoids per-row queries. */
  local+=9999; utc+=9999; assert(sd_run_snapshot()==1);
  local+=1; utc+=1;
  sd_card_set_launch_clock(100,utc+20000); /* Pilot T0, same session. */
  assert(sd_run_snapshot()==2 && sd_launch_key_locked()==2); /* Window opens at T0. */
  /* Rows are classified by capture UTC: [T0, T+20) only, regardless of when
   * their batch is enqueued; unknown UTC follows the receive-time route. */
  assert(sd_capture_key_locked(2,start+9999)==1 && sd_capture_key_locked(2,start+10000)==2);
  assert(sd_capture_key_locked(2,start+29999)==2 && sd_capture_key_locked(2,start+30000)==3);
  assert(sd_capture_key_locked(2,0)==2);
  for (unsigned i=0;i<19;++i) {
    local+=1000; utc+=1000;
    sd_card_set_launch_clock(100,start+30000); /* Retained replay/reconnect. */
    assert(!sd_launch_finished() && sd_run_snapshot()==2);
  }
  utc=0; local+=1000; /* Lose GS/time at T+20, still close. */
  assert(sd_launch_finished() && sd_run_snapshot()==3); /* New regular session file. */
  assert(sd_launch_key_locked()==2);
  sd_card_set_launch_clock(100,start+30000);
  utc=start; assert(sd_launch_finished() && sd_run_snapshot()==3); /* No reopen if time goes backwards. */
  sd_card_set_launch_clock(0,0); assert(!sd_launch_finished());
  assert(sd_run_snapshot()==3 && !sd_launch_key_locked()); /* Reset after the window: same file. */
  sd_card_set_launch_clock(100,start+30000);
  assert(sd_run_snapshot()==3); /* Old retained launch cannot revive a reset. */
  sd_card_set_launch_clock(101,start+30000);
  assert(sd_run_snapshot()==3 && sd_launch_key_locked()==4 && !sd_launch_finished());
  local+=10000; assert(sd_run_snapshot()==4); /* Second launch window. */
  sd_card_set_launch_clock(100,start+30000);
  assert(sd_run_snapshot()==4); /* Older session replay. */
  sd_card_set_launch_clock(0,0);
  assert(sd_run_snapshot()==5); /* Abort mid-window ends the launch file once. */
  utc=start+999999;
  sd_card_set_launch_clock(102,start+30000); /* GS restarts after completed run. */
  assert(sd_run_snapshot()==5 && sd_launch_finished() && !sd_launch_key_locked());
}
''')

    def test_no_permanent_brownout_latch_or_idle_file_creation(self):
        source = (ROOT / "Core/Src/sd_card.c").read_text()
        self.assertNotIn("g_power_loss_mode", source)
        self.assertNotIn("else if (serviced_work == 0U)", source)
        initialization = source.split("UINT sd_card_init", 1)[1]
        self.assertNotIn("sd_open_timestamped_log(NULL)", initialization)
        # A superseded file closes only after every queue, including raw, has
        # moved past its route key.
        from test_sd_raw_queue import function
        oldest = function(source, "sd_oldest_queued_key")
        for queue in ("g_sd_raw_queue", "g_sd_queue", "g_sd_pressure_queue"):
            self.assertIn(queue + ".tx_queue_enqueued != 0U", oldest)


if __name__ == "__main__":
    unittest.main()
