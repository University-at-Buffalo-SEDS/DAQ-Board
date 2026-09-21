"""Compile production logging lifecycle code with deterministic clocks/FileX."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class SdLifecycleTests(unittest.TestCase):
    def test_report_rows_drain_faster_than_two_channel_producer(self):
        source = (ROOT / "Core/Src/sd_card.c").read_text()
        functions = source[source.index("static uint8_t sd_service_telemetry_rows"):
                           source.index("void sd_card_writer_thread_entry")]
        self.run_c(r'''
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
typedef uintptr_t ULONG;
typedef unsigned sd_line_slot_t;
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
''' + functions + r'''
int main(void) {
  for (unsigned i=0;i<300000;i++) {
    queued+=2;
    assert(sd_service_telemetry_rows());
    assert(queued==0);
  }
  assert(freed==600000 && g_sd_csv_rows_written_count==600000);
  queued=24;
  assert(sd_service_telemetry_rows() && queued==20); /* bounded fairness */
  fail=1;
  assert(sd_service_telemetry_rows() && queued==16);
  assert(g_sd_write_error_count==4 && freed==600008);
  queued=0;
  assert(!sd_service_telemetry_rows());
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
                           source.index("static UINT sd_open_log")]
        self.run_c(r'''
#include <assert.h>
#include "sd_calendar.h"
typedef unsigned UINT;
typedef unsigned FX_FILE;
#define FX_SUCCESS 0U
static unsigned g_sd_media, g_sd_write_error_count, closes, year, day;
static uint64_t utc=1790035200000ULL;
static uint64_t telemetry_unix_ms(void) { return utc; }
static UINT fx_system_date_set(unsigned y,unsigned m,unsigned d) {
  year=y; day=d; (void)m; return 0;
}
static UINT fx_system_time_set(unsigned h,unsigned m,unsigned s) {
  (void)h; (void)m; (void)s; return 0;
}
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
  FX_FILE f=0;
  assert(sd_close_log(&f,"DAQ.CSV")==FX_SUCCESS);
  assert(closes==1 && g_sd_write_error_count==1 && year==2026 && day==22);
}
''')

    def test_reconnects_do_not_restart_launch_and_time_loss_still_closes(self):
        source = (ROOT / "Core/Src/sd_card.c").read_text()
        lifecycle = source[source.index("static uint64_t g_sd_session"):
                           source.index("volatile uint32_t g_sd_line_drop_count")]
        # File-specific session state is exercised by test_sd_log_separation.
        lifecycle = lifecycle.replace(
            "static uint64_t g_sd_raw_session, g_sd_telemetry_session;", "")
        self.run_c(r'''
#include <assert.h>
#include <stdint.h>
static uint64_t utc=1800000000000ULL, local=1000;
static unsigned utc_reads;
static uint32_t __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __enable_irq(void) {}
static uint64_t telemetry_unix_ms(void) { ++utc_reads; return utc; }
static uint64_t telemetry_now_ms(void) { return local; }
''' + lifecycle + r'''
int main(void) {
  const uint64_t start=utc;
  for (unsigned i=0;i<10000;i++) assert(!sd_launch_finished());
  assert(utc_reads==0); /* Idle acquisition must not contend for the router. */
  sd_card_set_launch_clock(100,start+130000); /* T-10 -> T+120 */
  assert(sd_run_snapshot()==100 && !sd_launch_finished());
  assert(utc_reads==1); /* Active local deadline also avoids per-row queries. */
  local+=10000; utc+=10000;
  sd_card_set_launch_clock(100,utc+120000); /* Pilot T0, same session. */
  for (unsigned i=0;i<119;++i) {
    local+=1000; utc+=1000;
    sd_card_set_launch_clock(100,start+130000); /* Retained replay/reconnect. */
    assert(!sd_launch_finished() && sd_run_snapshot()==100);
  }
  utc=0; local+=1000; /* Lose GS/time at T+120, still close. */
  assert(sd_launch_finished());
  sd_card_set_launch_clock(100,start+130000);
  utc=start; assert(sd_launch_finished()); /* No reopen if time goes backwards. */
  sd_card_set_launch_clock(0,0); assert(!sd_launch_finished());
  sd_card_set_launch_clock(100,start+130000);
  assert(sd_run_snapshot()==0); /* Old retained launch cannot revive a reset. */
  sd_card_set_launch_clock(101,start+130000);
  assert(sd_run_snapshot()==101 && !sd_launch_finished());
  sd_card_set_launch_clock(100,start+130000);
  assert(sd_run_snapshot()==101); /* Older session replay. */
  sd_card_set_launch_clock(0,0);
  utc=start+999999;
  sd_card_set_launch_clock(102,start+130000); /* GS restarts after completed run. */
  assert(sd_run_snapshot()==0 && !sd_launch_finished());
}
''')

    def test_no_permanent_brownout_latch_or_idle_file_creation(self):
        source = (ROOT / "Core/Src/sd_card.c").read_text()
        self.assertNotIn("g_power_loss_mode", source)
        self.assertNotIn("else if (serviced_work == 0U)", source)
        initialization = source.split("UINT sd_card_init", 1)[1]
        self.assertNotIn("sd_open_timestamped_log(NULL)", initialization)
        self.assertIn("g_sd_raw_queue.tx_queue_enqueued == 0U", source)


if __name__ == "__main__":
    unittest.main()
