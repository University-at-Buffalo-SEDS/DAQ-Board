"""FAT directory stamps: network UTC, unknown-time sentinel, steps, local zone.

The production stamping block of sd_card.c is compiled against a small model
of the FileX behaviour it relies on (b93ac4a): create stamps created, modified
and accessed from the system clock; write marks the file modified; media flush
and close write the whole open entry (modified time, size, created fields) when
it is marked modified; nothing advances the clock (FX_NO_TIMER).
"""
import datetime as dt
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "Core/Src/sd_card.c"
UTC = dt.timezone.utc
NEW_YORK = ZoneInfo("America/New_York")


def compile_and_run(code, *flags, stdin=None):
    with tempfile.TemporaryDirectory() as tmp:
        exe = Path(tmp) / "test"
        result = subprocess.run(
            ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", *flags,
             "-I", str(ROOT / "Core/Inc"), "-x", "c", "-", "-o", str(exe)],
            input=code, text=True, capture_output=True)
        if result.returncode != 0:
            raise AssertionError(result.stderr)
        run = subprocess.run([str(exe)], input=stdin, text=True, capture_output=True)
        if run.returncode != 0:
            raise AssertionError(run.stderr or f"exit {run.returncode}")
        return run.stdout


def unix_ms(*fields):
    return int(dt.datetime(*fields, tzinfo=UTC).timestamp() * 1000)


def fat(ms, zone=NEW_YORK):
    """(date, time) FAT words of a UTC instant shown in a zone."""
    t = dt.datetime.fromtimestamp(ms / 1000, UTC).astimezone(zone)
    return ((t.year - 1980) << 9 | t.month << 5 | t.day,
            t.hour << 11 | t.minute << 5 | t.second // 2)


def stamping_block():
    source = SOURCE.read_text()
    return source[source.index("static int64_t g_sd_fat_applied_offset;"):
                  source.index("static int sd_format_calibration_line")]


class FatZoneTests(unittest.TestCase):
    PROGRAM = r'''
#include <stdio.h>
#include <inttypes.h>
#include "sd_fat_time.h"
int main(void) {
  unsigned long long ms;
  while (scanf("%llu", &ms) == 1) {
    sd_calendar_t c;
    if (!sd_fat_local_calendar(ms, &c)) { puts("none"); continue; }
    printf("%u-%u-%u %u:%u:%u\n", c.year, c.month, c.day, c.hour, c.minute, c.second);
  }
}
'''

    def check_zone(self, zone, start_year, end_year, *flags):
        instants = []
        for year in range(start_year, end_year):
            base = unix_ms(year, 1, 1)
            instants += range(base, unix_ms(year + 1, 1, 1), 37 * 60 * 1000 + 1)
            for month in (3, 11):  # every second around each US transition window
                first = unix_ms(year, month, 1)
                instants += range(first, first + 15 * 86400 * 1000, 3600 * 1000)
        # Exact transitions, both sides, by second.
        for year in range(start_year, end_year):
            for month in (3, 11):
                for day in range(1, 15):
                    for hour in (5, 6, 7, 8, 9, 10):
                        edge = unix_ms(year, month, day, hour)
                        instants += (edge - 1000, edge - 1, edge, edge + 999)
        output = compile_and_run(self.PROGRAM, *flags,
                                 stdin="\n".join(map(str, instants)) + "\n").split("\n")
        for ms, line in zip(instants, output):
            t = dt.datetime.fromtimestamp(ms / 1000, UTC).astimezone(zone)
            self.assertEqual(line, f"{t.year}-{t.month}-{t.day} {t.hour}:{t.minute}:{t.second}",
                             f"{ms} {t.isoformat()}")

    def test_default_zone_is_new_york_with_us_dst(self):
        # US rules since 2007 (Energy Policy Act of 2005).
        self.check_zone(NEW_YORK, 2007, 2041)

    def test_dst_boundaries_new_york(self):
        cases = {
            unix_ms(2026, 3, 8, 6, 59, 59): "2026-3-8 1:59:59",    # last EST second
            unix_ms(2026, 3, 8, 7, 0, 0): "2026-3-8 3:0:0",        # first EDT second
            unix_ms(2026, 11, 1, 5, 59, 59): "2026-11-1 1:59:59",  # last EDT second
            unix_ms(2026, 11, 1, 6, 0, 0): "2026-11-1 1:0:0",      # first EST second
            unix_ms(2026, 11, 16, 19, 0, 0): "2026-11-16 14:0:0",  # fire day, EST
            unix_ms(2026, 12, 31, 23, 0, 0): "2026-12-31 18:0:0",
            unix_ms(2027, 1, 1, 3, 0, 0): "2026-12-31 22:0:0",     # local year differs
            unix_ms(1980, 1, 1, 4, 59, 59): "none",                # local before FAT epoch
            0: "none", 6830679: "none",                            # unknown / uptime only
        }
        output = compile_and_run(self.PROGRAM, stdin="\n".join(map(str, cases)) + "\n").split("\n")
        self.assertEqual(output[:len(cases)], list(cases.values()))

    def test_utc_and_other_zones_by_build_option(self):
        self.check_zone(UTC, 2024, 2028, "-DDAQ_SD_FAT_UTC_OFFSET_MIN=0", "-DDAQ_SD_FAT_US_DST=0")
        self.check_zone(ZoneInfo("America/Los_Angeles"), 2024, 2028, "-DDAQ_SD_FAT_UTC_OFFSET_MIN=-480")
        self.check_zone(dt.timezone(dt.timedelta(hours=5, minutes=30)), 2024, 2026,
                        "-DDAQ_SD_FAT_UTC_OFFSET_MIN=330", "-DDAQ_SD_FAT_US_DST=0")


FILEX_MODEL = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sd_fat_time.h"
typedef unsigned UINT;
typedef unsigned char UCHAR;
#define FX_SUCCESS 0U
#define FX_TRUE 1U
#define FX_FALSE 0U
typedef struct {
  UCHAR fx_dir_entry_created_time_ms;
  UINT fx_dir_entry_created_time, fx_dir_entry_created_date, fx_dir_entry_last_accessed_date;
  UINT fx_dir_entry_time, fx_dir_entry_date;
} FX_DIR_ENTRY;
typedef struct { FX_DIR_ENTRY fx_file_dir_entry; UCHAR fx_file_modified, model_open; } FX_FILE;
typedef struct { FX_FILE file; char name[64]; uint64_t key, created_ms; uint8_t open; } sd_log_t;
static sd_log_t g_sd_regular_log, g_sd_launch_log;
static FX_DIR_ENTRY disk[2];
static unsigned disk_writes[2], clock_sets;
static int64_t g_sd_clock_offset;
static uint64_t mono;
static UINT sys_date = 1U << 5 | 1U, sys_time; /* fx_system_date_set(1980,1,1) at init */
static uint32_t __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __set_PRIMASK(uint32_t v) { (void)v; }
static uint64_t daq_log_monotonic_ms(void) { return mono; }
static UINT fx_system_date_set(UINT y, UINT m, UINT d) {
  sys_date = (y - 1980U) << 9 | m << 5 | d; ++clock_sets; return FX_SUCCESS;
}
static UINT fx_system_time_set(UINT h, UINT m, UINT s) {
  sys_time = h << 11 | m << 5 | s / 2U; return FX_SUCCESS;
}
static int slot(const FX_FILE *f) { return f == &g_sd_launch_log.file; }
static void entry_write(FX_FILE *f) { disk[slot(f)] = f->fx_file_dir_entry; ++disk_writes[slot(f)]; }
/* _fx_directory_free_search + _fx_file_create, then fx_file_open. */
static void model_create(FX_FILE *f) {
  FX_DIR_ENTRY e = {0, sys_time, sys_date, sys_date, sys_time, sys_date};
  f->fx_file_dir_entry = e; f->fx_file_modified = FX_FALSE; f->model_open = 1U; entry_write(f);
}
static void model_write(FX_FILE *f) { f->fx_file_modified = FX_TRUE; }
/* _fx_media_flush: every open, modified file's entry at the system clock. */
static void model_media_flush(void) {
  sd_log_t *logs[2] = {&g_sd_regular_log, &g_sd_launch_log};
  for (unsigned i = 0; i < 2; ++i) {
    FX_FILE *f = &logs[i]->file;
    if (!f->model_open || !f->fx_file_modified) continue;
    f->fx_file_dir_entry.fx_dir_entry_time = sys_time;
    f->fx_file_dir_entry.fx_dir_entry_date = sys_date;
    entry_write(f); f->fx_file_modified = FX_FALSE;
  }
}
static UINT sd_release_file_space(FX_FILE *f) { (void)f; return FX_SUCCESS; }
/* _fx_file_close */
static UINT fx_file_close(FX_FILE *f) {
  if (f->fx_file_modified) {
    f->fx_file_dir_entry.fx_dir_entry_time = sys_time;
    f->fx_file_dir_entry.fx_dir_entry_date = sys_date;
    f->fx_file_dir_entry.fx_dir_entry_last_accessed_date = sys_date;
    entry_write(f);
  }
  f->model_open = 0U;
  return FX_SUCCESS;
}
@BLOCK@
/* sd_open_log's order: one creation instant, clock, create, header, flush. */
static void open_log(sd_log_t *log) {
  const uint64_t created = daq_log_monotonic_ms();
  sd_update_filesystem_clock_at(created);
  model_create(&log->file); model_write(&log->file); model_media_flush();
  log->open = 1U; log->created_ms = created;
}
/* The writer's flush turn: stepped clock or 1 s period. */
static unsigned writer_turn(int rows) {
  if (rows) { if (g_sd_regular_log.open) model_write(&g_sd_regular_log.file);
              if (g_sd_launch_log.open) model_write(&g_sd_launch_log.file); }
  const unsigned stepped = sd_fat_clock_stepped();
  sd_update_filesystem_clock();
  model_media_flush();
  return stepped;
}
#define CREATED(i) (disk[i].fx_dir_entry_created_date << 16 | disk[i].fx_dir_entry_created_time)
#define MODIFIED(i) (disk[i].fx_dir_entry_date << 16 | disk[i].fx_dir_entry_time)
#define FAT(pair) ((pair##_DATE) << 16 | (pair##_TIME))
'''


class FatStampLifecycleTests(unittest.TestCase):
    def run_model(self, constants, body):
        defines = "".join(f"#define {k} {v}ULL\n" for k, v in constants.items())
        compile_and_run(FILEX_MODEL.replace("@BLOCK@", stamping_block()) + defines +
                        "int main(void) {\n" + body + "\n}\n")

    @staticmethod
    def pair(name, ms):
        date, time = fat(ms)
        return {f"{name}_DATE": date, f"{name}_TIME": time}

    def test_open_file_through_sync_step_periodic_flush_and_close(self):
        wrong = unix_ms(2026, 11, 16, 13, 0, 0)       # network 3.8 h behind real time
        step = int(3.8 * 3600 * 1000)                 # operator set
        sync_offset = wrong - 8000                    # synced at mono 8000
        c = {"SYNC_OFFSET": sync_offset, "STEP": step}
        c |= self.pair("WRONG_CREATED", sync_offset + 5000)
        c |= self.pair("WRONG_NOW", sync_offset + 8000)
        c |= self.pair("PERIODIC", sync_offset + 20000 + 480)
        c |= self.pair("STEPPED_CREATED", sync_offset + step + 5000 + 480)
        c |= self.pair("STEPPED_NOW", sync_offset + step + 22000 + 480)
        c |= self.pair("LAUNCH_CREATED", sync_offset + step + 30000 + 480)
        c |= self.pair("CLOSED", sync_offset + step + 45000 + 480)
        self.run_model(c, r'''
  /* Creation before sync: the FAT epoch sentinel, never a guessed date. */
  mono = 5000; open_log(&g_sd_regular_log);
  assert(CREATED(0) == (33U << 16) && MODIFIED(0) == (33U << 16) && clock_sets == 0);
  for (mono = 6000; mono <= 7000; mono += 1000) assert(!writer_turn(1));
  assert(CREATED(0) == (33U << 16) && MODIFIED(0) == (33U << 16) && clock_sets == 0);
  assert(disk[0].fx_dir_entry_last_accessed_date == 33U);

  /* Sync mid-file: the next turn restamps creation from its monotonic instant. */
  mono = 8000; g_sd_clock_offset = (int64_t)SYNC_OFFSET;
  assert(sd_fat_clock_stepped());
  assert(writer_turn(0));                      /* idle file still gets fixed */
  assert(CREATED(0) == FAT(WRONG_CREATED) && MODIFIED(0) == FAT(WRONG_NOW));
  assert(disk[0].fx_dir_entry_last_accessed_date == WRONG_NOW_DATE);
  assert(!sd_fat_clock_stepped());

  /* Periodic update while open: each 1 s flush leaves a recent modified time,
   * which is what a power cut keeps; slew under one FAT tick is not a step. */
  for (mono = 9000; mono <= 20000; mono += 1000) {
    g_sd_clock_offset += 40;
    assert(!writer_turn(1));
  }
  assert(MODIFIED(0) == FAT(PERIODIC) && CREATED(0) == FAT(WRONG_CREATED));
  const unsigned writes = disk_writes[0];
  assert(mono == 21000); mono += 1000; assert(!writer_turn(0));
  assert(disk_writes[0] == writes);            /* no rows, no slew step: no I/O */

  /* Network time step while open (operator set +3.8 h): restamped at once. */
  g_sd_clock_offset += (int64_t)STEP;
  assert(sd_fat_clock_stepped() && writer_turn(0));
  assert(CREATED(0) == FAT(STEPPED_CREATED) && MODIFIED(0) == FAT(STEPPED_NOW));
  assert(!sd_fat_clock_stepped());

  /* A file created after sync is stamped right at creation, never rewritten
   * for its creation time alone. */
  mono = 30000; open_log(&g_sd_launch_log);
  assert(CREATED(1) == FAT(LAUNCH_CREATED) && MODIFIED(1) == FAT(LAUNCH_CREATED));
  const unsigned launch_writes = disk_writes[1];
  for (mono = 31000; mono <= 33000; mono += 1000) writer_turn(0);
  assert(disk_writes[1] == launch_writes && !g_sd_launch_log.file.fx_file_modified);

  /* Close stamps modified and accessed now, even with nothing left to flush. */
  mono = 42000; writer_turn(1);
  assert(!g_sd_regular_log.file.fx_file_modified && MODIFIED(0) != FAT(CLOSED));
  mono = 45000;
  assert(sd_close_file(&g_sd_regular_log) == FX_SUCCESS && !g_sd_regular_log.open);
  assert(MODIFIED(0) == FAT(CLOSED) && disk[0].fx_dir_entry_last_accessed_date == CLOSED_DATE);
  assert(CREATED(0) == FAT(STEPPED_CREATED));
  /* Closed files are no longer restamped by later steps. */
  g_sd_clock_offset += 3600000; mono += 1000; writer_turn(0);
  assert(MODIFIED(0) == FAT(CLOSED) && CREATED(0) == FAT(STEPPED_CREATED));
''')

    def test_close_before_any_sync_keeps_the_sentinel(self):
        self.run_model({}, r'''
  mono = 1000; open_log(&g_sd_regular_log);
  for (mono = 2000; mono < 600000; mono += 1000) writer_turn(1); /* ten minutes */
  assert(sd_close_file(&g_sd_regular_log) == FX_SUCCESS);
  /* FX_NO_TIMER: the sentinel does not drift with uptime. */
  assert(CREATED(0) == (33U << 16) && MODIFIED(0) == (33U << 16) && clock_sets == 0);
''')

    def test_file_open_across_the_dst_fall_back(self):
        start = unix_ms(2026, 11, 1, 5, 59, 50)       # 01:59:50 EDT
        offset = start - 1000
        c = {"OFFSET": offset}
        c |= self.pair("CREATED", start)
        c |= self.pair("CLOSED", offset + 41000)    # 01:00:30 EST
        self.assertEqual(c["CREATED_TIME"], 1 << 11 | 59 << 5 | 25)
        self.assertEqual(c["CLOSED_TIME"], 1 << 11 | 0 << 5 | 15)
        self.run_model(c, r'''
  g_sd_clock_offset = (int64_t)OFFSET;
  mono = 1000; open_log(&g_sd_regular_log);
  assert(CREATED(0) == FAT(CREATED));
  for (mono = 2000; mono <= 41000; mono += 1000) writer_turn(1);
  assert(sd_close_file(&g_sd_regular_log) == FX_SUCCESS);
  assert(MODIFIED(0) == FAT(CLOSED) && CREATED(0) == FAT(CREATED));
''')


class FatStampSourceContract(unittest.TestCase):
    def test_writer_sets_the_clock_before_every_stamp(self):
        source = SOURCE.read_text()
        open_log = source[source.index("static UINT sd_open_log"):source.index("static UINT sd_log_flush(")]
        self.assertRegex(open_log, r"const uint64_t created = daq_log_monotonic_ms\(\);\s*"
                                   r"sd_update_filesystem_clock_at\(created\);")
        self.assertLess(open_log.index("sd_update_filesystem_clock_at(created)"),
                        open_log.index("sd_create_log_file("))
        self.assertIn("log->created_ms = created;", open_log)
        writer = source[source.index("void sd_card_writer_thread_entry"):source.index("UINT sd_card_init")]
        flush = writer[writer.index("sd_fat_clock_stepped() ||"):]
        self.assertLess(flush.index("sd_update_filesystem_clock();"), flush.index("sd_media_flush()"))
        # No stamp outside the network clock path: UTC written as local was the bug.
        self.assertNotIn("fx_file_date_time_set", source)
        self.assertEqual(len(re.findall(r"fx_system_(?:date|time)_set\(", source)), 4)
        init = source[source.index("UINT sd_card_init"):]
        self.assertIn("(void)fx_system_date_set(1980U, 1U, 1U);", init)

    def test_filex_clock_does_not_free_run(self):
        self.assertRegex((ROOT / "Core/Inc/fx_user.h").read_text(), r"(?m)^#define FX_NO_TIMER$")


if __name__ == "__main__":
    unittest.main()
