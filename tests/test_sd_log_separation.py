"""Execute production route-key file selection against recording FileX mocks.

Every row (raw ADC, telemetry, VB pressure) carries a route key: odd keys are
regular session files, even keys are launch-window files. Interleaved queues
must never make the writer close and reopen files (the rotation storm that
created hundreds of tiny files and starved raw logging), calibration changes
must not rotate, and late rows must reach an existing file without loss.
"""
import pathlib
import subprocess
import tempfile
import unittest
from test_sd_raw_queue import function

ROOT = pathlib.Path(__file__).resolve().parents[1]


class LogSeparationTests(unittest.TestCase):
    def test_route_keys_open_each_file_once_without_loss(self):
        source = (ROOT / "Core/Src/sd_card.c").read_text()
        log_type = source[source.index("typedef struct {\n  FX_FILE file;"):
                          source.index("} sd_log_t;") + len("} sd_log_t;")]
        writer = source[source.index("static UINT sd_log_flush(sd_log_t *log)"):
                        source.index("static UINT sd_marker_exists(void)")]
        harness = r"""
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sd_file_limit.h"
#include "daq_calibration.h"
typedef unsigned UINT; typedef uint32_t ULONG; typedef unsigned char UCHAR; typedef void VOID;
#define FX_SUCCESS 0U
#define FX_IO_ERROR 1U
#define FX_PTR_ERROR 2U
typedef struct { uint64_t fx_file_current_file_size, fx_file_current_file_offset; unsigned id; } FX_FILE;
typedef struct { daq_calibration_t calibration; char line[64]; unsigned len; uint64_t session; } sd_line_slot_t;
""" + log_type + r"""
static sd_log_t g_sd_regular_log, g_sd_launch_log;
static uint64_t g_sd_regular_closed_key, g_sd_launch_done_key;
static unsigned g_sd_launch_flush_pending, g_sd_write_error_count;
static unsigned regular_opens, launch_opens, closes, rows, notes, fail_write, fail_close, fail_open;
static unsigned rows_in[2][16]; /* [launch][key] */
static unsigned next_id = 1;
static UINT sd_media_flush(void) { return 0; }
static UINT sd_prepare_file_space(sd_log_t *log, size_t n) { (void)log; (void)n; return 0; }
static UINT fx_file_seek(FX_FILE *f, ULONG o) { f->fx_file_current_file_offset = o; return 0; }
static UINT fx_file_write(FX_FILE *f, VOID *data, ULONG len) {
  assert(data && len);
  if (fail_write) return FX_IO_ERROR;
  const char *text = data;
  for (ULONG i = 0; i < len; ++i) {
    if (text[i] == 'R') {
      const int launch = f == &g_sd_launch_log.file;
      const unsigned key = (unsigned)(text[i + 1] - '0');
      ++rows_in[launch][key]; ++rows;
    } else if (text[i] == '#') ++notes;
  }
  f->fx_file_current_file_offset += len;
  f->fx_file_current_file_size = f->fx_file_current_file_offset;
  return 0;
}
static UINT sd_close_file(sd_log_t *log) {
  if (fail_close) { --fail_close; return FX_IO_ERROR; }
  ++closes; log->open = 0; return 0;
}
static int sd_format_calibration_line(char *line, size_t size, const daq_calibration_t *c) {
  return snprintf(line, size, "#cal %g\r\n", (double)c->kg1000_slope);
}
static UINT sd_open_log(sd_log_t *log, uint64_t key, const daq_calibration_t *calibration) {
  if (fail_open) { --fail_open; return FX_IO_ERROR; }
  if (log == &g_sd_launch_log) { assert((key & 1U) == 0U); ++launch_opens; }
  else { assert(key & 1U); ++regular_opens; }
  log->file.fx_file_current_file_size = log->file.fx_file_current_file_offset = 0;
  log->file.id = next_id++; log->len = 0; log->retry_active = 0;
  log->open = 1; log->key = key; log->calibration = *calibration;
  return 0;
}
""" + writer + r"""
static sd_line_slot_t row(uint64_t key, float slope) {
  sd_line_slot_t s = {{slope, 0, 1, 0, {0}, {0}, {0}}, "R0\r\n", 4, key};
  s.line[1] = (char)('0' + key);
  return s;
}
static UINT put(uint64_t key) { sd_line_slot_t s = row(key, 1); return sd_write_telemetry_row(&s); }
static void drain(void) { assert(sd_log_flush(&g_sd_regular_log) == 0 && sd_log_flush(&g_sd_launch_log) == 0); }
int main(void) {
  /* Boot session: raw, telemetry and pressure rows share one regular file. */
  for (unsigned i = 0; i < 1000; ++i) assert(put(1) == 0);
  assert(regular_opens == 1 && launch_opens == 0);
  /* Launch window: every launch-key row in ONE file. Pre-T0 pressure
   * captured before the window keeps arriving interleaved (the g6 storm was
   * exactly this alternation); it must not close or reopen either file. */
  for (unsigned i = 0; i < 10000; ++i) { assert(put(2) == 0); assert(put(1) == 0); }
  assert(regular_opens == 1 && launch_opens == 1 && closes == 0);
  /* A calibration change is an in-stream note, never a new file. */
  sd_line_slot_t changed = row(2, 7);
  const unsigned notes_before = notes;
  assert(sd_write_telemetry_row(&changed) == 0 && sd_write_telemetry_row(&changed) == 0);
  drain(); assert(notes == notes_before + 1 && launch_opens == 1);
  /* At T0+grace the writer finishes the pre-launch regular file once. */
  assert(sd_log_finish(&g_sd_regular_log) == 0 && g_sd_regular_closed_key == 1 && closes == 1);
  /* A very late pre-window row joins the open launch file, no reopened part. */
  assert(put(1) == 0 && regular_opens == 1 && launch_opens == 1);
  /* T+20: one NEW regular session file, while late window rows still reach
   * the launch file (both handles open only for this overlap). */
  for (unsigned i = 0; i < 1000; ++i) { assert(put(3) == 0); assert(put(2) == 0); }
  assert(regular_opens == 2 && launch_opens == 1 && closes == 1);
  assert(sd_log_finish(&g_sd_launch_log) == 0 && g_sd_launch_done_key == 2 && closes == 2);
  /* A finished launch window never reopens: a later window row is kept in
   * the current regular file instead of creating a second launch file. */
  assert(put(2) == 0 && launch_opens == 1 && regular_opens == 2);
  /* Write/close failures keep handles, bytes and keys; the caller retries the
   * same row, which is then written exactly once. */
  unsigned failed = 0;
  fail_write = 1;
  for (unsigned i = 0; i < 2000; ++i) if (put(3) != 0) ++failed;
  fail_write = 0;
  assert(failed > 0 && g_sd_regular_log.open && g_sd_write_error_count == failed);
  while (failed--) assert(put(3) == 0);
  /* Only the FAT32 part limit splits a key's file; a failed close retries. */
  drain();
  g_sd_regular_log.file.fx_file_current_file_size = SD_LOG_PART_LIMIT_BYTES - 2;
  fail_close = 1;
  assert(put(3) == FX_IO_ERROR && g_sd_regular_log.open && regular_opens == 2);
  assert(put(3) == 0 && regular_opens == 3 && g_sd_regular_log.key == 3 && g_sd_regular_closed_key == 1);
  /* A second launch: one more launch file and one more regular file. */
  assert(put(4) == 0 && put(5) == 0 && launch_opens == 2 && regular_opens == 4);
  drain();
  assert(rows_in[0][1] == 11000 && rows_in[1][1] == 1);
  assert(rows_in[1][2] == 11002 && rows_in[0][2] == 1);
  assert(rows_in[0][3] == 3001 && rows_in[1][4] == 1 && rows_in[0][5] == 1);
  assert(rows == 25007 && fail_open == 0);
  return 0;
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / "log-test"
            result = subprocess.run(
                ["cc", "-std=c11", "-O1", "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined",
                 "-I", str(ROOT / "Core/Inc"), "-x", "c", "-", "-o", str(binary)],
                input=harness, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(binary)], check=True)

    def test_every_row_kind_uses_the_route_key_selector(self):
        source = (ROOT / "Core/Src/sd_card.c").read_text()
        self.assertIn("sd_write_telemetry_row(pending)", source)
        self.assertIn("sd_log_for_key(slot->session, &slot->calibration", function(source, "sd_write_telemetry_row"))
        writer = function(source, "sd_card_writer_thread_entry")
        self.assertIn("sd_log_for_key(slot->session, &slot->calibration,", writer)
        self.assertIn("sd_write_telemetry_row(slot)", function(source, "sd_service_pressure_rows"))
        # No calibration-driven or session-driven rotation remains.
        self.assertNotIn("sd_select_calibration", source)
        self.assertNotIn("DAQ_TELEMETRY", source)
        self.assertNotIn("DAQ_LAUNCH_TELEMETRY", source)


if __name__ == "__main__":
    unittest.main()
