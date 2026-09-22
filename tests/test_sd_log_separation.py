"""Execute production stream-selection code against recording FileX mocks."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class LogSeparationTests(unittest.TestCase):
    def test_independent_calibration_epochs_and_write_failures(self):
        source = (ROOT / "Core/Src/sd_card.c").read_text()
        telemetry = "static UINT sd_write_telemetry_row" + source.split(
            "static UINT sd_write_telemetry_row", 1)[1].split(
            "static UINT sd_marker_exists", 1)[0]
        raw = "static UINT sd_select_calibration" + source.split(
            "static UINT sd_select_calibration", 1)[1].split(
            "static uint8_t sd_service_telemetry_rows", 1)[0]
        harness = r"""
#include <assert.h>
#include <stdint.h>
#include <string.h>
typedef unsigned UINT;
typedef void VOID;
typedef struct { unsigned id; } FX_FILE;
typedef struct { float slope, offset; } daq_calibration_t;
typedef struct { daq_calibration_t calibration; char line[64]; unsigned len; uint64_t session; } sd_line_slot_t;
#define FX_SUCCESS 0U
#define FX_IO_ERROR 1U
static FX_FILE g_sd_file={1}, g_sd_telemetry_file={2};
static unsigned g_sd_media, g_telemetry_file_open, g_file_open=1;
static uint64_t g_sd_telemetry_session;
static daq_calibration_t g_sd_telemetry_calibration, g_sd_open_calibration={1,0};
static char g_sd_telemetry_filename[64], g_sd_filename[64];
static unsigned raw_opens, telemetry_opens, raw_closes, telemetry_closes, writes, fail_write;
static UINT fx_media_flush(void *m) { (void)m; return 0; }
static UINT fx_file_close(FX_FILE *f) {
  if (f==&g_sd_file) raw_closes++; else telemetry_closes++;
  return 0;
}
static UINT sd_close_log(FX_FILE *f, char *name) { (void)name; return fx_file_close(f); }
static UINT sd_flush_pending(void) { return 0; }
static unsigned fail_flush;
static UINT sd_flush_telemetry_pending(void) { return fail_flush ? FX_IO_ERROR : 0; }
static UINT sd_open_timestamped_log(const daq_calibration_t *c) {
  raw_opens++; g_file_open=1; g_sd_open_calibration=*c; return 0;
}
static UINT sd_open_log(FX_FILE *f, char *n, size_t size, const char *prefix,
                        const daq_calibration_t *c) {
  (void)n; (void)size; (void)c;
  assert(f==&g_sd_telemetry_file && (strcmp(prefix,"DAQ_TELEMETRY")==0 ||
                                  strcmp(prefix,"DAQ_LAUNCH_TELEMETRY")==0));
  telemetry_opens++; return 0;
}
static UINT fx_file_write(FX_FILE *f, VOID *data, unsigned len) {
  assert(f==&g_sd_telemetry_file && data && len);
  writes++; return fail_write ? FX_IO_ERROR : FX_SUCCESS;
}
static UINT sd_write_telemetry_bytes(const void *data, size_t len) {
  return fx_file_write(&g_sd_telemetry_file, (void *)data, len);
}
""" + telemetry + raw + r"""
int main(void) {
  sd_line_slot_t row={{1,0},"kg1000_network",14,0};
  assert(sd_write_telemetry_row(&row)==0);
  assert(sd_write_telemetry_row(&row)==0 && telemetry_opens==1);
  daq_calibration_t next={2,3};
  assert(sd_select_calibration(&next)==0 && raw_opens==1 && raw_closes==1);
  /* A queued old network row cannot rotate or contaminate the new raw file. */
  assert(sd_write_telemetry_row(&row)==0 && raw_opens==1);
  row.calibration=next;
  fail_flush=1;
  assert(sd_write_telemetry_row(&row)==FX_IO_ERROR && telemetry_closes==0);
  fail_flush=0;
  assert(sd_write_telemetry_row(&row)==0 && telemetry_opens==2 && telemetry_closes==1);
  assert(sd_select_calibration(&next)==0 && raw_opens==1);
  fail_write=1;
  assert(sd_write_telemetry_row(&row)==FX_IO_ERROR);
  assert(writes==5 && g_sd_open_calibration.slope==2);
  fail_write=0;
  row.session=123;
  assert(sd_write_telemetry_row(&row)==0 && telemetry_opens==3);
  /* Ten minutes at 500 Hz, with duplicate retained launch/calibration updates:
   * keep appending to the same file instead of creating startup fragments. */
  for (unsigned i=0; i<600U*500U; ++i) {
    assert(sd_write_telemetry_row(&row)==0);
    assert(sd_select_calibration(&next)==0);
  }
  assert(telemetry_opens==3 && raw_opens==1);
  row.session=456;
  assert(sd_write_telemetry_row(&row)==0 && telemetry_opens==4);
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / "log-test"
            result = subprocess.run(
                ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-x", "c",
                 "-", "-o", str(binary)], input=harness, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(binary)], check=True)

    def test_worker_does_not_send_telemetry_to_raw_buffer(self):
        source = (ROOT / "Core/Src/sd_card.c").read_text()
        self.assertIn("sd_write_telemetry_row(slot)", source)
        self.assertNotIn("sd_write_bytes(slot->line", source)
