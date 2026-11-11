import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class TelemetryBufferTest(unittest.TestCase):
    def test_rows_batch_without_loss_and_failed_tail_is_retained(self):
        source = (ROOT / 'Core/Src/sd_card.c').read_text()
        log_type = source[source.index('typedef struct {\n  FX_FILE file;'):
                          source.index('} sd_log_t;') + len('} sd_log_t;')]
        helpers = source[source.index('static UINT sd_log_flush(sd_log_t *log)'):
                         source.index('/* A calibration change is recorded in-stream')]
        code = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
typedef unsigned UINT;
typedef uint32_t ULONG;
typedef unsigned char UCHAR;
enum {FX_SUCCESS=0,FX_PTR_ERROR=1,FX_IO_ERROR=2};
#include "daq_calibration.h"
typedef struct { ULONG fx_file_current_file_offset; } FX_FILE;
''' + log_type + r'''
static sd_log_t g_log = {.open = 1U};
static FX_FILE *const g_sd_telemetry_file = &g_log.file;
static unsigned g_sd_write_error_count, calls, fail, reserve_fail;
static char disk[20000];
static size_t written;
static UINT sd_prepare_file_space(sd_log_t *l,size_t n) {(void)l;(void)n;return reserve_fail?FX_IO_ERROR:0;}
static UINT fx_file_seek(FX_FILE *f, ULONG offset) { f->fx_file_current_file_offset=offset; return 0; }
static UINT fx_file_write(FX_FILE *f, void *p, ULONG bytes) {
  assert(f==g_sd_telemetry_file && bytes && bytes<=sizeof(g_log.buffer));
  ++calls;
  const ULONG n=fail ? bytes/2 : bytes; /* Real failures can advance file position. */
  memcpy(disk+f->fx_file_current_file_offset,p,n); f->fx_file_current_file_offset+=n;
  if (f->fx_file_current_file_offset>written) written=f->fx_file_current_file_offset;
  if(fail) return FX_IO_ERROR;
  return 0;
}
''' + helpers + r'''
int main(void) {
  const char row[]="12345,12345,kg50_network,3.0,,,,local,,\r\n";
  for(unsigned i=0;i<400;++i) assert(sd_log_write(&g_log,row,sizeof(row)-1)==0);
  assert(calls<8); /* Not one file write per row. */
  assert(written+g_log.len==400*(sizeof(row)-1));
  size_t tail=g_log.len;
  unsigned before_reserve=calls;reserve_fail=1;
  assert(sd_log_flush(&g_log)==FX_IO_ERROR && calls==before_reserve);
  assert(g_log.len==tail && g_sd_write_error_count==1);
  reserve_fail=0;fail=1;
  assert(sd_log_flush(&g_log)==FX_IO_ERROR);
  assert(g_log.len==tail && g_sd_write_error_count==2);
  fail=0;
  assert(sd_log_flush(&g_log)==0 && !g_log.len);
  assert(written==400*(sizeof(row)-1));
  for(unsigned i=0;i<400;++i) assert(!memcmp(disk+i*(sizeof(row)-1),row,sizeof(row)-1));
  unsigned prior=calls;
  assert(sd_log_flush(&g_log)==0 && calls==prior);
  g_log.open=0;
  assert(sd_log_write(&g_log,row,sizeof(row)-1)==FX_PTR_ERROR);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / 'telemetry-buffer'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-I', str(ROOT / 'Core/Inc'),
                            '-x', 'c', '-', '-o', str(binary)],
                           input=code, text=True, check=True)
            subprocess.run([str(binary)], check=True)
