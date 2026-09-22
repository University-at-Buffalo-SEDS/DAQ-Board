import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class TelemetryBufferTest(unittest.TestCase):
    def test_rows_batch_without_loss_and_failed_tail_is_retained(self):
        source = (ROOT / 'Core/Src/sd_card.c').read_text()
        helpers = source[source.index('static UINT sd_flush_telemetry_pending(void)'):
                         source.index('/* Delayed telemetry rows')]
        code = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
typedef unsigned UINT;
typedef uint32_t ULONG;
typedef unsigned char UCHAR;
enum {FX_SUCCESS=0,FX_PTR_ERROR=1,FX_IO_ERROR=2};
static int g_sd_telemetry_file, g_telemetry_file_open=1;
static UCHAR g_sd_telemetry_write_buffer[4096];
static size_t g_sd_telemetry_write_buffer_len;
static unsigned g_sd_write_error_count, calls, fail;
static char disk[20000];
static size_t written;
static UINT fx_file_write(int *f, void *p, ULONG bytes) {
  assert(f==&g_sd_telemetry_file && bytes && bytes<=4096);
  ++calls;
  if(fail) return FX_IO_ERROR;
  memcpy(disk+written,p,bytes); written+=bytes; return 0;
}
''' + helpers + r'''
int main(void) {
  const char row[]="12345,12345,kg50_network,3.0,,,,local,,\r\n";
  for(unsigned i=0;i<400;++i) assert(sd_write_telemetry_bytes(row,sizeof(row)-1)==0);
  assert(calls==400*(sizeof(row)-1)/4096); /* Not one file write per row. */
  assert(written+g_sd_telemetry_write_buffer_len==400*(sizeof(row)-1));
  fail=1; size_t tail=g_sd_telemetry_write_buffer_len;
  assert(sd_flush_telemetry_pending()==FX_IO_ERROR);
  assert(g_sd_telemetry_write_buffer_len==tail && g_sd_write_error_count==1);
  fail=0;
  assert(sd_flush_telemetry_pending()==0 && !g_sd_telemetry_write_buffer_len);
  assert(written==400*(sizeof(row)-1));
  for(unsigned i=0;i<400;++i) assert(!memcmp(disk+i*(sizeof(row)-1),row,sizeof(row)-1));
  unsigned prior=calls;
  assert(sd_flush_telemetry_pending()==0 && calls==prior);
  g_telemetry_file_open=0;
  assert(sd_write_telemetry_bytes(row,sizeof(row)-1)==FX_PTR_ERROR);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / 'telemetry-buffer'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-x', 'c', '-', '-o', str(binary)],
                           input=code, text=True, check=True)
            subprocess.run([str(binary)], check=True)
