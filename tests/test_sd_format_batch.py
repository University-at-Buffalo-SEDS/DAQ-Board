"""Verify the production provisioning adapter against a recording block device."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class SdFormatBatchTests(unittest.TestCase):
    def test_reused_buffers_ordering_tail_and_write_failures(self):
        source = (ROOT / 'Core/Src/sd_card.c').read_text()
        adapter = source[source.index('static UCHAR g_sd_format_buffer'):
                         source.index('static UINT sd_format_and_mark')]
        code = r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
typedef unsigned char UCHAR;
typedef unsigned UINT;
typedef uint32_t ULONG;
#define VOID void
#define SD_TRANSFER_SECTORS 8U
#define SD_SECTOR_SIZE 512U
#define FX_SUCCESS 0U
#define FX_IO_ERROR 1U
enum { FX_DRIVER_INIT, FX_DRIVER_WRITE, FX_DRIVER_FLUSH, FX_DRIVER_READ,
       FX_DRIVER_UNINIT, FX_DRIVER_BOOT_WRITE };
typedef struct {
  UINT fx_media_driver_request, fx_media_driver_status;
  ULONG fx_media_driver_logical_sector, fx_media_hidden_sectors, fx_media_driver_sectors;
  UCHAR *fx_media_driver_buffer;
} FX_MEDIA;
static UCHAR disk[200*512], expected[200*512];
static unsigned writes, fail, base_calls;
static UINT sd_hal_write(const UCHAR *p, ULONG sector, ULONG count) {
  assert(sector+count<=200 && count<=8);
  ++writes;
  if (fail) return FX_IO_ERROR;
  memcpy(disk+sector*512,p,count*512); return FX_SUCCESS;
}
static void sd_filex_driver(FX_MEDIA *media) {
  ++base_calls; media->fx_media_driver_status=FX_SUCCESS;
  if (media->fx_media_driver_request==FX_DRIVER_READ)
    memcpy(media->fx_media_driver_buffer,
      disk+(media->fx_media_driver_logical_sector+media->fx_media_hidden_sectors)*512,512);
}
''' + adapter + r'''
int main(void) {
  UCHAR buffer[512];
  FX_MEDIA m={.fx_media_driver_request=FX_DRIVER_INIT,.fx_media_hidden_sectors=7,
    .fx_media_driver_sectors=1,.fx_media_driver_buffer=buffer};
  sd_format_driver(&m);
  m.fx_media_driver_request=FX_DRIVER_WRITE;
  for(unsigned s=0;s<101;s++) {
    memset(buffer,s,sizeof(buffer));
    memcpy(expected+(s+7)*512,buffer,512);
    m.fx_media_driver_logical_sector=s; sd_format_driver(&m);
    assert(m.fx_media_driver_status==FX_SUCCESS);
  }
  memset(buffer,0xee,sizeof(buffer)); /* FileX reuses its memory immediately. */
  assert(writes==12);
  m.fx_media_driver_request=FX_DRIVER_FLUSH; sd_format_driver(&m);
  assert(writes==13 && g_sd_format_sectors_written==101);
  assert(!memcmp(disk,expected,sizeof(disk)));
  m.fx_media_driver_request=FX_DRIVER_WRITE;
  m.fx_media_driver_logical_sector=110; sd_format_driver(&m);
  m.fx_media_driver_request=FX_DRIVER_READ; memset(buffer,0,512);
  sd_format_driver(&m); assert(buffer[0]==0xee && writes==14);
  m.fx_media_driver_request=FX_DRIVER_WRITE;
  m.fx_media_driver_logical_sector=120; sd_format_driver(&m);
  m.fx_media_driver_logical_sector=122; sd_format_driver(&m);
  assert(writes==15); /* A gap flushes before starting the next batch. */
  m.fx_media_driver_request=FX_DRIVER_UNINIT; sd_format_driver(&m);
  assert(writes==16);
  m.fx_media_driver_request=FX_DRIVER_WRITE;
  m.fx_media_driver_logical_sector=130; sd_format_driver(&m);
  fail=1; m.fx_media_driver_request=FX_DRIVER_FLUSH; sd_format_driver(&m);
  assert(m.fx_media_driver_status==FX_IO_ERROR);
  unsigned before=writes;
  m.fx_media_driver_request=FX_DRIVER_WRITE; sd_format_driver(&m);
  assert(m.fx_media_driver_status==FX_IO_ERROR && writes==before);
  fail=0; m.fx_media_driver_request=FX_DRIVER_INIT; sd_format_driver(&m);
  assert(!g_sd_format_count && !g_sd_format_sectors_written);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            exe = Path(tmp) / 'format-test'
            result = subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                                     '-x', 'c', '-', '-o', str(exe)],
                                    input=code, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(exe)], check=True)
