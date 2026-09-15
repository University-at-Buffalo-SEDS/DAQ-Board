"""Run the production FileX driver against a recording block adapter."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class SdPartitionDriverTests(unittest.TestCase):
    def test_boot_writes_preserve_partition_table(self):
        source = (ROOT / "Core/Src/sd_card.c").read_text()
        driver = "static VOID sd_filex_driver" + source.split(
            "static VOID sd_filex_driver", 1)[1].split(
            "static UINT sd_open_timestamped_log", 1)[0]
        code = r"""
#include <assert.h>
#include <stdint.h>
typedef unsigned UINT;
typedef uint32_t ULONG;
typedef unsigned char UCHAR;
typedef void VOID;
#define FX_SUCCESS 0U
#define FX_IO_ERROR 1U
#define FX_FALSE 0U
enum { FX_DRIVER_INIT, FX_DRIVER_UNINIT, FX_DRIVER_FLUSH, FX_DRIVER_ABORT,
       FX_DRIVER_RELEASE_SECTORS, FX_DRIVER_BOOT_READ, FX_DRIVER_BOOT_WRITE,
       FX_DRIVER_READ, FX_DRIVER_WRITE };
typedef struct {
  UINT fx_media_driver_request, fx_media_driver_status;
  UINT fx_media_driver_write_protect, fx_media_driver_free_sector_update;
  ULONG fx_media_driver_sectors, fx_media_driver_logical_sector, fx_media_hidden_sectors;
  UCHAR *fx_media_driver_buffer;
} FX_MEDIA;
static unsigned g_sd_hardware_ready = 1U;
static ULONG partition, last_sector;
static unsigned reads, writes;
static UINT sd_hal_read(UCHAR *b, ULONG sector, ULONG count) {
  (void)b; assert(count == 1U); last_sector=sector; ++reads; return FX_SUCCESS;
}
static UINT sd_hal_write(const UCHAR *b, ULONG sector, ULONG count) {
  (void)b; assert(count == 1U); last_sector=sector; ++writes; return FX_SUCCESS;
}
static UINT _fx_partition_offset_calculate(void *b, UINT p, ULONG *start, ULONG *size) {
  (void)b; assert(p == 0U); *start=partition; *size=10000; return FX_SUCCESS;
}
""" + driver + r"""
int main(void) {
  UCHAR buffer[512] = {0};
  FX_MEDIA media = {0};
  media.fx_media_driver_buffer=buffer;
  media.fx_media_driver_sectors=1U;
  media.fx_media_hidden_sectors=2048U;
  media.fx_media_driver_request=FX_DRIVER_BOOT_WRITE;
  sd_filex_driver(&media);
  assert(writes==1 && last_sector==2048U && media.fx_media_driver_status==FX_SUCCESS);
  media.fx_media_hidden_sectors=0;
  sd_filex_driver(&media);
  assert(writes==2 && last_sector==0U);
  partition=2048U;
  media.fx_media_driver_request=FX_DRIVER_BOOT_READ;
  sd_filex_driver(&media);
  assert(reads==2 && last_sector==2048U);
  media.fx_media_hidden_sectors=2048U;
  media.fx_media_driver_logical_sector=17U;
  media.fx_media_driver_request=FX_DRIVER_WRITE;
  sd_filex_driver(&media);
  assert(writes==3 && last_sector==2065U);
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / "partition-test"
            result = subprocess.run(
                ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-x", "c",
                 "-", "-o", str(binary)], input=code, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(binary)], check=True)

