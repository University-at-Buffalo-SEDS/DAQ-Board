"""Run FileX's actual FAT mirror routine with the board's configuration."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class FatMirroringTest(unittest.TestCase):
    def test_large_card_mirrors_only_changed_sectors_and_retries_errors(self):
        candidates = list((ROOT / 'build').glob(
            '*/_deps/filex-src/common/src/fx_utility_FAT_map_flush.c'))
        if not candidates:
            self.skipTest('Configure a firmware build to fetch the pinned FileX source')
        source = candidates[0].read_text()
        routine = source[source.index('UINT  _fx_utility_FAT_map_flush('):]
        program = r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "fx_user.h"
typedef unsigned UINT;
typedef uint32_t ULONG;
typedef uint64_t ULONG64;
typedef unsigned char UCHAR;
enum { FX_SUCCESS=0, FX_FAT_SECTOR=1 };
typedef struct {
  ULONG fx_media_sectors_per_FAT, fx_media_reserved_sectors;
  UINT fx_media_number_of_FATs;
  UCHAR fx_media_fat_secondary_update_map[FX_FAT_MAP_SIZE];
  UCHAR *fx_media_memory_buffer;
} FX_MEDIA;
static unsigned reads, writes, fail_write;
static ULONG copied[8];
static UINT _fx_utility_logical_sector_read(FX_MEDIA *m, ULONG64 sector,
                                            UCHAR *buffer, ULONG count, UINT kind) {
  assert(count==1 && kind==FX_FAT_SECTOR);
  assert(sector>=m->fx_media_reserved_sectors);
  assert(sector<m->fx_media_reserved_sectors+m->fx_media_sectors_per_FAT);
  memcpy(buffer,&sector,sizeof(sector)); ++reads; return 0;
}
static UINT _fx_utility_logical_sector_write(FX_MEDIA *m, ULONG64 sector,
                                             UCHAR *buffer, ULONG count, UINT kind) {
  ULONG64 primary;
  memcpy(&primary,buffer,sizeof(primary));
  assert(count==1 && kind==FX_FAT_SECTOR);
  assert(sector==primary+m->fx_media_sectors_per_FAT);
  assert(writes<8); copied[writes++]=(ULONG)primary;
  return fail_write ? 99 : 0;
}
''' + routine + r'''
int main(void) {
  UCHAR buffer[512];
  FX_MEDIA m={.fx_media_reserved_sectors=32, .fx_media_number_of_FATs=2,
              .fx_media_memory_buffer=buffer};
  /* Representative 64/128 GB FATs with 32 KiB clusters. */
  for (ULONG sectors=16000;sectors<=32000;sectors+=16000) {
    m.fx_media_sectors_per_FAT=sectors;
    assert(sectors<=FX_FAT_MAP_SIZE*8U);
    reads=writes=0;
    m.fx_media_fat_secondary_update_map[125/8] |= 1U<<(125%8);
    m.fx_media_fat_secondary_update_map[(sectors-1)/8] |= 1U<<((sectors-1)%8);
    assert(_fx_utility_FAT_map_flush(&m)==0);
    assert(reads==2 && writes==2);
    assert(copied[0]==32+125 && copied[1]==32+sectors-1);
    assert(_fx_utility_FAT_map_flush(&m)==0 && writes==2);
  }
  reads=writes=0; fail_write=1;
  m.fx_media_fat_secondary_update_map[3]=1;
  assert(_fx_utility_FAT_map_flush(&m)==99);
  assert(m.fx_media_fat_secondary_update_map[3]==1);
  fail_write=0;
  assert(_fx_utility_FAT_map_flush(&m)==0);
  assert(writes==2 && m.fx_media_fat_secondary_update_map[3]==0);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / 'fat-mirror'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-I', str(ROOT / 'Core/Inc'), '-x', 'c', '-',
                            '-o', str(binary)], input=program, text=True, check=True)
            subprocess.run([str(binary)], check=True)
