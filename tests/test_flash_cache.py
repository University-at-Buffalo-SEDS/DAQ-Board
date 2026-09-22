"""Exercise production flash mutations with stale cache and partial failures."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FlashCacheTests(unittest.TestCase):
    def test_mutations_refresh_reads_before_verification(self):
        source = (ROOT / 'Bootloader/storage_internal_flash.c').read_text()
        functions = source[source.index('static bool flash_sync_reads('):
                           source.index('static launchcore_storage_status_t read_data(')]
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
typedef int HAL_StatusTypeDef;
typedef int launchcore_storage_status_t;
enum { HAL_OK, HAL_ERROR };
enum { LAUNCHCORE_STORAGE_OK, LAUNCHCORE_STORAGE_ERR_RANGE,
       LAUNCHCORE_STORAGE_ERR_ERASE, LAUNCHCORE_STORAGE_ERR_WRITE,
       LAUNCHCORE_STORAGE_ERR_VERIFY };
#define FLASH_PAGE_SIZE 8192U
#define FLASH_TYPEERASE_PAGES 0
#define FLASH_TYPEPROGRAM_QUADWORD 0
typedef struct { unsigned TypeErase, NbPages, Banks, Page; } FLASH_EraseInitTypeDef;
static unsigned dirty, invalidations, program_calls, erase_calls, locked;
static unsigned fail_program, fail_erase, fail_cache, verify_calls;
static void __DSB(void) {}
static void __ISB(void) {}
static bool writable(uint32_t a, uint32_t n) { (void)a; (void)n; return true; }
static void page(uint32_t a, uint32_t *b, uint32_t *p) { *b=1; *p=a/8192; }
static int HAL_FLASH_Unlock(void) { locked=0; return HAL_OK; }
static int HAL_FLASH_Lock(void) { locked=1; return HAL_OK; }
static int HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e, uint32_t *error) {
    (void)e; (void)error; ++erase_calls; dirty=1;
    return fail_erase ? HAL_ERROR : HAL_OK;
}
static int HAL_FLASH_Program(unsigned type, uint32_t a, uint32_t data) {
    (void)type; (void)a; (void)data; ++program_calls; dirty=1;
    return fail_program == program_calls ? HAL_ERROR : HAL_OK;
}
static int HAL_ICACHE_Invalidate(void) {
    assert(locked); ++invalidations;
    if (fail_cache) return HAL_ERROR;
    dirty=0; return HAL_OK;
}
/* Model stale CPU reads after a successful flash write. */
static int verify_flash(const void *a, const void *b, size_t n) {
    (void)a; (void)b; (void)n; ++verify_calls;
    return dirty ? 1 : 0;
}
#define memcmp verify_flash
''' + functions + r'''
int main(void) {
    unsigned char record[156]={0};
    assert(erase(0x081fa000,8192)==LAUNCHCORE_STORAGE_OK);
    assert(erase_calls==1 && invalidations==1 && !dirty);
    assert(write_data(0x081fa000,record,sizeof(record))==LAUNCHCORE_STORAGE_OK);
    assert(program_calls==10 && verify_calls==1 && invalidations==2);
    fail_program=program_calls+2;
    assert(write_data(0x081fa000,record,sizeof(record))==LAUNCHCORE_STORAGE_ERR_WRITE);
    assert(!dirty && invalidations==3 && verify_calls==1);
    fail_erase=1;
    assert(erase(0x081fa000,8192)==LAUNCHCORE_STORAGE_ERR_ERASE);
    assert(!dirty && invalidations==4);
    fail_cache=1;
    assert(write_data(0x081fa000,record,sizeof(record))==LAUNCHCORE_STORAGE_ERR_VERIFY);
    assert(verify_calls==1);
    fail_erase=0;
    assert(erase(0x081fa000,8192)==LAUNCHCORE_STORAGE_ERR_ERASE);
    assert(locked);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            exe = Path(tmp) / 'flash-cache-test'
            result = subprocess.run(
                ['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                 '-x', 'c', '-', '-o', str(exe)],
                input=code, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(exe)], check=True)
