"""Exercise finite production fixed pools: alignment, capacity and ownership."""
import pathlib
import re
import shutil
import subprocess
import tempfile
import unittest
from sd_fixed_pool_mock import TYPES, FUNCTIONS

ROOT = pathlib.Path(__file__).resolve().parents[1]


class CompactPoolTest(unittest.TestCase):
    def test_fixed_records_exhaust_and_reuse_the_actual_bounded_budgets(self):
        source = (ROOT / 'Core/Src/sd_card.c').read_text()
        types = source[source.index('typedef struct'):source.index('static TX_QUEUE')]
        record = (ROOT / 'Core/Inc/sd_card.h').read_text().split('typedef struct', 1)[1].split('sd_raw_adc_record_t;', 1)[0]
        record_definition = 'typedef struct' + record + 'sd_raw_adc_record_t;\n'
        helpers = source[source.index('static void *sd_pool_allocate'):
                         source.index('/* Every queued batch/row')]
        code = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "daq_rates.h"
#include "daq_calibration.h"
#define SD_RAW_BATCH_MAX DAQ_RAW_BATCH_CAPACITY
#define SD_LINE_MAX 384U
enum {TX_SUCCESS=0,TX_NO_WAIT=0};
static unsigned irq_mask;
static uint32_t __get_PRIMASK(void){return irq_mask;}
static void __disable_irq(void){irq_mask=1;}
static void __set_PRIMASK(uint32_t value){irq_mask=value;}
''' + record_definition + TYPES + types + r'''
static TX_BLOCK_POOL g_sd_line_pool,g_sd_raw_pool;
static unsigned char raw_storage[332U*1024U+8],line_storage[62U*1024U+8];
static unsigned g_sd_line_slots_used,g_sd_line_slots_peak,g_sd_raw_slots_used,g_sd_raw_slots_peak;
''' + FUNCTIONS + helpers + r'''
int main(void){
 mock_pool_init(&g_sd_raw_pool,raw_storage,332U*1024U,SD_RAW_BLOCK_PAYLOAD+SD_POOL_ALIGNMENT_RESERVE);
 mock_pool_init(&g_sd_line_pool,line_storage,62U*1024U,SD_LINE_BLOCK_PAYLOAD+SD_POOL_ALIGNMENT_RESERVE);
 assert(g_sd_raw_pool.tx_block_pool_total==g_sd_raw_pool.budget/(g_sd_raw_pool.tx_block_pool_block_size+sizeof(void*)));
 assert(g_sd_line_pool.tx_block_pool_total==g_sd_line_pool.budget/(g_sd_line_pool.tx_block_pool_block_size+sizeof(void*)));
 for(unsigned round=0;round<100;round++){
  sd_raw_slot_t *raw[1024];sd_line_slot_t *lines[1024];unsigned nr=0,nl=0;
  while((raw[nr]=sd_alloc_raw_slot((uint16_t)(1+nr%SD_RAW_SLOT_CAPACITY)))!=NULL){
   assert(nr<1024&&((uintptr_t)raw[nr]&7U)==0);
   raw[nr]->count=1+nr%SD_RAW_SLOT_CAPACITY;
   memset(raw[nr]->samples,0x5a,raw[nr]->count*sizeof(sd_raw_adc_record_t));++nr;
  }
  while((lines[nl]=sd_alloc_slot((uint16_t)(40+nl%300)))!=NULL){
   assert(nl<1024&&((uintptr_t)lines[nl]&7U)==0);lines[nl]->len=40+nl%300;
   memset(lines[nl]->line,'A',lines[nl]->len);lines[nl]->line[lines[nl]->len]=0;++nl;
  }
  assert(nr==g_sd_raw_pool.tx_block_pool_total&&nl==g_sd_line_pool.tx_block_pool_total);
  assert(g_sd_raw_slots_used==nr&&g_sd_line_slots_used==nl);
  assert(mock_pool_used_bytes(&g_sd_raw_pool)<=332U*1024U&&mock_pool_used_bytes(&g_sd_line_pool)<=62U*1024U);
  for(unsigned i=0;i<nr;i++){
   for(size_t j=0;j<raw[i]->count*sizeof(sd_raw_adc_record_t);j++)assert(((unsigned char*)raw[i]->samples)[j]==0x5a);
   sd_free_raw_slot(raw[i]);
  }
  for(unsigned i=0;i<nl;i++){assert(strlen(lines[i]->line)==lines[i]->len);sd_free_slot(lines[i]);}
  assert(!g_sd_raw_slots_used&&!g_sd_line_slots_used);
  assert(g_sd_raw_pool.tx_block_pool_available==nr&&g_sd_line_pool.tx_block_pool_available==nl);
 }
 assert(!sd_alloc_raw_slot(SD_RAW_SLOT_CAPACITY+1));
 assert(!sd_alloc_slot((uint16_t)g_sd_line_pool.tx_block_pool_block_size));
 printf("finite pools:raw%zu blocks/%zu bytes/%zu records;scalar%zu blocks/%zu bytes/%zu records;100 exhaustion/reuse rounds\n",
  g_sd_raw_pool.tx_block_pool_total,g_sd_raw_pool.budget,g_sd_raw_pool.tx_block_pool_total*SD_RAW_SLOT_CAPACITY,
  g_sd_line_pool.tx_block_pool_total,g_sd_line_pool.budget,g_sd_line_pool.tx_block_pool_total*SD_SCALAR_BATCH_MAX);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / 'compact-pools'
            subprocess.run(['cc', '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=address,undefined', '-I', str(ROOT / 'Core/Inc'),
                            '-x', 'c', '-', '-o', str(binary)], input=code, text=True, check=True)
            subprocess.run([str(binary)], check=True)

    def test_arm_pool_geometry_includes_alignment_and_threadx_link_overhead(self):
        compiler = shutil.which('arm-none-eabi-gcc')
        if compiler is None:
            self.skipTest('ARM compiler unavailable; native finite-pool contracts still run')
        source = (ROOT / 'Core/Src/sd_card.c').read_text()
        types = source[source.index('typedef struct'):source.index('static TX_QUEUE')]
        record = (ROOT / 'Core/Inc/sd_card.h').read_text().split('typedef struct', 1)[1].split('sd_raw_adc_record_t;', 1)[0]
        code = ("#include <stdint.h>\n#include <stddef.h>\n#include \"daq_rates.h\"\n#include \"daq_calibration.h\"\n"
                "#define SD_RAW_BATCH_MAX DAQ_RAW_BATCH_CAPACITY\n#define SD_LINE_MAX 384U\n"
                + 'typedef struct' + record + 'sd_raw_adc_record_t;\n' + types + r"""
#define ROUND_WORD(x) (((x)+sizeof(void*)-1)/sizeof(void*)*sizeof(void*))
unsigned char target_raw_blocks[(332U*1024U)/(ROUND_WORD(SD_RAW_BLOCK_PAYLOAD+SD_POOL_ALIGNMENT_RESERVE)+sizeof(void*))];
unsigned char target_line_blocks[(62U*1024U)/(ROUND_WORD(SD_LINE_BLOCK_PAYLOAD+SD_POOL_ALIGNMENT_RESERVE)+sizeof(void*))];
unsigned char target_raw_block_bytes[ROUND_WORD(SD_RAW_BLOCK_PAYLOAD+SD_POOL_ALIGNMENT_RESERVE)+sizeof(void*)];
unsigned char target_line_block_bytes[ROUND_WORD(SD_LINE_BLOCK_PAYLOAD+SD_POOL_ALIGNMENT_RESERVE)+sizeof(void*)];
unsigned char target_raw_records_per_block[SD_RAW_SLOT_CAPACITY];
unsigned char target_scalar_records_per_block[SD_SCALAR_BATCH_MAX];
""")
        with tempfile.TemporaryDirectory(prefix='daq-pool-geometry-') as directory:
            obj = pathlib.Path(directory) / 'geometry.o'
            subprocess.run([compiler, '-mcpu=cortex-m33', '-mthumb', '-std=c11', '-ffreestanding',
                            '-I', str(ROOT / 'Core/Inc'), '-x', 'c', '-', '-c', '-o', str(obj)],
                           input=code, text=True, check=True)
            output = subprocess.check_output([compiler.replace('gcc', 'nm'), '-S', '--radix=d', str(obj)], text=True)
        sizes = {}
        for line in output.splitlines():
            match = re.match(r'\d+\s+(\d+)\s+\w\s+(target_\w+)$', line)
            if match:
                sizes[match[2]] = int(match[1])
        raw_records = sizes['target_raw_blocks'] * sizes['target_raw_records_per_block']
        scalar_records = sizes['target_line_blocks'] * sizes['target_scalar_records_per_block']
        self.assertLessEqual(sizes['target_raw_blocks'] * sizes['target_raw_block_bytes'], 332 * 1024)
        self.assertLessEqual(sizes['target_line_blocks'] * sizes['target_line_block_bytes'], 62 * 1024)
        self.assertGreaterEqual(raw_records, 2500 * 3 + sizes['target_raw_records_per_block'])
        self.assertGreaterEqual(scalar_records, 500 * 3 + sizes['target_scalar_records_per_block'])
        print(f"ARM fixed pools: raw={sizes['target_raw_blocks']} blocks/{raw_records} records, "
              f"scalar={sizes['target_line_blocks']} blocks/{scalar_records} records; "
              "332/62KiB includes alignment+ThreadX link overhead")
