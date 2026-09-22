"""Exercise production record allocation, alignment, bounds and release."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class CompactPoolTest(unittest.TestCase):
    def test_variable_records_are_aligned_and_released_without_leaks(self):
        source = (ROOT / 'Core/Src/sd_card.c').read_text()
        types = source[source.index('typedef struct'):source.index('static TX_QUEUE')]
        helpers = source[source.index('static void *sd_pool_allocate'):
                         source.index('/* Every queued batch/row')]
        code = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
typedef struct {float coefficients[17];} daq_calibration_t;
typedef struct {uint64_t stamp; uint32_t words[8];} sd_raw_adc_record_t;
typedef struct {size_t used, limit;} TX_BYTE_POOL;
enum {TX_SUCCESS=0, TX_NO_WAIT=0, TX_WAIT_FOREVER=1};
static TX_BYTE_POOL g_sd_line_pool={0,4096},g_sd_raw_pool={0,4096};
static int g_sd_pool_mutex;
static unsigned g_sd_line_slots_used,g_sd_line_slots_peak;
static unsigned g_sd_raw_slots_used,g_sd_raw_slots_peak;
static size_t last_requested;
typedef struct {void *base,*returned; size_t bytes; TX_BYTE_POOL *pool;} Allocation;
static Allocation allocations[128];
static int tx_mutex_get(int *m,int wait) {(void)m;(void)wait;return 0;}
static int tx_mutex_put(int *m) {(void)m;return 0;}
static int tx_byte_allocate(TX_BYTE_POOL *pool,void **p,size_t bytes,int wait) {
  assert(wait==TX_NO_WAIT); last_requested=bytes;
  if(pool->used+bytes>pool->limit) return 1;
  for(unsigned i=0;i<128;++i) if(!allocations[i].base) {
    void *base=malloc(bytes+4); assert(base);
    *p=(char *)base+4; /* Deliberately not eight-byte aligned. */
    allocations[i]=(Allocation){base,*p,bytes,pool};
    pool->used+=bytes; return 0;
  }
  abort();
}
static int tx_byte_release(void *p) {
  for(unsigned i=0;i<128;++i) if(allocations[i].returned==p) {
    Allocation *a=&allocations[i]; a->pool->used-=a->bytes;
    free(a->base); memset(a,0,sizeof(*a)); return 0;
  }
  abort();
}
''' + types + helpers + r'''
int main(void) {
  for(unsigned round=0;round<1000;++round) {
    sd_raw_slot_t *raw[32]; sd_line_slot_t *lines[32];
    unsigned nr=0,nl=0;
    for(;nr<32;++nr) {
      unsigned count=1+nr%12;
      raw[nr]=sd_alloc_raw_slot(count);
      if(!raw[nr]) break;
      assert(((uintptr_t)raw[nr]&7U)==0);
      assert(last_requested==offsetof(sd_raw_slot_t,samples)+count*sizeof(sd_raw_adc_record_t)+sizeof(void *)+7);
      raw[nr]->count=count;
      memset(raw[nr]->samples,0x5a,count*sizeof(sd_raw_adc_record_t));
    }
    for(;nl<32;++nl) {
      unsigned len=40+nl%7;
      lines[nl]=sd_alloc_slot(len);
      if(!lines[nl]) break;
      assert(((uintptr_t)lines[nl]&7U)==0);
      assert(last_requested==offsetof(sd_line_slot_t,line)+len+1+sizeof(void *)+7);
      lines[nl]->len=len;
      memset(lines[nl]->line,'A',len); lines[nl]->line[len]=0;
    }
    assert(nr>0 && nr<32 && nl>0 && nl<32);
    assert(g_sd_raw_slots_used==nr && g_sd_line_slots_used==nl);
    for(unsigned i=0;i<nr;++i) {
      unsigned char *p=(unsigned char *)raw[i]->samples;
      for(size_t j=0;j<raw[i]->count*sizeof(sd_raw_adc_record_t);++j) assert(p[j]==0x5a);
      sd_free_raw_slot(raw[i]);
    }
    for(unsigned i=0;i<nl;++i) {
      assert(strlen(lines[i]->line)==lines[i]->len); sd_free_slot(lines[i]);
    }
    assert(!g_sd_raw_slots_used && !g_sd_line_slots_used);
    assert(!g_sd_raw_pool.used && !g_sd_line_pool.used);
  }
  assert(g_sd_raw_slots_peak>0 && g_sd_line_slots_peak>0);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / 'compact-pools'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=address,undefined', '-x', 'c', '-',
                            '-o', str(binary)], input=code, text=True, check=True)
            subprocess.run([str(binary)], check=True)
