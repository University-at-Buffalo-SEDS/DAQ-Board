import subprocess
import tempfile
import unittest
import re
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]

class TlsfAllocatorTests(unittest.TestCase):
    def test_board_adapter_churn_exhaustion_and_recovery(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)
            (p/'main.h').write_text('''#include <stdint.h>
extern uint32_t mock_irq;
static inline uint32_t __get_PRIMASK(void){return mock_irq;}
static inline void __disable_irq(void){mock_irq=1;}
static inline void __set_PRIMASK(uint32_t m){mock_irq=m;}
void Error_Handler(void);
''')
            (p/'tx_api.h').write_text('''#ifndef MOCK_TX_H
#define MOCK_TX_H
#include <stddef.h>
typedef unsigned UINT;
typedef unsigned long ULONG;
typedef struct { unsigned char *base; size_t used, bytes; } TX_BYTE_POOL;
#define TX_SUCCESS 0
#define TX_NO_MEMORY 1
#define TX_NO_WAIT 0
#define TX_NULL NULL
UINT tx_byte_allocate(TX_BYTE_POOL*,void**,ULONG,UINT);
UINT tx_byte_pool_info_get(TX_BYTE_POOL*,void*,ULONG*,ULONG*,void*,void*,void*);
#endif
''')
            telemetry = (ROOT/'Core/Src/telemetry.c').read_text()
            start = telemetry.index('static __attribute__((unused)) bool telemetry_memory_admission_probe(')
            end = telemetry.index('\n}\n', start) + 3
            registration = re.search(r'seds_set_memory_admission_probe(?:_v2)?\([^;]+\);', telemetry).group(0)
            adapter_code = r'''
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "telemetry_tlsf.h"
#define SEDS_MEMORY_WORK_SMALL_PROTECTED 2U
static bool (*registered_v2)(size_t,size_t,uint32_t);
static bool (*registered_legacy)(size_t,size_t);
static void seds_set_memory_admission_probe_v2(bool (*fn)(size_t,size_t,uint32_t)){
 registered_v2=fn;registered_legacy=NULL;
}
static void seds_set_memory_admission_probe(bool (*fn)(size_t,size_t)){
 registered_legacy=fn;registered_v2=NULL;
}
static bool registered_probe(size_t additional,size_t largest,uint32_t work){
 return registered_v2 ? registered_v2(additional,largest,work) : registered_legacy(additional,largest);
}
''' + telemetry[start:end] + '\nstatic void register_board_probe(void){' + registration + '}\n'
            (p/'test.c').write_text(adapter_code + r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "telemetry_tlsf.h"
#include "tlsf.h"
uint32_t mock_irq;
static unsigned calls, fail_init;
void Error_Handler(void){abort();}
UINT tx_byte_allocate(TX_BYTE_POOL*p,void**out,ULONG n,UINT wait){
 assert(wait==TX_NO_WAIT);calls++;
 size_t size=(n+7)&~(size_t)7;
 if(fail_init || size+16>p->bytes-p->used)return TX_NO_MEMORY;
 *out=p->base+p->used;p->used+=size+16;return TX_SUCCESS;
}
UINT tx_byte_pool_info_get(TX_BYTE_POOL*p,void*n,ULONG*a,ULONG*f,void*x,void*y,void*z){
 (void)n;(void)x;(void)y;(void)z;
 if(a)*a=p->bytes-p->used;if(f)*f=1;return TX_SUCCESS;
}
extern volatile uint32_t g_telemetry_tlsf_active,g_telemetry_tlsf_init_failed;
extern volatile uint32_t g_telemetry_tlsf_snapshot_count;
extern volatile uint32_t g_telemetry_tlsf_live_bytes,g_telemetry_tlsf_peak_bytes;
extern volatile uint32_t g_telemetry_tlsf_free_bytes,g_telemetry_tlsf_largest_free;
static uint64_t arenas[3][4096];
int main(int argc,char**argv){
 (void)argv;TX_BYTE_POOL pools[3];
 for(unsigned i=0;i<3;i++){pools[i]=(TX_BYTE_POOL){(unsigned char*)arenas[i],0,sizeof(arenas[i])};telemetry_tlsf_register_pool(&pools[i]);}
 if(argc>1){fail_init=1;assert(!telemetry_tlsf_malloc(32));assert(g_telemetry_tlsf_init_failed);assert(!g_telemetry_tlsf_active);assert(mock_irq==0);return 0;}
 void *first=telemetry_tlsf_malloc(0);assert(first&&g_telemetry_tlsf_active);telemetry_tlsf_free(first);
 assert(!telemetry_tlsf_malloc(SIZE_MAX));
 const uint32_t initial_free=g_telemetry_tlsf_free_bytes, initial_largest=g_telemetry_tlsf_largest_free;
 const unsigned startup_calls=calls;
 assert(telemetry_tlsf_admit(4096, 2048));
 assert(!telemetry_tlsf_admit(SIZE_MAX, 1));
 assert(!telemetry_tlsf_admit(0, SIZE_MAX));
 assert(mock_irq==0);
 {
  extern volatile uint32_t g_telemetry_tlsf_region_bytes;
  void *held_controls[1024];unsigned held_count=0;
  /* Reproduce the actuator's 5912-byte post-discovery admission refusal. */
  while(g_telemetry_tlsf_region_bytes > g_telemetry_tlsf_live_bytes +
        ((size_t)held_count+3U)*2U*sizeof(void*)+5912U){
   assert(held_count<1024);
   held_controls[held_count]=telemetry_tlsf_malloc(128);assert(held_controls[held_count++]);
  }
  assert(held_count>0);
  assert(!telemetry_tlsf_admit(2225,91));
  register_board_probe();
  assert(registered_probe(2225,91,SEDS_MEMORY_WORK_SMALL_PROTECTED));
  assert(!registered_probe(2225,91,0));
  assert(!registered_probe(2225,91,99));
  /* The former registration loses the protected class at this exact bound. */
  seds_set_memory_admission_probe(telemetry_tlsf_admit);
  assert(!registered_probe(2225,91,SEDS_MEMORY_WORK_SMALL_PROTECTED));
  register_board_probe();
  assert(telemetry_tlsf_admit_small_protected(2225,91));
  assert(!telemetry_tlsf_admit_small_protected(4096,91));
  assert(!telemetry_tlsf_admit_small_protected(2225,0));
  void *work=telemetry_tlsf_malloc(2225);assert(work);telemetry_tlsf_free(work);
  assert(telemetry_tlsf_admit(0,0));
  for(unsigned i=0;i<held_count;i++)telemetry_tlsf_free(held_controls[i]);
 }
 void *items[64]={0};size_t sizes[64]={0};uint32_t rng=42;
 for(unsigned step=0;step<100000;step++){
  rng=rng*1664525U+1013904223U;unsigned i=(rng>>16)%64;
  if(items[i]){
   for(unsigned j=0;j<sizes[i];j++)assert(((unsigned char*)items[i])[j]==i+1);
   telemetry_tlsf_free(items[i]);items[i]=0;
  }else{
   sizes[i]=1+rng%8192;items[i]=telemetry_tlsf_malloc(sizes[i]);
   if(items[i]){assert(((uintptr_t)items[i]&7)==0);memset(items[i],i+1,sizes[i]);}
  }
  assert(mock_irq==0);
  {
   const uint32_t walks=g_telemetry_tlsf_snapshot_count;
   const uint32_t live=g_telemetry_tlsf_live_bytes;
   size_t request=512U+rng%8192U;
   if (telemetry_tlsf_admit(request,request)) {
    void *scratch=telemetry_tlsf_malloc(request);
    assert(scratch);telemetry_tlsf_free(scratch);
   }
   assert(g_telemetry_tlsf_snapshot_count==walks);
   assert(g_telemetry_tlsf_live_bytes==live);
  }
 }
 for(unsigned i=0;i<64;i++)telemetry_tlsf_free(items[i]);
 assert(g_telemetry_tlsf_live_bytes==0&&g_telemetry_tlsf_peak_bytes>0);
 assert(!telemetry_tlsf_malloc(SIZE_MAX));
 assert(g_telemetry_tlsf_free_bytes==initial_free&&g_telemetry_tlsf_largest_free==initial_largest);
 // Exhaust then release every block; freed memory must be reusable.
 void*full[128];unsigned count=0;
 while(count<128&&(full[count]=telemetry_tlsf_malloc(1024)))count++;
 assert(count>0&&count<128);
 assert(!telemetry_tlsf_admit(4096, 2048));
 for(unsigned i=0;i<count;i++)telemetry_tlsf_free(full[i]);
 assert(telemetry_tlsf_admit(4096, 2048));
 mock_irq=1;assert(telemetry_tlsf_admit(4096,2048)&&mock_irq==1);first=telemetry_tlsf_malloc(4112);assert(first&&mock_irq==1);telemetry_tlsf_free(first);assert(mock_irq==1);mock_irq=0;
 assert(calls==startup_calls);telemetry_tlsf_free(NULL);
 assert(g_telemetry_tlsf_live_bytes==0);
}
''')
            exe=str(p/'test')
            subprocess.run(['cc','-std=c11','-g','-fsanitize=address,undefined','-I',tmp,'-I',str(ROOT/'Core/Inc'),'-I',str(ROOT/'third_party/tlsf'),str(p/'test.c'),str(ROOT/'Core/Src/telemetry_tlsf.c'),str(ROOT/'third_party/tlsf/tlsf.c'),'-o',exe],check=True)
            subprocess.run([exe],check=True)
            subprocess.run([exe,'init-failure'],check=True)
