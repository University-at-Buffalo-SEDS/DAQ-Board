from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
class AcquisitionClockTests(unittest.TestCase):
    def test_clock_extrapolation_invalidation_expiry_and_wrap(self):
        code=r'''#include "daq_clock_cache.h"
#include <assert.h>
int main(void) {
 const uint64_t utc=1791227300000ULL;
 daq_clock_cache_t c=daq_clock_cache_make(utc,100);
 assert(daq_clock_cache_read(c,100)==utc);
 assert(daq_clock_cache_read(c,150)==utc+50);
 assert(daq_clock_cache_read(c,5100)==utc+5000);
 assert(daq_clock_cache_read(c,5101)==0);
 c=daq_clock_cache_make(utc,UINT32_MAX-20);
 assert(daq_clock_cache_read(c,10)==utc+31);
 c=daq_clock_cache_make(1234,10);
 assert(daq_clock_cache_read(c,11)==0);
 c=daq_clock_cache_make(4354819200000ULL,10);
 assert(daq_clock_cache_read(c,11)==0);
 c=daq_clock_cache_make(utc-1000,10);
 assert(daq_clock_cache_read(c,11)==utc-999);
}'''
        with tempfile.TemporaryDirectory() as directory:
            exe=Path(directory)/'test'
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(exe)],input=code,text=True,check=True)
            subprocess.run([str(exe)],check=True,timeout=5)

    def test_acquisition_never_queries_router_clock(self):
        s=(ROOT/'Core/Src/daq_thread.c').read_text()
        drain=s.split('static uint16_t daq_drain_ext_adc',1)[1].split('static void daq_publish_loadcell',1)[0]
        self.assertIn('telemetry_unix_ms_cached()',drain)
        self.assertNotIn('telemetry_unix_ms()',drain)
        s=(ROOT/'Core/Src/telemetry.c').read_text()
        read=s.split('uint64_t telemetry_unix_ms_cached(void)',1)[1].split('uint64_t telemetry_unix_ms(void)',1)[0]
        self.assertNotIn('seds_router_',read)
        self.assertNotIn('tx_mutex_',read)
        self.assertIn('__set_PRIMASK(mask)',read)
