"""Exercise production firmware smoothing, without changing raw acquisition."""
import pathlib
import subprocess
import tempfile
import unittest
ROOT = pathlib.Path(__file__).resolve().parents[1]
class FilterTests(unittest.TestCase):
    def test_noise_steps_and_fault_recovery(self):
        source = r'''
#include <assert.h>
#include "daq_filter.h"
int main(void) {
 daq_filter_t f={0};
 assert(daq_filter_add(&f,0,0,100)==0);
 assert(fabsf(daq_filter_add(&f,1,100,100)-0.5f)<1e-6f);
 for(unsigned i=2;i<100;i++) daq_filter_add(&f,1,i*100,100);
 assert(fabsf(f.value-1)<1e-6f);
 assert(daq_filter_add(&f,5,20000,100)==5);
 assert(isnan(daq_filter_add(&f,NAN,20100,100)));
 assert(daq_filter_add(&f,6,20200,100)==6);
 assert(daq_filter_add(&f,8,20300,0)==8);
 daq_filter_t n={0}; float power=0;
 for(unsigned i=0;i<10000;i++) {
   float x=daq_filter_add(&n,2+(i%2?0.01f:-0.01f),i*2,100);
   if(i>1000) power+=(x-2)*(x-2);
 }
 assert(power/8999 < 0.000001f);
 f=(daq_filter_t){.value=0,.timestamp_ms=0xfffffff0,.valid=1};
 assert(fabsf(daq_filter_add(&f,1,0x10,32)-0.5f)<1e-6f);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary=pathlib.Path(tmp)/'filter'
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(binary)],input=source,text=True,check=True)
            subprocess.run([str(binary)],check=True)
