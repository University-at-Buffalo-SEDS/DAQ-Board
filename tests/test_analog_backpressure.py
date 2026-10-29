"""Analog offers cannot block behind a busy network or allocate retry queues."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class AnalogBackpressureTests(unittest.TestCase):
    def test_latest_network_offers_preserve_all_independent_analog_streams(self):
        source = (ROOT / 'Core/Src/daq_thread.c').read_text()
        start = source.index('static void daq_publish_analog_packet(')
        end = source.index('static void daq_publish_analog(', start)
        harness = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include "daq_network_schedule.h"
#define SEDS_DT_DAQ_SDADC_VOLTAGES 0
#define SEDS_DT_DAQ_SAR_VOLTAGES 1
#define SEDS_DT_DAQ_CURRENT_SENSE 2
#define SEDS_DT_DAQ_POWER_MONITOR 3
static daq_network_schedule_t state;
static unsigned calls,coalesced[7];
static void daq_network_offer(unsigned stream,uint32_t type,const float *values,size_t count,uint64_t time){
 int replaced=daq_network_schedule_offer(&state,stream,type,values,count,time);
 assert(replaced>=0);coalesced[stream]+=replaced;calls++;
}
''' + source[start:end] + r'''
int main(void){
 float values[8]={0};
 const uint64_t captured=UINT64_C(0x100000123);
 for(unsigned n=0;n<3;n++)for(unsigned type=0;type<4;type++){
   values[0]=(float)(10*n+type);daq_publish_analog_packet(type,values,8,captured+n);
 }
 assert(calls==12);
 for(unsigned type=0;type<4;type++){
   const unsigned stream=DAQ_NETWORK_SDADC+type;
   assert(coalesced[stream]==2 && state.latest[stream].pending);
   assert(state.latest[stream].values[0]==(float)(20+type));
   assert(state.latest[stream].captured_ms==captured+2 && state.latest[stream].type==type);
 }
}
'''
        with tempfile.TemporaryDirectory() as directory:
            exe=Path(directory)/'test'
            subprocess.run(['cc','-DTELEMETRY_ENABLED=1','-std=c11','-Wall','-Wextra','-Werror',
                            '-fsanitize=address,undefined','-I',str(ROOT/'Core/Inc'),
                            '-x','c','-','-o',str(exe)], input=harness,text=True,check=True)
            subprocess.run([str(exe)],check=True)
