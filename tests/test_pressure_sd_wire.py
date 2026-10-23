"""Execute the bounded source-timed 250 Hz pressure batch codec."""
from pathlib import Path
import subprocess,tempfile,unittest,json
ROOT=Path(__file__).resolve().parents[1]
class PressureSdWireTests(unittest.TestCase):
    def test_high_rate_type_is_daq_sd_only_and_does_not_raise_ui_rate(self):
        schema=json.loads((ROOT/'config/sedsnet.json').read_text())
        batch=next(t for t in schema['types'] if t['name']=='VALVE_PRESSURE_SD_V1')
        endpoint=next(e for e in schema['endpoints'] if e['rust']==batch['endpoints'][0])
        self.assertEqual(batch['endpoints'],['FillValvePressureLog'])
        self.assertEqual(endpoint['name'],'VALVE_PRESSURE_LOG')
        self.assertEqual(batch['element'],{'kind':'Static','data_type':'UInt8','count':44})
        self.assertEqual(batch['reliable'],'Unordered')
        self.assertTrue(all(t['endpoints']!=['FillValvePressureLog'] for t in schema['types'] if t['name']!='VALVE_PRESSURE_SD_V1'))

    def test_exact_wire_roundtrip_wrap_and_rejected_inputs(self):
        source=r'''
#include "pressure_sd_wire.h"
#include <assert.h>
int main(void){
 pressure_sd_batch_t a={.uptime_ms=4294967297ULL,.unix_ms=1792627000000ULL,
 .first_sequence=UINT32_MAX-1,.raw={0,1,3103,4095},.psi={0,0.25f,1999,2000}};
 uint8_t wire[PRESSURE_SD_BYTES+1];wire[PRESSURE_SD_BYTES]=0xa5;
 assert(pressure_sd_encode(wire,PRESSURE_SD_BYTES,&a)==44);
 assert(wire[0]==1 && wire[4]==1 && wire[20]==0 && wire[26]==1);
 assert(wire[44]==0xa5);
 pressure_sd_batch_t b={0};assert(pressure_sd_decode(&b,wire,44)==0);
 assert(b.uptime_ms==a.uptime_ms && b.unix_ms==a.unix_ms);
 assert(b.first_sequence+3U==1 && b.raw[3]==4095 && b.psi[1]==0.25f);
 assert(b.uptime_ms+3*PRESSURE_SD_PERIOD_MS==4294967309ULL);
 for(size_t n=0;n<44;++n)assert(pressure_sd_decode(&b,wire,n)==-1);
 assert(pressure_sd_decode(&b,wire,45)==-1);
 pressure_sd_batch_t before=b;wire[20]=255;wire[21]=255;
 assert(pressure_sd_decode(&b,wire,44)==-1 && !memcmp(&before,&b,sizeof(b)));
 a.psi[0]=NAN;assert(pressure_sd_encode(wire,44,&a)==-1);
 a.psi[0]=0;a.uptime_ms=UINT64_MAX;assert(pressure_sd_encode(wire,44,&a)==-1);
 assert(pressure_sd_encode(wire,43,&before)==-1);
 return 0;
}'''
        with tempfile.TemporaryDirectory() as d:
            exe=Path(d)/'pressure-wire'
            subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror','-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(exe)],input=source,text=True,check=True)
            subprocess.run([str(exe)],check=True)
