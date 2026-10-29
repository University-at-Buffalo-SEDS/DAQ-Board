import json
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

class ChamberPressureTests(unittest.TestCase):
    def compile_run(self, source, defines=(), valid=True):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp)/'test'
            result = subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I',str(ROOT/'Core/Inc'),
                *['-D'+x for x in defines],'-x','c','-','-lm','-o',str(binary)],input=source,text=True,capture_output=True)
            if not valid:
                self.assertNotEqual(result.returncode,0); return
            self.assertEqual(result.returncode,0,result.stderr)
            subprocess.run([str(binary)],check=True)

    def test_exact_omega_model_voltage_conversion_and_unassigned_default(self):
        self.compile_run('''#include "daq_chamber_pressure.h"
#include <assert.h>
int main(void) {
 assert(DAQ_CHAMBER_SCAN_MASK==0); assert(!daq_chamber_is_channel(6));
 assert(daq_chamber_pressure_psi(0)==0);
 assert(fabsf(daq_chamber_pressure_psi(5)-1000)<0.01f);
 assert(fabsf(daq_chamber_pressure_psi(2.5)-500)<0.01f);
 assert(daq_chamber_pressure_psi(5.1f)>1000); assert(isnan(daq_chamber_pressure_psi(NAN)));
 assert(daq_chamber_pressure_psi(-0.01f)<0); }
''')

    def test_fast_pin_binding_requires_explicit_compatible_acquisition_budget(self):
        source='''#include "daq_rates.h"
#include "mcp3564r_board_config.h"
#include <assert.h>
int main(void) { assert(DAQ_ADC_READ_INTERVAL_US<=180); assert(MCP3564R_BOARD_SCAN==(3U|DAQ_CHAMBER_SCAN_MASK));
 assert(!(MCP3564R_AUX_SCAN&DAQ_CHAMBER_SCAN_MASK));
 assert(daq_chamber_is_channel(DAQ_CHAMBER_ADC_CHANNEL)); }
'''
        for pin in (6,7):
            self.compile_run(source,[f'DAQ_CHAMBER_ADC_CHANNEL={pin}','DAQ_ADC_READ_RATE_HZ=5556','DAQ_ADC_OSR=512'])
            self.compile_run(source,[f'DAQ_CHAMBER_ADC_CHANNEL={pin}'],valid=False)
        self.compile_run(source,['DAQ_CHAMBER_ADC_CHANNEL=5'],valid=False)

    def test_combined_publication_keeps_aggregate_slot_cadence_and_control_isolation(self):
        self.compile_run('''#include "daq_network_schedule.h"
#include <assert.h>
int main(void) { daq_network_schedule_t s={0}; unsigned counts[DAQ_NETWORK_STREAMS]={0};
 for(unsigned now=0; now<10000; ++now) {
   for(unsigned k=0;k<DAQ_NETWORK_STREAMS;++k) {
     float v[]={1,1000,1.25}; assert(daq_network_schedule_offer(&s,k,k,v,k==DAQ_NETWORK_KG1000?3:1,now)>=0);
   }
   unsigned stream; daq_network_sample_t sample;
   if(daq_network_schedule_take(&s,now,&stream,&sample)) {counts[stream]++;if(stream==DAQ_NETWORK_KG1000) assert(sample.count==3);}
 }
 unsigned sum=0;for(unsigned i=0;i<DAQ_NETWORK_STREAMS;++i)sum+=counts[i];
 assert(sum==1000); assert(counts[DAQ_NETWORK_KG1000]==400); assert(counts[DAQ_NETWORK_KG50]==400);
}''')

    def test_raw_sd_path_precedes_network_and_keeps_both_clocks_and_raw_code(self):
        source=(ROOT/'Core/Src/daq_thread.c').read_text()
        self.assertLess(source.index('sd_card_enqueue_raw_adc_samples(raw_records'),source.index('daq_enqueue_report(&report)'))
        self.assertIn('records[count].raw_adc_code = sample.code',source)
        self.assertIn('records[count].network_unix_ms',source)
        self.assertIn('records[count].monotonic_ms',source)
        self.assertIn('daq_chamber_pressure_psi(corrected)',source)
        logger=(ROOT/'Core/Src/sd_card.c').read_text()
        self.assertIn('"chamber_pressure_raw_volts_psi"',logger)
        self.assertIn('Retain this exact record and batch',logger)

    def test_wire_contract_is_additive_and_fixed(self):
        d=json.loads((ROOT/'config/sedsnet.json').read_text())
        entry=next(x for x in d['types'] if x['name']=='DAQ_LOADCELL_CHAMBER')
        self.assertEqual(entry['element']['count'],3)
        self.assertEqual(entry['priority'],16)
        self.assertEqual(entry['reliable'],'Off')
        kg=next(x for x in d['types'] if x['name']=='KG1000')
        self.assertEqual(kg['element']['count'],1)

if __name__=='__main__':unittest.main()
