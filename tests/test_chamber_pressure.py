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

    def test_exact_omega_model_voltage_conversion_and_hvain1_default(self):
        self.compile_run('''#include "daq_chamber_pressure.h"
#include <assert.h>
int main(void) {
 assert(DAQ_CHAMBER_ADC_CHANNEL==6); assert(DAQ_CHAMBER_SCAN_MASK==(1U<<6)); assert(daq_chamber_is_channel(6));
 assert(daq_chamber_pressure_psi(0)==0);
 assert(fabsf(daq_chamber_pressure_psi(5)-1000)<0.01f);
 assert(fabsf(daq_chamber_pressure_psi(2.5)-500)<0.01f);
 assert(daq_chamber_pressure_psi(5.1f)>1000); assert(isnan(daq_chamber_pressure_psi(NAN)));
 assert(daq_chamber_pressure_psi(-0.01f)==0); }
''')

    def test_strict_computed_4_5_floor_and_exact_outside_bits(self):
        self.compile_run(r'''#include "daq_chamber_pressure.h"
#include <assert.h>
#include <float.h>
#include <string.h>
static uint32_t bits(float v) {uint32_t b;memcpy(&b,&v,4);return b;}
static float original(float v) {return isfinite(v)?v*200.0f:NAN;}
int main(void) {
 assert(DAQ_CHAMBER_NOISE_FLOOR_MILLIPSI==4500);
 assert(daq_chamber_pressure_psi(0.015f)==0.0f); /* observed roughly3psi */
 assert(daq_chamber_pressure_psi(-0.015f)==0.0f);
 assert(daq_chamber_pressure_psi(-5.0f)==0.0f); /* intentional one-sided change */
 float exact=4.5f/200.0f;assert(original(exact)==4.5f);
 float below=nextafterf(exact,-INFINITY),above=nextafterf(exact,INFINITY);
 while(original(below)>=4.5f)below=nextafterf(below,-INFINITY);
 while(original(above)<=4.5f)above=nextafterf(above,INFINITY);
 assert(original(below)==nextafterf(4.5f,-INFINITY));
 assert(original(above)==nextafterf(4.5f,INFINITY));
 assert(bits(daq_chamber_pressure_psi(below))==bits(0.0f));
 assert(bits(daq_chamber_pressure_psi(exact))==bits(original(exact)));
 assert(bits(daq_chamber_pressure_psi(above))==bits(original(above)));
 const float high[]={0.023f,0.025f,0.1f,1,2.5f,5,5.1f,6,FLT_MAX,-FLT_MAX};
 for(unsigned i=0;i<sizeof(high)/sizeof(high[0]);++i)
   assert(bits(daq_chamber_pressure_psi(high[i]))==bits(original(high[i])));
 assert(isnan(daq_chamber_pressure_psi(NAN)));
 assert(isnan(daq_chamber_pressure_psi(INFINITY)));
 assert(isnan(daq_chamber_pressure_psi(-INFINITY)));
 assert(isinf(daq_chamber_pressure_psi(FLT_MAX)));
 assert(isinf(daq_chamber_pressure_psi(-FLT_MAX)));
 uint32_t state=13579;
 for(unsigned i=0;i<100000;++i) {
   state^=state<<13;state^=state>>17;state^=state<<5;
   float v;memcpy(&v,&state,4);const uint32_t raw=bits(v);
   float old=original(v),now=daq_chamber_pressure_psi(v);
   if(isfinite(old)&&old<4.5f)assert(bits(now)==bits(0.0f));
   else if(isnan(old))assert(isnan(now));else assert(bits(now)==bits(old));
   assert(bits(v)==raw);
 }
}''')

    def test_floor_override_is_bounded_and_zero_restores_negative_and_small_values(self):
        self.compile_run(r'''#include "daq_chamber_pressure.h"
#include <assert.h>
#include <string.h>
int main(void){const float values[]={-5,-0.015f,-0.0f,0,0.015f,0.0225f,2.5f,6};
 for(unsigned i=0;i<sizeof(values)/sizeof(values[0]);++i){float old=values[i]*200.0f,now=daq_chamber_pressure_psi(values[i]);assert(memcmp(&old,&now,4)==0);}}
''', ['DAQ_CHAMBER_NOISE_FLOOR_MILLIPSI=0'])
        self.compile_run('#include "daq_chamber_pressure.h"\n#include <assert.h>\nint main(void){assert(daq_chamber_pressure_psi(0.025f)==0);assert(daq_chamber_pressure_psi(0.05f)==10);}', ['DAQ_CHAMBER_NOISE_FLOOR_MILLIPSI=10000'])
        for bad in (-1,10001):
            self.compile_run('#include "daq_chamber_pressure.h"\nint main(void){return 0;}', [f'DAQ_CHAMBER_NOISE_FLOOR_MILLIPSI={bad}'], valid=False)

    def test_actual_production_drain_preserves_raw_and_other_channel_bits(self):
        # Reuse the existing actual production-drain fixture, then compare
        # default conditioning to its disabled predecessor on identical samples.
        import runpy
        from types import SimpleNamespace
        from unittest import mock
        captured=[]
        def capture(*args, **kwargs):
            if 'input' in kwargs:
                captured.append(kwargs['input'])
            return SimpleNamespace(returncode=0, stderr='')
        fixture=runpy.run_path(str(ROOT/'tests/test_kg50_acquisition.py'))
        with mock.patch('subprocess.run', side_effect=capture):
            fixture['Kg50AcquisitionTests']().test_interleaved_channels_preserve_raw_values_calibration_and_counts()
        self.assertEqual(len(captured),1)
        harness='#include <stdio.h>\n'+captured[0]
        harness=harness.replace('int main(void) {', 'static uint32_t bits(float v){uint32_t b;memcpy(&b,&v,4);return b;}\nint main(void) {',1)
        end=harness.rindex('\n}')
        harness=harness[:end]+r'''
  const unsigned channels[]={0,1,2,3,4,5,6,7,9};
  const float inputs[]={0.001f,-0.001f,0.01f,0.6f};
  next=queued_count=0;
  for(unsigned i=0;i<sizeof(channels)/sizeof(channels[0]);++i){
    for(unsigned j=0;j<sizeof(inputs)/sizeof(inputs[0]);++j){
      memset(records,0,sizeof(records));
      snapshot=(daq_snapshot_t){.ext_adc_sample_valid=1,.ext_adc_channel=channels[i],
        .ext_adc_monotonic_ms=90,.ext_adc_code=-1234,.ext_adc_voltage_v=inputs[j],
        .ext_adc_loadcell_kg1000=inputs[j],.ext_adc_temp_c=25};
      assert(daq_drain_ext_adc(&snapshot,records,4,&cal,&window)==1);
      assert(records[0].raw_adc_code==-1234);
      printf("%u %u %d %08x %08x %u %u %llu\n",channels[i],j,records[0].raw_adc_code,
        bits(records[0].raw_value),bits(records[0].calibrated_value),records[0].monotonic_ms,
        records[0].record_sequence,(unsigned long long)records[0].network_unix_ms);
    }
  }
  memset(records,0xa5,sizeof(records));sd_raw_adc_record_t before[4];memcpy(before,records,sizeof(before));
  snapshot=(daq_snapshot_t){0};
  assert(daq_drain_ext_adc(&snapshot,records,4,&cal,&window)==0);
  assert(memcmp(records,before,sizeof(records))==0); /* missing is not zero */
  snapshot=(daq_snapshot_t){.ext_adc_sample_valid=1,.ext_adc_channel=6,
    .ext_adc_code=-1234,.ext_adc_voltage_v=NAN,.ext_adc_loadcell_kg1000=NAN};
  assert(daq_drain_ext_adc(&snapshot,records,4,&cal,&window)==1);
  assert(records[0].raw_adc_code==-1234 && isnan(records[0].raw_value));
  assert(isnan(records[0].calibrated_value)); /* invalid is not zero */
'''+harness[end:]
        outputs=[]
        with tempfile.TemporaryDirectory() as tmp:
            for floor in (0,4500):
                binary=Path(tmp)/f'drain-{floor}'
                r=subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',
                    '-I',str(ROOT/'Core/Inc'),f'-DDAQ_CHAMBER_NOISE_FLOOR_MILLIPSI={floor}',
                    '-x','c','-','-lm','-o',str(binary)],input=harness,text=True,capture_output=True)
                self.assertEqual(r.returncode,0,r.stderr)
                outputs.append(subprocess.check_output([str(binary)],text=True).splitlines())
        self.assertEqual(len(outputs[0]),36)
        changed=0
        for old,new in zip(*outputs):
            a,b=old.split(),new.split()
            if a[0]=='6' and int(a[1])<2:
                self.assertNotEqual(a[4],b[4]);self.assertEqual(b[4],'00000000');changed+=1
                a[4]=b[4]
            self.assertEqual(a,b,'all raw fields and other channel bits must be identical')
        self.assertEqual(changed,2)

    def test_actual_report_prepare_never_fabricates_missing_stale_or_invalid_zero(self):
        source=(ROOT/'Core/Src/daq_thread.c').read_text()
        start=source.index('    for (unsigned channel = 6U; channel <= 7U; ++channel)')
        end=source.index('    float filtered;',start)
        actual=source[start:end]
        self.compile_run(r'''#include "daq_rates.h"
#include "daq_downsample.h"
#include "mcp3564r_board_config.h"
#include <assert.h>
#include <float.h>
#include <string.h>
static daq_downsample_t downsample_chamber;
static float chamber[2];static uint8_t chamber_seen;static uint32_t chamber_sample_ms;
static struct {float chamber[2];unsigned chamber_valid;} report;
static void prepare(uint32_t report_ms,unsigned count,float adc_v) {
 struct {unsigned count[10];float voltage_sum[10];} window={0};
 window.count[6]=count;window.voltage_sum[6]=adc_v*count;
 report.chamber_valid=0;report.chamber[0]=report.chamber[1]=NAN;
'''+actual+r'''
}
int main(void) {
 prepare(0,0,0);assert(!report.chamber_valid && isnan(report.chamber[0]));
 prepare(1,1,0.015f/11.0f);assert(!report.chamber_valid);
 const uint32_t fresh=1+DAQ_BROADCAST_PERIOD_MS;
 prepare(fresh,1,0.015f/11.0f);
 assert(report.chamber_valid && report.chamber[0]==0 && report.chamber[1]>0);
 const float raw=report.chamber[1];
 prepare(fresh+100,0,0);assert(report.chamber_valid && report.chamber[1]==raw);
 prepare(fresh+101,0,0);assert(!report.chamber_valid && isnan(report.chamber[0]));
 prepare(fresh+110,1,NAN);assert(!report.chamber_valid && isnan(report.chamber[0]));
 prepare(fresh+120,1,INFINITY);assert(!report.chamber_valid && isnan(report.chamber[0]));
 prepare(fresh+130,1,FLT_MAX);assert(!report.chamber_valid && isnan(report.chamber[0]));
 const float voltage=0.025f/11.0f;
 prepare(fresh+140,1,voltage);assert(report.chamber_valid);
 float old=mcp3564r_connector_voltage(6,voltage)*200.0f;
 assert(memcmp(&old,&report.chamber[0],sizeof(float))==0);
}
''')

    def test_fast_pin_binding_selects_compatible_defaults_and_rejects_unsafe_overrides(self):
        source='''#include "daq_rates.h"
#include "mcp3564r_board_config.h"
#include <assert.h>
int main(void) { assert(DAQ_ADC_READ_INTERVAL_US<=180); assert(MCP3564R_BOARD_SCAN==(3U|DAQ_CHAMBER_SCAN_MASK));
 assert(!(MCP3564R_AUX_SCAN&DAQ_CHAMBER_SCAN_MASK));
 assert(daq_chamber_is_channel(DAQ_CHAMBER_ADC_CHANNEL)); }
'''
        for pin in (6,7):
            self.compile_run(source,[f'DAQ_CHAMBER_ADC_CHANNEL={pin}','DAQ_ADC_READ_RATE_HZ=5556','DAQ_ADC_OSR=512'])
            self.compile_run(source,[f'DAQ_CHAMBER_ADC_CHANNEL={pin}'])
            self.compile_run(source,[f'DAQ_CHAMBER_ADC_CHANNEL={pin}','DAQ_ADC_OSR=1024'],valid=False)
        self.compile_run(source,['DAQ_CHAMBER_ADC_CHANNEL=5'],valid=False)

    def test_explicit_unassigned_opt_out_preserves_legacy_two_channel_profile(self):
        self.compile_run('#include "daq_rates.h"\n#include <assert.h>\nint main(void) {assert(DAQ_CHAMBER_SCAN_MASK==0); assert(!daq_chamber_is_channel(6)); assert(DAQ_ADC_OSR==1024); assert(DAQ_ADC_READ_RATE_HZ==3700);}', ['DAQ_CHAMBER_ADC_CHANNEL=-1'])

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
        self.assertLess(source.index('sd_card_enqueue_raw_adc_samples_captured(raw_records'),source.index('daq_enqueue_report(&report)'))
        self.assertIn('records[count].raw_adc_code = sample.code',source)
        self.assertIn('records[count].network_unix_ms',source)
        self.assertIn('records[count].monotonic_ms',source)
        self.assertIn('daq_chamber_pressure_psi(corrected)',source)
        logger=(ROOT/'Core/Src/sd_card.c').read_text()
        self.assertIn('"chamber_pressure_raw_volts_psi"',logger)
        self.assertIn('noise_floor_millipsi=%u',logger)
        self.assertIn('noise_floor_rule=finite_raw_psig<floor_millipsi/1000:0',logger)
        self.assertIn('(unsigned)DAQ_CHAMBER_NOISE_FLOOR_MILLIPSI',logger)
        self.assertIn('chamber_seen = isfinite(chamber[0])',source)
        self.assertIn('window.count[channel] != 0U',source)
        self.assertIn('(uint32_t)(report_ms - chamber_sample_ms) <= 100U',source)
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
