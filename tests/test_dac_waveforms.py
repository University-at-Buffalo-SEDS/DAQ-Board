"""DAC waveform spec (docs/dac-waveforms.md): deterministic table generation,
golden CRC vectors shared with the GroundStation, timer/table-length
selection, chirp retuning, the chunked table upload with CRC, buffer limits,
and the production glue for uploads and chirps on register mocks."""
import pathlib
import tempfile
import unittest

from test_dac_validation import GLUE, ROOT, STUBS, run_c

SOURCES = [ROOT / 'Core/Src/daq_dac_control.c', ROOT / 'Core/Src/daq_dac_wave.c']
INCLUDES = [ROOT / 'Core/Inc']

# Golden vectors: (name, waveform, flags, amplitude, offset, frequency, phase,
# duty, steps, dwell_ms, frequency_stop, sweep_ms) -> (samples, psc, arr, CRC).
# GroundStation26 backend/src/daq_dac/waveform.rs asserts the same values.
GOLDEN = [
    ('sine10', 'SINE', 0, 1.0, 1.6, 10.0, 0, 32768, 0, 0, 0.0, 0, 256, 0, 62499, 0x81CDD413),
    ('sine123p90', 'SINE', 0, 1.0, 1.6, 123.4, 16384, 32768, 0, 0, 0.0, 0, 236, 0, 5493, 0xAF1D0167),
    ('square50d25', 'SQUARE', 0, 1.0, 1.6, 50.0, 0, 16384, 0, 0, 0.0, 0, 256, 0, 12499, 0x4C829B77),
    ('tri1', 'TRIANGLE', 0, 1.0, 1.6, 1.0, 0, 32768, 0, 0, 0.0, 0, 256, 9, 62499, 0x8BF0BFDD),
    ('saw7777', 'SAWTOOTH', 0, 1.0, 1.6, 7777.0, 0, 32768, 0, 0, 0.0, 0, 18, 0, 1142, 0xF74D8A08),
    ('sawdown2p5', 'SAWTOOTH', 1, 1.0, 1.6, 2.5, 0, 32768, 0, 0, 0.0, 0, 256, 3, 62499, 0xA444C715),
    ('stair11', 'STAIRCASE', 0, 1.0, 1.6, 0.0, 0, 32768, 11, 500, 0.0, 0, 11, 1249, 63999, 0x43675E77),
    ('noise1k', 'NOISE', 0, 0.5, 1.6, 1000.0, 0, 32768, 0, 0, 0.0, 0, 256, 3, 39999, 0x806C5080),
    ('chirp10to1k', 'CHIRP', 0, 1.0, 1.6, 10.0, 0, 32768, 0, 0, 1000.0, 10000, 200, 1, 39999, 0x70F8CF62),
    ('dc1p25', 'DC', 0, 0.0, 1.25, 0.0, 0, 32768, 0, 0, 0.0, 0, 1, 0, 0, 0x2F22AB05),
]

HEADER = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "daq_dac_control.h"
#define CLK 160000000U
static daq_dac_command_t cmd(uint8_t w,float a,float o,float f){
  daq_dac_command_t c={.channel=0,.enable=1,.waveform=w,.amplitude_v=a,.offset_v=o,.frequency_hz=f,
    .lease_ms=3000,.duty_q16=32768,.sequence=1};return c;}
'''


class SpecTests(unittest.TestCase):
    def test_golden_vectors(self):
        rows = []
        for (name, wave, flags, amp, off, freq, phase, duty, steps, dwell, fstop, sweep,
             n, psc, arr, crc) in GOLDEN:
            rows.append(
                f'  {{daq_dac_command_t k=cmd(DAQ_DAC_WAVE_{wave},{amp}f,{off}f,{freq}f);k.flags={flags};'
                f'k.phase_q16={phase};k.duty_q16={duty};k.steps={steps};k.dwell_ms={dwell};'
                f'k.frequency_stop_hz={fstop}f;k.sweep_ms={sweep};daq_dac_clamp(&k,&c);'
                f'assert(daq_dac_plan(&c,CLK,&p,t,NULL)==0);'
                f'if(p.samples!={n}u||p.psc!={psc}u||p.arr!={arr}u||p.table_crc!={crc:#x}u){{'
                f'printf("{name}: %u %u %u %08X\\n",p.samples,p.psc,p.arr,p.table_crc);return 1;}}}}')
        run_c(self, HEADER + 'int main(void){daq_dac_config_t c;daq_dac_plan_t p;uint16_t t[DAQ_DAC_ARB_LEN_MAX];\n'
              + '\n'.join(rows) + '\n puts("ok");return 0;}', INCLUDES, SOURCES)

    def test_sine_is_accurate_and_tables_are_bounded(self):
        run_c(self, HEADER + r'''
int main(void){
  /* The polynomial sine is within 1e-11 of libm everywhere (one code is 2.4e-4). */
  double worst=0;
  for(int i=-20000;i<=20000;i++){double x=i/7919.0;double e=fabs(daq_dac_wave_sin_turns(x)-sin(2*3.14159265358979323846*x));if(e>worst)worst=e;}
  assert(worst<1e-11);
  assert(daq_dac_wave_sin_turns(0)==0.0&&fabs(daq_dac_wave_sin_turns(0.25)-1.0)<1e-11&&daq_dac_wave_sin_turns(0.75)==-daq_dac_wave_sin_turns(0.25));
  /* Every built-in table stays inside offset +/- amplitude and the DAC's linear range. */
  uint16_t t[DAQ_DAC_ARB_LEN_MAX]; daq_dac_config_t c; daq_dac_plan_t p;
  const uint8_t w[]={DAQ_DAC_WAVE_SQUARE,DAQ_DAC_WAVE_SINE,DAQ_DAC_WAVE_TRIANGLE,DAQ_DAC_WAVE_SAWTOOTH,
                     DAQ_DAC_WAVE_NOISE,DAQ_DAC_WAVE_CHIRP,DAQ_DAC_WAVE_STAIRCASE};
  for(unsigned i=0;i<sizeof w;i++){
    daq_dac_command_t k=cmd(w[i],9.0f,1.65f,100.0f);k.steps=256;k.dwell_ms=10;k.frequency_stop_hz=900;k.sweep_ms=1000;
    daq_dac_clamp(&k,&c);assert(daq_dac_plan(&c,CLK,&p,t,NULL)==0);
    for(unsigned j=0;j<p.samples;j++)assert(t[j]>=DAQ_DAC_CODE_LO&&t[j]<=DAQ_DAC_CODE_HI);}
  assert(daq_dac_wave_volts_to_code(DAQ_DAC_MIN_V)==DAQ_DAC_CODE_LO&&daq_dac_wave_volts_to_code(DAQ_DAC_MAX_V)==DAQ_DAC_CODE_HI);
  puts("ok");return 0;}''', INCLUDES, SOURCES)

    def test_shapes_duty_phase_ramp_direction_and_noise(self):
        run_c(self, HEADER + r'''
int main(void){
  uint16_t t[DAQ_DAC_ARB_LEN_MAX]; daq_dac_wave_params_t p={.waveform=DAQ_DAC_WAVE_SQUARE,.duty_q16=16384,.amplitude_v=1.0f,.offset_v=1.6f};
  /* Square at 25 % duty on 64 samples: 16 high, 48 low. */
  assert(!daq_dac_wave_fill(&p,64,t));unsigned high=0;for(int i=0;i<64;i++)high+=t[i]>2048;assert(high==16&&t[0]>2048&&t[16]<2048);
  /* Quarter-turn phase moves a sine's peak to sample 0. */
  p.waveform=DAQ_DAC_WAVE_SINE;p.phase_q16=16384;assert(!daq_dac_wave_fill(&p,64,t));
  assert(t[0]==daq_dac_wave_volts_to_code(2.6f)&&t[32]==daq_dac_wave_volts_to_code((double)1.6f-1.0));
  /* Sawtooth rises from the minimum; ramp-down falls from the maximum. */
  p.phase_q16=0;p.waveform=DAQ_DAC_WAVE_SAWTOOTH;assert(!daq_dac_wave_fill(&p,32,t));
  for(int i=1;i<32;i++){assert(t[i]>t[i-1]);}
  assert(t[0]==daq_dac_wave_volts_to_code((double)1.6f-1.0));
  p.flags=DAQ_DAC_FLAG_RAMP_DOWN;assert(!daq_dac_wave_fill(&p,32,t));
  for(int i=1;i<32;i++){assert(t[i]<t[i-1]);}
  assert(t[0]==daq_dac_wave_volts_to_code((double)1.6f+1.0));
  /* Noise: fixed seed, so the sequence repeats exactly; it spans the range with a near-zero mean. */
  p.flags=0;p.waveform=DAQ_DAC_WAVE_NOISE;uint16_t u[256];assert(!daq_dac_wave_fill(&p,256,t)&&!daq_dac_wave_fill(&p,256,u));
  assert(!memcmp(t,u,sizeof u));double sum=0;uint16_t lo=4095,hi=0;
  for(int i=0;i<256;i++){sum+=daq_dac_wave_code_to_volts(t[i]);if(t[i]<lo)lo=t[i];if(t[i]>hi)hi=t[i];}
  assert(fabs(sum/256-1.6)<0.1&&lo<daq_dac_wave_volts_to_code(0.8)&&hi>daq_dac_wave_volts_to_code(2.4));
  /* An arbitrary waveform has no built-in table. */
  p.waveform=DAQ_DAC_WAVE_ARBITRARY;assert(daq_dac_wave_fill(&p,256,t)==-1);
  puts("ok");return 0;}''', INCLUDES, SOURCES)

    def test_rate_selection_minimises_frequency_error(self):
        out = run_c(self, HEADER + r'''
int main(void){
  daq_dac_rate_t r; double worst=0,worst_f=0;
  /* Log sweep 0.01 Hz .. 10 kHz: every request is plannable, within limits,
   * and within 10 ppm up to 500 Hz. Reports the worst error. */
  for(double f=0.01;f<=10000.0;f*=1.0137){
    assert(daq_dac_wave_choose(CLK,f,16,256,&r)==0);
    assert(r.samples>=16&&r.samples<=256&&!(r.samples&1)&&r.update_hz<=200000.0&&r.update_hz>=0.05);
    assert(fabs((double)CLK/((r.psc+1.0)*(r.arr+1.0))-r.update_hz)<1e-9*r.update_hz);
    double e=fabs(r.frequency_hz-f)/f; if(f<=500.0) assert(e<=1e-5);
    if(e>worst){worst=e;worst_f=f;}}
  printf("worst %.3g at %.4g Hz\n",worst,worst_f); assert(worst<5e-4);
  /* The choice is no worse than any single length (brute force). */
  const double fs[]={0.37,3.3,61.7,433.0,2718.0,9999.0};
  for(unsigned i=0;i<6;i++){assert(!daq_dac_wave_choose(CLK,fs[i],16,256,&r));double best=1e9;
    for(unsigned n=16;n<=256;n+=2){double rate=fs[i]*n;if(rate>200000.0)continue;uint16_t a,b;
      double got=daq_dac_wave_split(CLK,rate,&a,&b);if(got>0&&fabs(got/n-fs[i])<best)best=fabs(got/n-fs[i]);}
    assert(fabs(r.frequency_hz-fs[i])<=fmax(best,fs[i]*1e-5)+1e-12);}
  /* Longest table within tolerance: 10 Hz uses all 256 samples, exactly. */
  assert(!daq_dac_wave_choose(CLK,10.0,16,256,&r)&&r.samples==256&&r.frequency_hz==10.0);
  /* Outside the update-rate window nothing is plannable. */
  uint16_t a,b; assert(daq_dac_wave_split(CLK,200001.0,&a,&b)==0&&daq_dac_wave_split(CLK,0.04,&a,&b)==0);
  assert(daq_dac_wave_choose(CLK,20000.0,16,256,&r)==-1);
  /* Uploaded tables of any allowed length plan through the lone split. */
  assert(daq_dac_wave_split(CLK,1024*195.3125,&a,&b)==200000.0);
  puts("ok");return 0;}''', INCLUDES, SOURCES)
        self.assertIn('worst', out)

    def test_clamps_for_new_waveforms(self):
        run_c(self, HEADER + r'''
int main(void){
  daq_dac_config_t c; daq_dac_command_t k;
  k=cmd(DAQ_DAC_WAVE_SQUARE,0.5f,1.5f,10);k.duty_q16=0;assert(daq_dac_clamp(&k,&c)==DAQ_DAC_CLAMP_DUTY&&c.duty_q16==DAQ_DAC_DUTY_MIN_Q16);
  k.duty_q16=65535;daq_dac_clamp(&k,&c);assert(c.duty_q16==DAQ_DAC_DUTY_MAX_Q16);
  k=cmd(DAQ_DAC_WAVE_SINE,0.5f,1.5f,10);k.duty_q16=1;k.flags=3;k.phase_q16=99;
  assert(daq_dac_clamp(&k,&c)==0&&c.duty_q16==0&&c.flags==0&&c.phase_q16==99);
  k=cmd(DAQ_DAC_WAVE_SAWTOOTH,0.5f,1.5f,10);k.flags=3;daq_dac_clamp(&k,&c);assert(c.flags==DAQ_DAC_FLAG_RAMP_DOWN);
  k=cmd(DAQ_DAC_WAVE_CHIRP,0.5f,1.5f,0.001f);k.frequency_stop_hz=1e6f;k.sweep_ms=1;k.flags=3;
  assert(daq_dac_clamp(&k,&c)==(DAQ_DAC_CLAMP_FREQUENCY|DAQ_DAC_CLAMP_SWEEP));
  assert(c.frequency_hz==(float)DAQ_DAC_FREQ_MIN_HZ&&c.frequency_stop_hz==(float)DAQ_DAC_FREQ_MAX_HZ&&c.sweep_ms==DAQ_DAC_SWEEP_MIN_MS&&c.flags==DAQ_DAC_FLAG_LOG_SWEEP);
  k.sweep_ms=10000000;daq_dac_clamp(&k,&c);assert(c.sweep_ms==DAQ_DAC_SWEEP_MAX_MS);
  k=cmd(DAQ_DAC_WAVE_NOISE,0.5f,1.5f,1e7f);assert(daq_dac_clamp(&k,&c)==DAQ_DAC_CLAMP_FREQUENCY&&c.frequency_hz==200000.0f);
  /* Arbitrary: scaled like a built-in wave; update rate <= 200 kS/s. */
  k=cmd(DAQ_DAC_WAVE_ARBITRARY,3.0f,9.0f,1000.0f);k.table_len=512;k.table_crc=5;k.phase_q16=7;
  assert(daq_dac_clamp(&k,&c)==(DAQ_DAC_CLAMP_FREQUENCY|DAQ_DAC_CLAMP_OFFSET|DAQ_DAC_CLAMP_AMPLITUDE));
  assert(c.frequency_hz==390.625f&&c.amplitude_v==0&&c.offset_v==(float)DAQ_DAC_MAX_V&&c.table_len==512&&c.table_crc==5&&c.phase_q16==7);
  puts("ok");return 0;}''', INCLUDES, SOURCES)

    def test_chirp_retune(self):
        run_c(self, HEADER + r'''
int main(void){
  assert(daq_dac_wave_chirp_len(10,1000)==200&&daq_dac_wave_chirp_len(10000,1)==20&&daq_dac_wave_chirp_len(1,2)==256);
  /* Linear: halfway is the midpoint; the sweep repeats. */
  assert(fabs(daq_dac_wave_chirp_hz(100,1100,10000,0,5000)-600)<1e-9&&daq_dac_wave_chirp_hz(100,1100,10000,0,10000)==100);
  /* Log: halfway is the geometric mean; downward sweeps work too. */
  assert(fabs(daq_dac_wave_chirp_hz(10,1000,10000,DAQ_DAC_FLAG_LOG_SWEEP,5000)-100)<0.01);
  assert(fabs(daq_dac_wave_chirp_hz(1000,10,10000,0,2500)-752.5)<1e-9);
  daq_dac_config_t c; daq_dac_plan_t p; uint16_t t[DAQ_DAC_ARB_LEN_MAX];
  daq_dac_command_t k=cmd(DAQ_DAC_WAVE_CHIRP,1,1.6f,10);k.frequency_stop_hz=1000;k.sweep_ms=10000;daq_dac_clamp(&k,&c);
  assert(!daq_dac_plan(&c,CLK,&p,t,NULL)&&p.samples==200&&p.actual_frequency_hz==10.0f);
  uint16_t psc,arr;float f;
  for(uint32_t ms=0;ms<30000;ms+=DAQ_DAC_CHIRP_STEP_MS){assert(!daq_dac_chirp_retune(&c,p.samples,CLK,ms,&psc,&arr,&f));
    double want=daq_dac_wave_chirp_hz(10,1000,10000,0,ms);/* One timer count of resolution. */
    assert(fabs(f-want)<=want*(want*p.samples/CLK)+want*1e-6);
    assert(CLK/((psc+1.0)*(arr+1.0))<=200000.0);}
  k.waveform=DAQ_DAC_WAVE_SINE;daq_dac_clamp(&k,&c);assert(daq_dac_chirp_retune(&c,200,CLK,0,&psc,&arr,&f)==-1);
  puts("ok");return 0;}''', INCLUDES, SOURCES)


class UploadTests(unittest.TestCase):
    def test_golden_shape_vector(self):
        # A 64-sample triangle shape, amplitude 1 V, offset 1.6 V, 100 Hz, phase 90 deg.
        # GroundStation26 waveform.rs asserts the same CRCs.
        run_c(self, HEADER + r'''
static int16_t buf[DAQ_DAC_ARB_LEN_MAX];
int main(void){daq_dac_table_t t;daq_dac_table_init(&t,buf);
  for(int i=0;i<64;i++){double x=i/64.0;double y=x<0.5?-1+4*x:3-4*x;buf[i]=(int16_t)floor(32767.0*y+0.5);}
  t.state=DAQ_DAC_TABLE_READY;t.len=64;t.crc=daq_dac_wave_crc32((const uint16_t*)buf,64);assert(t.crc==0x79119F9Eu);
  daq_dac_command_t k=cmd(DAQ_DAC_WAVE_ARBITRARY,1.0f,1.6f,100.0f);k.phase_q16=16384;k.table_len=64;k.table_crc=t.crc;
  daq_dac_config_t c;daq_dac_plan_t p;uint16_t o[DAQ_DAC_ARB_LEN_MAX];daq_dac_clamp(&k,&c);
  assert(!daq_dac_plan(&c,CLK,&p,o,&t)&&p.samples==64&&p.psc==0&&p.arr==24999&&p.table_crc==0xA7B93781u);
  puts("ok");return 0;}''', INCLUDES, SOURCES)

    def test_chunked_upload_with_crc(self):
        run_c(self, HEADER + r'''
static int16_t buf[DAQ_DAC_ARB_LEN_MAX];
static daq_dac_table_t t;
static daq_dac_table_msg_t chunk(uint32_t id,uint16_t at,const int16_t *src,uint16_t len){
  daq_dac_table_msg_t m={.op=DAQ_DAC_TABLE_OP_DATA,.index=at,.upload_id=id};
  m.value=len-at<32?len-at:32;memcpy(m.samples,src+at,m.value*2);return m;}
int main(void){
  assert(DAQ_DAC_TABLE_BYTES==76&&DAQ_DAC_TABLE_CHUNKS_MAX==16);
  int16_t src[500];for(int i=0;i<500;i++)src[i]=(int16_t)(-32767+(i*997)%65535);
  const uint32_t crc=daq_dac_wave_crc32((const uint16_t*)src,500);
  /* Wire round trip, negative samples included. */
  {daq_dac_table_msg_t m=chunk(7,480,src,500),d;uint8_t b[76];daq_dac_table_encode(&m,b);
   assert(b[0]==1&&b[1]==2&&b[2]==(480&255)&&b[3]==(480>>8)&&b[4]==7&&b[8]==20);
   assert(!daq_dac_table_decode(b,76,&d)&&d.op==2&&d.index==480&&d.upload_id==7&&d.value==20&&!memcmp(d.samples,src+480,40));
   assert(daq_dac_table_decode(b,75,&d)==-1);b[0]=2;assert(daq_dac_table_decode(b,76,&d)==-1);b[0]=1;
   b[1]=9;assert(daq_dac_table_decode(b,76,&d)==-1);b[1]=2;b[8]=33;assert(daq_dac_table_decode(b,76,&d)==-1);
   b[8]=0;assert(daq_dac_table_decode(b,76,&d)==-1);}
  daq_dac_table_init(&t,buf);assert(t.state==DAQ_DAC_TABLE_EMPTY);
  /* Chunks before BEGIN are ignored. */
  {daq_dac_table_msg_t m=chunk(7,0,src,500);assert(daq_dac_table_data(&t,&m)==-1);}
  /* Bad lengths. */
  daq_dac_table_begin(&t,7,15,crc);assert(t.state==DAQ_DAC_TABLE_BAD_LENGTH);
  daq_dac_table_begin(&t,7,501,crc);assert(t.state==DAQ_DAC_TABLE_BAD_LENGTH);
  daq_dac_table_begin(&t,7,514,crc);assert(t.state==DAQ_DAC_TABLE_BAD_LENGTH);
  daq_dac_table_begin(&t,7,500,crc);assert(t.state==DAQ_DAC_TABLE_LOADING&&t.chunks==0);
  assert(daq_dac_table_full_mask(500)==0xFFFFu&&daq_dac_table_full_mask(16)==1&&daq_dac_table_full_mask(64)==3);
  /* Out of order, duplicated and one missing chunk. */
  for(int at=480;at>=0;at-=32){if(at==320)continue;daq_dac_table_msg_t m=chunk(7,(uint16_t)at,src,500);assert(!daq_dac_table_data(&t,&m));}
  {daq_dac_table_msg_t m=chunk(7,0,src,500);assert(!daq_dac_table_data(&t,&m));}
  /* Wrong id, misaligned, wrong count. */
  {daq_dac_table_msg_t m=chunk(8,320,src,500);assert(daq_dac_table_data(&t,&m)==-1);
   m=chunk(7,320,src,500);m.index=321;assert(daq_dac_table_data(&t,&m)==-1);
   m=chunk(7,320,src,500);m.value=31;assert(daq_dac_table_data(&t,&m)==-1);
   m=chunk(7,480,src,500);m.value=32;assert(daq_dac_table_data(&t,&m)==-1);}
  /* Commit while incomplete stays LOADING; the bitmap shows the gap. */
  assert(daq_dac_table_commit_begin(&t,7,500,crc)==0&&t.state==DAQ_DAC_TABLE_LOADING&&t.chunks==(0xFFFFu&~(1u<<10)));
  {daq_dac_table_msg_t m=chunk(7,320,src,500);assert(!daq_dac_table_data(&t,&m));}
  /* Commit with a different CRC or length than BEGIN is ignored. */
  assert(daq_dac_table_commit_begin(&t,7,500,crc^1)==0&&daq_dac_table_commit_begin(&t,7,498,crc)==0);
  assert(daq_dac_table_commit_begin(&t,7,500,crc)==1&&t.state==DAQ_DAC_TABLE_VERIFYING);
  /* No chunk lands while verifying. */
  {daq_dac_table_msg_t m=chunk(7,0,src,500);assert(daq_dac_table_data(&t,&m)==-1);}
  daq_dac_table_commit_finish(&t);assert(t.state==DAQ_DAC_TABLE_READY&&t.crc==crc&&!memcmp(buf,src,1000));
  /* Corruption (a station CRC that does not match the data) is caught. */
  daq_dac_table_begin(&t,9,32,0x12345678u);{daq_dac_table_msg_t m=chunk(9,0,src,32);assert(!daq_dac_table_data(&t,&m));}
  assert(daq_dac_table_commit_begin(&t,9,32,0x12345678u)==1);daq_dac_table_commit_finish(&t);
  assert(t.state==DAQ_DAC_TABLE_CRC_ERROR&&t.crc==daq_dac_wave_crc32((const uint16_t*)src,32));
  /* -32768 is outside -1..1 and poisons the upload. */
  daq_dac_table_begin(&t,10,32,0);{daq_dac_table_msg_t m=chunk(10,0,src,32);m.samples[5]=-32768;
   assert(daq_dac_table_data(&t,&m)==-1&&t.state==DAQ_DAC_TABLE_BAD_SAMPLE);
   m.samples[5]=-32767;assert(daq_dac_table_data(&t,&m)==-1);}
  daq_dac_table_clear(&t);assert(t.state==DAQ_DAC_TABLE_EMPTY&&t.samples==buf&&t.len==0);
  puts("ok");return 0;}''', INCLUDES, SOURCES)

    def test_shape_is_scaled_retimed_and_phased_like_a_builtin(self):
        run_c(self, HEADER + r'''
static int16_t buf[DAQ_DAC_ARB_LEN_MAX];
int main(void){
  daq_dac_table_t t; daq_dac_ctrl_t s; uint8_t a[2]; daq_dac_ctrl_init(&s); daq_dac_table_init(&t,buf);
  /* A 64-sample normalized sine shape. */
  for(int i=0;i<64;i++)buf[i]=(int16_t)floor(32767.0*daq_dac_wave_sin_turns(i/64.0)+0.5);
  const uint32_t shape_crc=daq_dac_wave_crc32((const uint16_t*)buf,64);
  daq_dac_command_t k=cmd(DAQ_DAC_WAVE_ARBITRARY,1.0f,1.6f,100);k.table_len=64;k.table_crc=shape_crc;
  /* No shape, or not verified: refused, nothing starts. */
  assert(daq_dac_ctrl_command(&s,&k,&t,true,1,0,a)==DAQ_DAC_RESULT_TABLE_MISMATCH&&a[0]==0&&!s.ch[0].active);
  assert(daq_dac_ctrl_command(&s,&k,NULL,true,1,0,a)==DAQ_DAC_RESULT_TABLE_MISMATCH);
  t.state=DAQ_DAC_TABLE_READY;t.len=64;t.crc=shape_crc;
  k.table_crc^=1;assert(daq_dac_ctrl_command(&s,&k,&t,true,1,0,a)==DAQ_DAC_RESULT_TABLE_MISMATCH);k.table_crc^=1;
  k.table_len=62;assert(daq_dac_ctrl_command(&s,&k,&t,true,1,0,a)==DAQ_DAC_RESULT_TABLE_MISMATCH);k.table_len=64;
  /* Matching length and CRC: plays, gated like every waveform. */
  assert(daq_dac_ctrl_command(&s,&k,&t,true,7,0,a)==DAQ_DAC_RESULT_REFUSED_STATE);
  assert(daq_dac_ctrl_command(&s,&k,&t,true,1,0,a)==DAQ_DAC_RESULT_OK&&a[0]==DAQ_DAC_ACTION_START);
  uint16_t p1[DAQ_DAC_ARB_LEN_MAX],p2[DAQ_DAC_ARB_LEN_MAX]; daq_dac_plan_t p,q;
  assert(!daq_dac_plan(&s.ch[0].config,CLK,&p,p1,&t));
  assert(p.samples==64&&p.actual_frequency_hz==100.0f&&p.update_hz==6400.0f&&p.table_crc==daq_dac_wave_crc32(p1,64));
  /* offset + amplitude x shape: the peak is 2.6 V, the trough 0.6 V. */
  assert(p1[16]==daq_dac_wave_volts_to_code((double)1.6f+1.0)&&p1[48]==daq_dac_wave_volts_to_code((double)1.6f-1.0));
  daq_dac_ctrl_started(&s,0,&p,true);
  /* Same shape at another amplitude, offset and frequency: rescaled and retimed. */
  daq_dac_command_t k2=k;k2.amplitude_v=0.25f;k2.offset_v=2.0f;k2.frequency_hz=1000;
  assert(daq_dac_ctrl_command(&s,&k2,&t,true,1,0,a)==DAQ_DAC_RESULT_OK&&a[0]==DAQ_DAC_ACTION_START);
  assert(!daq_dac_plan(&s.ch[0].config,CLK,&q,p2,&t)&&q.actual_frequency_hz==1000.0f&&q.update_hz==64000.0f);
  for(int i=0;i<64;i++)assert(p2[i]==daq_dac_wave_volts_to_code((double)2.0f+(double)0.25f*(buf[i]/32767.0)));
  /* Quarter-turn phase rotates the shape by n/4. */
  k2.phase_q16=16384;daq_dac_config_t c;daq_dac_clamp(&k2,&c);assert(!daq_dac_plan(&c,CLK,&q,p1,&t));
  for(int i=0;i<64;i++)assert(p1[i]==p2[(i+16)%64]);
  /* Amplitude clamps to the headroom, so a shape in -1..1 never leaves the range. */
  k2.amplitude_v=5;k2.offset_v=3.0f;daq_dac_clamp(&k2,&c);assert(c.clamp_flags&DAQ_DAC_CLAMP_AMPLITUDE);
  assert(!daq_dac_plan(&c,CLK,&q,p1,&t));for(int i=0;i<64;i++)assert(p1[i]>=DAQ_DAC_CODE_LO&&p1[i]<=DAQ_DAC_CODE_HI);
  /* Update rate stays <= 200 kS/s: frequency clamps to 200000 / 64. */
  k2.frequency_hz=9000;daq_dac_clamp(&k2,&c);assert(c.frequency_hz==3125.0f&&(c.clamp_flags&DAQ_DAC_CLAMP_FREQUENCY));
  /* A plan never scales an unverified or different shape. */
  t.state=DAQ_DAC_TABLE_LOADING;assert(daq_dac_plan(&s.ch[0].config,CLK,&q,p1,&t)==-1);t.state=DAQ_DAC_TABLE_READY;
  assert(daq_dac_plan(&s.ch[0].config,CLK,&q,p1,NULL)==-1);
  /* Unknown flag bits are refused. */
  {daq_dac_command_t bad=k;bad.flags=0x80;assert(daq_dac_ctrl_command(&s,&bad,&t,true,1,0,a)==DAQ_DAC_RESULT_BAD_REQUEST&&s.ch[0].active);}
  /* Status carries the table block and the per-channel CRC of the play table. */
  t.upload_id=0x11223344u;t.expected_crc=0xAABBCCDDu;t.chunks=3;uint8_t st[DAQ_DAC_STATUS_BYTES];
  daq_dac_status_encode(&s,&t,1,0,1,5,NULL,NULL,st);
  const uint8_t *c0=st+8,*tb=st+8+2*52;
  assert(c0[1]==DAQ_DAC_WAVE_ARBITRARY&&(c0[46]|(c0[47]<<8))==64);
  assert(tb[0]==DAQ_DAC_TABLE_READY&&tb[2]==64&&tb[4]==0x44&&tb[8]==0xDD&&tb[16]==3&&tb[17]==0);
  puts("ok");return 0;}''', INCLUDES, SOURCES)


GLUE_UPLOAD = GLUE[:GLUE.index('int main(void){')] + r'''
static void table_op(uint8_t op,uint16_t index,uint32_t id,uint32_t value,const int16_t *src){
  daq_dac_table_msg_t m={.op=op,.index=index,.upload_id=id,.value=value};
  if(src)memcpy(m.samples,src+index,value*2);
  uint8_t b[76];daq_dac_table_encode(&m,b);daq_dac_submit_table(b,76);}
static uint32_t u32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static void arb(uint8_t ch,float amp,float off,float f,uint32_t crc,uint32_t seq){
  daq_dac_command_t k={.channel=ch,.enable=1,.waveform=DAQ_DAC_WAVE_ARBITRARY,.amplitude_v=amp,.offset_v=off,
    .frequency_hz=f,.lease_ms=10000,.table_len=96,.table_crc=crc,.sequence=seq};
  uint8_t b[48];daq_dac_command_encode(&k,b);daq_dac_submit_command(b,48);}
int main(void){
  assert(daq_dac_init()==TX_SUCCESS); tick();
  int16_t src[96];for(int i=0;i<96;i++)src[i]=(int16_t)(-32767+i*689);
  const uint32_t crc=daq_dac_wave_crc32((const uint16_t*)src,96);
  const uint8_t *tb=last_status+8+2*52;
  table_op(DAQ_DAC_TABLE_OP_BEGIN,96,42,crc,NULL); tick();
  assert(tb[0]==DAQ_DAC_TABLE_LOADING&&u32(tb+4)==42);
  table_op(DAQ_DAC_TABLE_OP_DATA,0,42,32,src); table_op(DAQ_DAC_TABLE_OP_DATA,64,42,32,src); tick();
  assert(tb[16]==5);
  table_op(DAQ_DAC_TABLE_OP_COMMIT,96,42,crc,NULL); tick(); assert(tb[0]==DAQ_DAC_TABLE_LOADING);
  table_op(DAQ_DAC_TABLE_OP_DATA,32,42,32,src); table_op(DAQ_DAC_TABLE_OP_COMMIT,96,42,crc,NULL); tick();
  assert(tb[0]==DAQ_DAC_TABLE_READY&&u32(tb+12)==crc&&tb[16]==7);
  /* Play it on channel 1: DMA reads 96 scaled half-words; the first code is preloaded. */
  arb(1,1.0f,1.6f,50,crc,5); tick();
  assert(on(1)&&dma_on(1)&&tim_on(1)&&mock_dma[1].CBR1==96*2);
  assert(mock_dac.DOR2==daq_dac_wave_volts_to_code((double)1.6f+(double)1.0f*(-32767/32767.0)));
  /* Rescale without any upload: a new amplitude restarts from the same shape. */
  arb(1,0.5f,2.0f,100,crc,6); tick();
  assert(on(1)&&mock_dac.DOR2==daq_dac_wave_volts_to_code((double)2.0f-(double)0.5f)&&(mock_tim[1].PSC+1)*(mock_tim[1].ARR+1)==16667u);
  /* A new BEGIN does not disturb the running output (it plays its own copy). */
  table_op(DAQ_DAC_TABLE_OP_BEGIN,64,43,0,NULL); arb(1,0.5f,2.0f,100,crc,6); tick();
  assert(on(1)&&dma_on(1)&&tb[0]==DAQ_DAC_TABLE_LOADING);
  /* ...but a new configuration of the replaced shape is refused. */
  arb(1,0.4f,2.0f,100,crc,7); tick(); assert(on(1)&&last_status[60+2]==DAQ_DAC_RESULT_TABLE_MISMATCH);
  /* Chirp on channel 0: the service retunes TIM7 as the sweep advances. */
  {daq_dac_command_t k={.channel=0,.enable=1,.waveform=DAQ_DAC_WAVE_CHIRP,.amplitude_v=0.5f,.offset_v=1.5f,
     .frequency_hz=10,.frequency_stop_hz=1000,.sweep_ms=2000,.lease_ms=10000,.sequence=8};
   uint8_t b[48];daq_dac_command_encode(&k,b);daq_dac_submit_command(b,48);
   tick(); assert(on(0)&&tim_on(0)&&(mock_tim[0].CR1&TIM_CR1_ARPE));
   const uint32_t first=(mock_tim[0].PSC+1)*(mock_tim[0].ARR+1);
   for(int i=0;i<20;i++){daq_dac_submit_command(b,48);tick();}
   const uint32_t later=(mock_tim[0].PSC+1)*(mock_tim[0].ARR+1);
   assert(later<first/10);
   float f;memcpy(&f,last_status+8+16,4);assert(f>400.0f&&f<600.0f);}
  /* Malformed table payloads are counted. */
  {extern volatile uint32_t g_daq_dac_table_rejected;uint8_t junk[75]={1};uint32_t before=g_daq_dac_table_rejected;
   daq_dac_submit_table(junk,75);assert(g_daq_dac_table_rejected==before+1);}
  assert(!mock_primask);
  puts("ok");return 0;}
'''


class GlueTests(unittest.TestCase):
    def test_critical_sections_leave_the_acquisition_interrupts_alone(self):
        # DAC state is thread-only: BASEPRI masks priorities >= 1 so the
        # priority-0 TIM2/SPI2/GPDMA acquisition ISRs never wait on the DAC.
        source = (ROOT / 'Core/Src/daq_dac.c').read_text()
        self.assertNotIn('__disable_irq', source)
        self.assertNotIn('PRIMASK', source)
        self.assertIn('__set_BASEPRI_MAX(1U << (8U - __NVIC_PRIO_BITS))', source)
        msp = (ROOT / 'Core/Src/stm32u5xx_hal_msp.c').read_text()
        self.assertIn('HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0)', msp)
        # The DAC uses no interrupt and none of the acquisition resources.
        self.assertNotRegex(source, r'HAL_NVIC|IRQHandler|TIM2->|GPDMA1_Channel[0-3]\b')
        main = (ROOT / 'Core/Src/main.c').read_text()
        for resource in ('TIM7', 'TIM15', 'GPDMA1_Channel10', 'GPDMA1_Channel11'):
            self.assertNotIn(resource, main)


    def test_upload_and_chirp_on_register_mocks(self):
        with tempfile.TemporaryDirectory() as d:
            stub = pathlib.Path(d)
            for name, text in STUBS.items():
                (stub / name).write_text(text)
            src = stub / 'daq_dac_glue.c'
            src.write_text('#define TELEMETRY_ENABLED 1\n' + (ROOT / 'Core/Src/daq_dac.c').read_text())
            run_c(self, '#define TELEMETRY_ENABLED 1\n' + GLUE_UPLOAD, [stub, *INCLUDES], [src, *SOURCES])

    def test_wiring(self):
        telemetry = (ROOT / 'Core/Src/telemetry.c').read_text()
        handler = telemetry[telemetry.index('static SedsResult sd_card_endpoint_receive'):]
        handler = handler[:handler.index('\n}\n')]
        table = handler[handler.index('SEDS_DT_DAQ_DAC_TABLE'):]
        self.assertIn('"GS", 2U', table[:400])
        self.assertIn('daq_dac_submit_table(packet->payload, packet->payload_len)', table[:400])
        cmake = (ROOT / 'CMakeLists.txt').read_text()
        self.assertIn('Core/Src/daq_dac_wave.c', cmake)
        doc = (ROOT / 'docs/dac-waveforms.md').read_text()
        for name, *_rest, crc in GOLDEN:
            self.assertIn(f'{crc:08X}', doc, name)
        self.assertIn('79119F9E', doc)
        self.assertIn('A7B93781', doc)


if __name__ == '__main__':
    unittest.main()
