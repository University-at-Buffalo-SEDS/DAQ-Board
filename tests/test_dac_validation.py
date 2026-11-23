"""DAC signal generator: waveform maths, clamping, state gating, every auto-off
trigger, off at boot, status encoding and the production HAL glue on mocks."""
import json
import pathlib
import re
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
CC = ['cc', '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
      '-fsanitize=address,undefined', '-fno-sanitize-recover=all']


def run_c(test, code, include_dirs=(), sources=()):
    with tempfile.TemporaryDirectory() as d:
        main = pathlib.Path(d) / 'main_test.c'
        main.write_text(code)
        exe = pathlib.Path(d) / 'dac-test'
        args = CC + [f'-I{p}' for p in include_dirs] + [str(main), *map(str, sources), '-lm', '-o', str(exe)]
        built = subprocess.run(args, text=True, capture_output=True)
        test.assertEqual(built.returncode, 0, built.stderr)
        ran = subprocess.run([str(exe)], text=True, capture_output=True)
        test.assertEqual(ran.returncode, 0, ran.stdout + ran.stderr)
        return ran.stdout


CONTROL = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "daq_dac_control.h"
static float rdf(const uint8_t *p){uint32_t w=(uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);float f;memcpy(&f,&w,4);return f;}
static uint32_t rd32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static float v(uint32_t code){return daq_dac_code_to_volts(code);}
static int near(float a,float b,float tol){return fabsf(a-b)<=tol;}
#define LSB (3.3f/4095.0f)
static daq_dac_command_t cmd(uint8_t ch,uint8_t en,uint8_t wave,float amp,float off,float f){
  daq_dac_command_t c={.channel=ch,.enable=en,.waveform=wave,.amplitude_v=amp,.offset_v=off,
    .frequency_hz=f,.steps=8,.lease_ms=2000,.dwell_ms=500,.duty_q16=32768,.sequence=7};return c;}
'''


class WaveformAndClampTests(unittest.TestCase):
    def test_tables_codes_and_timer_plans(self):
        run_c(self, CONTROL + r'''
int main(void){
  uint16_t t[DAQ_DAC_ARB_LEN_MAX]; daq_dac_plan_t p; daq_dac_config_t c;
  /* Codes round to nearest and saturate at both ends (NaN is 0). */
  assert(daq_dac_volts_to_code(0.0f)==0&&daq_dac_volts_to_code(-1)==0&&daq_dac_volts_to_code(NAN)==0);
  assert(daq_dac_volts_to_code(3.3f)==4095&&daq_dac_volts_to_code(9)==4095);
  /* Table maths is double: 1.65 V is code 2048, the float 1.65f just below it. */
  assert(daq_dac_wave_volts_to_code(1.65)==2048&&daq_dac_volts_to_code(1.65f)==2047);
  /* Sine at 10 Hz: 256 samples x 10 Hz = 2560 S/s, exactly 62500 counts of 160 MHz. */
  daq_dac_command_t k=cmd(0,1,DAQ_DAC_WAVE_SINE,1.0f,1.6f,10.0f);
  daq_dac_clamp(&k,&c); assert(daq_dac_plan(&c,160000000U,&p,t,NULL)==0);
  assert(p.samples==256);
  assert(near(v(t[0]),1.6f,LSB)&&near(v(t[64]),2.6f,LSB)&&near(v(t[128]),1.6f,LSB)&&near(v(t[192]),0.6f,LSB));
  for(unsigned i=0;i<256;i++) assert(near(v(t[i]),1.6f+sinf(6.2831853f*i/256.0f),LSB));
  assert((p.psc+1u)*(p.arr+1u)==62500u&&p.actual_frequency_hz==10.0f&&p.table_crc==daq_dac_wave_crc32(t,256));
  /* Square: first half high at the default 50 % duty. */
  k.waveform=DAQ_DAC_WAVE_SQUARE; k.duty_q16=32768; daq_dac_clamp(&k,&c); assert(!daq_dac_plan(&c,160000000U,&p,t,NULL));
  for(unsigned i=0;i<256;i++) assert(near(v(t[i]),i<128?2.6f:0.6f,LSB));
  /* Triangle: minimum at 0, maximum at half period, linear in between. */
  k.waveform=DAQ_DAC_WAVE_TRIANGLE; daq_dac_clamp(&k,&c); assert(!daq_dac_plan(&c,160000000U,&p,t,NULL));
  assert(near(v(t[0]),0.6f,LSB)&&near(v(t[128]),2.6f,LSB)&&near(v(t[64]),1.6f,LSB)&&near(v(t[192]),1.6f,LSB));
  /* Staircase: steps from offset-amp to offset+amp, one sample per dwell. */
  k.waveform=DAQ_DAC_WAVE_STAIRCASE; k.steps=5; k.dwell_ms=250; daq_dac_clamp(&k,&c);
  assert(!daq_dac_plan(&c,160000000U,&p,t,NULL)&&p.samples==5);
  for(unsigned i=0;i<5;i++) assert(near(v(t[i]),0.6f+0.5f*i,LSB));
  assert(p.actual_dwell_ms==250&&p.actual_frequency_hz==0.0f&&near(p.update_hz,4.0f,1e-4f));
  /* DC: one sample, no timer. */
  k.waveform=DAQ_DAC_WAVE_DC; k.offset_v=1.0f; daq_dac_clamp(&k,&c);
  assert(!daq_dac_plan(&c,160000000U,&p,t,NULL)&&p.samples==1&&near(v(t[0]),1.0f,LSB)&&p.update_hz==0);
  /* Range ends: slowest and fastest periodic and the longest dwell fit 16-bit PSC/ARR. */
  float fs[]={0.01f,0.37f,1.0f,123.4f,1000.0f,7777.0f,10000.0f};
  for(unsigned i=0;i<7;i++){k=cmd(1,1,DAQ_DAC_WAVE_SINE,0.5f,1.5f,fs[i]);daq_dac_clamp(&k,&c);
    assert(!daq_dac_plan(&c,160000000U,&p,t,NULL));assert(fabsf(p.actual_frequency_hz-fs[i])<=fs[i]*1e-4f);
    assert(p.update_hz<=200000.0f&&p.samples>=16&&p.samples<=256&&p.samples%2==0);}
  k=cmd(1,1,DAQ_DAC_WAVE_STAIRCASE,0.5f,1.5f,0);k.steps=256;k.dwell_ms=20000;daq_dac_clamp(&k,&c);
  assert(!daq_dac_plan(&c,160000000U,&p,t,NULL)&&p.samples==256&&p.actual_dwell_ms==20000);
  puts("ok");return 0;}''', [ROOT / 'Core/Inc'], [ROOT / 'Core/Src/daq_dac_control.c', ROOT / 'Core/Src/daq_dac_wave.c'])

    def test_every_request_is_clamped_and_flagged(self):
        run_c(self, CONTROL + r'''
int main(void){
  daq_dac_config_t c; daq_dac_command_t k;
  k=cmd(0,1,DAQ_DAC_WAVE_DC,0,1.0f,0); assert(daq_dac_clamp(&k,&c)==0&&c.offset_v==1.0f&&c.amplitude_v==0&&c.lease_ms==2000);
  k.offset_v=-2; assert(daq_dac_clamp(&k,&c)==DAQ_DAC_CLAMP_OFFSET&&c.offset_v==(float)DAQ_DAC_MIN_V);
  k.offset_v=5; assert(daq_dac_clamp(&k,&c)==DAQ_DAC_CLAMP_OFFSET&&c.offset_v==(float)DAQ_DAC_MAX_V);
  /* Amplitude limited by the nearer rail; negative is zero. */
  k=cmd(0,1,DAQ_DAC_WAVE_SINE,2.0f,1.0f,1.0f); assert(daq_dac_clamp(&k,&c)==DAQ_DAC_CLAMP_AMPLITUDE&&near(c.amplitude_v,0.8f,1e-6f));
  k.offset_v=2.9f; daq_dac_clamp(&k,&c); assert(near(c.amplitude_v,0.2f,1e-6f));
  k.amplitude_v=-1; assert(daq_dac_clamp(&k,&c)==DAQ_DAC_CLAMP_AMPLITUDE&&c.amplitude_v==0);
  k=cmd(0,1,DAQ_DAC_WAVE_SQUARE,2.0f,9.0f,1.0f); assert(daq_dac_clamp(&k,&c)==(DAQ_DAC_CLAMP_OFFSET|DAQ_DAC_CLAMP_AMPLITUDE)&&c.amplitude_v==0);
  /* The output never leaves [min,max] for any wave. */
  for(int w=1;w<DAQ_DAC_WAVE_ARBITRARY;w++){k=cmd(0,1,w,5.0f,1.7f,5.0f);daq_dac_clamp(&k,&c);
    assert(c.offset_v-c.amplitude_v>=(float)DAQ_DAC_MIN_V-1e-6f&&c.offset_v+c.amplitude_v<=(float)DAQ_DAC_MAX_V+1e-6f);}
  k=cmd(0,1,DAQ_DAC_WAVE_TRIANGLE,0.1f,1.0f,0.0f); assert(daq_dac_clamp(&k,&c)==DAQ_DAC_CLAMP_FREQUENCY&&c.frequency_hz==(float)DAQ_DAC_FREQ_MIN_HZ);
  k.frequency_hz=1e6f; assert(daq_dac_clamp(&k,&c)==DAQ_DAC_CLAMP_FREQUENCY&&c.frequency_hz==(float)DAQ_DAC_FREQ_MAX_HZ);
  k=cmd(0,1,DAQ_DAC_WAVE_STAIRCASE,0.1f,1.0f,0); k.steps=1; k.dwell_ms=1;
  assert(daq_dac_clamp(&k,&c)==(DAQ_DAC_CLAMP_STEPS|DAQ_DAC_CLAMP_DWELL)&&c.steps==2&&c.dwell_ms==DAQ_DAC_DWELL_MIN_MS);
  k.steps=500; k.dwell_ms=99999; daq_dac_clamp(&k,&c); assert(c.steps==DAQ_DAC_STEPS_MAX&&c.dwell_ms==DAQ_DAC_DWELL_MAX_MS);
  k.lease_ms=0; daq_dac_clamp(&k,&c); assert(c.lease_ms==DAQ_DAC_LEASE_DEFAULT_MS);
  k.lease_ms=10; assert(daq_dac_clamp(&k,&c)&DAQ_DAC_CLAMP_LEASE); assert(c.lease_ms==DAQ_DAC_LEASE_MIN_MS);
  k.lease_ms=60000; daq_dac_clamp(&k,&c); assert(c.lease_ms==DAQ_DAC_LEASE_MAX_MS);
  puts("ok");return 0;}''', [ROOT / 'Core/Inc'], [ROOT / 'Core/Src/daq_dac_control.c', ROOT / 'Core/Src/daq_dac_wave.c'])


class StateMachineTests(unittest.TestCase):
    def test_gating_auto_off_triggers_and_status(self):
        run_c(self, CONTROL + r'''
static uint8_t a[2];
static daq_dac_ctrl_t s;
static void start(uint8_t ch,uint32_t now){daq_dac_command_t k=cmd(ch,1,DAQ_DAC_WAVE_SINE,0.5f,1.5f,1.0f);
  assert(daq_dac_ctrl_command(&s,&k,NULL,true,1,now,a)==DAQ_DAC_RESULT_OK&&a[ch]==DAQ_DAC_ACTION_START);
  daq_dac_plan_t p={.actual_frequency_hz=1.0f};daq_dac_ctrl_started(&s,ch,&p,true);}
int main(void){
  daq_dac_ctrl_init(&s);
  /* Off at boot: nothing active, status says off with NaN outputs. */
  assert(daq_dac_ctrl_active_mask(&s)==0);
  uint8_t st[DAQ_DAC_STATUS_BYTES]; float out[2]={1,1},rb[2]={1,1};
  daq_dac_status_encode(&s,NULL,0,0,3,1234,out,rb,st);
  assert(DAQ_DAC_STATUS_BYTES==136&&st[0]==2&&st[1]==0&&st[2]==0&&st[3]==3&&rd32(st+4)==1234);
  for(int c=0;c<2;c++){const uint8_t*p=st+8+52*c;assert(p[0]==0&&p[4]==DAQ_DAC_OFF_NONE&&isnan(rdf(p+28))&&isnan(rdf(p+32)));}
  /* Gate: only Startup/Idle with a known state. */
  assert(daq_dac_state_gate(true,0)==0&&daq_dac_state_gate(true,1)==0);
  const uint8_t refused[]={2,6,7,13,15,16};
  for(unsigned i=0;i<sizeof refused;i++){
    daq_dac_command_t k=cmd(0,1,DAQ_DAC_WAVE_DC,0,1,0);
    assert(daq_dac_ctrl_command(&s,&k,NULL,true,refused[i],0,a)==DAQ_DAC_RESULT_REFUSED_STATE&&a[0]==0&&!s.ch[0].active);
    assert(s.ch[0].last_result==DAQ_DAC_RESULT_REFUSED_STATE);}
  {daq_dac_command_t k=cmd(0,1,DAQ_DAC_WAVE_DC,0,1,0);
   assert(daq_dac_ctrl_command(&s,&k,NULL,false,0,0,a)==DAQ_DAC_RESULT_REFUSED_UNKNOWN_STATE&&!s.ch[0].active);
   k.waveform=9; assert(daq_dac_ctrl_command(&s,&k,NULL,true,1,0,a)==DAQ_DAC_RESULT_BAD_REQUEST&&!s.ch[0].active);
   k.waveform=0; k.offset_v=NAN; assert(daq_dac_ctrl_command(&s,&k,NULL,true,1,0,a)==DAQ_DAC_RESULT_BAD_REQUEST);}
  /* Allowed in Startup and Idle, per channel. */
  {daq_dac_command_t k=cmd(1,1,DAQ_DAC_WAVE_DC,0,1,0);
   assert(daq_dac_ctrl_command(&s,&k,NULL,true,0,0,a)==0&&a[1]==DAQ_DAC_ACTION_START&&a[0]==0&&s.ch[1].active);}
  start(0,0); assert(daq_dac_ctrl_active_mask(&s)==3);
  /* Identical keepalive: no restart, lease extended past the original deadline. */
  {daq_dac_command_t k=cmd(0,1,DAQ_DAC_WAVE_SINE,0.5f,1.5f,1.0f);
   assert(daq_dac_ctrl_command(&s,&k,NULL,true,1,1500,a)==0&&a[0]==0);
   daq_dac_ctrl_tick(&s,true,1,3000,a); assert(s.ch[0].active&&a[0]==0);
   /* A changed request restarts. */
   k.frequency_hz=2.0f; assert(daq_dac_ctrl_command(&s,&k,NULL,true,1,3000,a)==0&&a[0]==DAQ_DAC_ACTION_START);}
  /* Lease expiry (GS link loss) turns off only the expired channel. */
  {daq_dac_command_t k=cmd(1,1,DAQ_DAC_WAVE_DC,0,1,0);daq_dac_ctrl_command(&s,&k,NULL,true,1,4000,a);}
  daq_dac_ctrl_tick(&s,true,1,4999,a); assert(a[0]==0&&a[1]==0);
  daq_dac_ctrl_tick(&s,true,1,5000,a); assert(a[0]==DAQ_DAC_ACTION_STOP&&a[1]==0&&!s.ch[0].active&&s.ch[0].off_reason==DAQ_DAC_OFF_LEASE);
  daq_dac_ctrl_tick(&s,true,1,6000,a); assert(a[1]==DAQ_DAC_ACTION_STOP&&s.ch[1].off_reason==DAQ_DAC_OFF_LEASE);
  /* Lease survives uptime wrap. */
  start(0,0xFFFFFF00u); daq_dac_ctrl_tick(&s,true,1,0x100u,a); assert(s.ch[0].active);
  daq_dac_ctrl_tick(&s,true,1,0x800u,a); assert(!s.ch[0].active&&s.ch[0].off_reason==DAQ_DAC_OFF_LEASE);
  /* Every disallowed flight state turns both outputs off. */
  for(unsigned i=0;i<sizeof refused;i++){start(0,0);start(1,0);
    daq_dac_ctrl_tick(&s,true,refused[i],10,a);
    assert(a[0]==DAQ_DAC_ACTION_STOP&&a[1]==DAQ_DAC_ACTION_STOP&&daq_dac_ctrl_active_mask(&s)==0);
    assert(s.ch[0].off_reason==DAQ_DAC_OFF_FLIGHT_STATE&&s.ch[1].off_reason==DAQ_DAC_OFF_FLIGHT_STATE);}
  start(0,0); daq_dac_ctrl_tick(&s,false,0,10,a); assert(!s.ch[0].active&&s.ch[0].off_reason==DAQ_DAC_OFF_FLIGHT_STATE);
  /* Startup <-> Idle keeps running. */
  start(0,0); daq_dac_ctrl_tick(&s,true,0,10,a); assert(s.ch[0].active);
  /* Abort. */
  start(1,0); daq_dac_ctrl_force_off(&s,DAQ_DAC_OFF_ABORT,a);
  assert(a[0]==DAQ_DAC_ACTION_STOP&&a[1]==DAQ_DAC_ACTION_STOP&&s.ch[0].off_reason==DAQ_DAC_OFF_ABORT&&s.ch[1].off_reason==DAQ_DAC_OFF_ABORT);
  /* Commanded off, per channel and all; never gated by flight state. */
  start(0,0);start(1,0);
  {daq_dac_command_t k=cmd(0,0,0,0,0,0);assert(daq_dac_ctrl_command(&s,&k,NULL,true,7,0,a)==0&&a[0]==DAQ_DAC_ACTION_STOP&&a[1]==0);
   assert(s.ch[0].off_reason==DAQ_DAC_OFF_COMMANDED&&s.ch[1].active);
   k.channel=DAQ_DAC_CHANNEL_ALL;assert(daq_dac_ctrl_command(&s,&k,NULL,false,0,0,a)==0&&a[1]==DAQ_DAC_ACTION_STOP&&!s.ch[1].active);}
  /* Hardware refusal leaves it off with a hardware reason. */
  {daq_dac_command_t k=cmd(0,1,DAQ_DAC_WAVE_DC,0,1,0);daq_dac_ctrl_command(&s,&k,NULL,true,1,0,a);
   daq_dac_ctrl_started(&s,0,NULL,false);assert(!s.ch[0].active&&s.ch[0].off_reason==DAQ_DAC_OFF_HARDWARE&&s.ch[0].last_result==DAQ_DAC_RESULT_HARDWARE_ERROR);}
  /* Status: applied clamped values, actual frequency, sequence, readings. */
  {daq_dac_command_t k=cmd(1,1,DAQ_DAC_WAVE_STAIRCASE,9.0f,2.0f,0);k.sequence=0xABCDEF01u;k.steps=400;
   daq_dac_ctrl_command(&s,&k,NULL,true,1,0,a);daq_dac_plan_t p={.actual_dwell_ms=501};daq_dac_ctrl_started(&s,1,&p,true);
   (void)daq_dac_take_changes(&s);
   out[1]=2.5f;rb[1]=2.49f;daq_dac_status_encode(&s,NULL,1,0,1,77,out,rb,st);const uint8_t*q=st+8+52;
   assert(q[0]==1&&q[1]==DAQ_DAC_WAVE_STAIRCASE&&q[2]==0&&q[3]==(DAQ_DAC_CLAMP_AMPLITUDE|DAQ_DAC_CLAMP_STEPS));
   assert((q[6]|(q[7]<<8))==256&&near(rdf(q+8),1.1f,1e-6f)&&rdf(q+12)==2.0f&&rd32(q+20)==501&&rd32(q+24)==0xABCDEF01u);
   assert(rdf(q+28)==2.5f&&rdf(q+32)==2.49f);}
  /* Changes are reported once. */
  start(0,0); assert(daq_dac_take_changes(&s)&1); assert(daq_dac_take_changes(&s)==0);
  puts("ok");return 0;}''', [ROOT / 'Core/Inc'], [ROOT / 'Core/Src/daq_dac_control.c', ROOT / 'Core/Src/daq_dac_wave.c'])

    def test_command_wire_decode(self):
        run_c(self, CONTROL + r'''
static uint16_t rd16(const uint8_t *p){return (uint16_t)(p[0]|(p[1]<<8));}
int main(void){
  uint8_t b[48]; daq_dac_command_t k=cmd(1,1,DAQ_DAC_WAVE_CHIRP,0.25f,1.5f,3.0f),d;
  k.lease_ms=4000;k.sequence=99;k.dwell_ms=1234;k.steps=6;k.phase_q16=0x4000;k.duty_q16=0x8000;
  k.frequency_stop_hz=300.0f;k.sweep_ms=5000;k.table_crc=0xCAFEF00Du;k.table_len=512;k.flags=DAQ_DAC_FLAG_LOG_SWEEP;
  assert(DAQ_DAC_COMMAND_BYTES==48);
  daq_dac_command_encode(&k,b);
  assert(b[0]==2&&b[1]==1&&b[2]==1&&b[3]==6&&rdf(b+4)==0.25f&&rdf(b+8)==1.5f&&rdf(b+12)==3.0f);
  assert(b[16]==6&&b[18]==(4000&255)&&rd32(b+20)==1234&&rd32(b+24)==99&&rd16(b+28)==0x4000&&rd16(b+30)==0x8000);
  assert(rdf(b+32)==300.0f&&rd32(b+36)==5000&&rd32(b+40)==0xCAFEF00Du&&rd16(b+44)==512&&b[46]==2&&b[47]==0);
  assert(daq_dac_command_decode(b,48,&d)==0&&d.channel==1&&d.waveform==6&&d.steps==6&&d.lease_ms==4000&&d.sequence==99);
  assert(d.phase_q16==0x4000&&d.duty_q16==0x8000&&d.frequency_stop_hz==300.0f&&d.sweep_ms==5000);
  assert(d.table_crc==0xCAFEF00Du&&d.table_len==512&&d.flags==DAQ_DAC_FLAG_LOG_SWEEP);
  assert(daq_dac_command_decode(b,47,&d)!=0&&daq_dac_command_decode(NULL,48,&d)!=0);
  b[0]=1; assert(daq_dac_command_decode(b,48,&d)!=0); b[0]=2; /* v1 is gone. */
  b[2]=2; assert(daq_dac_command_decode(b,48,&d)!=0); b[2]=1;
  b[1]=2; assert(daq_dac_command_decode(b,48,&d)!=0);
  b[1]=0xFF; assert(daq_dac_command_decode(b,48,&d)!=0); /* all + enable */
  b[2]=0; assert(daq_dac_command_decode(b,48,&d)==0&&d.channel==DAQ_DAC_CHANNEL_ALL);
  puts("ok");return 0;}''', [ROOT / 'Core/Inc'], [ROOT / 'Core/Src/daq_dac_control.c', ROOT / 'Core/Src/daq_dac_wave.c'])


MOCK_MAIN = r'''
#ifndef MOCK_MAIN_H
#define MOCK_MAIN_H
#include <stdint.h>
#include <stddef.h>
#define DISABLE_SD_CARD 0U
typedef enum {HAL_OK, HAL_ERROR} HAL_StatusTypeDef;
typedef struct {uint32_t CR, SWTRIGR, DHR12R1, DHR12R2, DOR1, DOR2, MCR;} DAC_TypeDef;
typedef struct {uint32_t CR1, CR2, PSC, ARR, EGR, SR;} TIM_TypeDef;
typedef struct {uint32_t CLBAR, CFCR, CSR, CCR, CTR1, CTR2, CBR1, CSAR, CDAR, CLLR;} DMA_Channel_TypeDef;
typedef struct {uint32_t CFGR2;} RCC_TypeDef;
extern DAC_TypeDef mock_dac; extern TIM_TypeDef mock_tim[2]; extern DMA_Channel_TypeDef mock_dma[2]; extern RCC_TypeDef mock_rcc;
#define DAC1 (&mock_dac)
#define TIM7 (&mock_tim[0])
#define TIM15 (&mock_tim[1])
#define GPDMA1_Channel10 (&mock_dma[0])
#define GPDMA1_Channel11 (&mock_dma[1])
#define RCC (&mock_rcc)
#define RCC_CFGR2_PPRE1 (7u<<4)
#define RCC_CFGR2_PPRE1_Pos 4
#define RCC_CFGR2_PPRE2 (7u<<8)
#define RCC_CFGR2_PPRE2_Pos 8
#define DAC_CR_EN1 1u
#define DAC_CR_TEN1 2u
#define DAC_CR_TSEL1 (15u<<2)
#define DAC_CR_DMAEN1 (1u<<12)
#define TIM_CR1_CEN 1u
#define TIM_CR1_ARPE (1u<<7)
#define TIM_CR2_MMS (7u<<4)
#define TIM_CR2_MMS_1 (2u<<4)
#define TIM_EGR_UG 1u
#define DMA_CCR_EN 1u
#define DMA_CCR_RESET 2u
#define DMA_CCR_SUSP 4u
#define DMA_CSR_SUSPF (1u<<13)
#define DMA_CTR1_SINC (1u<<3)
#define DMA_CTR1_SDW_LOG2_Pos 0
#define DMA_CTR1_DDW_LOG2_Pos 16
#define DMA_CTR2_REQSEL_Pos 0
#define DMA_CTR2_DREQ (1u<<10)
#define DMA_CLLR_LA 0xFFFCu
#define DMA_CLLR_ULL (1u<<16)
#define DMA_CLLR_USA (1u<<28)
#define DMA_CLLR_UB1 (1u<<29)
#define GPDMA1_REQUEST_DAC1_CH1 2u
#define GPDMA1_REQUEST_DAC1_CH2 3u
#define ADC_CHANNEL_DAC1CH1_ADC4 21u
#define ADC_CHANNEL_DAC1CH2_ADC4 22u
#define DAC_CHANNEL_1 0u
#define DAC_CHANNEL_2 16u
#define DAC_TRIGGER_NONE 0u
#define DAC_TRIGGER_T7_TRGO 0x1Bu
#define DAC_TRIGGER_T15_TRGO 0x23u
#define DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC 2u
#define DAC_SAMPLEANDHOLD_DISABLE 0u
#define DAC_OUTPUTBUFFER_ENABLE 0u
#define DAC_CHIPCONNECT_BOTH 4u
#define DAC_TRIMMING_FACTORY 0u
#define DAC_AUTONOMOUS_MODE_DISABLE 0u
#define DAC_ALIGN_12B_R 0u
#define DISABLE 0u
#define HAL_DAC_STATE_READY 1u
typedef struct {DAC_TypeDef *Instance; uint32_t State;} DAC_HandleTypeDef;
typedef struct {uint32_t DAC_HighFrequency, DAC_DMADoubleDataMode, DAC_SignedFormat, DAC_SampleAndHold,
  DAC_Trigger, DAC_OutputBuffer, DAC_ConnectOnChipPeripheral, DAC_UserTrimming, DAC_AutonomousMode;} DAC_ChannelConfTypeDef;
HAL_StatusTypeDef HAL_DAC_ConfigChannel(DAC_HandleTypeDef *, const DAC_ChannelConfTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef *, uint32_t, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef *, uint32_t);
uint32_t HAL_DAC_GetValue(const DAC_HandleTypeDef *, uint32_t);
uint32_t HAL_RCC_GetPCLK1Freq(void); uint32_t HAL_RCC_GetPCLK2Freq(void);
#define __HAL_RCC_TIM7_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM15_CLK_ENABLE() ((void)0)
/* mock_primask holds the mocked BASEPRI (0 = nothing masked). */
extern uint32_t mock_primask;
#define __NVIC_PRIO_BITS 4U
static inline uint32_t __get_BASEPRI(void){return mock_primask;}
static inline void __set_BASEPRI_MAX(uint32_t v){if(v&&(mock_primask==0||v<mock_primask))mock_primask=v;}
static inline void __set_BASEPRI(uint32_t v){mock_primask=v;}
static inline void __DSB(void){}
static inline void __ISB(void){}
#endif
'''

STUBS = {
    'main.h': MOCK_MAIN,
    'tx_api.h': '#pragma once\ntypedef unsigned UINT;\n#define TX_SUCCESS 0u\n#define TX_PTR_ERROR 3u\n#define TX_NOT_DONE 0x20u\n',
    'sedsnet.h': '''#pragma once
#include <stdint.h>
#include <stddef.h>
typedef int32_t SedsResult; typedef uint32_t SedsDataType; typedef struct SedsRouter SedsRouter;
#define SEDS_OK 0
typedef struct {uint32_t ty; const uint8_t *payload; size_t payload_len; const char *sender; size_t sender_len;} SedsPacketView;
''',
    'telemetry.h': '''#pragma once
#include "sedsnet.h"
SedsResult log_telemetry_asynchronous(SedsDataType, const void *, size_t, size_t);
''',
    'daq_log_time.h': '#pragma once\n#include <stdint.h>\nuint64_t daq_log_monotonic_ms(void);\n',
    'daq_adc4.h': '#pragma once\n#include "tx_api.h"\n#include <stdint.h>\nUINT daq_adc4_read_internal(uint32_t channel, float *volts);\n',
    'sd_card.h': '''#pragma once
#include <stdint.h>
typedef struct daq_calibration daq_calibration_t;
typedef enum {SD_CARD_STATUS_OK=0} sd_card_status_t;
sd_card_status_t sd_card_enqueue_csv_row(const char *, uint64_t, float, const daq_calibration_t *);
''',
}

GLUE = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "daq_dac.h"
#include "daq_dac_control.h"
#include "flight_state_cache.h"
#include "sedsnet_config.h"
#include "sd_card.h"
#include "telemetry.h"
#include "daq_adc4.h"
#include "daq_log_time.h"
extern volatile uint32_t g_daq_dac_malformed;
DAC_TypeDef mock_dac; TIM_TypeDef mock_tim[2]; DMA_Channel_TypeDef mock_dma[2]; RCC_TypeDef mock_rcc; uint32_t mock_primask;
DAC_HandleTypeDef hdac1={.Instance=&mock_dac,.State=HAL_DAC_STATE_READY};
static uint32_t config_trigger[2]; static unsigned start_calls; static int fail_start;
HAL_StatusTypeDef HAL_DAC_ConfigChannel(DAC_HandleTypeDef *h,const DAC_ChannelConfTypeDef *c,uint32_t ch){
  assert(h==&hdac1&&!mock_primask&&c->DAC_ConnectOnChipPeripheral==DAC_CHIPCONNECT_BOTH&&c->DAC_OutputBuffer==DAC_OUTPUTBUFFER_ENABLE);
  config_trigger[ch?1:0]=c->DAC_Trigger;return HAL_OK;}
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef *h,uint32_t ch,uint32_t a,uint32_t v){(void)h;(void)a;if(ch)mock_dac.DHR12R2=v;else mock_dac.DHR12R1=v;return HAL_OK;}
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef *h,uint32_t ch){(void)h;if(fail_start)return HAL_ERROR;mock_dac.CR|=DAC_CR_EN1<<ch;
  if(ch)mock_dac.DOR2=mock_dac.DHR12R2;else mock_dac.DOR1=mock_dac.DHR12R1;++start_calls;return HAL_OK;}
uint32_t HAL_DAC_GetValue(const DAC_HandleTypeDef *h,uint32_t ch){(void)h;return ch?mock_dac.DOR2:mock_dac.DOR1;}
uint32_t HAL_RCC_GetPCLK1Freq(void){return 160000000u;} uint32_t HAL_RCC_GetPCLK2Freq(void){return 160000000u;}
UINT daq_adc4_read_internal(uint32_t channel,float *v){*v=channel==ADC_CHANNEL_DAC1CH1_ADC4?1.01f:2.02f;return TX_SUCCESS;}
static uint64_t mono; uint64_t daq_log_monotonic_ms(void){return mono;}
static bool has_state=true; static uint8_t state=1;
bool flight_state_cache_has_value(void){return has_state;}
uint8_t flight_state_cache_value(void){return state;}
static uint8_t last_status[136]; static unsigned statuses;
SedsResult log_telemetry_asynchronous(SedsDataType t,const void *d,size_t n,size_t sz){
  assert(t==SEDS_DT_DAQ_DAC_STATUS&&n==136&&sz==1);memcpy(last_status,d,136);++statuses;return SEDS_OK;}
static unsigned rows; static char last_row[64];
sd_card_status_t sd_card_enqueue_csv_row(const char *n,uint64_t t,float v,const daq_calibration_t *c){(void)t;(void)v;(void)c;
  assert(!strncmp(n,"dac_ch",6));strncpy(last_row,n,63);++rows;return SD_CARD_STATUS_OK;}
static uint32_t now=1000;
static void tick(void){now+=50;daq_dac_service(now);}
static void send(uint8_t ch,uint8_t en,uint8_t wave,float amp,float off,float f,uint32_t seq){
  daq_dac_command_t k={.channel=ch,.enable=en,.waveform=wave,.amplitude_v=amp,.offset_v=off,.frequency_hz=f,.steps=4,.dwell_ms=100,.lease_ms=1000,.sequence=seq};
  uint8_t b[48];daq_dac_command_encode(&k,b);daq_dac_submit_command(b,48);}
static int on(unsigned c){return (mock_dac.CR&(DAC_CR_EN1<<(c?16:0)))!=0;}
static int dma_on(unsigned c){return (mock_dma[c].CCR&DMA_CCR_EN)!=0;}
static int tim_on(unsigned c){return (mock_tim[c].CR1&TIM_CR1_CEN)!=0;}
static void all_off(void){assert(!on(0)&&!on(1)&&!dma_on(0)&&!dma_on(1)&&!tim_on(0)&&!tim_on(1));}
static SedsPacketView fstate(uint8_t *v){SedsPacketView p={.ty=SEDS_DT_FLIGHT_STATE,.payload=v,.payload_len=1};return p;}
int main(void){
  /* Boot: CubeMX left something enabled; init forces everything off. */
  mock_dac.CR=DAC_CR_EN1|(DAC_CR_EN1<<16);mock_dma[0].CCR=DMA_CCR_EN;
  assert(daq_dac_init()==TX_SUCCESS); all_off(); assert(!mock_primask);
  tick(); assert(statuses==1&&last_status[8]==0&&last_status[60]==0&&g_daq_dac_active_mask==0);
  daq_dac_sample_t s; assert(daq_dac_sample(&s,1)==TX_SUCCESS&&isnan(s.analog_outputs_v[0])&&isnan(s.analog_outputs_v[1]));
  /* Sine on channel 0: TIM7 trigger, self-linked circular DMA into DHR12R1. */
  send(0,1,DAQ_DAC_WAVE_SINE,0.5f,1.5f,10.0f,1); tick();
  assert(on(0)&&dma_on(0)&&tim_on(0)&&!on(1)&&config_trigger[0]==DAC_TRIGGER_T7_TRGO&&g_daq_dac_active_mask==1);
  assert(mock_dma[0].CDAR==(uint32_t)(uintptr_t)&mock_dac.DHR12R1&&mock_dma[0].CBR1==256*2&&mock_dma[0].CTR2==(2u|DMA_CTR2_DREQ));
  assert((mock_dma[0].CLLR&(DMA_CLLR_ULL|DMA_CLLR_USA|DMA_CLLR_UB1))==(DMA_CLLR_ULL|DMA_CLLR_USA|DMA_CLLR_UB1));
  assert((mock_dac.CR&DAC_CR_DMAEN1)&&(mock_tim[0].PSC+1)*(mock_tim[0].ARR+1)==62500u&&(mock_tim[0].CR1&TIM_CR1_ARPE)&&mock_dma[0].CTR1==(DMA_CTR1_SINC|(1u<<DMA_CTR1_SDW_LOG2_Pos)|(2u<<DMA_CTR1_DDW_LOG2_Pos)));
  assert(last_status[8]==1&&last_status[9]==DAQ_DAC_WAVE_SINE&&rows>=10);
  assert(daq_dac_sample(&s,1)==TX_SUCCESS&&fabsf(s.analog_outputs_v[0]-1.5f)<0.002f);
  /* Active status every 200 ms carries the ADC4 internal readback. */
  {unsigned before=statuses;for(int i=0;i<4;i++)tick();assert(statuses==before+1);
   float rb;memcpy(&rb,last_status+8+32,4);assert(rb==1.01f);}
  /* Identical keepalive does not restart the waveform. */
  unsigned starts=start_calls; send(0,1,DAQ_DAC_WAVE_SINE,0.5f,1.5f,10.0f,1); tick(); assert(start_calls==starts&&on(0));
  /* DC on channel 1 without a timer. */
  send(1,1,DAQ_DAC_WAVE_DC,0,1.0f,0,2); tick();
  assert(on(1)&&!dma_on(1)&&!tim_on(1)&&config_trigger[1]==DAC_TRIGGER_NONE&&g_daq_dac_active_mask==3);
  /* Lease expiry (no keepalive) turns both off. */
  for(int i=0;i<25;i++)tick();
  all_off(); assert(last_status[8+4]==DAQ_DAC_OFF_LEASE&&last_status[60+4]==DAQ_DAC_OFF_LEASE&&g_daq_dac_active_mask==0);
  /* Enable refused outside Startup/Idle. */
  state=2; send(0,1,DAQ_DAC_WAVE_DC,0,1,0,3); tick(); all_off(); assert(last_status[8+2]==DAQ_DAC_RESULT_REFUSED_STATE&&last_status[2]==1);
  has_state=false; send(0,1,DAQ_DAC_WAVE_DC,0,1,0,4); tick(); all_off(); assert(last_status[8+2]==DAQ_DAC_RESULT_REFUSED_UNKNOWN_STATE&&last_status[2]==2);
  has_state=true; state=0;
  /* FLIGHT_STATE hook: leaving Startup/Idle stops the hardware before the
   * service runs; Startup <-> Idle does not. */
  const uint8_t bad[]={2,6,7,13,15,16,3,9};
  for(unsigned i=0;i<sizeof bad;i++){
    state=1; send(0,1,DAQ_DAC_WAVE_SQUARE,0.3f,1.0f,1,10+i); send(1,1,DAQ_DAC_WAVE_DC,0,2,0,20+i); tick(); assert(on(0)&&on(1));
    uint8_t v=0; SedsPacketView p=fstate(&v); assert(flight_state_cache_apply_network_update(&p)==SEDS_OK); assert(on(0));
    v=bad[i]; p=fstate(&v); assert(flight_state_cache_apply_network_update(&p)==SEDS_OK); all_off();
    state=bad[i]; tick(); all_off(); assert(last_status[8+4]==DAQ_DAC_OFF_FLIGHT_STATE&&g_daq_dac_active_mask==0);}
  /* Polling backstop alone (no hook) also stops it. */
  state=1; send(0,1,DAQ_DAC_WAVE_DC,0,1,0,40); tick(); assert(on(0)); state=7; tick(); all_off(); assert(last_status[8+4]==DAQ_DAC_OFF_FLIGHT_STATE);
  /* Abort: immediate, and requests queued before it never start. */
  state=1; send(0,1,DAQ_DAC_WAVE_TRIANGLE,0.3f,1,1,50); tick(); assert(on(0));
  send(1,1,DAQ_DAC_WAVE_DC,0,1,0,51); daq_dac_emergency_off(DAQ_DAC_OFF_ABORT); all_off(); tick(); all_off();
  assert(last_status[8+4]==DAQ_DAC_OFF_ABORT&&g_daq_dac_active_mask==0);
  /* Commanded off per channel and all-off. */
  send(0,1,DAQ_DAC_WAVE_DC,0,1,0,60); send(1,1,DAQ_DAC_WAVE_DC,0,1,0,61); tick(); assert(on(0)&&on(1));
  send(0,0,0,0,0,0,62); tick(); assert(!on(0)&&on(1)&&last_status[8+4]==DAQ_DAC_OFF_COMMANDED);
  send(0xFF,0,0,0,0,0,63); tick(); all_off();
  /* A hardware start failure is reported, never left half-on. */
  fail_start=1; send(0,1,DAQ_DAC_WAVE_SINE,0.3f,1,1,70); tick(); all_off(); fail_start=0;
  assert(last_status[8+2]==DAQ_DAC_RESULT_HARDWARE_ERROR&&last_status[8+4]==DAQ_DAC_OFF_HARDWARE);
  /* Malformed payloads are counted and ignored. */
  uint8_t junk[47]={2}; daq_dac_submit_command(junk,47); tick(); all_off(); assert(g_daq_dac_malformed==1);
  assert(!mock_primask);
  puts("ok");return 0;}
'''


class HalGlueTests(unittest.TestCase):
    def test_production_glue_on_register_mocks(self):
        with tempfile.TemporaryDirectory() as d:
            stub = pathlib.Path(d)
            for name, text in STUBS.items():
                (stub / name).write_text(text)
            src = stub / 'daq_dac_glue.c'
            src.write_text('#define TELEMETRY_ENABLED 1\n' + (ROOT / 'Core/Src/daq_dac.c').read_text())
            run_c(self, '#define TELEMETRY_ENABLED 1\n' + GLUE, [stub, ROOT / 'Core/Inc'],
                  [src, ROOT / 'Core/Src/daq_dac_control.c', ROOT / 'Core/Src/daq_dac_wave.c'])


class WiringContractTests(unittest.TestCase):
    def test_boot_never_starts_the_dac(self):
        main = (ROOT / 'Core/Src/main.c').read_text()
        init = main[main.index('static void MX_DAC1_Init(void)\n{'):]
        init = init[:init.index('\n}\n')]
        self.assertNotIn('HAL_DAC_Start', init)
        source = (ROOT / 'Core/Src/daq_dac.c').read_text()
        boot = source[source.index('UINT daq_dac_init(void)'):source.index('UINT daq_dac_sample')]
        self.assertNotIn('HAL_DAC_Start', boot)
        self.assertIn('hw_stop(c)', boot)
        # Nothing about the DAC is persisted, so a reboot cannot resume it.
        self.assertNotIn('persist', source.lower())

    def test_abort_command_flight_state_and_service_are_wired(self):
        telemetry = (ROOT / 'Core/Src/telemetry.c').read_text()
        handler = telemetry[telemetry.index('static SedsResult sd_card_endpoint_receive'):]
        handler = handler[:handler.index('\n}\n')]
        self.assertRegex(handler, r'SEDS_DT_ABORT\) \{ daq_dac_emergency_off\(DAQ_DAC_OFF_ABORT\)')
        self.assertIn('SEDS_DT_DAQ_DAC_COMMAND', handler)
        self.assertIn('"GS", 2U', handler[handler.index('SEDS_DT_DAQ_DAC_COMMAND'):])
        self.assertIn('return sd_retry_receive(packet, context);', handler)
        self.assertIn('.endpoint = SEDS_EP_SD_CARD, .packet_handler = sd_card_endpoint_receive', telemetry)
        thread = (ROOT / 'Core/Src/daq_thread.c').read_text()
        network = thread[thread.index('static void daq_network_thread_entry'):]
        self.assertIn('daq_dac_service(HAL_GetTick())', network[:network.index('\n}\n')])
        self.assertIn('SedsResult flight_state_cache_apply_network_update',
                      (ROOT / 'Core/Src/daq_dac.c').read_text())
        self.assertIn('flight_state_cache_apply_network_update(packet)',
                      (ROOT / 'Core/Src/flight_state_cache.c').read_text())

    def test_schema_entries_and_canonical_ids(self):
        schema = {t['name']: t for t in json.loads((ROOT / 'config/sedsnet.json').read_text())['types']}
        header = (ROOT / 'Core/Inc/sedsnet_config.h').read_text()
        for name, count, endpoint in (('DAQ_DAC_COMMAND', 48, 'FillSdCard'),
                                      ('DAQ_DAC_STATUS', 136, 'ActuatorGroundStation'),
                                      ('DAQ_DAC_TABLE', 76, 'FillSdCard')):
            entry = schema[name]
            self.assertEqual(entry['element'], {'kind': 'Static', 'data_type': 'UInt8', 'count': count})
            self.assertEqual(entry['endpoints'], [endpoint])
            self.assertEqual(entry['reliable'], 'Off')
            value = 0x811c9dc5 ^ 0x5459
            for byte in name.encode():
                value = ((value ^ byte) * 0x01000193) & 0xffffffff
            ident = 0x10000 | (value & 0x0fffffff)
            self.assertRegex(header, rf'#define SEDS_DT_{name} \(\(uint32_t\){ident}U\)')
        control = (ROOT / 'Core/Inc/daq_dac_control.h').read_text()
        self.assertIn('#define DAQ_DAC_COMMAND_BYTES 48U', control)
        self.assertIn('#define DAQ_DAC_TABLE_BYTES (12U + 2U * DAQ_DAC_TABLE_CHUNK_SAMPLES)', control)
        # The ABORT type must keep reaching the DAQ's SD_CARD endpoint.
        self.assertIn('FillSdCard', schema['ABORT']['endpoints'])
        self.assertIn('FillSdCard', schema['FLIGHT_STATE']['endpoints'])

    def test_ram_cost_stays_small(self):
        # 2 x 512 play codes + a 512-sample uploaded shape, 16 bits each: 3 KiB.
        source = (ROOT / 'Core/Src/daq_dac.c').read_text()
        self.assertRegex(source, r'static uint16_t g_table\[DAQ_DAC_CHANNELS\]\[DAQ_DAC_ARB_LEN_MAX\]')
        self.assertRegex(source, r'static int16_t g_shape\[DAQ_DAC_ARB_LEN_MAX\]')
        wave = (ROOT / 'Core/Inc/daq_dac_wave.h').read_text()
        self.assertIn('#define DAQ_DAC_PERIODIC_LEN_MAX 256U', wave)
        self.assertIn('#define DAQ_DAC_ARB_LEN_MAX 512U', wave)
        self.assertIsNone(re.search(r'\bmalloc\b|tx_byte_allocate', source))


if __name__ == '__main__':
    unittest.main()
