"""Exercise production calibration update/persistence logic with legacy records."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class Kg50CalibrationTests(unittest.TestCase):
    def test_legacy_migration_independent_updates_and_failed_persistence(self):
        source = (ROOT / 'Core/Src/daq_calibration.c').read_text()
        source = '\n'.join(line for line in source.splitlines() if not line.startswith('#include "'))
        source = source.replace('__attribute__((used, externally_visible))', '')
        harness = r'''
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
typedef unsigned SedsResult;
typedef unsigned SedsRouter;
typedef unsigned launchcore_persist_status_t;
typedef struct { unsigned ty; const void *payload; size_t payload_len; } SedsPacketView;
typedef struct { float kg1000_slope, kg1000_intercept, iadc_slope, iadc_intercept, kg50[7], thermal[4], filter_tau_ms[2]; } daq_calibration_t;
#define SEDS_DT_DAQ_LOADCELL_CALIBRATION 137
#define SEDS_DT_DAQ_KG50_CALIBRATION 138
#define SEDS_DT_DAQ_THERMAL_CALIBRATION 141
#define SEDS_DT_DAQ_FILTER_CALIBRATION 142
#define SEDS_DT_DAQ_KG50_INPUT 147
static unsigned requested_input;
static void mcp3564r_request_kg50_input(uint8_t input) { requested_input=input; }
#define SEDS_OK 0
#define SEDS_BAD_ARG 1
#define SEDS_HANDLER_ERROR 2
#define LAUNCHCORE_PERSIST_OK 0
#define LAUNCHCORE_PERSIST_NOT_FOUND 1
volatile uint32_t g_telemetry_discovery_seen=1, g_reliable_refusal_count;
static unsigned ticks, fail_persist, requests[11], rotations, invalidations;
static int seds_router_invalidate_network_variable_cache(SedsRouter *r,unsigned ty,bool *out){(void)r;(void)ty;assert(!out);++invalidations;return 0;}
static unsigned char persisted[sizeof(daq_calibration_t)];
static size_t persisted_len;
static daq_calibration_t sd_calibration;
static uint32_t __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __enable_irq(void) {}
static uint32_t HAL_GetTick(void) { return ticks; }
static unsigned persistent_store_init(void) { return 0; }
static unsigned char persisted_input; static size_t persisted_input_len; static unsigned input_writes;
static unsigned persistent_store_get(unsigned key, void *value, size_t *size) {
  if (key==0x4441494Eu) {
    if (!persisted_input_len) return LAUNCHCORE_PERSIST_NOT_FOUND;
    memcpy(value,&persisted_input,1); *size=persisted_input_len; return 0;
  }
  if (!persisted_len) return LAUNCHCORE_PERSIST_NOT_FOUND;
  assert(*size >= persisted_len);
  memcpy(value,persisted,persisted_len); *size=persisted_len; return 0;
}
static unsigned persistent_store_set(unsigned key, const void *value, size_t size) {
  if(fail_persist) return 2;
  if (key==0x4441494Eu) { assert(size==1); memcpy(&persisted_input,value,1); persisted_input_len=1; input_writes++; return 0; }
  assert(size <= sizeof(persisted));
  memcpy(persisted,value,size); persisted_len=size; return 0;
}
static void sd_card_set_calibration(const daq_calibration_t *c) { sd_calibration=*c; rotations++; }
static unsigned seds_router_enable_network_variable(SedsRouter *r,unsigned ty,bool read,bool write) {
  assert(r && ty>=137 && ty<=147 && read && !write); return 0;
}
static unsigned seds_router_on_network_variable_update(SedsRouter *r,unsigned ty,
    SedsResult (*cb)(const SedsPacketView *, void *),void *user) {
  assert(r && user==r && ty>=137 && ty<=147 && cb); return 0;
}
static unsigned seds_router_request_managed_variable(SedsRouter *r,unsigned ty) {
  assert(r && ty>=137 && ty<=147); requests[ty-137]++; return 0;
}
static daq_calibration_t daq_calibration_current(void);
''' + source + r'''
int main(void) {
  float legacy[4]={2,3,4,5};
  memcpy(persisted,legacy,sizeof(legacy)); persisted_len=sizeof(legacy);
  SedsRouter router=0;
  assert(daq_calibration_init(&router)==SEDS_OK);
  assert(g_daq_calibration_restores==1 && g_calibration.kg1000_slope==2);
  assert(requested_input==0 && g_daq_kg50_input_restores==0); /* Blank board stays on AMP2. */
  assert(g_calibration.kg50[1]==1 && daq_calibration_apply_kg50(&g_calibration,5)==5);
  ticks=500; assert(daq_calibration_poll(&router)==0);
  assert(requests[10]==1);
  uint8_t input=1;
  SedsPacketView source_packet={147,&input,1};
  assert(apply_kg50_input(&source_packet,NULL)==SEDS_OK && requested_input==1);
  assert(persisted_input==1 && input_writes==1);
  /* Repeating the same selection must not spend a flash write. */
  assert(apply_kg50_input(&source_packet,NULL)==SEDS_OK && input_writes==1);
  /* Reboot: the selection comes back from flash before any network value. */
  requested_input=0; g_kg50_input_stored=0; g_restore_attempted=false;
  daq_calibration_restore();
  assert(requested_input==1 && g_kg50_input_stored==1 && g_daq_kg50_input_restores==1);
  /* A storage fault still applies the selection and is retried next time. */
  input=0; fail_persist=1;
  assert(apply_kg50_input(&source_packet,NULL)==SEDS_OK && requested_input==0 && persisted_input==1);
  unsigned errors=g_daq_calibration_errors; assert(errors>0);
  fail_persist=0; assert(apply_kg50_input(&source_packet,NULL)==SEDS_OK && persisted_input==0 && input_writes==2);
  /* A corrupt record is ignored rather than selecting an unknown input. */
  persisted_input=7; requested_input=1; g_restore_attempted=false;
  daq_calibration_restore();
  assert(requested_input==1 && g_daq_calibration_errors==errors+1);
  persisted_input=0;
  input=1; assert(apply_kg50_input(&source_packet,NULL)==SEDS_OK);
  /* Refused values are acknowledged, never applied, and leave no cache. */
  input=2; assert(apply_kg50_input(&source_packet,&router)==SEDS_OK && requested_input==1 && invalidations==1 && g_reliable_refusal_count==1);
  source_packet.payload_len=0; assert(apply_kg50_input(&source_packet,&router)==SEDS_OK && invalidations==2 && g_reliable_refusal_count==2);
  source_packet.payload_len=1; input=0;
  assert(apply_kg50_input(&source_packet,NULL)==SEDS_OK && requested_input==0);

  assert(requests[0]==1 && requests[1]==1);
  float kg50[7]={1,2,3,0,0,1,6};
  SedsPacketView packet={138,kg50,sizeof(kg50)};
  /* Storage failure is transient: no ACK, and the cached observation is
   * retired so the identical retry runs this callback again. */
  fail_persist=1; unsigned retired=invalidations;
  assert(apply_calibration(&packet,&router)==SEDS_HANDLER_ERROR && invalidations==retired+1);
  assert(g_calibration.kg50[1]==1 && !g_kg50_network_value_seen);
  assert(sd_calibration.kg50[1]==1 && rotations==1);
  fail_persist=0;
  assert(apply_calibration(&packet,NULL)==SEDS_OK);
  assert(g_calibration.kg1000_slope==2 && g_calibration.iadc_slope==4);
  assert(g_calibration.kg50[2]==3 && sd_calibration.kg50[2]==3);
  assert(persisted_len==sizeof(daq_calibration_t));
  assert(daq_calibration_apply_kg50(&g_calibration,3)==11);
  float thermal[4]={20,0.2f,25,-0.5f};
  packet=(SedsPacketView){141,thermal,sizeof(thermal)};
  assert(apply_calibration(&packet,NULL)==SEDS_OK);
  assert(g_thermal_network_value_seen && g_calibration.kg50[2]==3);
  assert(fabsf(daq_calibration_temperature_raw(&g_calibration,0,12,30)-10)<1e-5f);
  assert(fabsf(daq_calibration_temperature_raw(&g_calibration,1,5,35)-10)<1e-5f);
  assert(isnan(daq_calibration_temperature_raw(&g_calibration,0,12,NAN)));
  assert(isnan(daq_calibration_temperature_raw(&g_calibration,1,12,126)));
  float tau[2]={100,200};
  packet=(SedsPacketView){142,tau,sizeof(tau)};
  fail_persist=1; assert(apply_calibration(&packet,&router)==SEDS_HANDLER_ERROR);
  assert(g_calibration.filter_tau_ms[0]==0 && !g_filter_network_value_seen);
  fail_persist=0; assert(apply_calibration(&packet,NULL)==SEDS_OK);
  assert(g_calibration.filter_tau_ms[0]==100 && g_filter_network_value_seen);
  unsigned refused=g_reliable_refusal_count;
  tau[0]=-1; assert(apply_calibration(&packet,&router)==SEDS_OK && g_calibration.filter_tau_ms[0]==100);
  tau[0]=NAN; assert(apply_calibration(&packet,&router)==SEDS_OK && g_reliable_refusal_count==refused+2);
  legacy[0]=7; packet=(SedsPacketView){137,legacy,sizeof(legacy)};
  assert(apply_calibration(&packet,NULL)==SEDS_OK);
  assert(g_calibration.kg1000_slope==7 && g_calibration.kg50[2]==3);
  packet=(SedsPacketView){138,kg50,sizeof(kg50)-4};
  assert(apply_calibration(&packet,&router)==SEDS_OK);
  kg50[0]=NAN; packet.payload_len=sizeof(kg50);
  assert(apply_calibration(&packet,&router)==SEDS_OK && g_reliable_refusal_count==refused+4);
  assert(g_calibration.kg50[0]==1);
  g_restore_attempted=false; memset(&g_calibration,0,sizeof(g_calibration));
  daq_calibration_restore();
  assert(g_calibration.kg1000_slope==7 && g_calibration.kg50[2]==3);
  assert(g_calibration.thermal[1]==0.2f);
  assert(g_calibration.filter_tau_ms[1]==200);
  persisted_len=15*sizeof(float); /* Thermal record predating filters. */
  memset(&g_calibration,0,sizeof(g_calibration)); g_restore_attempted=false;
  daq_calibration_restore();
  assert(g_calibration.thermal[1]==0.2f && g_calibration.filter_tau_ms[1]==0);
  persisted_len=11*sizeof(float); /* Previous KG50 format without thermal fields. */
  memset(&g_calibration,0,sizeof(g_calibration)); g_restore_attempted=false;
  daq_calibration_restore();
  assert(g_calibration.kg50[2]==3 && g_calibration.thermal[1]==0);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / 'calibration-test'
            result = subprocess.run(['cc', '-DTELEMETRY_ENABLED=1', '-std=c11', '-Wall', '-Wextra', '-Werror',
                                     '-x', 'c', '-', '-o', str(binary)], input=harness,
                                    text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(binary)], check=True)
