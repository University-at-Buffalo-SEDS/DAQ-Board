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
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
typedef unsigned SedsResult;
typedef unsigned SedsRouter;
typedef unsigned launchcore_persist_status_t;
typedef struct { unsigned ty; const void *payload; size_t payload_len; } SedsPacketView;
typedef struct { float kg1000_slope, kg1000_intercept, iadc_slope, iadc_intercept, kg50[7]; } daq_calibration_t;
#define SEDS_DT_DAQ_LOADCELL_CALIBRATION 137
#define SEDS_DT_DAQ_KG50_CALIBRATION 138
#define SEDS_OK 0
#define SEDS_BAD_ARG 1
#define SEDS_HANDLER_ERROR 2
#define LAUNCHCORE_PERSIST_OK 0
#define LAUNCHCORE_PERSIST_NOT_FOUND 1
volatile uint32_t g_telemetry_discovery_seen=1;
static unsigned ticks, fail_persist, requests[2], rotations;
static unsigned char persisted[sizeof(daq_calibration_t)];
static size_t persisted_len;
static daq_calibration_t sd_calibration;
static uint32_t __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __enable_irq(void) {}
static uint32_t HAL_GetTick(void) { return ticks; }
static unsigned persistent_store_init(void) { return 0; }
static unsigned persistent_store_get(unsigned key, void *value, size_t *size) {
  (void)key;
  if (!persisted_len) return LAUNCHCORE_PERSIST_NOT_FOUND;
  assert(*size >= persisted_len);
  memcpy(value,persisted,persisted_len); *size=persisted_len; return 0;
}
static unsigned persistent_store_set(unsigned key, const void *value, size_t size) {
  (void)key;
  if(fail_persist) return 2;
  assert(size <= sizeof(persisted));
  memcpy(persisted,value,size); persisted_len=size; return 0;
}
static void sd_card_set_calibration(const daq_calibration_t *c) { sd_calibration=*c; rotations++; }
static unsigned seds_router_enable_network_variable(SedsRouter *r,unsigned ty,bool read,bool write) {
  assert(r && ty>=137 && ty<=138 && read && !write); return 0;
}
static unsigned seds_router_on_network_variable_update(SedsRouter *r,unsigned ty,
    SedsResult (*cb)(const SedsPacketView *, void *),void *user) {
  (void)user; assert(r && ty>=137 && ty<=138 && cb); return 0;
}
static unsigned seds_router_request_managed_variable(SedsRouter *r,unsigned ty) {
  assert(r && ty>=137 && ty<=138); requests[ty-137]++; return 0;
}
static daq_calibration_t daq_calibration_current(void);
''' + source + r'''
int main(void) {
  float legacy[4]={2,3,4,5};
  memcpy(persisted,legacy,sizeof(legacy)); persisted_len=sizeof(legacy);
  SedsRouter router=0;
  assert(daq_calibration_init(&router)==SEDS_OK);
  assert(g_daq_calibration_restores==1 && g_calibration.kg1000_slope==2);
  assert(g_calibration.kg50[1]==1 && daq_calibration_apply_kg50(&g_calibration,5)==5);
  ticks=500; assert(daq_calibration_poll(&router)==0);
  assert(requests[0]==1 && requests[1]==1);
  float kg50[7]={1,2,3,0,0,1,6};
  SedsPacketView packet={138,kg50,sizeof(kg50)};
  fail_persist=1;
  assert(apply_calibration(&packet,NULL)==SEDS_HANDLER_ERROR);
  assert(g_calibration.kg50[1]==1 && !g_kg50_network_value_seen);
  assert(sd_calibration.kg50[1]==1 && rotations==1);
  fail_persist=0;
  assert(apply_calibration(&packet,NULL)==SEDS_OK);
  assert(g_calibration.kg1000_slope==2 && g_calibration.iadc_slope==4);
  assert(g_calibration.kg50[2]==3 && sd_calibration.kg50[2]==3);
  assert(persisted_len==sizeof(daq_calibration_t));
  assert(daq_calibration_apply_kg50(&g_calibration,3)==11);
  legacy[0]=7; packet=(SedsPacketView){137,legacy,sizeof(legacy)};
  assert(apply_calibration(&packet,NULL)==SEDS_OK);
  assert(g_calibration.kg1000_slope==7 && g_calibration.kg50[2]==3);
  packet=(SedsPacketView){138,kg50,sizeof(kg50)-4};
  assert(apply_calibration(&packet,NULL)==SEDS_HANDLER_ERROR);
  kg50[0]=NAN; packet.payload_len=sizeof(kg50);
  assert(apply_calibration(&packet,NULL)==SEDS_HANDLER_ERROR);
  assert(g_calibration.kg50[0]==1);
  g_restore_attempted=false; memset(&g_calibration,0,sizeof(g_calibration));
  daq_calibration_restore();
  assert(g_calibration.kg1000_slope==7 && g_calibration.kg50[2]==3);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / 'calibration-test'
            result = subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                                     '-x', 'c', '-', '-o', str(binary)], input=harness,
                                    text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(binary)], check=True)
