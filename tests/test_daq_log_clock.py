"""Compile the production network-variable receiver with a recording router."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class DaqLogClockTests(unittest.TestCase):
    def test_clock_subscription_validation_and_router_reinitialization(self):
        source = (ROOT / "Core/Src/daq_log_clock.c").read_text()
        source = "\n".join(line for line in source.splitlines() if not line.startswith("#include"))
        code = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "sd_calendar.h"
typedef int SedsResult;
typedef int SedsRouter;
typedef struct { unsigned ty; const uint8_t *payload; size_t payload_len; } SedsPacketView;
enum { SEDS_OK, SEDS_HANDLER_ERROR };
#define SEDS_DT_DAQ_LOG_CLOCK 139U
volatile uint32_t g_telemetry_discovery_seen;
static uint32_t tick, requests, changes;
static uint64_t got_session, got_deadline;
static uint32_t HAL_GetTick(void) { return tick; }
static void sd_card_set_launch_clock(uint64_t s,uint64_t d) { got_session=s; got_deadline=d; ++changes; }
static int seds_router_enable_network_variable(SedsRouter *r,unsigned ty,bool read,bool write) {
  assert(r && ty==139 && read && !write); return 0;
}
static int seds_router_on_network_variable_update(SedsRouter *r,unsigned ty,
  SedsResult (*cb)(const SedsPacketView*,void*),void *context) {
  assert(r && ty==139 && cb && !context); return 0;
}
static int seds_router_request_managed_variable(SedsRouter *r,unsigned ty) {
  assert(r && ty==139); ++requests; return 0;
}
''' + source + r'''
int main(void) {
  SedsRouter router=1;
  assert(daq_log_clock_init(&router)==0);
  tick=1000; daq_log_clock_poll(&router); assert(requests==0);
  g_telemetry_discovery_seen=1; daq_log_clock_poll(&router); assert(requests==1);
  uint8_t payload[16]={0};
  SedsPacketView p={139,payload,15};
  assert(clock_update(&p,NULL)==SEDS_HANDLER_ERROR && changes==0);
  p.payload_len=16;
  payload[0]=7; /* Session without a valid deadline is rejected. */
  assert(clock_update(&p,NULL)==SEDS_HANDLER_ERROR && changes==0);
  const uint64_t end=1800000130000ULL;
  for (unsigned i=0;i<8;++i) payload[8+i]=(uint8_t)(end>>(8*i));
  assert(clock_update(&p,NULL)==SEDS_OK && changes==1);
  assert(got_session==7 && got_deadline==end);
  tick+=1000; daq_log_clock_poll(&router); assert(requests==1);
  /* A recreated router re-registers the read-only subscription and refetches. */
  assert(daq_log_clock_init(&router)==SEDS_OK);
  tick+=500; daq_log_clock_poll(&router); assert(requests==2);
  assert(clock_update(&p,NULL)==SEDS_OK && got_session==7 && got_deadline==end);
  for (unsigned i=0;i<16;++i) payload[i]=0;
  assert(clock_update(&p,NULL)==SEDS_OK && got_session==0 && got_deadline==0);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            exe = Path(directory) / "clock-test"
            result = subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                                     "-I", str(ROOT / "Core/Inc"),
                                     "-x", "c", "-", "-o", str(exe)],
                                    input=code, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()
