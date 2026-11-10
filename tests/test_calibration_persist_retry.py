"""An applied DAQ calibration is acknowledged; durability is retried from the poll.

apply_calibration persisted before applying and returned SEDS_HANDLER_ERROR
when storage was not ready or the write failed. The SDK then withholds the
end-to-end receipt, so the station's ordered calibration stream stalls
behind that write and newer values queue behind it. A valid value is now
applied and acknowledged; its persistence is retried from
daq_calibration_poll at most once per second.

Builds the actual daq_calibration.c against stub HAL/SDK/storage headers.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

HEADERS = {
    "main.h": "#pragma once\n#include <stdint.h>\nuint32_t HAL_GetTick(void);\n"
              "static inline uint32_t __get_PRIMASK(void){return 0U;}\n"
              "static inline void __disable_irq(void){}\nstatic inline void __enable_irq(void){}\n",
    "mcp3564r.h": "#pragma once\n#include <stdint.h>\nvoid mcp3564r_request_kg50_input(uint8_t input);\n",
    "sd_card.h": "#pragma once\n#include \"daq_calibration.h\"\n"
                 "void sd_card_set_calibration(const daq_calibration_t *calibration);\n",
    "persistent_store.h": "#pragma once\n#include <stddef.h>\n#include <stdint.h>\n"
                          "typedef int launchcore_persist_status_t;\n#define LAUNCHCORE_PERSIST_OK 0\n"
                          "#define LAUNCHCORE_PERSIST_NOT_FOUND 1\n#define LAUNCHCORE_PERSIST_STORAGE_ERROR 2\n"
                          "launchcore_persist_status_t persistent_store_init(void);\n"
                          "launchcore_persist_status_t persistent_store_get(uint32_t,void*,size_t*);\n"
                          "launchcore_persist_status_t persistent_store_set(uint32_t,const void*,size_t);\n",
    "sedsnet.h": "#pragma once\n#include <stdbool.h>\n#include <stddef.h>\n#include <stdint.h>\n"
                 "typedef int SedsResult;\ntypedef int SedsRouter;\n"
                 "typedef struct { uint32_t ty; const void *payload; size_t payload_len; } SedsPacketView;\n"
                 "#define SEDS_OK 0\n#define SEDS_BAD_ARG -10\n#define SEDS_HANDLER_ERROR -9\n"
                 "SedsResult seds_router_enable_network_variable(SedsRouter*,uint32_t,bool,bool);\n"
                 "SedsResult seds_router_on_network_variable_update(SedsRouter*,uint32_t,"
                 "SedsResult(*)(const SedsPacketView*,void*),void*);\n"
                 "SedsResult seds_router_request_managed_variable(SedsRouter*,uint32_t);\n"
                 "SedsResult seds_router_invalidate_network_variable_cache(SedsRouter*,uint32_t,bool*);\n",
    "sedsnet_config.h": "#pragma once\n#define SEDS_DT_DAQ_LOADCELL_CALIBRATION 1U\n"
                        "#define SEDS_DT_DAQ_KG50_CALIBRATION 2U\n#define SEDS_DT_DAQ_THERMAL_CALIBRATION 3U\n"
                        "#define SEDS_DT_DAQ_FILTER_CALIBRATION 4U\n#define SEDS_DT_DAQ_KG50_INPUT 5U\n",
    "board_recovery_transport.h": "#pragma once\n#include <stdint.h>\nextern volatile uint32_t g_reliable_refusal_count;\n",
}

HARNESS = r'''
#include "daq_calibration.h"
#include "sedsnet_config.h"
#include <assert.h>
#include <string.h>
volatile uint32_t g_telemetry_discovery_seen=1, g_reliable_refusal_count;
extern volatile uint32_t g_daq_calibration_persist_writes, g_daq_calibration_errors, g_daq_calibration_updates;
static uint32_t tick;
static int fail_write, store_ready=1;
static unsigned writes, clears, requests, sd_sets;
static daq_calibration_t stored;
static SedsResult (*callbacks[6])(const SedsPacketView*,void*);
uint32_t HAL_GetTick(void){return tick;}
void mcp3564r_request_kg50_input(uint8_t input){(void)input;}
void sd_card_set_calibration(const daq_calibration_t *c){(void)c;++sd_sets;}
int persistent_store_init(void){return store_ready?0:2;}
int persistent_store_get(uint32_t k,void*v,size_t*n){(void)k;(void)v;(void)n;return 1;}
int persistent_store_set(uint32_t k,const void*v,size_t n){
 ++writes;if(fail_write)return 2;
 if(k==0x44414341u){assert(n==sizeof stored);memcpy(&stored,v,n);}return 0;
}
SedsResult seds_router_enable_network_variable(SedsRouter*r,uint32_t t,bool a,bool b){(void)r;(void)t;(void)a;(void)b;return 0;}
SedsResult seds_router_on_network_variable_update(SedsRouter*r,uint32_t t,SedsResult(*cb)(const SedsPacketView*,void*),void*u){(void)r;(void)u;callbacks[t]=cb;return 0;}
SedsResult seds_router_request_managed_variable(SedsRouter*r,uint32_t t){(void)r;(void)t;++requests;return 0;}
SedsResult seds_router_invalidate_network_variable_cache(SedsRouter*r,uint32_t t,bool*o){(void)r;(void)t;(void)o;++clears;return 0;}
static SedsRouter router=1;
static SedsResult loadcell(float slope){float v[4]={slope,0.0f,1.0f,0.0f};SedsPacketView p={SEDS_DT_DAQ_LOADCELL_CALIBRATION,v,sizeof v};return callbacks[SEDS_DT_DAQ_LOADCELL_CALIBRATION](&p,&router);}
static void poll_at(uint32_t at){tick=at;assert(daq_calibration_poll(&router)==SEDS_OK);}
int main(int argc,char**argv){
 assert(argc==2);
 if(!strcmp(argv[1],"not_ready"))store_ready=0;
 assert(daq_calibration_init(&router)==SEDS_OK);
 const unsigned base=writes;poll_at(10000); /* the 500 ms poll cadence starts here */
 if(!strcmp(argv[1],"write_failure")){
  fail_write=1;
  /* Applied and acknowledged; durability is retried, not the receipt. */
  assert(loadcell(2.0f)==SEDS_OK&&daq_calibration_current().kg1000_slope==2.0f&&writes==base+1&&clears==0);
  fail_write=0;
  poll_at(10500);assert(writes==base+1);              /* every second 500 ms tick: once per second */
  poll_at(10999);assert(writes==base+1);
  poll_at(11000);assert(writes==base+2&&stored.kg1000_slope==2.0f);
  poll_at(30000);assert(writes==base+2);              /* nothing pending */
  assert(loadcell(2.0f)==SEDS_OK&&writes==base+2);     /* unchanged: no write */
 }else if(!strcmp(argv[1],"retry_cadence")){
  fail_write=1;assert(loadcell(3.0f)==SEDS_OK);
  const unsigned w=writes;
  for(uint32_t t=10500;t<=14000;t+=500)poll_at(t);  /* the 500 ms GET poll keeps running */
  assert(writes-w==4U);                               /* 11.0, 12.0, 13.0, 14.0 s: never closer than 1 s */
 }else if(!strcmp(argv[1],"newer_value")){
  fail_write=1;assert(loadcell(4.0f)==SEDS_OK&&loadcell(5.0f)==SEDS_OK);
  fail_write=0;poll_at(10500);poll_at(11000);assert(stored.kg1000_slope==5.0f);
 }else if(!strcmp(argv[1],"not_ready")){
  assert(loadcell(6.0f)==SEDS_OK&&daq_calibration_current().kg1000_slope==6.0f&&writes==base);
  poll_at(10500);poll_at(11000);assert(writes==base); /* init retried, still unready: no write */
 }else assert(!"unknown case");
 (void)g_daq_calibration_errors;(void)g_daq_calibration_updates;(void)sd_sets;(void)requests;
 return 0;
}
'''


class CalibrationPersistRetryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="daq-calibration-retry-")
        p = Path(cls.tmp.name)
        for name, text in HEADERS.items():
            (p / name).write_text(text)
        (p / "harness.c").write_text(HARNESS)
        cls.exe = p / "test"
        subprocess.run(["cc", "-DTELEMETRY_ENABLED=1", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-fsanitize=address,undefined", "-I", str(p), "-I", str(ROOT / "Core/Inc"),
                        str(ROOT / "Core/Src/daq_calibration.c"), str(p / "harness.c"),
                        "-lm", "-o", str(cls.exe)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def case(self, name):
        subprocess.run([str(self.exe), name], check=True)

    def test_failed_write_is_acknowledged_and_retried_once_per_second(self):
        self.case("write_failure")

    def test_retry_runs_at_most_once_per_second_beside_the_get_poll(self):
        self.case("retry_cadence")

    def test_pending_retry_persists_the_latest_applied_value(self):
        self.case("newer_value")

    def test_unready_storage_still_applies_and_acknowledges(self):
        self.case("not_ready")


if __name__ == "__main__":
    unittest.main()
