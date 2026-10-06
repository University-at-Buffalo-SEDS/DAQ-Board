import json
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

class CanPriorityTests(unittest.TestCase):
    def test_opaque_frames_use_logical_priority(self):
        source = (ROOT / "Core/Src/telemetry.c").read_text()
        self.assertIn('seds_router_add_side_packed_profile_with_priority(\n      r, "can", 3U, tx_send_with_priority,', source)
        start = source.index("static SedsResult tx_send_with_priority(")
        end = source.index("\nSedsResult tx_send(", start)
        callback = source[start:end]
        node = 7
        code = r"""
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
typedef int SedsResult;
typedef int HAL_StatusTypeDef;
#define HAL_ERROR 1
#define HAL_OK 0
#define SEDS_OK 0
#define SEDS_BAD_ARG -1
#define SEDS_IO -2
#define SEDS_DT_HEARTBEAT 120
#define SEDS_DT_KG1000 118
#define LED1_GPIO_Port 0
#define LED1_Pin 0
static uint32_t g_daq_loadcell_wire_tx_count, g_daq_loadcell_wire_fail_count;
#define GREEN_LED_GPIO_Port 0
#define GREEN_LED_Pin 0
static uint32_t observed;
static int result;
static void HAL_GPIO_TogglePin(int a,int b){(void)a;(void)b;}
static uint32_t sim_probe_packed_data_type(const uint8_t *p,size_t n){(void)n;return p[0]==120?120:0;}
static int can_bus_send_large(const uint8_t *p,size_t n,uint32_t id){(void)p;(void)n;observed=id;return result;}
static void sim_probe_observe_can_tx(const uint8_t *p,size_t n){(void)p;(void)n;}
""" + callback + r"""
int main(void) {
 uint8_t opaque[][8]={{83,68,84,1},{83,68,84,2},{83,68,7,1}};
 for(unsigned i=0;i<3;i++) {
   for(unsigned p=0;p<256;p++) {
     result=HAL_OK;
     assert(tx_send_with_priority(opaque[i],8,p,NULL)==SEDS_OK);
     assert(observed==(p>=200?NODE:0x100+NODE));
   }
   result=HAL_ERROR;
   assert(tx_send_with_priority(opaque[i],8,255,NULL)==SEDS_IO);
 }
 assert(tx_send_with_priority(NULL,8,255,NULL)==SEDS_BAD_ARG);
 assert(tx_send_with_priority(opaque[0],0,255,NULL)==SEDS_BAD_ARG);
}
"""
        code=code.replace("NODE",str(node))
        with tempfile.TemporaryDirectory() as tmp:
            exe=str(Path(tmp)/"priority")
            subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror","-fsanitize=address,undefined","-x","c","-","-o",exe], input=code,text=True,check=True)
            subprocess.run([exe],check=True)

    def test_commands_and_confirmations_have_protected_priority(self):
        config=json.loads((ROOT/"config/sedsnet.json").read_text())
        types={item["name"]:item for item in config["types"]}
        for name in ["ACTUATOR_COMMAND","VALVE_COMMAND","FLIGHT_COMMAND","UMBILICAL_STATUS"]:
            if name in types:
                self.assertGreaterEqual(types[name]["priority"],200,name)
