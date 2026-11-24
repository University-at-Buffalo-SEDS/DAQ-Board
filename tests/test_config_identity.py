"""The DAQ reports the identity of the calibration it applied, for the station's resync.

BOARD_HEALTH words 13..15 carry the "CFG1" tag, a CRC-32 of the applied
calibration in the station's payload order (loadcell 4, KG50 7, thermal 4,
filter 2 little-endian floats, then the MCP3564R reference float), and flags:
which values came from the station this boot, whether flash supplied them, the
applied 50 kg input, and where the reference came from (bits 9, 10). The
station hashes the payloads it sends, so a match proves the DAQ holds exactly
those bytes, whatever their units.

Builds the actual daq_calibration.c against the stub headers of
test_calibration_persist_retry, with a store that can hold a saved record.
"""
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest
import zlib

from test_calibration_persist_retry import HEADERS

ROOT = Path(__file__).resolve().parents[1]

DEFAULT = [1.0, 0.0, 1.0, 0.0] + [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0] + [0.0] * 4 + [0.0, 0.0]
LOADCELL = [2.5, -1.25, 0.5, 0.125]
KG50 = [0.1, 2.0, 0.0, 0.0, 0.0, 0.25, 0.5]
THERMAL = [25.0, 0.0, 25.0, 0.0]
FILTER = [100.0, 50.0]
STORED = [3.0, 0.0, 1.0, 0.0] + KG50 + THERMAL + FILTER


NOMINAL_VREF = 2.4


def crc(values, vref=NOMINAL_VREF):
    values = list(values) + [vref]
    return zlib.crc32(struct.pack("<%df" % len(values), *values))


def f32(value):
    return struct.unpack("<f", struct.pack("<f", value))[0]


HARNESS = r'''
#include "daq_calibration.h"
#include "sedsnet_config.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
volatile uint32_t g_telemetry_discovery_seen=1, g_reliable_refusal_count;
static uint32_t tick;
static int have_record;
static SedsResult (*callbacks[7])(const SedsPacketView*,void*);
/* A record saved by volts firmware (calibration, then the volts mark), so
 * restore applies it as stored without the raw-scale migration. */
static const struct {float v[17]; uint32_t units;} stored_record={{STORED},0x56544C56u};
uint32_t HAL_GetTick(void){return tick;}
void mcp3564r_request_kg50_input(uint8_t input){(void)input;}
void sd_card_set_calibration(const daq_calibration_t *c){(void)c;}
int persistent_store_init(void){return 0;}
int persistent_store_get(uint32_t k,void*v,size_t*n){
 if(!have_record)return 1;
 if(k==0x44414341u){assert(*n>=sizeof stored_record);memcpy(v,&stored_record,sizeof stored_record);*n=sizeof stored_record;return 0;}
 if(k==0x4441494Eu){*(uint8_t*)v=1;*n=1;return 0;}
 if(k==0x44415652u){const float vref=2.3f;assert(*n==sizeof vref);memcpy(v,&vref,sizeof vref);return 0;}
 return 1;
}
int persistent_store_set(uint32_t k,const void*v,size_t n){(void)k;(void)v;(void)n;return 0;}
SedsResult seds_router_enable_network_variable(SedsRouter*r,uint32_t t,bool a,bool b){(void)r;(void)t;(void)a;(void)b;return 0;}
SedsResult seds_router_on_network_variable_update(SedsRouter*r,uint32_t t,SedsResult(*cb)(const SedsPacketView*,void*),void*u){(void)r;(void)u;callbacks[t]=cb;return 0;}
SedsResult seds_router_request_managed_variable(SedsRouter*r,uint32_t t){(void)r;(void)t;return 0;}
SedsResult seds_router_invalidate_network_variable_cache(SedsRouter*r,uint32_t t,bool*o){(void)r;(void)t;(void)o;return 0;}
static SedsRouter router=1;
static void deliver(uint32_t type,const void*payload,size_t len){SedsPacketView p={type,payload,len};assert(callbacks[type](&p,&router)==SEDS_OK);}
static void report(const char*label){uint32_t flags=0;uint32_t id=daq_calibration_identity(&flags);printf("%s %u %u\n",label,id,flags);}
static void values(const char*label){const daq_calibration_t c=daq_calibration_current();uint32_t source=0;
 const float vref=daq_calibration_mcp_vref(&source);
 printf("%s %.9e %.9e %.9e %.9e %.9e %.9e %u\n",label,(double)vref,(double)c.kg1000_slope,(double)c.kg1000_intercept,
        (double)c.kg50[1],(double)c.kg50[2],(double)c.kg50[5],source);}
int main(int argc,char**argv){
 assert(argc==2);
 have_record=!strcmp(argv[1],"restored");
 assert(daq_calibration_init(&router)==SEDS_OK);
 report("boot");
 const float loadcell[4]={LOADCELL},kg50[7]={KG50},thermal[4]={THERMAL},filter[2]={FILTER};
 const float bad_filter[2]={5000.0f,0.0f};
 const uint8_t input=1;
 deliver(SEDS_DT_DAQ_LOADCELL_CALIBRATION,loadcell,sizeof loadcell);
 report("partial");
 deliver(SEDS_DT_DAQ_KG50_CALIBRATION,kg50,sizeof kg50);
 deliver(SEDS_DT_DAQ_THERMAL_CALIBRATION,thermal,sizeof thermal);
 deliver(SEDS_DT_DAQ_FILTER_CALIBRATION,filter,sizeof filter);
 deliver(SEDS_DT_DAQ_KG50_INPUT,&input,1);
 report("station");
 deliver(SEDS_DT_DAQ_FILTER_CALIBRATION,bad_filter,sizeof bad_filter);
 report("refused");
 values("before");
 /* The station sets this board's measured reference. */
 const float vref=2.2f,too_high=2.7f,too_low=1.8f;
 deliver(SEDS_DT_DAQ_MCP_REFERENCE,&vref,sizeof vref);
 report("vref");
 values("after");
 deliver(SEDS_DT_DAQ_MCP_REFERENCE,&too_high,sizeof too_high);
 deliver(SEDS_DT_DAQ_MCP_REFERENCE,&too_low,sizeof too_low);
 report("vref_refused");
 return 0;
}
'''


def floats(values):
    return ",".join("%.9ef" % v for v in values)


class ConfigIdentityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="daq-config-identity-")
        p = Path(cls.tmp.name)
        for name, text in HEADERS.items():
            (p / name).write_text(text)
        source = (HARNESS.replace("{STORED}", "{" + floats(STORED) + "}")
                  .replace("{LOADCELL}", "{" + floats(LOADCELL) + "}")
                  .replace("{KG50}", "{" + floats(KG50) + "}")
                  .replace("{THERMAL}", "{" + floats(THERMAL) + "}")
                  .replace("{FILTER}", "{" + floats(FILTER) + "}"))
        (p / "harness.c").write_text(source)
        cls.exe = p / "test"
        subprocess.run(["cc", "-DTELEMETRY_ENABLED=1", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-fsanitize=address,undefined", "-I", str(p), "-I", str(ROOT / "Core/Inc"),
                        str(ROOT / "Core/Src/daq_calibration.c"), str(p / "harness.c"),
                        "-lm", "-o", str(cls.exe)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_case(self, name):
        out = subprocess.run([str(self.exe), name], check=True, capture_output=True, text=True).stdout
        rows = {}
        for line in out.splitlines():
            label, *rest = line.split()
            rows[label] = tuple(int(x) for x in rest) if len(rest) == 2 else tuple(float(x) for x in rest)
        return rows

    def test_wiped_storage_reports_the_default_identity_until_the_station_values_arrive(self):
        rows = self.run_case("wiped")
        self.assertEqual(rows["boot"], (crc(DEFAULT), 0), 'the nominal 2.4 V reference until the station sends one')
        self.assertEqual(rows["partial"], (crc(LOADCELL + DEFAULT[4:]), 0x01))
        self.assertEqual(rows["station"], (crc(LOADCELL + KG50 + THERMAL + FILTER), 0x1F | (1 << 8)))

    def test_a_refused_value_leaves_the_identity_of_the_applied_one(self):
        rows = self.run_case("wiped")
        self.assertEqual(rows["refused"], rows["station"])

    def test_flash_restore_is_flagged_and_hashes_the_restored_values(self):
        rows = self.run_case("restored")
        self.assertEqual(rows["boot"], (crc(STORED, f32(2.3)), 0x20 | 0x80 | (1 << 8) | (1 << 10)))
        self.assertEqual(rows["station"][0], crc(LOADCELL + KG50 + THERMAL + FILTER, f32(2.3)))
        self.assertEqual(rows["station"][1], 0x1F | 0x20 | 0x80 | (1 << 8) | (1 << 10))

    def test_reference_change_converts_the_calibration_and_keeps_kg_per_code(self):
        rows = self.run_case("wiped")
        vref, slope, intercept, c1, c2, x0, source = rows["after"]
        before = rows["before"]
        self.assertAlmostEqual(before[0], 2.4, places=6)
        self.assertAlmostEqual(vref, 2.2, places=6)
        self.assertEqual(source, 1, 'from the station this boot')
        r = f32(2.2) / f32(2.4)
        # The same code now reads r times the volts: kg per code is unchanged.
        code_volts_old = 0.5
        self.assertAlmostEqual(before[1] * code_volts_old + before[2], slope * code_volts_old * r + intercept, places=5)
        self.assertAlmostEqual(c1, before[3] / r, places=5)
        self.assertAlmostEqual(c2, before[4] / r ** 2, places=5)
        self.assertAlmostEqual(x0, before[5] * r, places=6)
        self.assertEqual(rows["vref"][1], 0x1F | (1 << 8) | (1 << 9))
        self.assertNotEqual(rows["vref"][0], rows["refused"][0])

    def test_out_of_range_references_are_refused(self):
        rows = self.run_case("wiped")
        self.assertEqual(rows["vref_refused"], rows["vref"])


if __name__ == "__main__":
    unittest.main()
