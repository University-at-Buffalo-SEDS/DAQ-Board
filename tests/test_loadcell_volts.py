"""KG1000/KG50 report ADC-pin volts; legacy raw-scale calibrations keep their kg.

Firmware before this change reported both amplified load cells on a historical
scale, code * 2.2104 / 2^27. The volts scale is code * 2.4 / 2^23, so one raw
unit is exactly k = 16000/921 volts. A stored raw-scale calibration is
converted once at boot; the same ADC code must give the same kg afterwards.
"""
from fractions import Fraction
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

from test_calibration_persist_retry import HEADERS

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r'''
#include "daq_calibration.h"
#include "mcp3564r_board_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
volatile uint32_t g_telemetry_discovery_seen=1, g_reliable_refusal_count;
extern volatile uint32_t g_daq_calibration_restores, g_daq_calibration_migrations, g_daq_calibration_errors;
uint32_t HAL_GetTick(void){return 0;}
void mcp3564r_request_kg50_input(uint8_t input){(void)input;}
void sd_card_set_calibration(const daq_calibration_t *c){(void)c;}
static unsigned char flash[256]; static size_t flash_len; static unsigned writes;
int persistent_store_init(void){return 0;}
int persistent_store_get(uint32_t k,void*v,size_t*n){
 if(k!=0x44414341u||!flash_len){return 1;}
 assert(*n>=flash_len);memcpy(v,flash,flash_len);*n=flash_len;return 0;}
int persistent_store_set(uint32_t k,const void*v,size_t n){
 if(k!=0x44414341u){return 0;}
 assert(n<=sizeof flash);memcpy(flash,v,n);flash_len=n;++writes;return 0;}
SedsResult seds_router_enable_network_variable(SedsRouter*r,uint32_t t,bool a,bool b){(void)r;(void)t;(void)a;(void)b;return 0;}
SedsResult seds_router_on_network_variable_update(SedsRouter*r,uint32_t t,SedsResult(*cb)(const SedsPacketView*,void*),void*u){(void)r;(void)t;(void)cb;(void)u;return 0;}
SedsResult seds_router_request_managed_variable(SedsRouter*r,uint32_t t){(void)r;(void)t;return 0;}
SedsResult seds_router_invalidate_network_variable_cache(SedsRouter*r,uint32_t t,bool*o){(void)r;(void)t;(void)o;return 0;}

static const int32_t codes[]={0,1,-1,1000,-1000,790001,1576000,2097152,-2097152,4000000,
                              8388607,-8388608,0xB40000,-0xB40000,16777215,-16777216};
#define NCODES (sizeof codes/sizeof codes[0])

static float kg1000(const daq_calibration_t*c,float x,float t){
 return c->kg1000_slope*daq_calibration_temperature_raw(c,0,x,t)+c->kg1000_intercept;}
static float kg50(const daq_calibration_t*c,float x,float t){
 return daq_calibration_apply_kg50(c,daq_calibration_temperature_raw(c,1,x,t));}
static void close_kg(float before,float after,float scale){
 /* binary32 coefficients and inputs: a few ulps of the largest term. */
 if(!(fabsf(before-after)<=2e-6f*scale+1e-6f)){fprintf(stderr,"%.9g vs %.9g (scale %g)\n",before,after,scale);assert(0);}}

static void exact_scale(void){
 const double k=MCP3564R_LEGACY_RAW_TO_VOLTS;
 for(unsigned i=0;i<NCODES;++i){
  const double v=mcp3564r_code_to_voltage(codes[i]);
  const double raw=mcp3564r_code_to_legacy_raw_value(codes[i]);
  /* Each float formula rounds at most 4 times; together <= 2^-21 relative. */
  assert(fabs(v-raw*k)<=fabs(v)*4.77e-7);
  /* In exact arithmetic the formulas differ by exactly k. */
  assert(fabs((double)codes[i]*2.4/8388608.0-(double)codes[i]*2.2104/134217728.0*k)<=fabs((double)codes[i])*1e-22);
 }
 assert(mcp3564r_code_to_voltage(8388608)==2.4f);
}

static daq_calibration_t legacy_calibration(void){
 /* The 2026-11-16 test-fire KG1000 fit, a cubic KG50 with shift and tare,
  * and nonzero thermal terms (raw/C). */
 daq_calibration_t c={9481.619f,-115.9939f,1.5f,-0.25f,
   {-2.0f,8000.0f,-150000.0f,4.0e6f,-3.0e7f,0.012f,0.75f},
   {25.0f,1.5e-5f,22.5f,-2.0e-5f},{120.0f,40.0f}};
 return c;
}

static void same_kg_for_same_code(void){
 const daq_calibration_t raw=legacy_calibration();
 daq_calibration_t volts=raw;daq_calibration_from_legacy_raw(&volts);
 const double k=MCP3564R_LEGACY_RAW_TO_VOLTS;
 assert(volts.kg1000_intercept==raw.kg1000_intercept&&volts.kg50[0]==raw.kg50[0]&&volts.kg50[6]==raw.kg50[6]);
 assert(volts.iadc_slope==raw.iadc_slope&&volts.iadc_intercept==raw.iadc_intercept);
 assert(volts.thermal[0]==raw.thermal[0]&&volts.thermal[2]==raw.thermal[2]);
 assert(!memcmp(volts.filter_tau_ms,raw.filter_tau_ms,sizeof raw.filter_tau_ms));
 assert(volts.kg1000_slope==(float)(raw.kg1000_slope/k)&&volts.kg50[5]==(float)(raw.kg50[5]*k));
 assert(volts.kg50[4]==(float)(raw.kg50[4]/(k*k*k*k))&&volts.thermal[1]==(float)(raw.thermal[1]*k));
 const float temps[]={25.0f,18.0f,41.5f};
 for(unsigned t=0;t<3;++t)for(unsigned i=0;i<NCODES;++i){
  if(codes[i]>1600000||codes[i]<-1600000)continue; /* keep the quartic in float range */
  const float r=mcp3564r_code_to_legacy_raw_value(codes[i]),v=mcp3564r_code_to_voltage(codes[i]);
  close_kg(kg1000(&raw,r,temps[t]),kg1000(&volts,v,temps[t]),fabsf(raw.kg1000_slope*r)+fabsf(raw.kg1000_intercept));
  const float x=r-raw.kg50[5];
  float scale=0;for(int p=0;p<5;++p)scale+=fabsf(raw.kg50[p]*powf(x,(float)p));
  close_kg(kg50(&raw,r,temps[t]),kg50(&volts,v,temps[t]),scale+fabsf(raw.kg50[6]));
 }
 /* daq_calibration_apply_kg1000 uses the restored coefficients too. */
}

static void boot_migrates_once(size_t legacy_size){
 daq_calibration_t raw=legacy_calibration();
 memset(flash,0,sizeof flash);memcpy(flash,&raw,legacy_size);flash_len=legacy_size;writes=0;
 const uint32_t migrations=g_daq_calibration_migrations,errors=g_daq_calibration_errors;
 daq_calibration_restore();
 daq_calibration_t expected={1.0f,0.0f,1.0f,0.0f,{0.0f,1.0f},{0},{0}};
 memcpy(&expected,&raw,legacy_size);daq_calibration_from_legacy_raw(&expected);
 const daq_calibration_t now=daq_calibration_current();
 assert(!memcmp(&now,&expected,sizeof now));
 assert(writes==1&&flash_len==sizeof(daq_calibration_t)+4&&g_daq_calibration_migrations==migrations+1&&g_daq_calibration_errors==errors);
 uint32_t mark;memcpy(&mark,flash+sizeof(daq_calibration_t),4);assert(mark==0x56544C56u);
 const float v=mcp3564r_code_to_voltage(790001),r=mcp3564r_code_to_legacy_raw_value(790001);
 close_kg(raw.kg1000_slope*r+raw.kg1000_intercept,daq_calibration_apply_kg1000(v),
          fabsf(raw.kg1000_slope*r)+fabsf(raw.kg1000_intercept));
}

int main(int argc,char**argv){
 assert(argc==2);
 if(!strcmp(argv[1],"exact"))exact_scale();
 else if(!strcmp(argv[1],"same_kg"))same_kg_for_same_code();
 else if(!strcmp(argv[1],"boot")){
  extern void daq_calibration_test_reset(void);
  const size_t sizes[]={4*sizeof(float),11*sizeof(float),15*sizeof(float),sizeof(daq_calibration_t)};
  for(unsigned i=0;i<4;++i){daq_calibration_test_reset();boot_migrates_once(sizes[i]);}
  /* The next boot reads the volts record and does not convert again. */
  const daq_calibration_t migrated=daq_calibration_current();
  daq_calibration_test_reset();writes=0;daq_calibration_restore();
  const daq_calibration_t again=daq_calibration_current();
  assert(!memcmp(&again,&migrated,sizeof again)&&writes==0);
  /* No record: the identity defaults stay untouched. */
  flash_len=0;daq_calibration_test_reset();daq_calibration_restore();
  assert(daq_calibration_current().kg1000_slope==1.0f&&daq_calibration_current().kg50[1]==1.0f&&writes==0);
 }else assert(!"unknown case");
 return 0;
}
'''


class LoadcellVoltsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="daq-loadcell-volts-")
        p = Path(cls.tmp.name)
        for name, text in HEADERS.items():
            (p / name).write_text(text)
        (p / "daq_chamber_pressure.h").write_text("#pragma once\n#define DAQ_CHAMBER_SCAN_MASK 0U\n")
        # Reset the restore latch between simulated boots without touching production code.
        source = (ROOT / "Core/Src/daq_calibration.c").read_text()
        source += ("\nvoid daq_calibration_test_reset(void){g_restore_attempted=false;"
                   "g_calibration=(daq_calibration_t){1.0f,0.0f,1.0f,0.0f,{0.0f,1.0f},{0},{0}};}\n")
        (p / "daq_calibration_under_test.c").write_text(source)
        (p / "harness.c").write_text(HARNESS)
        cls.exe = p / "test"
        subprocess.run(["cc", "-DTELEMETRY_ENABLED=1", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-Wno-missing-prototypes", "-fsanitize=address,undefined",
                        "-I", str(p), "-I", str(ROOT / "Core/Inc"),
                        str(p / "daq_calibration_under_test.c"), str(p / "harness.c"),
                        "-lm", "-o", str(cls.exe)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def case(self, name):
        subprocess.run([str(self.exe), name], check=True)

    def test_factor_is_the_exact_ratio_of_the_two_formulas(self):
        volts_per_code = Fraction(24, 10) / 2**23
        legacy_raw_per_code = Fraction(22104, 10000) / 2**24 * 2 / 16
        self.assertEqual(volts_per_code / legacy_raw_per_code, Fraction(16000, 921))
        header = (ROOT / "Core/Inc/mcp3564r_board_config.h").read_text()
        self.assertRegex(header, r"#define MCP3564R_LEGACY_RAW_TO_VOLTS \(16000\.0 / 921\.0\)")
        self.assertIn("2.2104f", header)  # legacy formula retained bit-exact for migration

    def test_voltage_equals_legacy_raw_times_factor_for_every_code(self):
        self.case("exact")

    def test_converted_coefficients_give_the_same_kg_for_the_same_code(self):
        self.case("same_kg")

    def test_boot_converts_each_legacy_record_size_once_and_stores_volts(self):
        self.case("boot")

    def test_load_cells_report_volts_on_the_wire_and_sd(self):
        driver = (ROOT / "Core/Src/mcp3564r.c").read_text()
        self.assertIn("sample->raw_value = sample->voltage_v;", driver)
        self.assertNotIn("code_to_legacy_raw_value", driver)
        sd = (ROOT / "Core/Src/sd_card.c").read_text()
        line = re.search(r'"# calibration,[^"]*"', sd).group(0)
        self.assertTrue(line.startswith('"# calibration,kg1000_slope='))
        self.assertTrue(line.endswith(',loadcell_input=adc_volts,loadcell_report=median_bin,kg1000_median=%u,'
                                      'kg1000_bin_us=%u,kg50_median=%u,kg50_bin_us=%u,adc_uptime_hz=1000000\\r\\n"'), line)


if __name__ == "__main__":
    unittest.main()
