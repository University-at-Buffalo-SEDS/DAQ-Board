"""Compare hot raw serialization with a reference snprintf formatter: the
record's own uptime clock (us for MCP3564R rows), and temperature columns
written only when the value changes (or every SD_RAW_TEMPERATURE_REPEAT rows)."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class RawRecordFormatTests(unittest.TestCase):
    def test_byte_identity_bounds_and_cached_conversion_work(self):
        program = r'''
#include "sd_float_format.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <time.h>
static unsigned conversions;
static void counted_float(char output[24], float value) {++conversions;sd_format_float(output,value);}
#define sd_format_float counted_float
#include "sd_raw_record_format.h"
#undef sd_format_float
static int original(char *line,size_t cap,uint64_t utc,uint64_t uptime,uint64_t fine,uint32_t hz,const char *sensor,
    int32_t code,float raw,float cal,float temp,int32_t tempcode,uint64_t sequence,int with_temp) {
 char raw_text[24],cal_text[24],temp_text[24],stamp[21],ticks[21],fine_text[21],seq[21],code_text[16];
 sd_format_float(raw_text,raw);sd_format_float(cal_text,cal);sd_format_float(temp_text,temp);
 sd_format_u64(stamp,utc?utc:uptime);sd_format_u64(ticks,uptime);sd_format_u64(fine_text,fine);sd_format_u64(seq,sequence);
 snprintf(code_text,sizeof code_text,"%ld",(long)tempcode);
 return snprintf(line,cap,"%s,%s,%s,,%ld,%s,%s,%s,%s,%s,%s,%u,%s\r\n",
   stamp,ticks,sensor,(long)code,raw_text,cal_text,utc?"network":"local",with_temp?temp_text:"",with_temp?code_text:"",
   fine_text,hz,seq);
}
static float value(uint32_t bits) {float v;memcpy(&v,&bits,4);return v;}
int main(void) {
 sd_raw_format_cache_t cache={0};char expected[512],actual[512];
 uint32_t state=42;
 const uint32_t edges[]={0,0x80000000,1,0x7fffff,0x7f7fffff,0x7f800000,0xff800000,0x7fc00000,0xffc00000};
 const char *sensors[]={"kg1000_v","chamber_pressure_v","kg50_p6_v"};
 uint32_t last_temp=0;int32_t last_code=0;unsigned since=0;int have=0;
 for(unsigned i=0;i<20000;i++) {
  state=state*1664525U+1013904223U;float raw=value(i<sizeof(edges)/sizeof(edges[0])?edges[i]:state);
  state=state*1664525U+1013904223U;float cal=i%2?raw:value(state);
  float temp=i%100?26.125f:value(state);int32_t code=(int32_t)state,tempcode=i%50<25?INT32_MIN:INT32_MAX;
  uint64_t utc=i%3?1789136975096ULL-i:0,uptime=i%5?4294967297ULL+i:UINT64_MAX;
  const uint64_t fine=i%4?uptime*1000U+i%1000U:uptime;const uint32_t hz=i%4?1000000U:1000U;
  uint64_t sequence=i%7?i:UINT64_MAX;const char *sensor=sensors[i%3];
  uint32_t bits;memcpy(&bits,&temp,4);
  const int with_temp=!have||bits!=last_temp||tempcode!=last_code||since+1U>=SD_RAW_TEMPERATURE_REPEAT;
  if(with_temp){have=1;last_temp=bits;last_code=tempcode;since=0;}else ++since;
  int old=original(expected,sizeof(expected),utc,uptime,fine,hz,sensor,code,raw,cal,temp,tempcode,sequence,with_temp);
  int fresh=sd_format_csv_raw_record(actual,sizeof(actual),utc,uptime,fine,hz,sensor,code,raw,cal,temp,tempcode,sequence,&cache);
  assert(old==fresh && !strcmp(expected,actual));
 }
 conversions=0;cache=(sd_raw_format_cache_t){0};
 for(unsigned i=0;i<1000;i++)assert(sd_format_csv_raw_record(actual,sizeof(actual),1,i,i,1000,"chamber_pressure_v",10,1.25f,1.25f,26.0f,20,i,&cache)>0);
 assert(conversions==1004); /* One raw format per row; the temperature again every 256 rows. */
 assert(sd_format_csv_raw_record(actual,sizeof(actual),1,1001,1001000,1000000,"chamber_pressure_v",10,1.25f,250.0f,27.0f,20,1001,&cache)>0);
 assert(conversions==1007); /* Independently changing calibration/temperature still formats both. */
 for(size_t capacity=1;capacity<=strlen(actual);capacity++) {
  unsigned char box[520];memset(box,0xa5,sizeof(box));cache=(sd_raw_format_cache_t){0};
  int n=sd_format_csv_raw_record((char*)box,capacity,1,1001,1001000,1000000,"chamber_pressure_v",10,1.25f,250.0f,27.0f,20,1001,&cache);
  assert(n==-1 && !box[0]);for(size_t i=capacity;i<sizeof(box);i++)assert(box[i]==0xa5);
 }
 assert(sd_format_csv_raw_record(NULL,0,0,0,0,1000,"x",0,0,0,0,0,0,&cache)==-1);
 clock_t start=clock();volatile unsigned checksum=0;
 for(unsigned i=0;i<100000;i++) checksum+=(unsigned)original(expected,sizeof(expected),1789136975096ULL,i,i*1000ULL,1000000,"chamber_pressure_v",123,1.25f,250,26,123,i,1);
 double before=(double)(clock()-start)/CLOCKS_PER_SEC;start=clock();
 for(unsigned i=0;i<100000;i++) checksum+=(unsigned)sd_format_csv_raw_record(actual,sizeof(actual),1789136975096ULL,i,i*1000ULL,1000000,"chamber_pressure_v",123,1.25f,250,26,123,i,&cache);
 double after=(double)(clock()-start)/CLOCKS_PER_SEC;
 printf("raw CSV host benchmark: old%.3fs new%.3fs checksum%u; all byte/precision/bounds checks passed\n",before,after,checksum);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary=Path(tmp)/'raw-csv'
            subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
                '-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(binary)],input=program,text=True,check=True)
            subprocess.run([str(binary)],check=True)
