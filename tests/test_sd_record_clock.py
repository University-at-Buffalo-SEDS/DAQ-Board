"""Execute the production clock/CSV helpers, including counter wrap and UTC corrections."""
from pathlib import Path
import csv
import io
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]
class RecordClockTests(unittest.TestCase):
    def test_csv_escaping_counter_wrap_and_backwards_utc(self):
        code = r'''
#include "sd_record_format.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
    board_uptime_state state={0};
    assert(board_uptime_extend(&state,0xfffffffeU)==4294967294ULL);
    assert(board_uptime_extend(&state,1U)==4294967297ULL);
    assert(board_uptime_project(state.ticks,0xfffffffeU)==4294967294ULL);
    assert(board_uptime_project(2U,0xfffffffeU)==0U);
    char digits[21];
    sd_format_u64(digits, UINT64_MAX);
    assert(strcmp(digits, "18446744073709551615") == 0);
    sd_format_u64(digits, 0U); assert(strcmp(digits, "0") == 0);
    sd_format_u64(digits, UINT32_MAX); assert(strcmp(digits, "4294967295") == 0);
    char line[512];
    fputs(SD_CSV_HEADER,stdout);
    assert(sd_format_csv_record(line,sizeof(line),1000000,4294967294ULL,"GYRO","1.0 2.0 -3.0",1)>0);
    fputs(line,stdout);
    assert(sd_format_csv_record(line,sizeof(line),500000,4294967297ULL,"Message","a,\"quoted\"\nsecond line",2)>0);
    fputs(line,stdout);
    struct {char text[4]; char guard[4];} tiny={{0},{1,2,3,4}};
    assert(sd_format_csv_record(tiny.text,4,0,1,"IMU","data",3)==-1);
    assert(!tiny.text[0] && tiny.guard[0]==1 && tiny.guard[3]==4);
}
'''
        with tempfile.TemporaryDirectory() as temp:
            binary=Path(temp)/'clock-record'
            subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror','-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(binary)],input=code,text=True,check=True)
            result=subprocess.check_output([str(binary)],text=True)
        rows=list(csv.DictReader(io.StringIO(result)))
        self.assertEqual(len(rows),2)
        self.assertEqual(rows[0]['value'],'1.0 2.0 -3.0')
        self.assertEqual(rows[1]['value'],'a,"quoted"\nsecond line')
        self.assertEqual(rows[1]['uptime_frequency_hz'],'1000')
        self.assertGreater(int(rows[1]['uptime_ticks']),int(rows[0]['uptime_ticks']))
        self.assertLess(int(rows[1]['timestamp_ms']),int(rows[0]['timestamp_ms']))
        self.assertTrue(all(None not in row for row in rows))
