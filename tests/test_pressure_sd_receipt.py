"""A pressure delivery receipt means its entire batch entered the SD writer."""
from pathlib import Path
import subprocess,tempfile,unittest
from test_sd_raw_queue import function
ROOT=Path(__file__).resolve().parents[1]
class PressureReceiptTests(unittest.TestCase):
    def test_no_success_receipt_for_refused_atomic_sd_handoff(self):
        source=(ROOT/'Core/Src/telemetry.c').read_text()
        helper=function(source,'pressure_sd_receive')
        harness=r'''#include "pressure_sd_wire.h"
#include <assert.h>
typedef int SedsResult;
#define SEDS_OK 0
#define SEDS_IO -14
#define SEDS_HANDLER_ERROR -7
#define SEDS_DT_VALVE_PRESSURE_SD_V1 157797238
#define SD_CARD_STATUS_OK 0
static unsigned g_pressure_sd_batches_received,g_pressure_sd_invalid_batches,calls;
static int status;
typedef struct {int ty;const uint8_t *payload;size_t payload_len;}SedsPacketView;
int sd_card_log_pressure_batch(const pressure_sd_batch_t *batch){++calls;assert(batch->first_sequence==42);return status;}
''' + helper + r'''
int main(void){
 pressure_sd_batch_t batch={.first_sequence=42,.uptime_ms=100};uint8_t wire[44];
 assert(pressure_sd_encode(wire,44,&batch)==44);
 SedsPacketView packet={SEDS_DT_VALVE_PRESSURE_SD_V1,wire,44};
 assert(pressure_sd_receive(NULL,NULL)==SEDS_OK && calls==0);
 status=1;assert(pressure_sd_receive(&packet,NULL)==SEDS_IO);
 assert(calls==1 && g_pressure_sd_batches_received==0);
 status=0;assert(pressure_sd_receive(&packet,NULL)==SEDS_OK);
 assert(calls==2 && g_pressure_sd_batches_received==1);
 packet.payload_len=43;assert(pressure_sd_receive(&packet,NULL)==SEDS_HANDLER_ERROR);
 assert(calls==2 && g_pressure_sd_invalid_batches==1);
 return 0;
}'''
        with tempfile.TemporaryDirectory() as d:
            exe=Path(d)/'receipt'
            subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(exe)],input=harness,text=True,check=True)
            subprocess.run([str(exe)],check=True)
