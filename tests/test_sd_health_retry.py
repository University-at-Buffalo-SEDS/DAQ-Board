"""Production SD progress ages, nonblocking retry lifecycle and command admission."""
from pathlib import Path
import json
import subprocess
import tempfile
import unittest
from test_sd_raw_queue import function
ROOT=Path(__file__).resolve().parents[1]

class SdHealthRetryTests(unittest.TestCase):
    def run_c(self, code):
        with tempfile.TemporaryDirectory() as d:
            exe=Path(d)/'sd-health'
            compiled=subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror',
                '-fsanitize=address,undefined','-x','c','-','-o',str(exe)],input=code,text=True,capture_output=True)
            self.assertEqual(compiled.returncode,0,compiled.stderr)
            subprocess.run([str(exe)],check=True)

    def test_actual_progress_retry_dedup_boundary_and_monotonic_ages(self):
        source=(ROOT/'Core/Src/sd_card.c').read_text()
        code=r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
typedef unsigned UINT;typedef unsigned char UCHAR;typedef uint32_t ULONG;typedef unsigned sd_card_status_t;
enum{FX_SUCCESS=0,FX_IO_ERROR=1,SD_CARD_STATUS_OK=0,SD_CARD_STATUS_BUSY=1,SD_CARD_STATUS_IO_ERROR=3};
#define SD_TRANSFER_SECTORS 8U
#define SD_SECTOR_SIZE 512U
static unsigned mask;static uint64_t now;static uint8_t g_sd_launch_closed;
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t m){mask=m;}
static uint64_t board_uptime_ticks(void){return now;}
static uint32_t HAL_GetTick(void){return (uint32_t)now;}
static uint8_t g_sd_manual_no_format,g_sd_services_initialized=1,g_sd_ready=1;
static uint32_t g_sd_retry_pending_token,g_sd_retry_active_token,g_sd_retry_accepted_token,g_sd_retry_completed_token;
static uint32_t g_sd_retry_recent[8],g_sd_retry_recent_next;
static uint64_t g_sd_last_sector_write_ms,g_sd_last_media_flush_ms;
static uint8_t g_sd_write_progress_valid,g_sd_flush_progress_valid;
static uint32_t g_sd_init_stage=5,g_sd_retry_count=20,g_sd_init_failures=4,g_sd_max_write_ms;
static _Alignas(32) UCHAR g_sd_transfer_buffer[4096];
static unsigned io_calls,fail_io_call,needs_reinit;
static UINT sd_storage_io_write(const UCHAR *b,ULONG s,ULONG n){assert(!mask&&b==g_sd_transfer_buffer&&n<=8);(void)s;now+=10;++io_calls;return io_calls==fail_io_call?FX_IO_ERROR:FX_SUCCESS;}
static unsigned sd_storage_io_requires_reinit(void){return needs_reinit;}
static unsigned g_sd_media,flush_result;
static UINT fx_media_flush(unsigned *m){assert(!mask&&m==&g_sd_media);return flush_result;}
'''
        for name in ['sd_card_request_retry','sd_take_manual_retry','sd_complete_manual_retry','sd_progress_age','sd_card_health_snapshot','sd_hal_write','sd_media_flush']:
            code+=function(source,name)+'\n'
        code+=r'''
int main(void){
 uint32_t h[8];sd_card_health_snapshot(h);assert(h[0]==3&&h[1]==UINT32_MAX&&h[2]==UINT32_MAX);
 assert(sd_card_request_retry(0)==SD_CARD_STATUS_IO_ERROR);
 assert(sd_card_request_retry(101)==0&&sd_card_request_retry(101)==0);
 assert(sd_card_request_retry(102)==SD_CARD_STATUS_BUSY);
 sd_card_health_snapshot(h);assert(h[0]==11&&h[5]==101&&!h[6]);
 assert(sd_take_manual_retry()==101&&g_sd_manual_no_format&&!g_sd_retry_pending_token&&g_sd_retry_active_token==101);
 assert(!sd_take_manual_retry()&&sd_card_request_retry(102)==SD_CARD_STATUS_BUSY);
 sd_card_health_snapshot(h);assert(h[0]&8);sd_complete_manual_retry();
 sd_card_health_snapshot(h);assert(!(h[0]&8)&&h[6]==101&&h[4]==20&&h[7]==4);
 assert(sd_card_request_retry(101)==0&&!g_sd_retry_pending_token);
 assert(sd_card_request_retry(102)==0);assert(sd_take_manual_retry()==102);
 g_sd_launch_closed=1;g_sd_ready=0;sd_complete_manual_retry();sd_card_health_snapshot(h);
 assert(h[6]==102&&(h[0]&4)&&!(h[0]&8)); /* completed attempt does not imply ready */
 for(unsigned token=103;token<110;token++){assert(!sd_card_request_retry(token));assert(sd_take_manual_retry()==token);sd_complete_manual_retry();}
 assert(sd_card_request_retry(105)==0&&!g_sd_retry_pending_token); /* bounded recent dedup */
 UCHAR bytes[8192]={0};now=10;fail_io_call=2;needs_reinit=1;
 assert(sd_hal_write(bytes,0,16)==FX_IO_ERROR&&!g_sd_ready);
 now=100;sd_card_health_snapshot(h);assert(h[1]==80&&h[2]==UINT32_MAX); /* only first successful chunk */
 flush_result=FX_IO_ERROR;assert(sd_media_flush()==FX_IO_ERROR);
 now=200;flush_result=FX_SUCCESS;assert(sd_media_flush()==FX_SUCCESS);
 now=240;sd_card_health_snapshot(h);assert(h[2]==40);
 now=250;flush_result=FX_IO_ERROR;assert(sd_media_flush()==FX_IO_ERROR);
 sd_card_health_snapshot(h);assert(h[2]==50); /* failed flush cannot look fresh */
 now+=(uint64_t)UINT32_MAX+500;sd_card_health_snapshot(h);
 assert(h[1]==UINT32_MAX-1&&h[2]==UINT32_MAX-1&&!mask);
}
'''
        self.run_c(code)

    def test_direct_retry_command_validates_exact_sender_type_and_payload(self):
        source=(ROOT/'Core/Src/telemetry.c').read_text()
        code=r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
typedef int SedsResult;typedef int sd_card_status_t;
enum{SEDS_OK=0,SEDS_IO=1,SEDS_HANDLER_ERROR=2,SD_CARD_STATUS_OK=0,SD_CARD_STATUS_BUSY=1};
#define SEDS_DT_DAQ_SD_RETRY 24116261U
typedef struct{uint32_t ty;const char *sender;size_t sender_len;const uint8_t *payload;size_t payload_len;} SedsPacketView;
static unsigned calls;static uint32_t accepted;static int status;
static sd_card_status_t sd_card_request_retry(uint32_t token){calls++;accepted=token;return status;}
'''+function(source,'sd_retry_receive')+r'''
int main(void){
 uint8_t p[]={0x78,0x56,0x34,0x12};SedsPacketView packet={SEDS_DT_DAQ_SD_RETRY,"GS",2,p,4};
 assert(sd_retry_receive(&packet,0)==SEDS_OK&&calls==1&&accepted==0x12345678);
 packet.sender="RRLS";packet.sender_len=4;status=SD_CARD_STATUS_BUSY;
 assert(sd_retry_receive(&packet,0)==SEDS_IO&&calls==2);
 packet.sender="VB";packet.sender_len=2;assert(sd_retry_receive(&packet,0)==SEDS_HANDLER_ERROR&&calls==2);
 packet.sender="GSx";packet.sender_len=3;assert(sd_retry_receive(&packet,0)==SEDS_HANDLER_ERROR);
 packet.sender="GS";packet.sender_len=2;packet.payload_len=8;assert(sd_retry_receive(&packet,0)==SEDS_HANDLER_ERROR);
 packet.payload_len=3;assert(sd_retry_receive(&packet,0)==SEDS_HANDLER_ERROR);
 packet.payload_len=4;packet.ty++;assert(sd_retry_receive(&packet,0)==SEDS_OK&&calls==2);
 assert(sd_retry_receive(0,0)==SEDS_HANDLER_ERROR&&calls==2);
}
'''
        self.run_c(code)

    def test_schema_identity_and_legacy_health_shape(self):
        schema=json.loads((ROOT/'config/sedsnet.json').read_text())
        entries={entry['name']:entry for entry in schema['types']}
        self.assertEqual(entries['BOARD_HEALTH']['element']['count'],24)
        self.assertEqual(entries['DAQ_SD_HEALTH']['element'],dict(kind='Static',data_type='UInt32',count=8))
        retry=entries['DAQ_SD_RETRY']
        self.assertEqual((retry['reliable'],retry['endpoints']),('Unordered',['FillSdCard']))
        self.assertEqual(retry['element'],dict(kind='Static',data_type='UInt32',count=1))
        self.assertNotIn('managed',str(retry.get('class','')).lower())
        header=(ROOT/'Core/Inc/sedsnet_config.h').read_text()
        for name in ['DAQ_SD_RETRY','DAQ_SD_HEALTH']:
            value=0x811c9dc5^0x5459
            for byte in name.encode():value=((value^byte)*0x01000193)&0xffffffff
            value=0x10000|(value&0x0fffffff)
            self.assertIn(f'#define SEDS_DT_{name} ((uint32_t){value}U)',header)

    def test_existing_gateway_can_boundary_preserves_new_unknown_type_bytes(self):
        gateway = ROOT.parent/'gateway-board/Core/Src/telemetry.c'
        if not gateway.is_file():
            self.skipTest('Workspace-level check requires the gateway peer checkout')
        source = gateway.read_text()
        code = r"""
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#define TELEMETRY_ENABLED
#define SEDS_OK 0
typedef int SedsResult;
static struct{void *r;}g_router={(void*)1};static int g_can_side_id=3;
static unsigned g_gateway_can_rx_handoff_fail,g_gateway_can_admission_fail,g_gateway_can_queue_fail,g_telemetry_discovery_seen;
static uint32_t g_gateway_can_rejected_word0,g_gateway_can_rejected_word1,g_gateway_can_rejected_word2,g_gateway_can_rejected_word3;
static int g_gateway_can_rx_handoff_last;
static unsigned delivered,locked;static const uint8_t *expected;static size_t expected_length;
static unsigned init_telemetry_router(void){return 0;}
static size_t seds_memory_admission_rejected(void){return 0;}
static void telemetry_lock(void){assert(!locked);locked=1;}
static void telemetry_unlock(void){assert(locked);locked=0;}
static void telemetry_signal_deserialize_failure(void){assert(0);}
static int seds_router_rx_packed_packet_to_queue_from_side(void *r,uint32_t side,const uint8_t *p,size_t n){
 assert(r&&side==3&&locked&&n==expected_length&&!memcmp(p,expected,n));delivered++;return 0;
}
static int seds_router_rx_packed_packet_to_queue(void *r,const uint8_t *p,size_t n){(void)r;(void)p;(void)n;assert(0);return -1;}
""" + function(source[source.index('void rx_asynchronous('):],'rx_asynchronous')
        def uleb(value):
            out=[]
            while value>=128:out.append((value&127)|128);value>>=7
            return bytes(out+[value])
        import zlib
        frames=[]
        for ty,payload in [(92486907,bytes(range(32))),(24116261,b'\x01\x02\x03\x04')]:
            frame=b'\x00\x01'+uleb(ty)+uleb(len(payload))+uleb(1000)+uleb(123456)+payload
            frames.append(frame+zlib.crc32(frame).to_bytes(4,'little'))
        code+='\nint main(void){\n'
        for i,frame in enumerate(frames):
            code+=f'const uint8_t p{i}[]={{'+','.join(str(v) for v in frame)+'};\n'
            code+=f'expected=p{i};expected_length=sizeof(p{i});rx_asynchronous(p{i},sizeof(p{i}));\n'
        code+='assert(delivered==2&&!g_gateway_can_rx_handoff_fail&&!locked&&g_telemetry_discovery_seen);return 0;}\n'
        self.run_c(code)
