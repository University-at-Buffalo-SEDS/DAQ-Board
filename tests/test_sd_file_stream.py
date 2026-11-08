"""SD download service: actual wire codec and stream/slot owner with fake SDK,
HAL and flight-state boundaries. No FileX, card or packed-transport claim."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

HEADERS = {
    'main.h': r'''#pragma once
#include <stdint.h>
extern uint32_t fake_ms, fake_primask;
static inline uint32_t HAL_GetTick(void){return fake_ms;}
static inline uint32_t __get_PRIMASK(void){return fake_primask;}
static inline void __disable_irq(void){fake_primask=1;}
static inline void __set_PRIMASK(uint32_t p){fake_primask=p;}
''',
    'flight_state_cache.h': r'''#pragma once
#include <stdbool.h>
#include <stdint.h>
extern bool fake_has_state; extern uint8_t fake_state;
static inline bool flight_state_cache_has_value(void){return fake_has_state;}
static inline uint8_t flight_state_cache_value(void){return fake_state;}
''',
    'sedsnet_config.h': r'''#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef int32_t SedsResult;
#define SEDS_OK 0
#define SEDS_ERR (-1)
#define SEDS_BAD_ARG (-2)
#define SEDS_P2P_STREAM_ACCEPTED 1U
#define SEDS_P2P_STREAM_CONNECTED 2U
#define SEDS_P2P_STREAM_DATA 3U
#define SEDS_P2P_STREAM_CLOSED 4U
#define SEDS_P2P_STREAM_RESET 5U
typedef struct SedsRouter { int id; } SedsRouter;
typedef struct { uint8_t kind; uint32_t stream_id, peer_stream_id, sequence; const char *peer_hostname;
  size_t peer_hostname_len; uint32_t peer_address; uint16_t local_port, peer_port;
  const uint8_t *payload; size_t payload_len; } SedsP2pStreamEventView;
typedef SedsResult (*SedsP2pStreamHandlerFn)(const SedsP2pStreamEventView *, void *);
SedsResult seds_router_bind_p2p_stream_port(SedsRouter *, uint16_t, SedsP2pStreamHandlerFn, void *);
SedsResult seds_router_send_p2p_stream(SedsRouter *, uint32_t, const uint8_t *, size_t);
SedsResult seds_router_reset_p2p_stream(SedsRouter *, uint32_t);
int32_t seds_router_has_queued_work(SedsRouter *);
bool seds_router_sender_matches_hostname(const SedsRouter *, const char *, size_t, const char *, size_t);
''',
    'sedsnet.h': '#pragma once\n#include "sedsnet_config.h"\n',
    'telemetry_tlsf.h': r'''#pragma once
#include <stdbool.h>
#include <stddef.h>
bool telemetry_tlsf_admit(size_t additional, size_t largest);
void *telemetry_tlsf_malloc(size_t size);
void telemetry_tlsf_free(void *ptr);
''',
}

HARNESS = r'''
#include "sd_file_stream.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
uint32_t fake_ms=1000, fake_primask; bool fake_has_state=true; uint8_t fake_state=1;
static SedsP2pStreamHandlerFn handler; static SedsRouter router;
static uint8_t frames[64][128]; static size_t frame_len[64]; static unsigned nframes, resets; static int queued;
SedsResult seds_router_bind_p2p_stream_port(SedsRouter *r, uint16_t port, SedsP2pStreamHandlerFn f, void *u)
{ (void)u; assert(r==&router && port==4511); handler=f; return SEDS_OK; }
SedsResult seds_router_send_p2p_stream(SedsRouter *r, uint32_t id, const uint8_t *p, size_t n)
{ assert(r==&router && id==7 && n<=SD_FILE_FRAME_MAX && nframes<64); memcpy(frames[nframes],p,n); frame_len[nframes++]=n; return SEDS_OK; }
SedsResult seds_router_reset_p2p_stream(SedsRouter *r, uint32_t id){ assert(r==&router && id==7); resets++; return SEDS_OK; }
int32_t seds_router_has_queued_work(SedsRouter *r){ (void)r; return queued; }
bool seds_router_sender_matches_hostname(const SedsRouter *r, const char *s, size_t n, const char *h, size_t hn)
{ (void)r; return n==hn && !memcmp(s,h,n); }
static bool heap_ok=true; static unsigned heap_live; static size_t heap_last;
bool telemetry_tlsf_admit(size_t a, size_t l){ (void)a; (void)l; return heap_ok; }
void *telemetry_tlsf_malloc(size_t n){ heap_live++; heap_last=n; return malloc(n); }
void telemetry_tlsf_free(void *p){ assert(p && heap_live); heap_live--; free(p); }
size_t sd_card_download_handle_size(void){ return 300; }

static SedsResult event(uint8_t kind, const char *host, const uint8_t *p, size_t n)
{
  SedsP2pStreamEventView e={kind,7,9,0,host,strlen(host),0x1001,4511,50000,p,n};
  return handler(&e,NULL);
}
static void poll_many(unsigned n){ for(unsigned i=0;i<n;i++){ fake_ms+=3; sd_file_stream_poll(); } }
static size_t read_req(uint8_t *b, uint32_t offset, const char *name)
{ size_t n=strlen(name); b[0]=SD_FILE_OP_READ; sd_file_put_u32(b+1,offset); b[5]=SD_FILE_DIR_LAUNCH; b[6]=(uint8_t)n; memcpy(b+7,name,n); return 7+n; }

static void wire_cases(void)
{
  sd_file_request_t r; uint8_t b[128];
  assert(sd_file_parse_request((const uint8_t[]){0x21,0,5,1},4,&r)==0 && r.op==0x21 && r.index==261);
  assert(sd_file_parse_request((const uint8_t[]){0x21,3,0,0},4,&r)==SD_FILE_STATUS_BAD_MESSAGE);
  assert(sd_file_parse_request((const uint8_t[]){0x21,0,0},3,&r)==SD_FILE_STATUS_BAD_MESSAGE);
  assert(sd_file_parse_request((const uint8_t[]){0x23},1,&r)==0 && r.op==0x23);
  assert(sd_file_parse_request((const uint8_t[]){0x23,0},2,&r)==SD_FILE_STATUS_BAD_MESSAGE);
  size_t n=read_req(b,1234567,"DAQ_LAUNCH_1700000000000_000.CSV");
  assert(sd_file_parse_request(b,n,&r)==0 && r.offset==1234567 && !strcmp(r.name,"DAQ_LAUNCH_1700000000000_000.CSV"));
  assert(sd_file_parse_request(b,n-1,&r)==SD_FILE_STATUS_BAD_MESSAGE);
  const char *bad[]={"..","a/b","..\\x","x y","",".", "a:b"};
  for(unsigned i=0;i<sizeof bad/sizeof bad[0];i++){ n=read_req(b,0,bad[i]); assert(sd_file_parse_request(b,n,&r)!=0); }
  char longname[65]; memset(longname,'A',64); longname[64]=0; n=read_req(b,0,longname); assert(sd_file_parse_request(b,n,&r)!=0);
  longname[63]=0; n=read_req(b,0,longname); assert(sd_file_parse_request(b,n,&r)==0);
  char path[80]; assert(sd_file_path(path,sizeof path,SD_FILE_DIR_NO_UTC,"X.CSV",5)==13 && !strcmp(path,"/NO_UTC/X.CSV"));
  assert(sd_file_path(path,sizeof path,9,"X.CSV",5)==0);
  for(unsigned s=0;s<16;s++) assert(sd_file_state_allowed(true,(uint8_t)s)==(s==1||s==13||s==14));
  assert(!sd_file_state_allowed(false,1));
  uint8_t f[SD_FILE_FRAME_MAX]; uint8_t data[100]; memset(data,0x5a,sizeof data);
  assert(sd_file_encode_data(f,sizeof f,0,1,512,9000,data,100)==111 && f[0]==0xa2 && sd_file_get_u32(f+3)==512 && sd_file_get_u32(f+7)==9000);
  assert(sd_file_encode_data(f,sizeof f,0,1,0,0,data,101)==0);
  assert(sd_file_encode_list(f,sizeof f,0,1,3,77,1,"A.CSV",5)==17 && f[0]==0xa1 && f[2]==1 && sd_file_get_u16(f+3)==3 && f[9]==1 && f[10]==5);
}

static void read_flow(void)
{
  uint8_t b[128]; size_t n;
  assert(sd_file_stream_init(&router)==SEDS_OK && handler);
  assert(event(SEDS_P2P_STREAM_ACCEPTED,"RF",NULL,0)==SEDS_BAD_ARG);
  fake_state=4; assert(event(SEDS_P2P_STREAM_ACCEPTED,"GS",NULL,0)==SEDS_ERR); /* Fill refused */
  /* No heap headroom: the session is refused and nothing is allocated. */
  heap_ok=false; assert(event(SEDS_P2P_STREAM_ACCEPTED,"GS",NULL,0)==SEDS_ERR && heap_live==0 && !sd_file_stream_block());
  heap_ok=true;
  fake_state=1; assert(event(SEDS_P2P_STREAM_ACCEPTED,"GS",NULL,0)==SEDS_OK && sd_file_stream_session_active());
  assert(heap_live==1 && heap_last==812 && sd_file_stream_block() && (uint8_t*)sd_file_stream_handle()==sd_file_stream_block()+512);
  n=read_req(b,700,"F.CSV"); assert(event(SEDS_P2P_STREAM_DATA,"GS",b,n)==SEDS_OK);
  /* A second request while one is outstanding is answered BUSY, never queued. */
  assert(event(SEDS_P2P_STREAM_DATA,"GS",b,n)==SEDS_OK && nframes==1 && frames[0][1]==SD_FILE_STATUS_BUSY);
  nframes=0;
  sd_file_request_t req; uint32_t ticket;
  assert(sd_file_stream_claim(&req,&ticket) && req.offset==700 && !sd_file_stream_claim(&req,&ticket));
  uint8_t *blk=sd_file_stream_block(); for(unsigned i=0;i<512;i++) blk[i]=(uint8_t)(i*7);
  sd_file_result_t res={.status=0,.file_size=5000,.offset=700,.data_start=700-512,.data_len=512-(700-512)};
  sd_file_stream_finish(ticket,&res);
  /* Busy router: no frame before the long gap, then exactly one. */
  queued=1; fake_ms+=3; sd_file_stream_poll(); assert(nframes==1); /* long idle gap */
  fake_ms+=3; sd_file_stream_poll(); assert(nframes==1);
  fake_ms+=30; sd_file_stream_poll(); assert(nframes==2);
  queued=0; poll_many(10);
  assert(nframes==4); /* 324 bytes -> 100,100,100,24 */
  uint32_t expect=700; for(unsigned i=0;i<nframes;i++){
    assert(frames[i][0]==0xa2 && frames[i][1]==0 && sd_file_get_u32(frames[i]+3)==expect && sd_file_get_u32(frames[i]+7)==5000);
    size_t len=frame_len[i]-SD_FILE_DATA_HEADER; for(size_t k=0;k<len;k++) assert(frames[i][SD_FILE_DATA_HEADER+k]==(uint8_t)((expect-512+k)*7));
    assert((frames[i][2]&SD_FILE_DATA_LAST)==(i==nframes-1)); expect+=(uint32_t)len; }
  assert(expect==1024);
  /* Minimum spacing: two polls in the same millisecond send at most one frame. */
  nframes=0; n=read_req(b,1024,"F.CSV"); assert(event(SEDS_P2P_STREAM_DATA,"GS",b,n)==SEDS_OK);
  assert(sd_file_stream_claim(&req,&ticket)); res.offset=1024; res.data_start=0; res.data_len=512; sd_file_stream_finish(ticket,&res);
  fake_ms+=5; sd_file_stream_poll(); sd_file_stream_poll(); assert(nframes==1);
  /* CANCEL drops the remaining answer immediately. */
  uint8_t cancel=SD_FILE_OP_CANCEL; assert(event(SEDS_P2P_STREAM_DATA,"GS",&cancel,1)==SEDS_OK);
  assert(nframes==2 && frames[1][0]==0xa3 && frames[1][1]==0); poll_many(10); assert(nframes==2);
  /* CANCEL while the writer is busy discards its late answer. */
  assert(event(SEDS_P2P_STREAM_DATA,"GS",b,n)==SEDS_OK && sd_file_stream_claim(&req,&ticket));
  assert(event(SEDS_P2P_STREAM_DATA,"GS",&cancel,1)==SEDS_OK); sd_file_stream_finish(ticket,&res); poll_many(10); assert(nframes==3);
  /* LIST answer carries the name from the block buffer. */
  nframes=0; assert(event(SEDS_P2P_STREAM_DATA,"GS",(const uint8_t[]){0x21,0,2,0},4)==SEDS_OK);
  assert(sd_file_stream_claim(&req,&ticket) && req.op==0x21 && req.index==2);
  memcpy(sd_file_stream_block(),"L.CSV",5); sd_file_result_t lres={.status=0,.flags=1,.file_size=42,.name_len=5};
  sd_file_stream_finish(ticket,&lres); poll_many(2);
  assert(nframes==1 && frames[0][0]==0xa1 && sd_file_get_u32(frames[0]+5)==42 && frames[0][10]==5 && !memcmp(frames[0]+12,"L.CSV",5));
}

static void state_abort(void)
{
  uint8_t b[128]; size_t n=read_req(b,0,"F.CSV"); sd_file_request_t req; uint32_t ticket;
  nframes=0; assert(event(SEDS_P2P_STREAM_DATA,"GS",b,n)==SEDS_OK && sd_file_stream_claim(&req,&ticket));
  sd_file_result_t res={.status=0,.file_size=512,.offset=0,.data_start=0,.data_len=512};
  sd_file_stream_finish(ticket,&res); fake_ms+=5; sd_file_stream_poll(); assert(nframes==1);
  fake_state=6; /* Armed: the next poll resets the stream and sends nothing more. */
  unsigned before=resets; fake_ms+=5; sd_file_stream_poll(); assert(resets==before+1 && nframes==1 && !sd_file_stream_session_active());
  poll_many(10); assert(nframes==1);
  assert(event(SEDS_P2P_STREAM_DATA,"GS",b,n)==SEDS_ERR); /* old session gone */
  assert(event(SEDS_P2P_STREAM_ACCEPTED,"GS",NULL,0)==SEDS_ERR);
  fake_state=7; assert(event(SEDS_P2P_STREAM_ACCEPTED,"GS",NULL,0)==SEDS_ERR);
  fake_state=13; assert(event(SEDS_P2P_STREAM_ACCEPTED,"GS",NULL,0)==SEDS_OK);
  /* A request admitted in Landed and claimed after a change to Launch is refused by the writer gate. */
  assert(event(SEDS_P2P_STREAM_DATA,"GS",b,n)==SEDS_OK); fake_state=7; assert(!sd_file_stream_state_allowed());
  assert(sd_file_stream_claim(&req,&ticket));
  /* Session closed by the station before the writer answers: answer discarded. */
  fake_state=14; assert(event(SEDS_P2P_STREAM_CLOSED,"GS",NULL,0)==SEDS_OK);
  nframes=0; sd_file_stream_finish(ticket,&res); poll_many(10); assert(nframes==0);
  /* A non-idle state rejects new requests on a live session with BAD_STATE. */
  assert(event(SEDS_P2P_STREAM_ACCEPTED,"GS",NULL,0)==SEDS_OK);
  fake_state=3; assert(event(SEDS_P2P_STREAM_DATA,"GS",b,n)==SEDS_OK);
  assert(nframes==1 && frames[0][1]==SD_FILE_STATUS_BAD_STATE && !sd_file_stream_claim(&req,&ticket));
  /* Workspace lifetime: kept while the writer holds its handle open, released
   * once no session, BUSY request or open handle remains. */
  assert(heap_live==1); sd_file_stream_handle_open(true);
  fake_state=1; assert(event(SEDS_P2P_STREAM_CLOSED,"GS",NULL,0)==SEDS_OK);
  poll_many(3); assert(heap_live==1 && sd_file_stream_block());
  sd_file_stream_handle_open(false); poll_many(1); assert(heap_live==0 && !sd_file_stream_block() && !sd_file_stream_handle());
  /* BUSY at session end also defers the release until the writer finishes. */
  assert(event(SEDS_P2P_STREAM_ACCEPTED,"GS",NULL,0)==SEDS_OK && heap_live==1);
  assert(event(SEDS_P2P_STREAM_DATA,"GS",b,n)==SEDS_OK && sd_file_stream_claim(&req,&ticket));
  assert(event(SEDS_P2P_STREAM_RESET,"GS",NULL,0)==SEDS_OK); poll_many(3); assert(heap_live==1);
  sd_file_stream_finish(ticket,&res); poll_many(1); assert(heap_live==0);
}

int main(void){ wire_cases(); read_flow(); state_abort(); puts("PASS sd_file_stream"); return 0; }
'''


class SdFileStreamTests(unittest.TestCase):
    def test_actual_wire_and_stream_owner(self):
        with tempfile.TemporaryDirectory(prefix='sd-file-stream-') as tmp:
            p = Path(tmp)
            for name, text in HEADERS.items():
                (p / name).write_text(text)
            (p / 'harness.c').write_text(HARNESS)
            exe = p / 'test'
            subprocess.run(['cc', '-std=c11', '-DTELEMETRY_ENABLED', '-DTELEMETRY_USE_TLSF', '-Wall', '-Wextra', '-Werror', '-g',
                            '-fsanitize=address,undefined', '-I' + str(p), '-I' + str(ROOT / 'Core/Inc'),
                            str(ROOT / 'Core/Src/sd_file_stream.c'), str(p / 'harness.c'), '-o', str(exe)],
                           check=True)
            out = subprocess.run([str(exe)], check=True, capture_output=True, text=True, timeout=20)
            self.assertIn('PASS sd_file_stream', out.stdout)

    def test_writer_gate_is_idle_only_and_launch_excluded(self):
        text = (ROOT / 'Core/Src/sd_card.c').read_text()
        loop = text[text.index('void sd_card_writer_thread_entry'):]
        # Download work only fills a turn that serviced no logging work.
        self.assertIn('if (serviced_work == 0U) serviced_work = sd_file_service_turn();', loop)
        turn = text[text.index('static uint8_t sd_file_service_turn(void)'):text.index('void sd_card_writer_thread_entry')]
        for guard in ('sd_file_stream_state_allowed()', 'g_sd_launch_log.open == 0U',
                      'g_sd_launch_flush_pending == 0U'):
            self.assertIn(guard, turn)
        service = text[text.index('#define SD_DL_IDLE_CLOSE_MS'):text.index('static uint8_t sd_file_service_turn(void)')]
        # The download owner only reads: no write/create/delete/rename/truncate.
        self.assertIn('FX_OPEN_FOR_READ', service)
        self.assertNotIn('static FX_FILE', service)  # handle lives in the on-demand workspace
        self.assertIsNone(re.search(r'fx_file_(write|create|delete|rename|truncate|allocate)|fx_directory_(create|delete|rename)|FX_OPEN_FOR_WRITE', service))
        # Directory borrowing is restored within the same turn.
        self.assertEqual(service.count('fx_directory_default_set(&g_sd_media, FX_NULL)'), 2)


if __name__ == '__main__':
    unittest.main()
