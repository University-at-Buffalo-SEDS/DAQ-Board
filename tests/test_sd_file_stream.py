"""DAQ SD transfer service v2 (P2P port 4511): wire, gating, session and FileX side.

Runs the production sources on the host: Core/Inc/sd_file_wire.h directly,
Core/Src/sd_file_stream.c with fake SDK/HAL/flight-state/heap boundaries
(together with the real sd_xfer.c engine and SDX encoder), and the SD writer's
service block of Core/Src/sd_card.c against a small FileX model with
LaunchCore's software SHA-256 (Core/Src/sd_file_sha256.c). The engine and the
encoder themselves are covered by tests/test_sd_xfer.py and
tests/test_sd_xfer_codec.py; the wire is GroundStation26
docs/backend/sd-transfer.md."""
import hashlib
import pathlib
import re
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tests'))
import sdx_reference as ref  # noqa: E402

ENGINE = [str(ROOT / 'Core/Src/sd_xfer.c'), str(ROOT / 'Core/Src/sd_xfer_codec.c')]
# The firmware's sizes (CMakeLists.txt): 1 KiB DATA messages, 128 SDX contexts.
SIZES = ['-DSD_XFER_MSG_MAX=1024U', '-DSDX_CTX_ENTRIES=128U']
BASE = ['cc', '-std=gnu11', '-O1', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
        '-fsanitize=address,undefined', '-fno-sanitize-recover=all', *SIZES]
CFLAGS = BASE + ['-I', str(ROOT / 'Core/Inc')]


def crc_source():
    """board_crc32_software() from Core/Src/board_crc32.c, without its HAL side."""
    src = (ROOT / 'Core/Src/board_crc32.c').read_text()
    table = src[src.index('static const uint32_t k_crc_nibble'):]
    table = table[:table.index('};') + 2]
    fn = src[src.index('uint32_t board_crc32_software('):]
    return table + '\n' + fn[:fn.index('\n}\n') + 3]


def run_c(code, extra=(), flags=(), args=()):
    if extra:
        code = '#include <stddef.h>\n#include <stdint.h>\n' + crc_source() + code
    with tempfile.TemporaryDirectory(prefix='sd-file-stream-') as tmp:
        exe = pathlib.Path(tmp) / 'check'
        subprocess.run(CFLAGS + list(flags) + ['-x', 'c', '-', '-x', 'none', *extra, '-o', str(exe)],
                       input=code, text=True, check=True)
        return subprocess.run([str(exe), *args], capture_output=True, text=True, check=True, timeout=60).stdout


FRAME_PRINT = r'''
static void hex(const char *tag, const uint8_t *p, size_t n)
{ printf("%s ", tag); for (size_t i = 0; i < n; ++i) printf("%02x", p[i]); printf("\n"); }
'''


def frames(out, tag):
    return [bytes.fromhex(l.split(' ', 1)[1]) if ' ' in l else b'' for l in out.splitlines()
            if l == tag or l.startswith(tag + ' ')]


def value(out, tag):
    return int(next(l.split()[1] for l in out.splitlines() if l.startswith(tag + ' ')))


class WireTests(unittest.TestCase):
    def test_request_parsing_and_bounds(self):
        out = run_c(r'''#include "sd_file_wire.h"
#include <assert.h>
#include <stdio.h>
static uint8_t parse(const uint8_t *p, size_t n) { sd_file_request_t r; return sd_file_parse_request(p, n, &r); }
static size_t open_req(uint8_t *p, uint8_t xfer, uint8_t dir, const char *name)
{
    size_t n = strlen(name);
    const uint8_t fixed[19] = {0x25, xfer, dir, 1, 0x10, 0, 0, 0, 0, 0, 0, 0, 96, 0, 0x00, 4, 8, 16, (uint8_t)n};
    memcpy(p, fixed, 19); memcpy(p + 19, name, n); return 19 + n;
}
static uint8_t hash_req(uint8_t dir, const char *name, int len_adjust)
{
    uint8_t p[100]; size_t n = strlen(name);
    p[0] = 0x27; p[1] = dir; p[2] = (uint8_t)n; memcpy(p + 3, name, n);
    return parse(p, 3 + n + len_adjust);
}
int main(void)
{
    sd_file_request_t r;
    const uint8_t cancel[] = {0x23}, cancel_long[] = {0x23, 0};
    assert(sd_file_parse_request(cancel, 1, &r) == 0 && r.op == 0x23 && parse(cancel_long, 2) == 1);
    /* LIST: the three DAQ directories only. */
    const uint8_t list[] = {0x21, 2, 5, 1}, list_dir3[] = {0x21, 3, 0, 0}, list_short[] = {0x21, 0, 0};
    assert(sd_file_parse_request(list, 4, &r) == 0 && r.op == 0x21 && r.dir == 2 && r.index == 261);
    assert(parse(list_dir3, 4) == 1 && parse(list_short, 3) == 1);
    const uint8_t unknown[] = {0x24}, empty[] = {0};
    const uint8_t read_v1[] = {0x22, 0, 0, 0, 0, 0, 5, 'F', '.', 'C', 'S', 'V'};
    assert(parse(unknown, 1) == 1 && parse(empty, 0) == 1 && parse(NULL, 1) == 1);
    assert(parse(read_v1, sizeof read_v1) == 1);           /* v1 READ is gone */
    uint8_t p[120];
    size_t n = open_req(p, 7, 0, "DAQ_LAUNCH_1700000000000_000.CSV");
    assert(sd_file_parse_request(p, n, &r) == 0 && r.op == 0x25 && r.dir == 0 && r.open.xfer == 7 &&
           r.open.flags == 1 && r.open.offset == 16 && r.open.length == 0 && r.open.rate == 96 * 16 &&
           r.open.msg == 1024 && r.open.window == 8 && r.open.block_kib == 16 && r.name_len == 32 &&
           !strcmp(r.name, "DAQ_LAUNCH_1700000000000_000.CSV"));
    assert(parse(p, n - 1) == 1 && parse(p, n + 1) == 1);  /* name length mismatch */
    n = open_req(p, 0, 0, "X.CSV"); assert(parse(p, n) == 1);          /* xfer 0 is reserved */
    n = open_req(p, 1, 2, "X.CSV"); assert(parse(p, n) == 0);
    n = open_req(p, 1, 3, "X.CSV"); assert(parse(p, n) == 1);          /* no directory 3 */
    const char *bad[] = {"..", "a/b", "..\\x", "x y", ".", "a:b", "UTC/X.CSV"};
    for (unsigned i = 0; i < sizeof bad / sizeof *bad; ++i) {
        n = open_req(p, 1, 1, bad[i]); assert(parse(p, n) == 1);
        assert(hash_req(1, bad[i], 0) == 1);
    }
    n = open_req(p, 1, 1, ""); assert(parse(p, n) == 1);
    assert(hash_req(1, "notes.txt", 0) == 0);               /* any plain leaf name */
    assert(hash_req(1, "X.CSV", 1) == 1 && hash_req(1, "X.CSV", -1) == 1 && hash_req(3, "X.CSV", 0) == 1);
    char longname[65]; memset(longname, 'A', 64); longname[64] = 0;
    assert(hash_req(0, longname, 0) == 1);                  /* 64 > SD_FILE_NAME_MAX */
    longname[63] = 0; assert(hash_req(0, longname, 0) == 0);
    n = open_req(p, 1, 0, longname); assert(n == SD_FILE_REQUEST_MAX && parse(p, n) == 0);
    assert(parse(p, SD_FILE_REQUEST_MAX + 1) == 1);         /* over the request bound */
    const uint8_t ack[11] = {0x26, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1};
    assert(parse(ack, 11) == 1);                            /* ACK is not a request */
    char path[80];
    assert(sd_file_path(path, sizeof path, SD_FILE_DIR_NO_UTC, "X.CSV", 5) == 13 && !strcmp(path, "/NO_UTC/X.CSV"));
    assert(sd_file_path(path, sizeof path, 9, "X.CSV", 5) == 0 && sd_file_path(path, 10, 0, "X.CSV", 5) == 0);
    puts("ok");
}
''', ENGINE)
        self.assertEqual(out.strip(), 'ok')

    def test_answer_encoding(self):
        out = run_c(r'''#include "sd_file_wire.h"
#include <stdio.h>
''' + FRAME_PRINT + r'''
int main(void)
{
    uint8_t f[SD_FILE_ANSWER_MAX];
    hex("L", f, sd_file_encode_list(f, 0, 2, 0x0102, 0x11223344u, SD_FILE_ENTRY_OPEN, "DAQ_1_000.CSV", 13));
    hex("L", f, sd_file_encode_list(f, 6, 1, 3, 99, 1, "ignored", 7));
    hex("S", f, sd_file_encode_status(f, SD_FILE_OP_OPEN, 2));
    hex("S", f, sd_file_encode_status(f, SD_FILE_OP_HASH, 3));
    hex("S", f, sd_file_encode_status(f, SD_FILE_OP_CANCEL, 0));
    hex("S", f, sd_file_encode_status(f, SD_FILE_OP_LIST, 4));
    hex("S", f, sd_file_encode_status(f, 0x22, 1));
}
''', ENGINE)
        lists = frames(out, 'L')
        self.assertEqual(lists[0], bytes([0xa1, 0, 2, 2, 1, 0x44, 0x33, 0x22, 0x11, 1, 13, 0]) + b'DAQ_1_000.CSV')
        self.assertEqual(lists[1], bytes([0xa1, 6, 1, 3, 0, 0, 0, 0, 0, 0, 0, 0]))
        st = frames(out, 'S')
        self.assertEqual(st[0], bytes([0xa5, 2] + [0] * 17))
        self.assertEqual(st[1], bytes([0xa7, 3] + [0] * 37))
        self.assertEqual(st[2], bytes([0xa3, 0]))
        self.assertEqual(st[3], bytes([0xa1, 4] + [0] * 10))
        self.assertEqual(st[4], bytes([0xa2, 1]))

    def test_gating_table(self):
        out = run_c(r'''#include "sd_file_wire.h"
#include <stdio.h>
int main(void)
{
    printf("%d", sd_file_state_allowed(false, 1));
    for (unsigned s = 0; s < 32; ++s) printf("%d", sd_file_state_allowed(true, (uint8_t)s));
    printf("\n");
}
''')
        row = out.strip()
        self.assertEqual(row[0], '0', 'unknown flight state is refused')
        # Idle 1, Landed 13 and the retired Recovery 14 only.
        self.assertEqual([i for i, bit in enumerate(row[1:]) if bit == '1'], [1, 13, 14])


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
    'launchcore/image.h': r'''#pragma once
#include <stddef.h>
#include <stdint.h>
typedef struct { uint32_t h[8]; uint64_t len; uint8_t block[64]; size_t used; } launchcore_sha256_ctx_t;
''',
}

STREAM_HARNESS = r'''
#include "sd_file_stream.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
''' + FRAME_PRINT + r'''
uint32_t fake_ms = 1000, fake_primask; bool fake_has_state = true; uint8_t fake_state = 13;
static SedsP2pStreamHandlerFn handler; static SedsRouter router;
static uint8_t sent[64][SD_XFER_MSG_MAX]; static size_t sent_len[64]; static unsigned nsent, resets, send_fail;
static int queued;
SedsResult seds_router_bind_p2p_stream_port(SedsRouter *r, uint16_t port, SedsP2pStreamHandlerFn f, void *u)
{ (void)u; assert(r == &router && port == 4511); handler = f; return SEDS_OK; }
SedsResult seds_router_send_p2p_stream(SedsRouter *r, uint32_t id, const uint8_t *p, size_t n)
{
    assert(r == &router && id == 7 && n <= SD_XFER_MSG_MAX && !fake_primask && nsent < 64);
    if (send_fail) return SEDS_ERR;
    memcpy(sent[nsent], p, n); sent_len[nsent++] = n;
    return SEDS_OK;
}
SedsResult seds_router_reset_p2p_stream(SedsRouter *r, uint32_t id) { assert(r == &router && id == 7); resets++; return SEDS_OK; }
int32_t seds_router_has_queued_work(SedsRouter *r) { (void)r; return queued; }
bool seds_router_sender_matches_hostname(const SedsRouter *r, const char *s, size_t n, const char *h, size_t hn)
{ (void)r; return n == hn && !memcmp(s, h, n); }
static bool heap_ok = true; static unsigned heap_live, allocs; static size_t heap_last, admit_extra;
bool telemetry_tlsf_admit(size_t a, size_t l) { admit_extra = a - l; return heap_ok; }
void *telemetry_tlsf_malloc(size_t n) { heap_live++; allocs++; heap_last = n; return malloc(n); }
void telemetry_tlsf_free(void *p) { assert(p && heap_live); heap_live--; free(p); }
size_t sd_card_download_handle_size(void) { return 300; }
static uint8_t *file_bytes; static uint32_t file_size;
static bool file_read(void *ctx, uint32_t off, uint8_t *dst, uint32_t len)
{ (void)ctx; if (off + len > file_size) return false; memcpy(dst, file_bytes + off, len); return true; }
static bool fail_read(void *ctx, uint32_t off, uint8_t *dst, uint32_t len) { (void)ctx; (void)off; (void)dst; (void)len; return false; }

static SedsResult ev(uint8_t kind, const char *peer, const uint8_t *p, size_t n)
{
    SedsP2pStreamEventView e = {.kind = kind, .stream_id = 7, .peer_stream_id = 9, .peer_hostname = peer,
        .peer_hostname_len = strlen(peer), .peer_address = 0x1001, .local_port = 4511, .peer_port = 50000,
        .payload = p, .payload_len = n};
    return handler(&e, NULL);
}
/* OPEN xfer 5, SDX, dir 1, offset 0, whole file, rate 100*16 B/s, msg 1024, window 2, 16 KiB blocks. */
static const uint8_t OPEN[] = {0x25, 5, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0x00, 4, 2, 16, 5, 'F', '.', 'C', 'S', 'V'};
static const uint8_t LIST[] = {0x21, 1, 0, 0}, CANCEL[] = {0x23};
static const uint8_t READ_V1[] = {0x22, 0, 0, 0, 0, 1, 5, 'F', '.', 'C', 'S', 'V'};
static void pump(void) { for (unsigned i = 0; i < 400; ++i) { fake_ms += 1; sd_file_stream_poll(); } }
static uint8_t answer12[12] = {0xa1, 6};

static void session_flow(void)
{
    assert(sd_file_stream_init(&router) == SEDS_OK && handler);
    /* Peer check: the GroundStation only. */
    assert(ev(SEDS_P2P_STREAM_ACCEPTED, "RRLS", NULL, 0) == SEDS_BAD_ARG && allocs == 0);
    assert(ev(SEDS_P2P_STREAM_ACCEPTED, "RF", NULL, 0) == SEDS_BAD_ARG && allocs == 0);
    fake_state = 4; assert(ev(SEDS_P2P_STREAM_ACCEPTED, "GS", NULL, 0) == SEDS_ERR && allocs == 0); /* Fill */
    fake_state = 13;
    /* No heap headroom: the session is refused and nothing is allocated. */
    heap_ok = false; assert(ev(SEDS_P2P_STREAM_ACCEPTED, "GS", NULL, 0) == SEDS_ERR && allocs == 0 && !sd_file_stream_block());
    heap_ok = true;
    assert(ev(SEDS_P2P_STREAM_ACCEPTED, "GS", NULL, 0) == SEDS_OK && allocs == 1);
    assert(ev(SEDS_P2P_STREAM_ACCEPTED, "GS", NULL, 0) == SEDS_OK && allocs == 1); /* replaces it */
    printf("AREA %zu\nADMIT %zu\nWORK %zu\nXFER %zu\n", heap_last, admit_extra, sizeof(sd_xfer_work_t), sizeof(sd_xfer_t));
    assert(sd_file_stream_session_active() && !sd_file_stream_transfer_active());
    uint8_t *block = sd_file_stream_block();
    assert(block && sd_file_stream_handle() && sd_file_stream_scratch());
    assert((uint8_t *)sd_file_stream_handle() + 300 <= block + heap_last && (uintptr_t)sd_file_stream_handle() % 8 == 0);
    /* OPEN: the router callback only queues it; a second request answers BUSY. */
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", OPEN, sizeof OPEN) == SEDS_OK && nsent == 0);
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", LIST, sizeof LIST) == SEDS_OK && nsent == 1);
    hex("BUSY", sent[0], sent_len[0]); nsent = 0;
    sd_file_request_t rq; uint32_t ticket;
    assert(sd_file_stream_claim(&rq, &ticket) && rq.op == 0x25 && rq.dir == 1 && rq.open.xfer == 5 && !strcmp(rq.name, "F.CSV"));
    assert(!sd_file_stream_claim(&rq, &ticket) && sd_file_stream_ticket_current(ticket));
    sd_file_stream_open(ticket, &rq.open, file_size, true);
    assert(sd_file_stream_transfer_active());
    fake_ms += 1000; sd_file_stream_poll(); assert(nsent == 1);
    hex("OPENED", sent[0], sent_len[0]); nsent = 0;
    /* DATA: one message per produce; paced at the station's rate and window. */
    assert(sd_file_stream_produce(file_read, NULL, fake_ms));
    assert(!sd_file_stream_produce(file_read, NULL, fake_ms));  /* slot still full */
    fake_ms += 2; sd_file_stream_poll(); assert(nsent == 1);
    const uint32_t first = fake_ms;
    assert(sd_file_stream_produce(file_read, NULL, fake_ms));
    while (nsent == 1 && fake_ms < first + 2000) { fake_ms += 1; sd_file_stream_poll(); }
    printf("GAP %u\n", fake_ms - first);                      /* first message at 1600 B/s */
    assert(!sd_file_stream_produce(file_read, NULL, fake_ms)); /* window of 2 */
    /* ACK from the station: the window opens again; stale ids are ignored. */
    const uint8_t stale[] = {0x26, 4, 2, 0, 0, 0, 0, 0, 100, 0, 2};
    const uint8_t ack[] = {0x26, 5, 2, 0, 0, 0, 0, 0, 100, 0, 2};
    const uint8_t ack_short[] = {0x26, 5, 2, 0, 0, 0, 0, 0, 100, 0};
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", ack_short, sizeof ack_short) == SEDS_BAD_ARG && nsent == 2);
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", stale, sizeof stale) == SEDS_OK && !sd_file_stream_produce(file_read, NULL, fake_ms));
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", ack, sizeof ack) == SEDS_OK);
    assert(sd_file_stream_produce(file_read, NULL, fake_ms));
    /* Busy router: at least 25 ms between frames; answers go before DATA. */
    queued = 1; fake_ms += 2000; unsigned before = nsent;
    sd_file_stream_poll(); assert(nsent == before + 1);
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", LIST, sizeof LIST) == SEDS_OK && sd_file_stream_claim(&rq, &ticket));
    sd_file_stream_answer(ticket, answer12, sizeof answer12);
    fake_ms += 10; sd_file_stream_poll(); assert(nsent == before + 1);
    fake_ms += 16; sd_file_stream_poll(); assert(nsent == before + 2 && sent[before + 1][0] == 0xa1);
    queued = 0;
    /* Rate 0 pauses DATA. */
    const uint8_t pause[] = {0x26, 5, 3, 0, 0, 0, 0, 0, 0, 0, 2};
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", pause, sizeof pause) == SEDS_OK && !sd_file_stream_produce(file_read, NULL, fake_ms));
    /* A failed send is retried on a later poll. */
    const uint8_t resume[] = {0x26, 5, 3, 0, 0, 0, 0, 0, 100, 0, 2};
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", resume, sizeof resume) == SEDS_OK && sd_file_stream_produce(file_read, NULL, fake_ms));
    before = nsent; send_fail = 1; fake_ms += 2000; sd_file_stream_poll(); assert(nsent == before);
    send_fail = 0; fake_ms += 3; sd_file_stream_poll(); assert(nsent == before + 1);
    for (unsigned i = 0; i < nsent; ++i) if (sent[i][0] == 0xa6) hex("DATA", sent[i], sent_len[i]);
    nsent = 0;
    /* CANCEL ends the transfer and drops a queued message. */
    const uint8_t ack4[] = {0x26, 5, 4, 0, 0, 0, 0, 0, 100, 0, 2};
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", ack4, sizeof ack4) == SEDS_OK && sd_file_stream_produce(file_read, NULL, fake_ms));
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", CANCEL, 1) == SEDS_OK);
    assert(nsent == 1 && sent_len[0] == 2 && sent[0][0] == 0xa3 && sent[0][1] == 0); nsent = 0;
    assert(!sd_file_stream_transfer_active()); pump(); assert(nsent == 0);
    /* Bad requests answer at once with BAD_MESSAGE, v1 READ included. */
    const uint8_t bad[] = {0x21, 3, 0, 0};
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", bad, 4) == SEDS_OK && nsent == 1);
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", READ_V1, sizeof READ_V1) == SEDS_OK && nsent == 2);
    hex("BAD", sent[0], sent_len[0]); hex("BAD", sent[1], sent_len[1]); nsent = 0;
    /* A CANCEL stales a busy answer; a later OPEN answer for it is dropped. */
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", OPEN, sizeof OPEN) == SEDS_OK && sd_file_stream_claim(&rq, &ticket));
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", CANCEL, 1) == SEDS_OK); nsent = 0;
    assert(!sd_file_stream_ticket_current(ticket));
    sd_file_stream_open(ticket, &rq.open, file_size, false);
    pump(); assert(nsent == 0 && !sd_file_stream_transfer_active());
    /* A read error ends the transfer. */
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", OPEN, sizeof OPEN) == SEDS_OK && sd_file_stream_claim(&rq, &ticket));
    sd_file_stream_open(ticket, &rq.open, file_size, false); pump(); nsent = 0;
    assert(!sd_file_stream_produce(fail_read, NULL, fake_ms) && !sd_file_stream_transfer_active());
    /* A new OPEN replaces the running transfer. */
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", OPEN, sizeof OPEN) == SEDS_OK && sd_file_stream_claim(&rq, &ticket));
    sd_file_stream_open(ticket, &rq.open, file_size, false); pump(); nsent = 0;
    assert(sd_file_stream_produce(file_read, NULL, fake_ms));
    uint8_t open6[sizeof OPEN]; memcpy(open6, OPEN, sizeof OPEN); open6[1] = 6;
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", open6, sizeof open6) == SEDS_OK && sd_file_stream_claim(&rq, &ticket));
    sd_file_stream_open(ticket, &rq.open, file_size, false);
    pump(); assert(nsent == 1 && sent[0][0] == 0xa5 && sent[0][2] == 6); nsent = 0;  /* old message dropped */
    assert(sd_file_stream_produce(file_read, NULL, fake_ms)); pump();
    assert(nsent == 1 && sent[0][0] == 0xa6 && sent[0][1] == 6 && sent[0][3] == 0 && sent[0][4] == 0); nsent = 0;
    /* A non-allowed state refuses requests on a live session with BAD_STATE
     * (the OPEN answer carries the xfer id), then the poll resets the session. */
    fake_state = 7;                                          /* Launch */
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", OPEN, sizeof OPEN) == SEDS_OK && nsent == 1);
    hex("STATE", sent[0], sent_len[0]); nsent = 0;
    sd_file_stream_handle_open(true);                        /* the writer still owns its handle */
    sd_file_stream_poll();
    assert(resets == 1 && !sd_file_stream_session_active() && !sd_file_stream_transfer_active() && heap_live == 1);
    pump(); assert(resets == 1 && heap_live == 1 && nsent == 0);
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", LIST, sizeof LIST) == SEDS_ERR);    /* old session gone */
    sd_file_stream_handle_open(false); sd_file_stream_poll(); assert(heap_live == 0 && !sd_file_stream_block() && !sd_file_stream_handle());
    /* New sessions: refused without a known state or outside Idle/Landed/Recovery. */
    assert(ev(SEDS_P2P_STREAM_ACCEPTED, "GS", NULL, 0) == SEDS_ERR);
    fake_state = 1; fake_has_state = false; assert(ev(SEDS_P2P_STREAM_ACCEPTED, "GS", NULL, 0) == SEDS_ERR);
    fake_has_state = true; fake_state = 14;                   /* retired Recovery */
    assert(ev(SEDS_P2P_STREAM_ACCEPTED, "GS", NULL, 0) == SEDS_OK && heap_live == 1);
    /* A request admitted in Recovery and claimed after a change to Launch is still
     * claimable: the writer re-checks the gate and answers BAD_STATE itself. */
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", LIST, sizeof LIST) == SEDS_OK);
    fake_state = 7; assert(!sd_file_stream_state_allowed() && sd_file_stream_claim(&rq, &ticket));
    /* The session closes before the writer answers: the answer is discarded,
     * and the area waits for the BUSY owner before it is released. */
    fake_state = 1; assert(ev(SEDS_P2P_STREAM_CLOSED, "GS", NULL, 0) == SEDS_OK && !sd_file_stream_session_active());
    pump(); assert(heap_live == 1);
    sd_file_stream_answer(ticket, answer12, sizeof answer12); pump(); assert(nsent == 0 && heap_live == 0);
    /* A transfer in flight ends with a stream reset; a late answer publishes nothing. */
    assert(ev(SEDS_P2P_STREAM_ACCEPTED, "GS", NULL, 0) == SEDS_OK);
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", OPEN, sizeof OPEN) == SEDS_OK && sd_file_stream_claim(&rq, &ticket));
    sd_file_stream_open(ticket, &rq.open, file_size, false); pump(); nsent = 0;
    assert(sd_file_stream_produce(file_read, NULL, fake_ms));
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", LIST, sizeof LIST) == SEDS_OK && sd_file_stream_claim(&rq, &ticket));
    assert(ev(SEDS_P2P_STREAM_RESET, "GS", NULL, 0) == SEDS_OK && !sd_file_stream_transfer_active());
    sd_file_stream_answer(ticket, answer12, sizeof answer12);
    pump(); assert(nsent == 0 && heap_live == 0);
}

/* A whole transfer through the stream layer at the firmware's 1 KiB message
 * size: every DATA message is printed for the station model in Python. */
static void full_transfer(void)
{
    fake_state = 1;
    assert(ev(SEDS_P2P_STREAM_ACCEPTED, "GS", NULL, 0) == SEDS_OK);
    /* xfer 9, SDX, dir 2, whole file, rate 4000*16 B/s, msg 1024, window 8, 8 KiB blocks. */
    const uint8_t open[] = {0x25, 9, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0xa0, 0x0f, 0x00, 4, 8, 8, 5, 'F', '.', 'C', 'S', 'V'};
    assert(ev(SEDS_P2P_STREAM_DATA, "GS", open, sizeof open) == SEDS_OK);
    sd_file_request_t rq; uint32_t ticket;
    assert(sd_file_stream_claim(&rq, &ticket)); sd_file_stream_open(ticket, &rq.open, file_size, false);
    nsent = 0; fake_ms += 10; sd_file_stream_poll(); assert(nsent == 1); hex("FULLOPEN", sent[0], sent_len[0]);
    uint16_t una = 0; unsigned max_len = 0, msgs = 0;
    for (unsigned t = 0; t < 200000 && sd_file_stream_transfer_active(); ++t) {
        nsent = 0;
        (void)sd_file_stream_produce(file_read, NULL, fake_ms);
        fake_ms += 1; sd_file_stream_poll();
        if (nsent) {
            assert(sent[0][0] == 0xa6 && sent[0][1] == 9);
            hex("M", sent[0], sent_len[0]); ++msgs;
            if (sent_len[0] > max_len) max_len = (unsigned)sent_len[0];
            una = (uint16_t)(sent[0][3] | sent[0][4] << 8) + 1U;
            const uint8_t a[] = {0x26, 9, (uint8_t)una, (uint8_t)(una >> 8), 0, 0, 0, 0, 0xa0, 0x0f, 8};
            assert(ev(SEDS_P2P_STREAM_DATA, "GS", a, sizeof a) == SEDS_OK);
        }
    }
    printf("FULL %u %u %d\n", msgs, max_len, (int)sd_file_stream_transfer_active());
    assert(ev(SEDS_P2P_STREAM_CLOSED, "GS", NULL, 0) == SEDS_OK); pump(); assert(heap_live == 0);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    FILE *f = fopen(argv[1], "rb"); assert(f); fseek(f, 0, SEEK_END); file_size = (uint32_t)ftell(f); fseek(f, 0, SEEK_SET);
    file_bytes = malloc(file_size); assert(fread(file_bytes, 1, file_size, f) == file_size); fclose(f);
    session_flow();
    full_transfer();
    free(file_bytes);
    puts("PASS sd_file_stream");
    return 0;
}
'''


def daq_csv(rows):
    from test_sd_xfer_codec import daq_rows
    return daq_rows(rows)


class StreamTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = daq_csv(1500)
        with tempfile.TemporaryDirectory(prefix='sd-file-stream-') as tmp:
            p = pathlib.Path(tmp)
            for name, text in HEADERS.items():
                (p / name).parent.mkdir(parents=True, exist_ok=True)
                (p / name).write_text(text)
            (p / 'harness.c').write_text('#include <stddef.h>\n#include <stdint.h>\n' + crc_source() + STREAM_HARNESS)
            (p / 'file.csv').write_bytes(cls.data)
            exe = p / 'test'
            subprocess.run(BASE + ['-DTELEMETRY_ENABLED', '-DTELEMETRY_USE_TLSF', '-I' + str(p), '-I' + str(ROOT / 'Core/Inc'),
                                     str(ROOT / 'Core/Src/sd_file_stream.c'), str(p / 'harness.c'), *ENGINE,
                                     '-o', str(exe)], check=True)
            cls.out = subprocess.run([str(exe), str(p / 'file.csv')], check=True, capture_output=True,
                                     text=True, timeout=60).stdout

    def test_session_gating_and_answers(self):
        self.assertIn('PASS sd_file_stream', self.out)
        self.assertEqual(frames(self.out, 'BUSY')[0], bytes([0xa1, 3] + [0] * 10))
        self.assertEqual(frames(self.out, 'BAD')[0], bytes([0xa1, 1] + [0] * 10))
        self.assertEqual(frames(self.out, 'BAD')[1], bytes([0xa2, 1]))
        self.assertEqual(frames(self.out, 'STATE')[0], bytes([0xa5, 2, 5] + [0] * 16))
        opened = frames(self.out, 'OPENED')[0]
        size = len(self.data).to_bytes(4, 'little')
        self.assertEqual(opened[:4], bytes([0xa5, 0, 5, 0x01 | 0x04]))   # SDX, file still growing
        self.assertEqual(opened[4:16], size + bytes(4) + size)
        self.assertEqual(opened[16:], bytes([0x00, 4, 16]))              # 1024-byte messages, 16 KiB blocks

    def test_borrowed_area_and_admission(self):
        area, work, xfer = value(self.out, 'AREA'), value(self.out, 'WORK'), value(self.out, 'XFER')
        # Block + transfer state + workspace + the writer's handle, nothing static.
        self.assertGreaterEqual(area, 512 + xfer + work + 300)
        self.assertLess(area, 512 + xfer + work + 300 + 16)
        self.assertEqual(value(self.out, 'ADMIT'), 4096, 'normal 4 KiB heap reserve on top of the area')
        src = (ROOT / 'Core/Src/sd_file_stream.c').read_text()
        self.assertNotRegex(src, r'static\s+uint8_t\s+\w+\[\s*SD_FILE_BLOCK_BYTES')

    def test_data_is_paced_windowed_and_ordered(self):
        gap = value(self.out, 'GAP')
        data = frames(self.out, 'DATA')
        self.assertTrue(960 < len(data[0]) <= 1024, 'first message nearly fills the 1 KiB limit (whole rows)')
        self.assertEqual(gap, -(-len(data[0]) * 1000 // 1600), 'second message waits for the 1600 B/s rate')
        self.assertEqual([int.from_bytes(d[3:5], 'little') for d in data], list(range(len(data))))
        self.assertEqual(data[0][1:3], bytes([5, 0x01 | 0x04 | 0x20]))  # xfer 5, FIRST | SDX | GROWING
        self.assertTrue(all(len(d) <= 1024 for d in data))

    def test_full_transfer_at_1k_messages_rebuilds_the_file(self):
        msgs, max_len, active = (int(v) for v in next(l.split()[1:] for l in self.out.splitlines()
                                                      if l.startswith('FULL ')))
        self.assertEqual(active, 0, 'the transfer ends once the END block is acknowledged')
        self.assertTrue(960 < max_len <= 1024, max_len)
        opened = frames(self.out, 'FULLOPEN')[0]
        self.assertEqual(opened[:4], bytes([0xa5, 0, 9, 0x01]))
        out, blocks, body, raw_len, crc = bytearray(), 0, b'', 0, 0
        for m in frames(self.out, 'M'):
            flags, block = m[2], struct.unpack_from('<I', m, 5)[0]
            if flags & 0x01:
                self.assertEqual(block, len(out))
                body = b''
            if flags & 0x02:
                raw_len, crc = struct.unpack_from('<II', m, 9)
                body += m[17:]
                raw = ref.decode_block(body)
                self.assertEqual((len(raw), zlib.crc32(raw)), (raw_len, crc))
                out += raw
                blocks += 1
                self.assertEqual(bool(flags & 0x10), len(out) == len(self.data))
            else:
                body += m[9:]
        self.assertEqual(bytes(out), self.data)
        self.assertEqual(msgs, len(frames(self.out, 'M')))
        self.assertGreater(blocks, 3)
        self.assertLess(sum(len(m) for m in frames(self.out, 'M')) * 3, len(self.data), 'SDX at least 3x')


FILEX_STUBS = r'''
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include "sd_file_stream.h"
#include "launchcore/image.h"
typedef unsigned UINT; typedef unsigned long ULONG; typedef unsigned long long ULONG64; typedef char CHAR;
#define FX_NULL NULL
#define FX_SUCCESS 0U
#define FX_NOT_FOUND 4U
#define FX_NO_MORE_ENTRIES 0x0FU
#define FX_INVALID_PATH 0x0DU
#define FX_NOT_DIRECTORY 0x0EU
#define FX_IO_ERROR 0x90U
#define FX_END_OF_FILE 0x09U
#define FX_OPEN_FOR_READ 0U
#define FX_DIRECTORY 0x10U
#define FX_VOLUME 0x08U
#define FX_HIDDEN 0x02U
#define FX_SYSTEM 0x04U
#define FX_MAX_LONG_NAME_LEN 256U
typedef struct { CHAR fx_file_name_buffer[FX_MAX_LONG_NAME_LEN]; ULONG64 fx_file_current_file_size;
                 int entry; ULONG pos; } FX_FILE;
typedef struct { struct { ULONG fx_path_current_entry; } fx_media_default_path; int dir; } FX_MEDIA;
static FX_MEDIA g_sd_media;
typedef struct { uint8_t open; char name[64]; } sd_log_t;
static sd_log_t g_sd_regular_log, g_sd_launch_log;
static volatile uint8_t g_sd_launch_flush_pending;
static uint32_t clock_ms, opens, closes, io_fail, seek_fail, short_read, dir_fail;
static uint32_t HAL_GetTick(void) { return clock_ms; }
struct entry { int dir; const char *name; UINT attr; ULONG size; };
static struct entry entries[] = {
    {1, "SYSTEM~1", FX_DIRECTORY | FX_HIDDEN, 0}, {1, "SUB", FX_DIRECTORY, 0},
    {1, "DAQ_1700000000000_000.CSV", 0, 1300}, {1, "bad name.csv", 0, 5}, {1, "HIDDEN.CSV", FX_HIDDEN, 9},
    {1, "DAQ_1700000000000_001.CSV", 0, 5000}, {1, "VOL", FX_VOLUME, 0}, {1, "SYS.CSV", FX_SYSTEM, 3},
    {1, "DAQ_1700000100000_000.CSV", 0, 0},
    {0, "DAQ_LAUNCH_1700000000000_000.CSV", 0, 512},
};
#define NENT (sizeof entries / sizeof *entries)
static const char *dirs[] = {"/LAUNCH", "/UTC"};   /* /NO_UTC does not exist */
static UINT fx_directory_default_set(FX_MEDIA *m, CHAR *path)
{
    if (!path) { m->dir = -1; return FX_SUCCESS; }
    if (dir_fail) { m->dir = -2; return FX_IO_ERROR; }
    for (int i = 0; i < 2; ++i) if (!strcmp(path, dirs[i])) { m->dir = i; return FX_SUCCESS; }
    m->dir = -2; return FX_NOT_FOUND;
}
static UINT fx_directory_next_full_entry_find(FX_MEDIA *m, CHAR *name, UINT *attr, ULONG *size,
    UINT *y, UINT *mo, UINT *d, UINT *h, UINT *mi, UINT *s)
{
    (void)y; (void)mo; (void)d; (void)h; (void)mi; (void)s;
    assert(m->dir >= 0);
    if (io_fail) return FX_IO_ERROR;
    for (ULONG i = m->fx_media_default_path.fx_path_current_entry; i < NENT; ++i) {
        if (entries[i].dir != m->dir) continue;
        strcpy(name, entries[i].name); *attr = entries[i].attr; *size = entries[i].size;
        m->fx_media_default_path.fx_path_current_entry = i + 1;
        return FX_SUCCESS;
    }
    return FX_NO_MORE_ENTRIES;
}
static UINT fx_file_open(FX_MEDIA *m, FX_FILE *f, CHAR *path, UINT mode)
{
    (void)m; assert(mode == FX_OPEN_FOR_READ);
    for (unsigned i = 0; i < NENT; ++i) {
        char full[96]; snprintf(full, sizeof full, "%s/%s", dirs[entries[i].dir], entries[i].name);
        if (!strcasecmp(full, path) && !(entries[i].attr & FX_DIRECTORY)) {
            if (io_fail) return FX_IO_ERROR;
            ++opens; strcpy(f->fx_file_name_buffer, entries[i].name);
            f->fx_file_current_file_size = entries[i].size; f->entry = (int)i; return FX_SUCCESS;
        }
    }
    return FX_NOT_FOUND;
}
static UINT fx_file_close(FX_FILE *f) { (void)f; ++closes; return FX_SUCCESS; }
static UINT fx_file_seek(FX_FILE *f, ULONG at) { if (seek_fail) return FX_IO_ERROR; f->pos = at; return FX_SUCCESS; }
static UINT fx_file_read(FX_FILE *f, void *buf, ULONG want, ULONG *actual)
{
    assert(f->pos % 512 == 0 && want <= 1024);
    if (f->pos >= f->fx_file_current_file_size) { *actual = 0; return FX_END_OF_FILE; }  /* as FileX */
    ULONG size = (ULONG)f->fx_file_current_file_size, n = f->pos + want > size ? size - f->pos : want;
    if (short_read && n) --n;
    for (ULONG i = 0; i < n; ++i) ((uint8_t *)buf)[i] = (uint8_t)((f->pos + i) * 31 + f->entry);
    *actual = n; f->pos += n; return io_fail ? FX_IO_ERROR : FX_SUCCESS;
}
/* sd_file_stream.c side */
static uint8_t area[512]; static uint64_t handle[64]; static launchcore_sha256_ctx_t sha_ctx;
static bool have_area = true, allowed = true, session = true, handle_open, xfer_active;
static sd_file_request_t pending; static bool has_pending; static uint32_t ticket_now = 1;
static uint8_t last[SD_FILE_ANSWER_MAX]; static size_t last_len; static unsigned answered, produced_calls;
static uint32_t opened_size; static bool opened_growing; static uint8_t opened_xfer;
static uint8_t produced[8192]; static bool produce_ok;
bool sd_file_stream_claim(sd_file_request_t *r, uint32_t *t) { if (!has_pending) return false; *r = pending; *t = ticket_now; has_pending = false; return true; }
bool sd_file_stream_ticket_current(uint32_t t) { return t == ticket_now; }
uint8_t *sd_file_stream_block(void) { return have_area ? area : NULL; }
void *sd_file_stream_handle(void) { return have_area ? handle : NULL; }
void *sd_file_stream_scratch(void) { return &sha_ctx; }
void sd_file_stream_handle_open(bool open) { handle_open = open; }
static unsigned released;   /* NULL answers: the slot freed, nothing sent (sd_file_stream.c) */
void sd_file_stream_answer(uint32_t t, const uint8_t *f, size_t n) { if (!f) { ++released; return; } if (t != ticket_now) return; memcpy(last, f, n); last_len = n; ++answered; }
void sd_file_stream_open(uint32_t t, const sd_xfer_open_t *o, uint32_t size, bool growing)
{ opened_xfer = o->xfer; opened_size = size; opened_growing = growing; xfer_active = true;
  uint8_t a[SD_XFER_OPEN_ANSWER] = {0xa5}; sd_file_stream_answer(t, a, sizeof a); }
bool sd_file_stream_transfer_active(void) { return xfer_active; }
bool sd_file_stream_produce(sd_xfer_read_fn read, void *ctx, uint32_t now)
{   /* reads the whole file the way the engine does: a 1024-byte window, then the next sectors */
    (void)now; ++produced_calls;
    produce_ok = true;
    for (uint32_t off = 0; off < opened_size && produce_ok; off += 1024U) {
        const uint32_t n = opened_size - off < 1024U ? opened_size - off : 1024U;
        produce_ok = read(ctx, off, produced + off, n);
    }
    if (!produce_ok) xfer_active = false;
    return produce_ok;
}
bool sd_file_stream_session_active(void) { return session; }
bool sd_file_stream_state_allowed(void) { return allowed; }
'''

FILEX_MAIN = FRAME_PRINT + r'''
static void request(uint8_t op, uint8_t dir, uint16_t index, const char *name)
{
    pending = (sd_file_request_t){.op = op, .dir = dir, .index = index, .name_len = (uint8_t)(name ? strlen(name) : 0)};
    pending.open.xfer = 0x42;
    if (name) strcpy(pending.name, name);
    has_pending = true; (void)sd_file_service_turn();
}
static unsigned hash(uint8_t dir, const char *name)
{
    const unsigned before = answered; unsigned turns = 0;
    request(SD_FILE_OP_HASH, dir, 0, name);
    while (answered == before && turns < 100) { (void)sd_file_service_turn(); ++turns; }
    return turns;
}
int main(void)
{
    assert(sd_file_service_turn() == 0 && answered == 0);   /* nothing claimed */
    /* Enumeration: regular plain names only, FileX order, END after the last;
     * the media default directory is restored after every step. */
    for (uint16_t i = 0; i < 4; ++i) { request(SD_FILE_OP_LIST, 1, i, NULL); hex("LIST", last, last_len); assert(g_sd_media.dir == -1); }
    request(SD_FILE_OP_LIST, 1, 1, NULL); assert(last[1] == 0 && !memcmp(last + 12, "DAQ_1700000000000_001.CSV", 25)); /* backwards restarts */
    request(SD_FILE_OP_LIST, 0, 0, NULL); hex("LIST", last, last_len);                /* another directory */
    request(SD_FILE_OP_LIST, 2, 0, NULL); hex("LIST", last, last_len);                /* missing directory: END */
    assert(g_sd_media.dir == -1);
    /* The writer's open files are flagged, in LIST and OPEN. */
    g_sd_regular_log.open = 1; strcpy(g_sd_regular_log.name, "/UTC/DAQ_1700000100000_000.CSV");
    request(SD_FILE_OP_LIST, 1, 2, NULL); assert(last[1] == 0 && last[9] == SD_FILE_ENTRY_OPEN);
    request(SD_FILE_OP_OPEN, 1, 0, "DAQ_1700000100000_000.CSV");
    assert(last[0] == 0xa5 && opened_growing && opened_size == 0 && opened_xfer == 0x42);
    xfer_active = false; g_sd_regular_log.open = 0;
    request(SD_FILE_OP_LIST, 1, 2, NULL); assert(last[1] == 0 && last[9] == 0);
    /* OPEN keeps one read handle; DATA reads go through it. */
    uint32_t o = opens; const unsigned pc = produced_calls;
    request(SD_FILE_OP_OPEN, 1, 0, "DAQ_1700000000000_001.CSV"); assert(opened_size == 5000 && !opened_growing && handle_open);
    assert(produced_calls == pc);                             /* one step per turn: the answer only */
    assert(sd_file_service_turn() == 1 && produce_ok); hex("DATA", produced, 5000);
    assert(opens == o + 1);
    /* The handle stays open for a transfer however long the gaps. */
    clock_ms += 10000; (void)sd_file_service_turn(); assert(handle_open);
    /* HASH: refused while a transfer runs, then SHA-256 of the whole file in bounded steps. */
    request(SD_FILE_OP_HASH, 1, 0, "DAQ_1700000000000_001.CSV");
    assert(last[0] == 0xa7 && last[1] == SD_FILE_STATUS_BUSY && last_len == 39);
    xfer_active = false;
    printf("TURNS %u\n", hash(1, "DAQ_1700000000000_001.CSV")); hex("HASH", last, last_len);
    assert(opens == o + 1);                                   /* same handle reused */
    printf("TURNS %u\n", hash(0, "DAQ_LAUNCH_1700000000000_000.CSV")); hex("HASH", last, last_len);
    hash(1, "DAQ_1700000100000_000.CSV"); hex("HASH", last, last_len);   /* empty file */
    g_sd_regular_log.open = 1; hash(1, "DAQ_1700000000000_000.CSV"); hex("HASH", last, last_len);
    g_sd_regular_log.open = 0;
    request(SD_FILE_OP_OPEN, 1, 0, "MISSING.CSV"); assert(last[0] == 0xa5 && last[1] == SD_FILE_STATUS_NOT_FOUND && last[2] == 0x42);
    request(SD_FILE_OP_HASH, 1, 0, "MISSING.CSV"); assert(last[0] == 0xa7 && last[1] == SD_FILE_STATUS_NOT_FOUND);
    request(SD_FILE_OP_HASH, 0, 0, "DAQ_1700000000000_000.CSV"); assert(last[1] == SD_FILE_STATUS_NOT_FOUND); /* wrong dir */
    /* I/O failures: the handle closes and the answer says IO_ERROR. */
    request(SD_FILE_OP_OPEN, 0, 0, "DAQ_LAUNCH_1700000000000_000.CSV"); assert(handle_open);
    /* The engine's zero-length read at a sector-aligned range end succeeds
     * (FileX itself would answer FX_END_OF_FILE); reading past the end fails. */
    uint8_t probe[512];
    assert(sd_dl_read(NULL, 512, probe, 0) && handle_open);
    assert(sd_dl_read(NULL, 0, probe, 512) && handle_open && probe[1] == (uint8_t)(31 + 9));
    io_fail = 1; (void)sd_file_service_turn(); assert(!produce_ok && !handle_open && !xfer_active);
    request(SD_FILE_OP_LIST, 1, 0, NULL); assert(last[1] == SD_FILE_STATUS_IO_ERROR);
    request(SD_FILE_OP_HASH, 1, 0, "DAQ_1700000000000_000.CSV"); assert(last[1] == SD_FILE_STATUS_IO_ERROR);
    request(SD_FILE_OP_OPEN, 1, 0, "DAQ_1700000000000_000.CSV"); assert(last[1] == SD_FILE_STATUS_IO_ERROR && last[2] == 0x42);
    io_fail = 0;
    dir_fail = 1; request(SD_FILE_OP_LIST, 1, 0, NULL); assert(last[1] == SD_FILE_STATUS_IO_ERROR && g_sd_media.dir == -1); dir_fail = 0;
    request(SD_FILE_OP_OPEN, 1, 0, "DAQ_1700000000000_001.CSV"); assert(handle_open);
    short_read = 1; (void)sd_file_service_turn(); assert(!produce_ok && !handle_open); short_read = 0;
    request(SD_FILE_OP_OPEN, 1, 0, "DAQ_1700000000000_001.CSV"); assert(handle_open);
    seek_fail = 1; (void)sd_file_service_turn(); assert(!produce_ok && !handle_open); seek_fail = 0;
    hash(1, "DAQ_1700000000000_001.CSV"); assert(last[1] == 0);
    /* A cancelled hash stops quietly and releases its request slot (else the
     * session answers BUSY to everything after it). */
    request(SD_FILE_OP_HASH, 1, 0, "DAQ_1700000000000_001.CSV"); const unsigned a = answered, r0 = released;
    ++ticket_now; (void)sd_file_service_turn(); (void)sd_file_service_turn(); assert(answered == a && released == r0 + 1);
    /* Gate re-checked in the writer (state, launch file open, its close flush):
     * a request is answered BAD_STATE and the handle closes. */
    request(SD_FILE_OP_OPEN, 1, 0, "DAQ_1700000000000_001.CSV"); assert(handle_open);
    const unsigned produced_before = produced_calls;
    g_sd_launch_log.open = 1; request(SD_FILE_OP_LIST, 1, 0, NULL);
    assert(last[1] == SD_FILE_STATUS_BAD_STATE && !handle_open && produced_calls == produced_before);
    g_sd_launch_log.open = 0; xfer_active = false;
    request(SD_FILE_OP_OPEN, 1, 0, "DAQ_1700000000000_001.CSV"); assert(handle_open); xfer_active = false;
    g_sd_launch_flush_pending = 1; request(SD_FILE_OP_HASH, 1, 0, "DAQ_1700000000000_001.CSV");
    assert(last[0] == 0xa7 && last[1] == SD_FILE_STATUS_BAD_STATE && !handle_open);
    g_sd_launch_flush_pending = 0;
    allowed = false; request(SD_FILE_OP_OPEN, 1, 0, "DAQ_1700000000000_001.CSV");
    assert(last[0] == 0xa5 && last[1] == SD_FILE_STATUS_BAD_STATE && last[2] == 0x42 && !xfer_active);
    allowed = true;
    /* A gated hash in progress stops. */
    request(SD_FILE_OP_HASH, 1, 0, "DAQ_1700000000000_001.CSV"); const unsigned b = answered, r1 = released;
    allowed = false; (void)sd_file_service_turn(); allowed = true; (void)sd_file_service_turn(); assert(answered == b && released == r1 + 1);
    /* Idle close after 3 s without a transfer; session end closes at once. */
    request(SD_FILE_OP_OPEN, 1, 0, "DAQ_1700000000000_001.CSV"); assert(handle_open); xfer_active = false;
    clock_ms += 2999; (void)sd_file_service_turn(); assert(handle_open);
    clock_ms += 1; (void)sd_file_service_turn(); assert(!handle_open);
    request(SD_FILE_OP_OPEN, 1, 0, "DAQ_1700000000000_001.CSV"); xfer_active = false; session = false;
    assert(sd_file_service_turn() == 0 && !handle_open);
    /* Without a session nothing is claimed. */
    has_pending = true; assert(sd_file_service_turn() == 0 && has_pending); has_pending = false;
    session = true;
    have_area = false; request(SD_FILE_OP_LIST, 1, 0, NULL); assert(last[1] == SD_FILE_STATUS_BAD_STATE);
    puts("done");
}
'''


def service_block():
    src = (ROOT / 'Core/Src/sd_card.c').read_text()
    start = src.index('/* Network SD transfer service v2')
    end = src.index('#else\nstatic uint8_t sd_file_service_turn(void) { return 0U; }')
    block = src[start:end]
    return block.replace('#ifdef TELEMETRY_ENABLED\n', '', 1)


class FilexSideTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from native_sdk import configured_dependency_source
        cls.block = service_block()
        try:
            lc = configured_dependency_source(ROOT, 'sedslaunchcore', 'SDK_FREE_LAUNCHCORE_SOURCE')
        except RuntimeError as error:
            raise unittest.SkipTest(str(error))
        with tempfile.TemporaryDirectory(prefix='sd-file-service-') as tmp:
            exe = pathlib.Path(tmp) / 'check'
            subprocess.run(CFLAGS + ['-I', str(lc / 'bootloader/include'), '-I', str(lc / 'bootloader/src'),
                                     '-x', 'c', '-', '-x', 'none', str(ROOT / 'Core/Src/sd_file_sha256.c'),
                                     '-o', str(exe)],
                           input=FILEX_STUBS + cls.block + FILEX_MAIN, text=True, check=True)
            cls.out = subprocess.run([str(exe)], capture_output=True, text=True, check=True, timeout=60).stdout

    @staticmethod
    def content(entry, size):
        return bytes((i * 31 + entry) & 0xff for i in range(size))

    def test_enumeration_filter(self):
        self.assertIn('done', self.out)
        lists = frames(self.out, 'LIST')
        self.assertEqual(lists[0], bytes([0xa1, 0, 1, 0, 0]) + (1300).to_bytes(4, 'little') + bytes([0, 25, 0])
                         + b'DAQ_1700000000000_000.CSV')
        self.assertEqual(lists[1][12:], b'DAQ_1700000000000_001.CSV')
        self.assertEqual(lists[2][12:], b'DAQ_1700000100000_000.CSV')
        self.assertEqual(lists[3], bytes([0xa1, 6, 1, 3, 0] + [0] * 7))   # END
        self.assertEqual(lists[4][:3], bytes([0xa1, 0, 0]))
        self.assertEqual(lists[4][12:], b'DAQ_LAUNCH_1700000000000_000.CSV')
        self.assertEqual(lists[5][:3], bytes([0xa1, 6, 2]))              # /NO_UTC absent: END

    def test_data_reads_and_hashes_match_the_card(self):
        self.assertEqual(frames(self.out, 'DATA')[0], self.content(5, 5000))
        hashes = frames(self.out, 'HASH')
        for answer, (entry, size, growing) in zip(hashes, ((5, 5000, 0), (9, 512, 0), (8, 0, 0), (2, 1300, 0))):
            self.assertEqual(answer[:3], bytes([0xa7, 0, growing]))
            self.assertEqual(int.from_bytes(answer[3:7], 'little'), size)
            self.assertEqual(answer[7:], hashlib.sha256(self.content(entry, size)).digest())
        self.assertEqual(len(hashes), 4)
        turns = [int(l.split()[1]) for l in self.out.splitlines() if l.startswith('TURNS ')]
        # 8 sectors (4 KiB) per writer turn: 5000 bytes need one more turn, 512 none.
        self.assertEqual(turns, [1, 0])

    def test_growing_file_hash_is_flagged(self):
        src = self.block
        self.assertIn('answer[2] = g_sd_dl_hash.growing ? SD_XFER_ANSWER_GROWING : 0U;', src)

    def test_read_only_and_bounded(self):
        # The transfer owner only reads: no write/create/delete/rename/truncate.
        self.assertIn('FX_OPEN_FOR_READ', self.block)
        self.assertNotIn('static FX_FILE', self.block)        # handle lives in the borrowed area
        self.assertIsNone(re.search(r'fx_file_(write|create|delete|rename|truncate|allocate)|'
                                    r'fx_directory_(create|delete|rename)|FX_OPEN_FOR_WRITE|fx_media_close', self.block))
        # Directory borrowing is restored within the same turn.
        self.assertEqual(self.block.count('fx_directory_default_set(&g_sd_media, FX_NULL)'), 2)
        self.assertIn('#define SD_DL_HASH_SECTORS_PER_TURN 8U', self.block)
        self.assertIn('#define SD_DL_LIST_SCAN_PER_TURN 32U', self.block)
        # The HASH uses the private software SHA-256, never the shared HASH peripheral.
        self.assertNotIn('launchcore_sha256_init', self.block)
        self.assertIn('sd_file_sha256_update', self.block)


class WiringTests(unittest.TestCase):
    def test_writer_gate_is_idle_only_and_launch_excluded(self):
        text = (ROOT / 'Core/Src/sd_card.c').read_text()
        loop = text[text.index('void sd_card_writer_thread_entry'):]
        # Transfer work only fills a turn that serviced no logging work.
        self.assertIn('if (serviced_work == 0U) serviced_work = sd_file_service_turn();', loop)
        turn = text[text.index('static uint8_t sd_file_service_turn(void)\n{'):text.index('void sd_card_writer_thread_entry')]
        for guard in ('sd_file_stream_state_allowed()', 'g_sd_launch_log.open == 0U',
                      'g_sd_launch_flush_pending == 0U'):
            self.assertIn(guard, turn)

    def test_router_and_build_wiring(self):
        tel = (ROOT / 'Core/Src/telemetry.c').read_text()
        self.assertIn('sd_file_stream_init(r)', tel)
        self.assertIn('sd_file_stream_poll();', (ROOT / 'Core/Src/telemetry_thread.c').read_text())
        cmake = (ROOT / 'CMakeLists.txt').read_text()
        self.assertIn('Core/Src/sd_file_stream.c', cmake)
        for src in ('sd_xfer.c', 'sd_xfer_codec.c', 'sd_file_sha256.c'):
            self.assertIn('Core/Src/' + src, cmake)
        self.assertIn('SD_XFER_MSG_MAX=1024U SDX_CTX_ENTRIES=128U', cmake)
        self.assertIn('sedslaunchcore_SOURCE_DIR}/bootloader/src', cmake)


if __name__ == '__main__':
    unittest.main()
