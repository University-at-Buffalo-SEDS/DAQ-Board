"""SD transfer v2 sender engine (Core/Src/sd_xfer.c) against a station model.

The production engine and encoder run on the host (ASan/UBSan) behind a small
command loop; this test plays the GroundStation side of
GroundStation26 docs/backend/sd-transfer.md: it drops DATA and ACKs at random,
reassembles blocks, decodes SDX with the reference decoder, checks CRCs, ACKs
selectively and resumes, and requires the byte-identical file every time."""
import pathlib
import random
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tests'))
import sdx_reference as ref  # noqa: E402
from test_sd_xfer_codec import fc_rows, daq_rows  # noqa: E402
from test_sd_file_stream import crc_source  # noqa: E402

CFLAGS = ['cc', '-std=gnu11', '-O1', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
          '-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-I', str(ROOT / 'Core/Inc')]

DRIVER = r'''
#include "sd_xfer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static uint8_t *data; static uint32_t size; static unsigned reads;
static bool rd(void *ctx, uint32_t off, uint8_t *dst, uint32_t len)
{ (void)ctx; ++reads; if (off + len > size || len > SD_XFER_RAW_WINDOW) return false; memcpy(dst, data + off, len); return true; }
static void hex(char tag, const uint8_t *p, size_t n)
{ printf("%c ", tag); for (size_t i = 0; i < n; ++i) printf("%02x", p[i]); printf("\n"); fflush(stdout); }
int main(int argc, char **argv)
{
    FILE *f = fopen(argv[1], "rb"); fseek(f, 0, SEEK_END); size = (uint32_t)ftell(f); fseek(f, 0, SEEK_SET);
    data = malloc(size + 1); if (fread(data, 1, size, f) != size) return 3; fclose(f);
    (void)argc;
    static sd_xfer_t x; static sd_xfer_work_t w;
    char cmd[256];
    while (fgets(cmd, sizeof cmd, stdin)) {
        if (cmd[0] == 'O') {
            unsigned xf, fl, off, len, rate, msg, win, blk, now; sd_xfer_open_t o;
            sscanf(cmd + 1, "%u %u %u %u %u %u %u %u %u", &xf, &fl, &off, &len, &rate, &msg, &win, &blk, &now);
            o.xfer = (uint8_t)xf; o.flags = (uint8_t)fl; o.offset = off; o.length = len; o.rate = rate;
            o.msg = (uint16_t)msg; o.window = (uint8_t)win; o.block_kib = (uint8_t)blk;
            uint8_t ans[SD_XFER_OPEN_ANSWER];
            sd_xfer_begin(&x, &o, size, false, now, ans);
            hex('A', ans, sizeof ans);
        } else if (cmd[0] == 'K') {
            unsigned xf, una, bm, rate, win, now; sd_xfer_flow_t fl;
            sscanf(cmd + 1, "%u %u %u %u %u %u", &xf, &una, &bm, &rate, &win, &now);
            fl.una = (uint16_t)una; fl.bitmap = bm; fl.rate = rate; fl.window = (uint8_t)win;
            printf("k %d\n", sd_xfer_ack(&x, (uint8_t)xf, &fl, now)); fflush(stdout);
        } else if (cmd[0] == 'P') {
            unsigned now; sscanf(cmd + 1, "%u", &now);
            const uint8_t *out = NULL;
            sd_xfer_flow_t fl = x.flow;
            size_t n = sd_xfer_produce(&x, &w, &fl, rd, NULL, now, &out);
            if (n == SIZE_MAX) printf("E\n");
            else if (n) hex('D', out, n);
            else printf("%c %u\n", x.id ? 'N' : 'X', reads);
            fflush(stdout);
        } else if (cmd[0] == 'Q') break;
    }
    free(data);
    return 0;
}
'''


class Sender:
    def __init__(self, exe, path):
        self.p = subprocess.Popen([str(exe), str(path)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)

    def ask(self, line):
        self.p.stdin.write(line + '\n')
        self.p.stdin.flush()
        return self.p.stdout.readline().split()

    def close(self):
        self.p.stdin.write('Q\n')
        self.p.stdin.close()
        self.p.stdout.close()
        assert self.p.wait() == 0


class Station:
    """GS receiver: reassembly, SDX decode, CRC, contiguous commit, SACK."""

    def __init__(self, xfer, start, end):
        self.xfer, self.next, self.end = xfer, start, end
        self.got = {}           # seq -> (flags, block, raw_len, crc, body)
        self.blocks = {}        # offset -> bytes (verified)
        self.out = bytearray()
        self.una = 0
        self.bad = 0

    def data(self, msg):
        assert msg[0] == 0xA6
        if msg[1] != self.xfer:
            return
        flags, seq, block = msg[2], struct.unpack_from('<H', msg, 3)[0], struct.unpack_from('<I', msg, 5)[0]
        raw_len = crc = None
        body = msg[9:]
        if flags & 2:
            raw_len, crc = struct.unpack_from('<II', msg, 9)
            body = msg[17:]
        if seq < self.una or seq in self.got:
            return
        self.got[seq] = (flags, block, raw_len, crc, body)
        while self.una in self.got:
            self.una += 1
        self.assemble()

    def assemble(self):
        for seq, (flags, block, _, _, _) in sorted(self.got.items()):
            if not flags & 1 or block in self.blocks:
                continue
            parts, s = [], seq
            while s in self.got and self.got[s][1] == block:
                parts.append(self.got[s])
                if self.got[s][0] & 2:
                    break
                s += 1
            if not parts or not parts[-1][0] & 2:
                continue
            body = b''.join(p[4] for p in parts)
            raw = ref.decode_block(body) if flags & 4 else body
            if len(raw) != parts[-1][2] or zlib.crc32(raw) != parts[-1][3]:
                self.bad += 1
                continue
            self.blocks[block] = raw
        while self.next in self.blocks:
            raw = self.blocks.pop(self.next)
            self.out += raw
            self.next += len(raw)

    def ack(self, rate, window):
        bitmap = 0
        for i in range(32):
            if self.una + 1 + i in self.got:
                bitmap |= 1 << i
        return 'K %d %d %d %d %d' % (self.xfer, self.una & 0xffff, bitmap, rate, window)

    def complete(self):
        return self.next == self.end


def run_transfer(exe, path, data, *, start=0, length=0, loss_down=0.0, loss_up=0.0, seed=1, sdx=1,
                 msg=460, window=8, block=16, rate=1536, max_ms=10_000_000, stop_after=None):
    """Simulated time in ms. One produce per ms tick while pacing allows."""
    rnd = random.Random(seed)
    s = Sender(exe, path)
    ans = bytes.fromhex(s.ask('O 7 %d %d %d %d %d %d %d 0' % (sdx, start, length, rate, msg, window, block))[1])
    assert ans[0] == 0xA5 and ans[1] == 0, ans
    st_off, st_len = struct.unpack_from('<II', ans, 8)
    st = Station(7, st_off, st_off + st_len)
    now, next_send, last_ack, since_ack, sent, sent_bytes = 0, 0, 0, 0, 0, 0
    in_flight = []          # (arrive_ms, msg)
    while not st.complete() and now < max_ms:
        now += 1
        for item in [m for m in in_flight if m[0] <= now]:
            in_flight.remove(item)
            st.data(item[1])
            since_ack += 1
        if stop_after is not None and len(st.out) >= stop_after:
            break
        if since_ack >= window // 2 or (since_ack and now - last_ack >= 1000) or now - last_ack >= 5000:
            line = st.ack(rate, window) + ' %d' % now
            last_ack, since_ack = now, 0
            if rnd.random() >= loss_up:
                s.ask(line)
        if now >= next_send:
            r = s.ask('P %d' % now)
            if r[0] == 'E':
                raise AssertionError('read error')
            if r[0] in 'NX':
                next_send = now + 10      # nothing due: look again shortly
            if r[0] == 'D':
                m = bytes.fromhex(r[1])
                assert len(m) <= msg
                sent += 1
                sent_bytes += len(m)
                next_send = now + max(1, len(m) * 1000 // rate)
                if rnd.random() >= loss_down:
                    in_flight.append((now + 150, m))
    reads = None
    if st.complete():
        # Final ACK: the sender ends the transfer once everything is acknowledged.
        s.ask(st.ack(rate, window) + ' %d' % (now + 1))
        r = s.ask('P %d' % (now + 2))
        assert r[0] == 'X', r
        reads = int(r[1])
    s.close()
    return st, sent, sent_bytes, now, reads


class TransferTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        exe = pathlib.Path(cls.tmp.name) / 'xfer'
        subprocess.run(CFLAGS + ['-x', 'c', '-', str(ROOT / 'Core/Src/sd_xfer.c'), str(ROOT / 'Core/Src/sd_xfer_codec.c'),
                                 '-o', str(exe)],
                       input='#include <stddef.h>\n#include <stdint.h>\n' + crc_source() + DRIVER, text=True, check=True)
        cls.exe = exe
        cls.data = fc_rows(2500) + daq_rows(2500)
        cls.path = pathlib.Path(cls.tmp.name) / 'irec_LAUNCH-1-000.csv'
        cls.path.write_bytes(cls.data)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_lossless_stream_is_byte_identical_and_sends_each_message_once(self):
        st, sent, sent_bytes, _, _ = run_transfer(self.exe, self.path, self.data)
        self.assertEqual(bytes(st.out), self.data)
        self.assertEqual(sent, st.una, 'no retransmission without loss')
        self.assertLess(sent_bytes * 4, len(self.data), 'DATA bytes at least 4x below the CSV')

    def test_lossy_links_still_reconstruct_exactly(self):
        for loss, seed in ((0.05, 3), (0.15, 4), (0.30, 5)):
            with self.subTest(loss=loss):
                st, sent, _, ms, _ = run_transfer(self.exe, self.path, self.data, loss_down=loss, loss_up=loss, seed=seed)
                self.assertTrue(st.complete(), 'stalled at %d of %d after %d ms' % (st.next, st.end, ms))
                self.assertEqual(bytes(st.out), self.data)
                self.assertEqual(st.bad, 0)
                # Only gaps are resent: the overhead stays near the loss rate.
                self.assertLess(sent, st.una * (1 + 3 * loss) + 8, (sent, st.una))

    def test_resume_from_a_committed_offset_and_range_reads(self):
        st, _, _, _, _ = run_transfer(self.exe, self.path, self.data, stop_after=len(self.data) // 3)
        done = len(st.out)
        self.assertGreater(done, 0)
        rest, _, _, _, _ = run_transfer(self.exe, self.path, self.data, start=done, seed=9, loss_down=0.1)
        self.assertEqual(bytes(st.out[:done]) + bytes(rest.out), self.data)
        tail, _, _, _, _ = run_transfer(self.exe, self.path, self.data, start=len(self.data) - 1000, length=1000)
        self.assertEqual(bytes(tail.out), self.data[-1000:])
        mid, _, _, _, _ = run_transfer(self.exe, self.path, self.data, start=12345, length=777, msg=64)
        self.assertEqual(bytes(mid.out), self.data[12345:12345 + 777])

    def test_raw_mode_and_odd_sizes(self):
        for sdx, msg, block, window in ((0, 460, 16, 8), (1, 64, 1, 1), (1, 768, 64, 32), (0, 64, 2, 3)):
            with self.subTest(sdx=sdx, msg=msg, block=block, window=window):
                st, _, _, _, _ = run_transfer(self.exe, self.path, self.data, sdx=sdx, msg=msg, block=block,
                                              window=window, loss_down=0.1, seed=msg)
                self.assertEqual(bytes(st.out), self.data)

    def test_window_pause_stale_acks_and_timeout(self):
        s = Sender(self.exe, self.path)
        ans = bytes.fromhex(s.ask('O 9 1 0 0 1536 460 4 16 0')[1])
        self.assertEqual(ans[1], 0)
        sent = [s.ask('P %d' % t) for t in range(1, 10)]
        self.assertEqual([r[0] for r in sent], ['D'] * 4 + ['N'] * 5, 'window of 4')
        self.assertEqual(s.ask('K 8 4 0 1536 4 20'), ['k', '0'], 'stale transfer id')
        self.assertEqual(s.ask('K 9 40 0 1536 4 20'), ['k', '0'], 'una beyond what was sent')
        self.assertEqual(s.ask('K 9 2 0 0 4 20'), ['k', '1'])
        self.assertEqual(s.ask('P 21')[0], 'N', 'rate 0 pauses')
        self.assertEqual(s.ask('K 9 2 0 1536 4 30'), ['k', '1'])
        self.assertEqual(s.ask('P 31')[0], 'D', 'resumed')
        self.assertEqual(s.ask('P 15100')[0], 'X', 'no ACK for 15 s ends the transfer')
        s.close()

    def test_open_answers_end_and_bad_offset(self):
        s = Sender(self.exe, self.path)
        self.assertEqual(bytes.fromhex(s.ask('O 3 1 %d 0 1536 460 4 16 0' % len(self.data))[1])[1], 6)
        self.assertEqual(bytes.fromhex(s.ask('O 3 1 %d 0 1536 460 4 16 0' % (len(self.data) + 1))[1])[1], 7)
        s.close()


if __name__ == '__main__':
    unittest.main()
