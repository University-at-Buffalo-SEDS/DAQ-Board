"""SDX encoder (Core/Src/sd_xfer_codec.c): byte-exact round trip.

The production encoder runs on the host under ASan/UBSan inside a driver that
feeds it the way the SD thread does (a bounded raw window refilled in 512-byte
sectors, blocks of a target size, messages of a fixed capacity); the
reference decoder (tests/sdx_reference.py, mirroring the GroundStation26
decoder) must reproduce every block's original bytes."""
import pathlib
import random
import struct
import subprocess
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tests'))
import sdx_reference as ref  # noqa: E402

CFLAGS = ['cc', '-std=gnu11', '-O1', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
          '-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-I', str(ROOT / 'Core/Inc')]

DRIVER = r'''
#include "sd_xfer_codec.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* argv: input block_bytes msg_bytes window_bytes output. Output records:
 * [u32 block_offset][u32 raw_len][u32 nmsgs] then per message [u16 len][bytes]. */
int main(int argc, char **argv)
{
    if (argc != 6) return 2;
    FILE *f = fopen(argv[1], "rb"); fseek(f, 0, SEEK_END); long size = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *data = malloc((size_t)size + 1); if (fread(data, 1, (size_t)size, f) != (size_t)size) return 3; fclose(f);
    const size_t block = (size_t)atol(argv[2]), cap = (size_t)atol(argv[3]), window = (size_t)atol(argv[4]);
    FILE *o = fopen(argv[5], "wb");
    static sdx_enc_t enc;
    uint8_t *msg = malloc(cap), *msgs = malloc(1 << 20);
    for (size_t start = 0; start < (size_t)size;) {
        sdx_enc_reset(&enc);
        size_t pos = start, used = 0, nmsg = 0, mlen = 0;
        while (pos < (size_t)size && pos - start < block) {
            /* The board holds at most 'window' raw bytes from pos, refilled in
             * 512-byte sectors aligned to the file, never past the file end. */
            size_t avail = window - (pos % 512U);
            if (pos + avail > (size_t)size) avail = (size_t)size - pos;
            const unsigned flags = (pos + avail == (size_t)size ? SDX_FINAL : 0U) | (mlen == 0 ? SDX_FRESH : 0U);
            size_t written = 0;
            size_t n = sdx_enc_item(&enc, data + pos, avail, flags, msg + mlen, cap - mlen, &written);
            if (n == 0) n = sdx_enc_item(&enc, data + pos, avail, flags | SDX_FINAL, msg + mlen, cap - mlen, &written);
            if (n == SDX_FLUSH) {
                if (mlen == 0) return 4;
                msgs[used] = (uint8_t)mlen; msgs[used + 1] = (uint8_t)(mlen >> 8);
                memcpy(msgs + used + 2, msg, mlen); used += 2 + mlen; ++nmsg; mlen = 0;
                continue;
            }
            if (n == 0 || written > cap - mlen) return 5;
            mlen += written; pos += n;
        }
        if (mlen) {
            msgs[used] = (uint8_t)mlen; msgs[used + 1] = (uint8_t)(mlen >> 8);
            memcpy(msgs + used + 2, msg, mlen); used += 2 + mlen; ++nmsg;
        }
        uint32_t hdr[3] = {(uint32_t)start, (uint32_t)(pos - start), (uint32_t)nmsg};
        fwrite(hdr, 4, 3, o); fwrite(msgs, 1, used, o);
        start = pos;
    }
    fclose(o);
    free(data); free(msg); free(msgs);
    return 0;
}
'''


def build(tmp):
    exe = pathlib.Path(tmp) / 'drv'
    subprocess.run(CFLAGS + ['-x', 'c', '-', str(ROOT / 'Core/Src/sd_xfer_codec.c'), '-o', str(exe)],
                   input=DRIVER, text=True, check=True)
    return exe


def encode(exe, tmp, data, block=16384, cap=480, window=1024):
    src = pathlib.Path(tmp) / 'in.csv'
    out = pathlib.Path(tmp) / 'out.bin'
    src.write_bytes(data)
    subprocess.run([str(exe), str(src), str(block), str(cap), str(window), str(out)], check=True)
    raw = out.read_bytes()
    blocks = []
    pos = 0
    while pos < len(raw):
        start, length, nmsg = struct.unpack_from('<III', raw, pos)
        pos += 12
        msgs = []
        for _ in range(nmsg):
            (n,) = struct.unpack_from('<H', raw, pos)
            msgs.append(raw[pos + 2:pos + 2 + n])
            pos += 2 + n
        blocks.append((start, length, msgs))
    return blocks


def fmt_float(value):
    """sd_float_format.h, bit for bit (binary64 normalisation of a binary32)."""
    value = struct.unpack('<f', struct.pack('<f', value))[0]
    bits = struct.unpack('<I', struct.pack('<f', value))[0]
    sign = '-' if bits >> 31 else ''
    mag = bits & 0x7fffffff
    if mag >= 0x7f800000:
        return sign + ('inf' if mag == 0x7f800000 else 'nan')
    if mag == 0:
        return sign + '0'
    n, e = abs(value), 0
    while n >= 10.0:
        n /= 10.0
        e += 1
    while n < 1.0:
        n *= 10.0
        e -= 1
    d = int(n * 100000000.0 + 0.5)
    if d == 1000000000:
        d, e = 100000000, e + 1
    return '%s%d.%08de%s%02d' % (sign, d // 100000000, d % 100000000, '-' if e < 0 else '+', abs(e))


HEADER = ('timestamp_ms,monotonic_ms,sensor,value,raw_adc_code,raw_value,calibrated_value,time_source,'
          'adc_temperature_c,adc_temperature_code,uptime_ticks,uptime_frequency_hz,record_sequence\r\n'
          '# seds-sd,format=fc,version=2,board=FC,firmware=0123456789ab,tick_hz=1000,schema=e59ae8a7\r\n')


def fc_rows(n, seed=1):
    """Rows as sd_format_csv_common writes them, with sensor-like values."""
    rnd = random.Random(seed)
    out = [HEADER]
    t, seq = 1794877191000, 48000
    for i in range(n):
        t += rnd.choice((0, 1, 1, 2, 9, 10))
        up = t - 1794877191000 + 734000
        utc = t if i % 97 else 0
        kind = i % 5
        if kind < 2:
            raw = [rnd.randint(-2000, 2000) for _ in range(3)]
            val = ' '.join(fmt_float(r * 0.00718) for r in raw)
            rawt = ' '.join(fmt_float(r) for r in raw)
            sensor = 'accelerometer_sample' if kind == 0 else 'gyroscope_sample'
        elif kind == 2:
            val = ' '.join(fmt_float(x) for x in (91000 + rnd.gauss(0, 3), 30 + rnd.gauss(0, 0.1), rnd.gauss(0, 2)))
            rawt = ' '.join([fmt_float(8300000 + rnd.randint(-50, 50)), fmt_float(5100000 + rnd.randint(-9, 9)), 'nan'])
            sensor = 'barometer_sample'
        elif kind == 3:
            val = ' '.join(fmt_float(rnd.gauss(0, 10 ** rnd.randint(-6, 3))) for _ in range(4))
            rawt, sensor = None, 'DKF'
        else:
            val = rnd.choice(('EV new flight state: 9', 'drogue', 'he said "go", then, ok', 'line one\nline two', ''))
            rawt, sensor = None, rnd.choice(('Ordered', 'pyro_gpio_high', 'Message'))
        v = val.replace('"', '""')
        out.append('%d,%d,%s,"%s","%s","%s","%s",%s,,,%d,1000,%d\r\n' % (
            utc or up, up, sensor, v, rawt or '', rawt or '', val if rawt else '',
            'network' if utc else 'local', up, seq))
        seq += 1
    return ''.join(out).encode()


def daq_rows(n, seed=2):
    """Raw MCP3564R/supply rows as sd_format_csv_raw_record, plus scalars."""
    rnd = random.Random(seed)
    out = [HEADER.replace('format=fc,version=2,board=FC', 'format=daq,version=3,board=DAQ')]
    up, seq = 5200000, 7000000
    temp = '2.45000000e+01'
    for i in range(n):
        if i % 3 == 0:
            up += 1
        if i % 31 == 0:
            out.append('# calibration,kg50_c0=1.00000000e+00,kg50_tare=%s\r\n' % fmt_float(rnd.random()))
        code = rnd.randint(600000, 620000) if i % 3 == 0 else rnd.randint(-900, 900)
        volts = code * 2.5 / 8388608
        rep = i % 17 == 0
        if i % 7 == 6:
            out.append('%d,%d,vmon_v,"%s",,,,network,,,%d,1000,%d\r\n' % (
                1794877191000 + up, up, fmt_float(1.17 + rnd.gauss(0, 0.01)), up, seq))
        else:
            out.append(','.join([str(1794877191000 + up), str(up), ('kg1000_v', 'kg50_v', 'chamber_pressure_v')[i % 3], '',
                                 str(code), fmt_float(volts), fmt_float(volts * 9481.62), 'network',
                                 temp if rep else '', '-200' if rep else '', str(up * 1000 + 333 * (i % 3)),
                                 '1000000', str(seq)]) + '\r\n')
        seq += 1
    return ''.join(out).encode()


EDGE = (b'only header\r\n# comment, with commas\r\n'
        b'1,2,x,"",,,,local,,,2,1000,3\r\n'                       # empty quoted field
        b'1,2,x,"a  b",,,,local,,,2,1000,3\r\n'                   # double space: not a vector
        b'1,2,x," a",,,,local,,,2,1000,3\r\n'                     # leading space
        b'1,2,x,"-0 007 1e5 1E-05 +3 .5 5. 1.5e+100 99999999999999999999",,,,local,,,2,1000,3\r\n'
        b'-0,0007,x,"nan inf -inf",,,,local,,,2,1000,3\r\n'
        b'1,2,x,"q""uote",,,,local,,,2,1000,3\n'                 # LF only
        b'1,2,"quoted label",v,,,,local,,,2,1000,3\r\n'
        b'1,2,x,v,,,,local,,,2,1000\r\n'                          # 12 fields
        b'1,2,x,v,,,,local,,,2,1000,3,4\r\n'                      # 14 fields
        b'1,2,x,v,,,,local,,,2,1000,3\r'                          # CR only
        b'1,2,' + b'y' * 40 + b',v,,,,local,,,2,1000,3\r\n'        # label too long
        + b'1,2,z,"' + b' '.join(b'%d' % i for i in range(20)) + b'",,,,local,,,2,1000,3\r\n'
        + b'1,2,z,"' + b'w' * 3000 + b'",,,,local,,,2,1000,3\r\n'  # longer than the window
        + bytes(range(256)) + b'\r\n'
        + b''.join(b'%d,%d,lab%d,"1.0e+0%d 2",,,,local,,,2,1000,3\r\n' % (i, i, i, i % 10) for i in range(40))
        + b'9,9,x,"1.5e+00 2.5e-01",,,,local,,,9,1000,9\r\n'
        + b'9,9,x,"1.6e+03 2.5e-04",,,,local,,,9,1000,9\r\n'       # exponent jumps beyond 3
        + b'unterminated "quote,1,2\r\n1,2,x,v,,,,local,,,2,1000,3')


class CodecTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.exe = build(cls.tmp.name)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def roundtrip(self, data, **kw):
        blocks = encode(self.exe, self.tmp.name, data, **kw)
        pos = 0
        sizes = 0
        for start, length, msgs in blocks:
            self.assertEqual(start, pos)
            cap = kw.get('cap', 480)
            self.assertTrue(all(0 < len(m) <= cap for m in msgs), [len(m) for m in msgs])
            body = b''.join(msgs)
            self.assertEqual(ref.decode_block(body), data[start:start + length], 'block at %d' % start)
            # Items never span messages: each message decodes on its own
            # once the block state before it is known.
            pos += length
            sizes += len(body)
        self.assertEqual(pos, len(data))
        return sizes

    def test_fc_rows_round_trip_and_shrink(self):
        data = fc_rows(3000)
        size = self.roundtrip(data)
        self.assertLess(size * 3.5, len(data), 'binary encoding at least 3.5x on FC rows')

    def test_daq_rows_round_trip_and_shrink(self):
        data = daq_rows(6000)
        size = self.roundtrip(data)
        self.assertLess(size * 3.5, len(data))

    def test_edge_cases_are_carried_exactly(self):
        for block, cap, window in ((16384, 480, 1024), (64, 16, 512), (700, 100, 600), (1 << 20, 4096, 4096)):
            self.roundtrip(EDGE, block=block, cap=cap, window=window)

    def test_any_bytes_and_any_cut_are_lossless(self):
        rnd = random.Random(7)
        base = fc_rows(300) + daq_rows(300)
        for _ in range(12):
            cut = rnd.randrange(len(base))
            noisy = bytearray(base[cut:cut + 6000])
            for _ in range(20):
                noisy[rnd.randrange(len(noisy))] = rnd.choice(b',"\r\n -.e0123456789xX')
            self.roundtrip(bytes(noisy), block=rnd.choice((512, 4096, 16384)),
                           cap=rnd.choice((64, 300, 480, 1024)), window=rnd.choice((512, 1024, 2048)))

    def test_blocks_are_independent(self):
        data = fc_rows(1500)
        blocks = encode(self.exe, self.tmp.name, data, block=4096)
        self.assertGreater(len(blocks), 3)
        for start, length, msgs in reversed(blocks):   # any order, alone
            self.assertEqual(ref.decode_block(b''.join(msgs)), data[start:start + length])


if __name__ == '__main__':
    unittest.main()
