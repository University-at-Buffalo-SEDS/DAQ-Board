"""Compile the real diagnostic parser against valid and malformed wire frames."""
import json
import os
from pathlib import Path
import random
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
PEERS = ("RF", "PB", "FC", "GB", "AB", "VB", "DAQ")


def source_address(name):
    # Independent reference: SEDSnet packet.rs's stable logical-sender hash.
    value = 0xA6D38C214B7F19E5
    for byte in name.encode():
        value = ((value ^ byte) * 0x9E3779B1) & ((1 << 64) - 1)
        value ^= value >> 27
    return (value & 0xFFFFFFFF) or 1


def uleb(value):
    encoded = bytearray()
    while value >= 128:
        encoded.append((value & 127) | 128)
        value >>= 7
    encoded.append(value)
    return bytes(encoded)


def frame(address, timestamp=0, nonce=False, data_type=3):
    return (bytes((8 if nonce else 0, 1)) + uleb(data_type) + uleb(0)
            + uleb(timestamp) + (uleb(65535) if nonce else b"") + uleb(address))


class SimulationProbeParser(unittest.TestCase):
    def compile_and_run(self, cases, check_type=False):
        compiler = shutil.which(os.environ.get("CC", "cc"))
        if compiler is None:
            self.skipTest("host C compiler unavailable")
        prefix = r"""
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
typedef int SedsResult;
typedef struct { int dummy; } SedsRouter;
typedef struct { uint32_t ty; const char *sender; size_t sender_len; } SedsPacketView;
enum { SEDS_OK=0, SEDS_DT_HEARTBEAT=3 };
volatile uint32_t g_telemetry_peer_mask, g_sim_heartbeat_attempts;
volatile uint32_t g_sim_heartbeat_ok, g_sim_heartbeat_fail, g_sim_heartbeat_wire_tx;
int seds_router_log_bytes_ex(SedsRouter *r, int t, const uint8_t *p,
                            size_t n, void *d, int q) {
  (void)r; (void)t; (void)p; (void)n; (void)d; (void)q; return 0;
}
"""
        include = '#include ' + json.dumps(str(ROOT / "Core/Inc/sim_network_probe.h")) + '\n'
        for enabled in (False, True):
            with self.subTest(simulation=enabled), tempfile.TemporaryDirectory(
                    prefix="seds-probe-parser-") as directory:
                body = []
                for index, (wire, expected) in enumerate(cases):
                    array = ",".join(str(byte) for byte in wire) or "0"
                    body.append(f"static const uint8_t b{index}[]={{{array}}};\n")
                    if check_type:
                        body.append(f"assert(sim_probe_packed_data_type(b{index}, {len(wire)})"
                                    f" == UINT32_C({expected}));\n")
                        body.append("g_sim_heartbeat_wire_tx=0;\n")
                        body.append(f"sim_probe_observe_can_tx(b{index}, {len(wire)});\n")
                        body.append("assert(g_sim_heartbeat_wire_tx == "
                                    f"{int(enabled and expected == 3)}U);\n")
                    else:
                        body.append("g_telemetry_peer_mask=0;\n")
                        body.append(f"sim_probe_observe_packed(b{index}, {len(wire)});\n")
                        body.append(f"assert(g_telemetry_peer_mask == {expected if enabled else 0}U);\n")
                source = Path(directory) / "test.c"
                source.write_text(prefix + include + "int main(void) {\n"
                                  + "".join(body) + "return 0;\n}\n")
                binary = Path(directory) / "test"
                flags = ["-DSEDS_FIRMWARE_SIM_TEST=1"] if enabled else []
                subprocess.run([compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                                *flags, str(source), "-o", str(binary)], check=True,
                               capture_output=True, text=True)
                subprocess.run([str(binary)], check=True, capture_output=True, text=True)

    def test_all_peers_wide_timestamps_nonce_and_every_truncated_prefix(self):
        cases = []
        for index, name in enumerate(PEERS):
            for timestamp in (0, 127, 128, (1 << 32) - 1, 1 << 40, (1 << 64) - 1):
                for nonce in (False, True):
                    wire = frame(source_address(name), timestamp, nonce)
                    cases.append((wire, 1 << index))
                    cases.extend((wire[:end], 0) for end in range(len(wire)))
        cases.extend((frame(address), 0) for address in (0, 1, 0xFFFFFFFF, 123456789))
        self.compile_and_run(cases)

    def test_wide_source_ids_cannot_impersonate_known_peers(self):
        self.compile_and_run([(frame(source_address(name) + (1 << 32)), 0)
                              for name in PEERS])

    def test_overflowing_timestamp_does_not_mark_peer_alive(self):
        wire = b"\x00\x01\x03\x00" + b"\x80" * 9 + b"\x02" + uleb(source_address("RF"))
        self.compile_and_run([(wire, 0)])

    def test_type_ids_and_heartbeat_counter_keep_full_u32_range(self):
        random_ids = random.Random(20261019)
        values = [0, 3, 127, 128, (1 << 14), (1 << 28), 0xFFFFFFFF]
        values += [random_ids.getrandbits(32) for _ in range(64)]
        cases = [(frame(1, data_type=value), value) for value in values]
        for value in (1 << 32, 1 << 40, (1 << 64) - 1):
            cases.append((frame(1, data_type=value), 0xFFFFFFFF))
        for length in range(1, 3):
            cases.append((b"\x00" * length, 0xFFFFFFFF))
        self.compile_and_run(cases, check_type=True)


if __name__ == "__main__":
    unittest.main()
