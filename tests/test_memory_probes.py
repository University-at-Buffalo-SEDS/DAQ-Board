import json
import unittest
from pathlib import Path


class MemoryProbeContractTests(unittest.TestCase):
    def test_simulator_guards_firmware_allocator_and_locking(self):
        root = Path(__file__).resolve().parents[1]
        layout = json.loads((root / "sim" / "board.json").read_text())
        probes = {
            probe["symbol"]: probe.get("maximum")
            for probe in layout["execution"]["memory_probes"]
        }
        self.assertEqual(
            probes,
            {
                "g_telemetry_alloc_fail": 0,
                "g_telemetry_panic_count": 0,
                "g_telemetry_lock_get_fail": 0,
                "g_telemetry_lock_put_fail": 0,
            },
        )

        hooks = (root / "Core" / "Src" / "telemetry_hooks.c").read_text()
        for symbol in probes:
            self.assertIn(symbol, hooks)


if __name__ == "__main__":
    unittest.main()

