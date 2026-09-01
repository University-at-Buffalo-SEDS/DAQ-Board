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
            {key: probes[key] for key in (
                "g_telemetry_alloc_fail",
                "g_telemetry_panic_count",
                "g_telemetry_lock_get_fail",
                "g_telemetry_lock_put_fail",
            )},
            {
                "g_telemetry_alloc_fail": 0,
                "g_telemetry_panic_count": 0,
                "g_telemetry_lock_get_fail": 0,
                "g_telemetry_lock_put_fail": 0,
            },
        )
        self.assertIn("g_telemetry_pool_available", probes)
        self.assertIn("g_telemetry_network_ready", probes)

        hooks = (root / "Core" / "Src" / "telemetry_hooks.c").read_text()
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text()
        can_bus = (root / "Core" / "Src" / "can_bus.c").read_text()
        for symbol in probes:
            self.assertIn(symbol, hooks + telemetry + can_bus)


if __name__ == "__main__":
    unittest.main()
