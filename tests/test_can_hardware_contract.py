import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

class CanHardwareContract(unittest.TestCase):
    def test_uses_known_good_fill_bus_timing(self):
        source = (ROOT / "Core/Src/main.c").read_text()
        ioc = (ROOT / "DAQ-Board.ioc").read_text()
        self.assertIn("hfdcan1.Init.AutoRetransmission = ENABLE", source)
        self.assertIn("hfdcan1.Init.NominalPrescaler = 9", source)
        self.assertIn("hfdcan1.Init.NominalTimeSeg1 = 1", source)
        self.assertIn("hfdcan1.Init.NominalTimeSeg2 = 1", source)
        self.assertIn("FDCAN1.CalculateBaudRateNominal=3555555", ioc)
        self.assertIn("FDCAN1.AutoRetransmission=ENABLE", ioc)
        can = (ROOT / "Core/Src/can_bus.c").read_text()
        self.assertIn("can_bus_wait_for_tx_slot", can)
        self.assertIn("can_bus_recover_if_bus_off", can)
