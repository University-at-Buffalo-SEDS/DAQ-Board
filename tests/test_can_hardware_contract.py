import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

class CanHardwareContract(unittest.TestCase):
    def test_enqueue_timeout_is_counted_as_tx_failure(self):
        source = (ROOT / "Core/Src/can_bus.c").read_text()
        path = source.split("if (slot_status != HAL_OK)", 1)[1].split("return slot_status;", 1)[0]
        self.assertIn("g_fdcan_tx_fail_count++;", path)

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
        self.assertIn("CAN_BUS_TX_ENQUEUE_TIMEOUT_MS 5U", can)
        self.assertIn("can_bus_recover_if_bus_off", can)
        self.assertNotIn("< (uint32_t)frag_cnt", can)
