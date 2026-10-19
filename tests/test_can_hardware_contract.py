import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

class CanHardwareContract(unittest.TestCase):
    def test_owned_queue_keeps_backpressure_out_of_receive_dispatch(self):
        source = (ROOT / "Core/Src/can_bus.c").read_text()
        self.assertIn("return can_tx_queue_submit(bytes, len, std_id);", source)
        self.assertIn("if (HAL_FDCAN_GetTxFifoFreeLevel(g_hfdcan) == 0U) return HAL_BUSY;", source)
        self.assertNotIn("can_bus_wait_for_tx_slot", source)
        self.assertIn("can_tx_queue_service();", source)
        self.assertIn("FDCAN_IT_TX_COMPLETE", source)

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
        self.assertIn("can_bus_recover_if_bus_off", can)
        self.assertNotIn("< (uint32_t)frag_cnt", can)
