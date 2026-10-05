from pathlib import Path
import re
import unittest

ROOT=Path(__file__).resolve().parents[1]
class RtosAcquisitionClockTests(unittest.TestCase):
    def test_scheduler_tick_matches_all_startup_ports(self):
        user=(ROOT/'Core/Inc/tx_user.h').read_text()
        frequency=int(re.search(r'#define\s+TX_TIMER_TICKS_PER_SECOND\s+(\d+)',user).group(1))
        assembly=(ROOT/'Core/Src/tx_initialize_low_level.S').read_text()
        frequencies=[int(v) for v in re.findall(r'SYSTEM_CLOCK\s*/\s*(\d+)',assembly)]
        self.assertEqual(frequencies,[frequency]*3)
        self.assertEqual(frequency,1000)

    def test_adc_uses_independent_microsecond_hardware_timer(self):
        source=(ROOT/'Core/Src/mcp3564r.c').read_text()
        timer=source.split('static HAL_StatusTypeDef mcp3564r_schedule',1)
        self.assertIn('__HAL_TIM_SET_AUTORELOAD(&htim2, delay_us - 1U)',source)
        self.assertIn('HAL_TIM_Base_Start_IT(&htim2)',source)
        self.assertNotIn('TX_TIMER_TICKS_PER_SECOND',source)

    def test_acquisition_pool_lock_inherits_priority(self):
        source=(ROOT/'Core/Src/sd_card.c').read_text()
        self.assertIn('tx_mutex_create(&g_sd_pool_mutex, "sd_pool", TX_INHERIT)',source)
