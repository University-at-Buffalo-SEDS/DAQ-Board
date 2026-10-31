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

    def test_adc_uses_drdy_exti_and_never_blocks_spi_in_interrupt(self):
        source=(ROOT/'Core/Src/mcp3564r.c').read_text()
        main=(ROOT/'Core/Src/main.c').read_text()
        irq=(ROOT/'Core/Src/stm32u5xx_it.c').read_text()
        header=(ROOT/'Core/Inc/main.h').read_text()
        self.assertIn('#define ADC_IRQ_Pin GPIO_PIN_10',header)
        self.assertIn('#define ADC_IRQ_GPIO_Port GPIOB',header)
        self.assertIn('GPIO_MODE_IT_FALLING',main)
        self.assertIn('void HAL_GPIO_EXTI_Falling_Callback(uint16_t pin)',main)
        self.assertIn('mcp3564r_data_ready_callback(pin)',main)
        self.assertIn('void EXTI10_IRQHandler(void)',irq)
        self.assertIn('HAL_GPIO_EXTI_IRQHandler(ADC_IRQ_Pin)',irq)
        self.assertNotIn('HAL_TIM_Base_Start_IT',source)
        self.assertNotIn('TX_TIMER_TICKS_PER_SECOND',source)
        read=source.split('static HAL_StatusTypeDef mcp3564r_start_read_dma(void)',1)[1].split('static HAL_StatusTypeDef mcp3564r_clear_stale_ready',1)[0]
        self.assertIn('HAL_SPI_TransmitReceive_DMA',read)
        self.assertNotRegex(read,r'HAL_SPI_TransmitReceive\s*\(')
        self.assertNotIn('HAL_Delay',read)

    def test_acquisition_sd_admission_cannot_wait_on_a_shared_allocator(self):
        source=(ROOT/'Core/Src/sd_card.c').read_text()
        self.assertNotIn('tx_mutex_get(',source)
        self.assertIn('tx_block_allocate(pool, &allocation, TX_NO_WAIT)',source)
        self.assertNotIn('tx_byte_allocate(',source)
