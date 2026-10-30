"""Link the actual MCU scripts, rejecting DMA placement outside uncached SRAM."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class AdcDmaPlacementTests(unittest.TestCase):
    def test_both_linker_scripts_pin_zeroed_dma_and_reject_external_ram(self):
        cc = shutil.which('arm-none-eabi-gcc')
        nm = shutil.which('arm-none-eabi-nm')
        if not cc or not nm:
            self.skipTest('ARM linker/nm are required for the actual linker contract')
        source = '''
#include <stdint.h>
__attribute__((section(".bss.adc_dma"),aligned(32),used)) uint8_t tx_frame[32];
__attribute__((section(".bss.adc_dma"),aligned(32),used)) uint8_t rx_frame[32];
void Reset_Handler(void) {}
'''
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder/'fixture.c').write_text(source)
            subprocess.run([cc, '-mcpu=cortex-m33', '-mthumb', '-c', str(folder/'fixture.c'), '-o', str(folder/'fixture.o')], check=True, capture_output=True)
            library = subprocess.check_output([cc, '-print-file-name=libc.a'], text=True).strip()
            for name in ('STM32U585xx_FLASH.ld', 'STM32U585xx_RAM.ld'):
                with self.subTest(script=name):
                    script = (ROOT/name).read_text()
                    for external in (False, True):
                        candidate = script.replace('ORIGIN = 0x20000000', 'ORIGIN = 0x60000000') if external else script
                        linker = folder/'fixture.ld'
                        linker.write_text(candidate)
                        image = folder/'fixture.elf'
                        command = [cc, '-mcpu=cortex-m33', '-mthumb', '-nostdlib', '-L'+str(Path(library).parent), '-Wl,-T,'+str(linker), str(folder/'fixture.o'), '-o', str(image)]
                        result = subprocess.run(command, capture_output=True, text=True)
                        if external:
                            self.assertNotEqual(result.returncode, 0)
                            self.assertIn('ADC DMA buffers require internal uncached SRAM', result.stderr)
                            # The same external region links if this exact safety
                            # guard is removed: failure is not missing libraries.
                            linker.write_text(re.sub(r'  ASSERT\(__adc_dma_start__ >= 0x20000000.*?"ADC DMA buffers require internal uncached SRAM"\)\s*', '', candidate, flags=re.S))
                            control = subprocess.run(command, capture_output=True, text=True)
                            self.assertEqual(control.returncode, 0, control.stderr)
                        else:
                            self.assertEqual(result.returncode, 0, result.stderr)
                            output = subprocess.check_output([nm, '-n', str(image)], text=True)
                            symbols = {line.split()[-1]:int(line.split()[0],16) for line in output.splitlines() if len(line.split()) == 3}
                            start, end = symbols['__adc_dma_start__'], symbols['__adc_dma_end__']
                            self.assertEqual(end-start, 64)
                            self.assertEqual(start % 32, 0)
                            self.assertGreaterEqual(start, 0x20000000)
                            self.assertLessEqual(end, 0x200c0000)
                            self.assertLessEqual(symbols['_sbss'], start)
                            self.assertGreaterEqual(symbols['_ebss'], end)
                            self.assertTrue(start <= symbols['tx_frame'] < end)
                            self.assertTrue(start <= symbols['rx_frame'] < end)
        startup = (ROOT/'startup_stm32u585xx.s').read_text()
        self.assertIn('ldr\tr2, =_sbss', startup)
        self.assertIn('ldr\tr3, = _ebss', startup)
        self.assertIn('str\tr3, [r2], #4', startup)
