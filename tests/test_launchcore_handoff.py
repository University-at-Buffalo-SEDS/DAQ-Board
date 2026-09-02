import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIXED_LAUNCHCORE = "7cdbca87b7fad2c72d73257f4fb1b14df6b280a0"
UNSAFE_HANDOFFS = {
    "1ab6cd3dcddb7acaacb9dbfc16159f36f19363a8",
    "709474c68b83d259ba8657038340577ed4e8c6e4",
}


class LaunchCoreHandoffContract(unittest.TestCase):
    def test_bootloader_uses_stack_safe_application_handoff(self):
        cmake = (ROOT / "cmake/launchcore_stm32.cmake").read_text()
        self.assertIn(f"GIT_TAG {FIXED_LAUNCHCORE}", cmake)
        for unsafe_revision in UNSAFE_HANDOFFS:
            self.assertNotIn(unsafe_revision, cmake)

    def test_memory_report_uses_bsp_ram_capacity(self):
        cmake = (ROOT / "cmake/launchcore_stm32.cmake").read_text()
        self.assertIn("LAUNCHCORE_INTERNAL_SRAM_SIZE _launchcore_total_ram", cmake)
        self.assertIn('RAM_CAPACITY "${_launchcore_total_ram}"', cmake)

    def test_persistent_store_uses_u5_flash_write_granularity(self):
        storage = (ROOT / "Bootloader/storage_internal_flash.c").read_text()
        self.assertIn(".persistent_data_write_size=16u", storage)


if __name__ == "__main__":
    unittest.main()
