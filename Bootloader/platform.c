#include "board_config.h"
#include "launchcore/platform.h"
#include "launchcore/storage.h"
#include "stm32u5xx.h"
extern const launchcore_storage_driver_t launchcore_board_storage_driver;
void platform_early_init(void) { launchcore_storage_set_driver(&launchcore_board_storage_driver); }
void platform_clock_init(void) {}
void platform_external_ram_init(void) {}
void platform_external_flash_init(void) {}
void platform_deinit_before_jump(void) {}
bool platform_recovery_requested(void) { return false; }
void platform_system_reset(void) { NVIC_SystemReset(); for (;;) {} }
uint32_t platform_get_reset_reason(void) { return RCC->CSR; }
void platform_feed_watchdog(void) {}
bool platform_validate_app_vector(uint32_t vector, uint32_t sp, uint32_t reset) { return vector == BOARD_VECTOR_TABLE && sp >= LAUNCHCORE_INTERNAL_SRAM_BASE && sp <= LAUNCHCORE_INTERNAL_SRAM_BASE + LAUNCHCORE_INTERNAL_SRAM_SIZE && reset >= BOARD_VECTOR_TABLE && reset < BOARD_SLOT_A_BASE + BOARD_SLOT_A_SIZE && (reset & 1u); }
uint32_t HAL_GetTick(void) { static uint32_t tick; return tick++; }
