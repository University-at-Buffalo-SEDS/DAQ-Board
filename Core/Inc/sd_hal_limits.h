#ifndef DAQ_SD_HAL_LIMITS_H
#define DAQ_SD_HAL_LIMITS_H

/* Read the vendor definitions first, then override the effectively infinite
 * status-read timeout. Applied by the board CMake, including after CubeMX
 * regeneration. This is milliseconds, not ThreadX's 20 us ticks. */
#include "stm32u5xx_hal.h"
#undef SDMMC_SWDATATIMEOUT
#define SDMMC_SWDATATIMEOUT 2000U

#endif
