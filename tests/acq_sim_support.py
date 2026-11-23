"""Build and run tests/acq_engine_sim.c: the production MCP3564R frame engine
(Core/Src/mcp3564r.c) on a modelled TIM2, SPI DMA, DRDY pin and ADC."""
import atexit
import functools
import pathlib
import shutil
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]

HAL = r'''
#pragma once
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef struct { int unused; } GPIO_TypeDef;
typedef struct { int unused; } DMA_HandleTypeDef;
typedef struct { DMA_HandleTypeDef *hdmatx, *hdmarx; } SPI_HandleTypeDef;
typedef struct { volatile uint32_t CR1, DIER, SR, EGR, CCMR1, CCER, CNT, PSC, ARR, CCR1, CCR2; } TIM_TypeDef;
extern TIM_TypeDef g_tim2, g_tim6;
#define TIM2 (&g_tim2)
#define TIM6 (&g_tim6)
#define TIM_SR_UIF 1U
#define TIM_SR_CC1IF 2U
#define TIM_SR_CC2IF 4U
#define TIM_DIER_CC1IE 2U
#define TIM_DIER_CC2IE 4U
#define TIM_EGR_UG 1U
#define TIM_EGR_CC2G 4U
#define TIM_CR1_CEN 1U
/* rc_w0 status flags: writing 0 clears, 1 keeps. */
#define ACQ_TIM_CLEAR(flags) (g_tim2.SR &= ~(uint32_t)(flags))
extern uint32_t g_primask;
static inline uint32_t __get_PRIMASK(void) { return g_primask; }
static inline void __disable_irq(void) { g_primask = 1U; }
static inline void __enable_irq(void) { g_primask = 0U; }
static inline void __set_PRIMASK(uint32_t v) { g_primask = v; }
#define __DMB() do {} while (0)
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
uint32_t HAL_RCC_GetPCLK1Freq(void);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *h, uint8_t *tx, uint8_t *rx, uint16_t len);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *h);
'''
MAIN = r'''
#pragma once
#include "stm32u5xx_hal.h"
#define ADC_IRQ_Pin 1024U
#define ADC_IRQ_GPIO_Port ((GPIO_TypeDef *)0)
#define ADC_NCS_Pin 2U
#define ADC_NCS_GPIO_Port ((GPIO_TypeDef *)0)
#define EN_12V_Pin 4U
#define EN_12V_GPIO_Port ((GPIO_TypeDef *)0)
'''
TX = r'''
#pragma once
typedef unsigned int UINT;
#define TX_SUCCESS 0U
#define TX_PTR_ERROR 3U
#define TX_NOT_DONE 0x20U
'''


@functools.lru_cache(maxsize=None)
def build(defines=()):
    folder = pathlib.Path(tempfile.mkdtemp(prefix='acq-sim-'))
    atexit.register(shutil.rmtree, folder, True)
    (folder / 'stm32u5xx_hal.h').write_text(HAL)
    (folder / 'main.h').write_text(MAIN)
    (folder / 'tx_api.h').write_text(TX)
    binary = folder / 'acq_engine_sim'
    result = subprocess.run(['cc', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                    *('-D' + d for d in defines), '-I', str(folder), '-I', str(ROOT / 'Core/Inc'),
                    str(ROOT / 'Core/Src/mcp3564r.c'), str(ROOT / 'tests/acq_engine_sim.c'),
                    '-lm', '-o', str(binary)], capture_output=True, text=True)
    if not binary.exists():
        raise AssertionError('acq_engine_sim build failed:\n' + result.stderr)
    return binary


def run(seconds=2.0, slow_ppm=0, drop_every=0, launch=(0, 0), kg50=1, waveform='', defines=()):
    binary = build(tuple(defines))
    args = [str(binary), str(seconds), str(slow_ppm), str(drop_every), str(launch[0]), str(launch[1]), str(kg50)]
    if waveform:
        args.append(waveform)
    out = subprocess.run(args, check=True, capture_output=True, text=True).stdout
    samples, diag, configs = [], {}, []
    for line in out.splitlines():
        parts = line.split()
        if parts[0] == 'S':
            channel, t_us, frame, profile, slot, code = map(int, parts[1:])
            samples.append((channel, t_us, frame, profile, slot, code))
        elif parts[0] == 'D':
            diag[parts[1]] = int(parts[2])
        elif parts[0] == 'C':
            configs.append(tuple(map(int, parts[1:])))
    if configs:
        diag['configs'] = configs
    return samples, diag
