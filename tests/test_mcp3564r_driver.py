"""Run the complete production ADC driver against a mocked SPI/HAL boundary.

The register widths, scan IDs, output format and active-low data-ready flag
follow MCP3561/2/4R DS20006391C sections 5.6, 5.15, 6.2 and 8.
"""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]

HAL = r'''
#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stddef.h>
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
typedef struct { void *hdmatx, *hdmarx; } SPI_HandleTypeDef;
typedef struct { void *Instance; uint32_t counter, reload; } TIM_HandleTypeDef;
typedef struct { int unused; } DCACHE_HandleTypeDef;
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define TIM2 ((void *)2)
#define TIM_EVENTSOURCE_UPDATE 1
#define TIM_FLAG_UPDATE 1
#define __HAL_TIM_SET_COUNTER(t, n) ((t)->counter = (n))
#define __HAL_TIM_SET_AUTORELOAD(t, n) ((t)->reload = (n))
#define __HAL_TIM_CLEAR_FLAG(t, f) ((void)(t), (void)(f))
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
void HAL_GPIO_WritePin(void *, uint16_t, unsigned);
void HAL_Delay(uint32_t);
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_DCACHE_CleanByAddr(DCACHE_HandleTypeDef *, const uint32_t *, uint32_t);
HAL_StatusTypeDef HAL_DCACHE_InvalidateByAddr(DCACHE_HandleTypeDef *, const uint32_t *, uint32_t);
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *, unsigned);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *);
#endif
'''

HARNESS = r'''
#include <assert.h>
#include <math.h>
#include <string.h>
#include "mcp3564r.h"
#include "main.h"

DCACHE_HandleTypeDef hdcache1;
TIM_HandleTypeDef htim2 = {.Instance = TIM2};
static SPI_HandleTypeDef spi = {.hdmatx = (void *)1, .hdmarx = (void *)1};
/* Byte-wide CONFIG0..MUX, then 24-bit SCAN..GAINCAL. */
static uint32_t registers[16];
static unsigned selected, command, reg, byte_index, powered, timer_running;
static unsigned ready, dma_pending, dma_calls, start_commands, fail_next_dma;
static uint32_t tick, wire_word;

void HAL_GPIO_WritePin(void *port, uint16_t pin, unsigned value) {
  (void)pin;
  if (port == EN_12V_GPIO_Port) { powered = value; return; }
  assert(port == ADC_NCS_GPIO_Port);
  if (value == GPIO_PIN_RESET) {
    assert(!selected);
    selected = 1; command = 0; byte_index = 0;
  } else {
    selected = 0;
  }
}
void HAL_Delay(uint32_t ms) { tick += ms; }
uint32_t HAL_GetTick(void) { return tick; }

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *s, uint8_t *tx,
    uint8_t *rx, uint16_t len, uint32_t timeout) {
  (void)timeout;
  assert(s == &spi && powered && selected);
  for (unsigned i = 0; i < len; ++i) {
    rx[i] = 0;
    if (!command) {
      command = tx[i];
      assert((command >> 6) == 1); /* Device address 01. */
      rx[i] = ready ? 0x13 : 0x17; /* STATUS bit 2, not IRQ register bit 6. */
      ready = 0;
      if (command == 0x78) memset(registers, 0, sizeof(registers));
      else if (command == 0x68) {
        assert(registers[5] & 2); /* Fast commands enabled. */
        registers[1] |= 3;
        start_commands++;
      } else {
        reg = (command >> 2) & 15;
        assert(command == 0x41 || (command & 3) == 2);
      }
    } else {
      assert((command & 3) == 2 && reg >= 1 && reg <= 10);
      const unsigned width = reg <= 6 ? 1 : 3;
      if (!byte_index) registers[reg] = 0;
      registers[reg] = (registers[reg] << 8) | tx[i];
      if (++byte_index == width) { reg++; byte_index = 0; }
    }
  }
  return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *s,
    uint8_t *tx, uint8_t *rx, uint16_t len) {
  assert(s == &spi && selected && command == 0x41 && !dma_pending);
  assert(len == 4 && ((registers[4] >> 4) & 3) == 3);
  dma_calls++;
  if (fail_next_dma) { fail_next_dma = 0; return HAL_ERROR; }
  for (unsigned i = 0; i < len; ++i) {
    assert(tx[i] == 0);
    rx[i] = (uint8_t)(wire_word >> (24 - 8 * i));
  }
  dma_pending = 1;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_DCACHE_CleanByAddr(DCACHE_HandleTypeDef *c,
    const uint32_t *p, uint32_t n) { (void)c; (void)p; (void)n; return HAL_OK; }
HAL_StatusTypeDef HAL_DCACHE_InvalidateByAddr(DCACHE_HandleTypeDef *c,
    const uint32_t *p, uint32_t n) { (void)c; (void)p; (void)n; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *t) {
  assert(t == &htim2); timer_running = 0; return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *t) {
  assert(t == &htim2); timer_running = 1; return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *t, unsigned event) {
  assert(t == &htim2 && event == TIM_EVENTSOURCE_UPDATE); return HAL_OK;
}

static void poll_adc(void) {
  assert(timer_running);
  tick++;
  mcp3564r_timer_elapsed_callback(&htim2);
  if (dma_pending) {
    assert(selected);
    dma_pending = 0;
    HAL_SPI_TxRxCpltCallback(&spi);
  }
}
static void receive(unsigned channel, int32_t code) {
  /* DATA_FORMAT=11 repeats the sign through bits 27:24, with CH_ID in 31:28. */
  wire_word = (channel << 28) | ((uint32_t)code & 0x0FFFFFFFU);
  ready = 1;
  poll_adc();
}
static mcp3564r_sample_t take(unsigned channel, int32_t code) {
  mcp3564r_sample_t sample;
  assert(mcp3564r_get_sample(&sample) == TX_SUCCESS);
  assert(sample.sample_valid && sample.channel == channel && sample.code == code);
  assert(sample.monotonic_ms <= tick && sample.overrun_count == 0);
  return sample;
}

static void test_configuration(void) {
  /* The working input's reference, clock, OSR, gain and auto-zero settings
   * apply to CH1 as well. No channel-specific PGA or CONFIG registers exist. */
  assert(registers[1] == 0x83); /* 0x82 plus START: internal ref, external clock. */
  assert(registers[2] == 0x14); /* MCLK/1, OSR=1024. */
  assert(registers[3] == 0xCF); /* Gain 1, boost 2, AZ_MUX/AZ_REF enabled. */
  assert(registers[4] == 0xF0); /* Continuous scan cycles, 32-bit tagged data. */
  assert(registers[7] == 0x1003);    /* TEMP, CH1-AGND and CH0-AGND. */
  assert(registers[8] == 0);    /* No extra inter-scan timer delay. */
  assert(start_commands == 1 && timer_running);
}
static void test_equal_input_response(void) {
  const int32_t codes[] = {0, 1, -1, 790001, 2097152, -2097152, 8388607,
                           -8388608, 8388608, 16777215, -16777216};
  for (unsigned i = 0; i < sizeof(codes) / sizeof(codes[0]); ++i) {
    /* SCAN priority is MSb first: CH1 then CH0. Equal ADC codes must receive
     * identical scaling regardless of the tag and sign extension. */
    receive(1, codes[i]);
    receive(0, codes[i]);
    assert(mcp3564r_pending_samples() == 2);
    const mcp3564r_sample_t ch1 = take(1, codes[i]);
    const mcp3564r_sample_t ch0 = take(0, codes[i]);
    assert(ch0.raw_value == ch1.raw_value && ch0.voltage_v == ch1.voltage_v);
    const float legacy = (((float)codes[i] * 2.2104f) / 16777216.0f) * 2 / 16;
    assert(ch1.raw_value == legacy);
  }
  /* Move only CH1 through enough frames to wrap the ring buffer indices. */
  float previous = -1;
  for (int32_t step = 0; step < 160; ++step) {
    receive(1, 2097152 + 1000 * step);
    receive(0, 790001);
    const mcp3564r_sample_t ch1 = take(1, 2097152 + 1000 * step);
    (void)take(0, 790001);
    assert(ch1.raw_value > previous);
    previous = ch1.raw_value;
  }
  assert(start_commands == 1); /* Polling must not restart the scan on CH1. */
}
static void test_not_ready_and_recovery(void) {
  receive(1, 2097152);
  (void)take(1, 2097152);
  const unsigned calls = dma_calls;
  for (unsigned i = 0; i < 8; ++i) poll_adc();
  assert(dma_calls == calls && !mcp3564r_pending_samples());
  mcp3564r_sample_t empty;
  assert(mcp3564r_get_sample(&empty) == TX_SUCCESS && !empty.sample_valid);
  receive(0, 790001);
  (void)take(0, 790001);
  receive(1, 2197152);
  (void)take(1, 2197152);
  /* A diagnostic-channel result cannot masquerade as a load-cell sample. */
  receive(12, 305656); /* Approximately 25 C. */
  receive(0, 790001);
  mcp3564r_sample_t warm = take(0, 790001);
  assert(warm.temperature_code == 305656);
  assert(warm.temperature_c > 24.9f && warm.temperature_c < 25.1f);
  tick += 2001;
  receive(0, 790001);
  assert(isnan(take(0, 790001).temperature_c));
  receive(12, 12345); /* Implausible temperature invalidates it. */
  receive(1, 790001);
  assert(isnan(take(1, 790001).temperature_c));
  assert(!mcp3564r_pending_samples());
  fail_next_dma = 1;
  receive(1, 2297152);
  assert(!selected && !timer_running && !mcp3564r_pending_samples());
  assert(mcp3564r_start_dma() == HAL_OK);
  receive(1, 2397152);
  (void)take(1, 2397152);
  assert(start_commands == 1);
}
int main(int argc, char **argv) {
  assert(argc == 2 && mcp3564r_init(&spi) == TX_SUCCESS);
  if (!strcmp(argv[1], "config")) test_configuration();
  else if (!strcmp(argv[1], "response")) test_equal_input_response();
  else if (!strcmp(argv[1], "polling")) test_not_ready_and_recovery();
  else assert(0);
}
'''


class Mcp3564rDriverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.tmp.cleanup)
        folder = pathlib.Path(cls.tmp.name)
        (folder / 'stm32u5xx_hal.h').write_text(HAL)
        (folder / 'tx_api.h').write_text(
            'typedef unsigned UINT;\n#define TX_SUCCESS 0\n'
            '#define TX_PTR_ERROR 1\n#define TX_NOT_DONE 2\n')
        (folder / 'main.h').write_text(
            '#include "stm32u5xx_hal.h"\n'
            '#define EN_12V_GPIO_Port ((void *)1)\n#define EN_12V_Pin 1\n'
            '#define ADC_NCS_GPIO_Port ((void *)2)\n#define ADC_NCS_Pin 2\n')
        harness = folder / 'driver_test.c'
        harness.write_text(HARNESS)
        cls.binary = folder / 'driver_test'
        subprocess.run(['cc', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-I', str(folder), '-I', str(ROOT / 'Core/Inc'),
                        str(ROOT / 'Core/Src/mcp3564r.c'), str(harness),
                        '-o', str(cls.binary)], check=True, capture_output=True, text=True)

    def test_spi_configures_both_channels_like_the_working_input(self):
        subprocess.run([str(self.binary), 'config'], check=True)

    def test_both_channels_have_identical_scaling_and_independent_response(self):
        subprocess.run([str(self.binary), 'response'], check=True)

    def test_not_ready_polls_and_failed_dma_do_not_freeze_acquisition(self):
        subprocess.run([str(self.binary), 'polling'], check=True)
