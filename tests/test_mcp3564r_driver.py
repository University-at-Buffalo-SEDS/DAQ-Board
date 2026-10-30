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
void test_memory_barrier(void);
#define __DMB() test_memory_barrier()
void test_clear_exti(uint16_t);
#define __HAL_GPIO_EXTI_CLEAR_FALLING_IT(p) test_clear_exti(p)
void HAL_GPIO_WritePin(void *, uint16_t, unsigned);
unsigned HAL_GPIO_ReadPin(void *, uint16_t);
void HAL_Delay(uint32_t);
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *);
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
#include "daq_rates.h"

DCACHE_HandleTypeDef hdcache1;
TIM_HandleTypeDef htim2 = {.Instance = TIM2};
static SPI_HandleTypeDef spi = {.hdmatx = (void *)1, .hdmarx = (void *)1};
/* Byte-wide CONFIG0..MUX, then 24-bit SCAN..GAINCAL. */
static uint32_t registers[16];
static unsigned selected, command, reg, byte_index, powered, timer_running;
static unsigned ready, dma_pending, dma_calls, start_commands, fail_next_dma;
static uint32_t tick, wire_word;
static unsigned fail_command;
static unsigned in_isr, abort_fail, abort_calls, status_override, clear_exti_calls, inject_config_edge;
static unsigned force_power_low, power_writes;
static unsigned memory_barriers, launch_barrier, cache_calls, pending_rx_valid, command_published;
static uint8_t pending_rx[5], *pending_rx_target;
extern volatile uint32_t g_mcp3564r_init_status;

void HAL_GPIO_WritePin(void *port, uint16_t pin, unsigned value) {
  (void)pin;
  if (port == EN_12V_GPIO_Port) { assert(pin==EN_12V_Pin); powered = value; power_writes++; return; }
  assert(port == ADC_NCS_GPIO_Port);
  if (value == GPIO_PIN_RESET) {
    assert(!selected);
    selected = 1; command = 0; byte_index = 0;
  } else {
    selected = 0;
  }
}
unsigned HAL_GPIO_ReadPin(void *port, uint16_t pin) {
  if (port==ADC_IRQ_GPIO_Port) { assert(pin==ADC_IRQ_Pin); return ready ? GPIO_PIN_RESET : GPIO_PIN_SET; }
  assert(port==EN_12V_GPIO_Port && pin==EN_12V_Pin && tick>=10);
  return force_power_low ? GPIO_PIN_RESET : powered;
}
void HAL_Delay(uint32_t ms) { tick += ms; }
uint32_t HAL_GetTick(void) { return tick; }

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *s, uint8_t *tx,
    uint8_t *rx, uint16_t len, uint32_t timeout) {
  (void)timeout;
  assert(!in_isr && s == &spi && powered && selected);
  if (!command && inject_config_edge && (tx[0]&3U)==2U) {
    inject_config_edge=0;ready=1;const unsigned before=dma_calls;
    in_isr=1;mcp3564r_data_ready_callback(ADC_IRQ_Pin);in_isr=0;
    assert(dma_calls==before); /* Software disarmed before first config byte. */
  }
  if (!command && fail_command && tx[0] == fail_command) { fail_command=0; return HAL_ERROR; }
  for (unsigned i = 0; i < len; ++i) {
    rx[i] = 0;
    if (!command) {
      command = tx[i];
      assert((command >> 6) == 1); /* Device address 01. */
      rx[i] = ready ? 0x13 : 0x17; /* Address01, complemented ACK bit0, STATUS data-ready bit2. */
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
    } else if (command == 0x41) {
      assert(byte_index<4); rx[i]=(uint8_t)(wire_word>>(24-8*byte_index)); byte_index++;
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
  assert(s == &spi && selected && command == 0 && !dma_pending);
  assert(len == 5 && tx[0] == 0x41);
  assert(status_override || ((registers[4] >> 4) & 3) == 3);
  assert(command_published && memory_barriers>launch_barrier);
  assert((((uintptr_t)tx | (uintptr_t)rx) & 31U)==0);
  command_published=0;launch_barrier=memory_barriers;
  dma_calls++;
  if (fail_next_dma) { fail_next_dma = 0; return HAL_ERROR; }
  pending_rx_target=rx;pending_rx_valid=1;
  memset(rx,0,5); /* Stale until the completion memory-order handoff. */
  pending_rx[0] = status_override ? (uint8_t)(status_override-1) : ready ? 0x13 : 0x17;
  ready=0; command=0x41;
  for (unsigned i = 1; i < len; ++i) {
    assert(tx[i] == 0);
    pending_rx[i] = (uint8_t)(wire_word >> (32 - 8 * i));
  }
  dma_pending = 1;
  return HAL_OK;
}
void test_memory_barrier(void) {
  memory_barriers++;
  if(pending_rx_valid && !dma_pending) {
    memcpy(pending_rx_target,pending_rx,5);pending_rx_valid=0;
  } else {
    command_published=1;
  }
}
void test_clear_exti(uint16_t pin) { assert(pin==ADC_IRQ_Pin); clear_exti_calls++; }
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *s) {
  assert(!in_isr && s==&spi);abort_calls++;
  if(abort_fail){abort_fail=0;return HAL_ERROR;}
  dma_pending=0;pending_rx_valid=0;return HAL_OK;
}
HAL_StatusTypeDef HAL_DCACHE_CleanByAddr(DCACHE_HandleTypeDef *c,
    const uint32_t *p, uint32_t n) { assert(!in_isr && "INTENDED_NO_POLLING_DCACHE_IN_ADC_ISR");
  cache_calls++;(void)c;(void)p;(void)n;return HAL_OK; }
HAL_StatusTypeDef HAL_DCACHE_InvalidateByAddr(DCACHE_HandleTypeDef *c,
    const uint32_t *p, uint32_t n) { assert(!in_isr && "INTENDED_NO_POLLING_DCACHE_IN_ADC_ISR");
  cache_calls++;(void)c;(void)p;(void)n;return HAL_OK; }
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
  assert(!timer_running);
  tick++;
  in_isr=1;mcp3564r_data_ready_callback(ADC_IRQ_Pin);in_isr=0;
  if (dma_pending) {
    assert(selected);
    dma_pending = 0;
    in_isr=1;HAL_SPI_TxRxCpltCallback(&spi);in_isr=0;
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
  assert(registers[7] == 3);    /* Fast load-cell scan; temperature is separate. */
  assert(registers[5] == 6); /* Driven-high IRQ, fast commands, no startup pulse. */
  assert(registers[8] == 0);    /* No extra inter-scan timer delay. */
  assert(start_commands == 1 && !timer_running);
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
  tick += 100;
  assert(mcp3564r_start_dma() == HAL_OK);
  receive(12, 305656); /* Approximately 25 C at the temperature clock. */
  assert(mcp3564r_start_dma() == HAL_OK);
  receive(0, 790001);
  mcp3564r_sample_t warm = take(0, 790001);
  assert(warm.temperature_code == 305656);
  assert(warm.temperature_c > 24.9f && warm.temperature_c < 25.1f);
  tick += 2001;
  receive(0, 790001);
  assert(isnan(take(0, 790001).temperature_c));
  assert(mcp3564r_start_dma() == HAL_OK);
  receive(12, 12345); /* Implausible temperature invalidates it. */
  assert(mcp3564r_start_dma() == HAL_OK);
  receive(1, 790001);
  assert(isnan(take(1, 790001).temperature_c));
  assert(!mcp3564r_pending_samples());
  fail_next_dma = 1;
  receive(1, 2297152);
  assert(selected && !timer_running && !mcp3564r_pending_samples()); /* Failed launch remains owned until thread abort. */
  assert(mcp3564r_start_dma() == HAL_OK);
  if (registers[7] == 0xfc) {
    for (int ch=7; ch>=2; --ch) { receive(ch, 100000); (void)take(ch, 100000); }
    assert(mcp3564r_start_dma()==HAL_OK && registers[7]==3);
  }
  receive(1, 2397152);
  (void)take(1, 2397152);
  /* Auxiliary scan can be due after recovery. Complete it if selected. */
  if (registers[7] == 0xfc) {
    for (int ch=7; ch>=2; --ch) { receive(ch, 100000); (void)take(ch, 100000); }
    assert(mcp3564r_start_dma()==HAL_OK && registers[7]==3);
  }
}

static void test_temperature_clock_switch(void) {
  /* Unexpected TEMP tagged data at 16 MHz must not become a temperature. */
  receive(12, 245000);
  receive(0, 790001);
  assert(isnan(take(0, 790001).temperature_c));
  for (unsigned i=0; i<10; ++i) {
    tick += 100;
    assert(mcp3564r_start_dma() == HAL_OK);
    assert(registers[2] == 0x8c && registers[7] == 0x1000);
    assert(registers[3] == 0xcf && !timer_running);
    receive(12, 305656);
    assert(!timer_running && !mcp3564r_pending_samples());
    assert(mcp3564r_start_dma() == HAL_OK);
    assert(registers[2] == 0x14 && registers[7] == 3 && registers[3] == 0xcf);
    receive(1, 2097152);
    mcp3564r_sample_t sample=take(1,2097152);
    assert(sample.temperature_c>24.9f && sample.temperature_c<25.1f);
    assert(sample.voltage_v>0.599f && sample.voltage_v<0.601f);
  }
  tick+=100; fail_command=0x4a;
  assert(mcp3564r_start_dma()==HAL_ERROR && !selected && !timer_running);
  assert(mcp3564r_start_dma()==HAL_OK && registers[2]==0x8c && registers[7]==0x1000);
  receive(12,305656);
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==3);
  /* A missing temperature conversion cannot stop load-cell acquisition. */
  tick+=100;
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==0x1000);
  tick+=50;
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==3);
  receive(0,790001); (void)take(0,790001);
  /* No interleaving SPI writes while a DMA frame is outstanding. */
  tick+=100; wire_word=790001; ready=1;
  mcp3564r_data_ready_callback(ADC_IRQ_Pin);
  assert(dma_pending);
  assert(mcp3564r_start_dma()==HAL_BUSY && registers[7]==3);
  dma_pending=0; HAL_SPI_TxRxCpltCallback(&spi);
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==0x1000);
}
static void test_auxiliary_pass(void) {
  tick += 100;
  /* Temperature has priority, then auxiliary gets a bounded turn. */
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==0x1000);
  receive(12,305656);
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==3);
  receive(0,790001); (void)take(0,790001);
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==0xfc && registers[2]==0x14);
  receive(1,123); assert(!mcp3564r_pending_samples());
  for (int ch=7; ch>=2; --ch) {
    receive(ch,2097152);
    mcp3564r_sample_t sample=take(ch,2097152);
    assert(sample.raw_value==sample.voltage_v && sample.raw_value==0.6f);
    const float connector=mcp3564r_connector_voltage(ch,sample.voltage_v);
    assert(fabsf(connector-(ch>=6 ? 6.6f : 0.6f))<0.00001f);
  }
  assert(!timer_running);
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==3);
  receive(1,2097152); assert(take(1,2097152).raw_value!=0.6f);
  /* Incomplete aux scan times out and returns to load cells. */
  tick+=100;
  assert(mcp3564r_start_dma()==HAL_OK);receive(12,305656);
  assert(mcp3564r_start_dma()==HAL_OK);
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==0xfc);
  tick+=25;
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==3);
  receive(0,790001);(void)take(0,790001);
}
static void test_p6_fast_selection(void) {
  mcp3564r_request_kg50_input(1);
  assert(mcp3564r_kg50_input()==0); /* callback performs no SPI I/O */
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==1 && registers[3]==0xcf);
  assert(mcp3564r_kg50_input()==1 && registers[2]==0x14);
  receive(9,123); assert(!mcp3564r_pending_samples());
  receive(0,790001); assert(take(0,790001).raw_value!=0.0f);
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==0x200 && registers[3]==0xef);
  /* First poll after each gain change allows full digital-filter settling. */
  assert(!timer_running); /* Hardware DRDY follows the first settled conversion. */
  receive(0,123); assert(!mcp3564r_pending_samples());
  receive(9,-2097152);
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==1 && registers[3]==0xcf);
  /* A queued x16 sample is scaled by its own channel, not today's gain. */
  assert(take(9,-2097152).raw_value==-0.0375f);
  receive(1,123); assert(!mcp3564r_pending_samples());
  receive(0,790001); (void)take(0,790001);
  fail_command=0x4e; /* Fail the CONFIG2 write; retry the intended P6 phase. */
  assert(mcp3564r_start_dma()==HAL_ERROR && !timer_running);
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==0x200 && registers[3]==0xef);
  tick+=100;
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==0x1000 && registers[3]==0xcf);
  receive(12,305656);
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==1 && registers[3]==0xcf);
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==0xfc && registers[3]==0xcf);
  for (int ch=7;ch>=2;--ch) { receive(ch,1000); (void)take(ch,1000); }
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==1);
  receive(0,790001); (void)take(0,790001);
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==0x200 && registers[3]==0xef);
  /* A request during DMA must wait until the transfer completes. */
  ready=1; wire_word=0x90000000U|1000U; mcp3564r_data_ready_callback(ADC_IRQ_Pin);
  assert(dma_pending); mcp3564r_request_kg50_input(0);
  assert(mcp3564r_start_dma()==HAL_BUSY && mcp3564r_kg50_input()==1 && registers[3]==0xef);
  dma_pending=0; HAL_SPI_TxRxCpltCallback(&spi); (void)take(9,1000);
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==3 && registers[3]==0xcf && mcp3564r_kg50_input()==0);
  receive(9,123); assert(!mcp3564r_pending_samples());
  receive(1,123); (void)take(1,123);
  mcp3564r_request_kg50_input(255); assert(mcp3564r_start_dma()==HAL_OK);
  assert(registers[7]==3);
}

static void complete_irq_dma(void) {
  assert(dma_pending);dma_pending=0;in_isr=1;HAL_SPI_TxRxCpltCallback(&spi);in_isr=0;
}
static void test_drdy_ownership_and_faults(void) {
  assert(!timer_running && registers[5]==6);
  const unsigned before=dma_calls;
  mcp3564r_data_ready_callback(1);mcp3564r_data_ready_callback(ADC_IRQ_Pin);
  assert(dma_calls==before); /* Wrong pin and high line are spurious. */
  ready=1;wire_word=790001;
  in_isr=1;mcp3564r_data_ready_callback(ADC_IRQ_Pin);in_isr=0;
  assert(dma_pending && dma_calls==before+1 && selected);
  ready=1;mcp3564r_data_ready_callback(ADC_IRQ_Pin);
  assert(dma_calls==before+1); /* Busy IRQ cannot reuse aligned DMA buffers. */
  mcp3564r_request_kg50_input(1);
  assert(mcp3564r_start_dma()==HAL_BUSY && registers[7]==3);
  complete_irq_dma();(void)take(0,790001);
  ready=0;inject_config_edge=1;
  assert(mcp3564r_start_dma()==HAL_OK && !inject_config_edge);
  assert(!dma_pending && !ready && registers[7]==1); /* Stale low drained before START. */
  mcp3564r_request_kg50_input(0);assert(mcp3564r_start_dma()==HAL_OK);
  /* An edge lost while masked is recovered by one acquisition-thread kick. */
  ready=1;wire_word=790002;
  assert(mcp3564r_start_dma()==HAL_OK && dma_pending);
  complete_irq_dma();(void)take(0,790002);
  assert(mcp3564r_start_dma()==HAL_OK && !dma_pending);
  /* Healthy not-ready STATUS is not a conversion or an error/restart. */
  status_override=0x17+1;ready=1;wire_word=790003;
  mcp3564r_data_ready_callback(ADC_IRQ_Pin);complete_irq_dma();
  assert(!mcp3564r_pending_samples());status_override=0;
  const unsigned faults[]={0x00,0xff,0x11,0x12,0x3b,0x1b};
  for(unsigned i=0;i<sizeof(faults)/sizeof(faults[0]);i++) {
    status_override=faults[i]+1;ready=1;
    mcp3564r_data_ready_callback(ADC_IRQ_Pin);
    if(faults[i]==0x12)memset(registers,0,sizeof(registers)); /* Actual POR loses format+IRQ. */
    complete_irq_dma();
    assert(!mcp3564r_pending_samples());
    const unsigned n=dma_calls;ready=1;
    mcp3564r_data_ready_callback(ADC_IRQ_Pin);assert(dma_calls==n);
    abort_fail=1;
    assert(mcp3564r_start_dma()==HAL_ERROR && selected); /* Error cannot release CS/buffers. */
    status_override=0;
    assert(mcp3564r_start_dma()==HAL_OK && !selected && !dma_pending);
    assert(registers[4]==0xf0 && registers[5]==6 && registers[7]==3);
    receive(0,790010+(int)i);(void)take(0,790010+(int)i);
  }
  /* Real asynchronous error, no immediate ISR restart or blocking abort. */
  ready=1;wire_word=790020;mcp3564r_data_ready_callback(ADC_IRQ_Pin);
  in_isr=1;mcp3564r_dma_error();in_isr=0;
  const unsigned n=dma_calls;ready=1;mcp3564r_data_ready_callback(ADC_IRQ_Pin);
  assert(dma_calls==n && dma_pending && selected);
  assert(mcp3564r_start_dma()==HAL_OK && !dma_pending && !selected);
  receive(0,790021);(void)take(0,790021);
  const unsigned timeouts=g_mcp3564r_dma_timeout_count;
  /* An acquisition-thread stall after an already completed DMA is healthy. */
  tick+=20;assert(mcp3564r_start_dma()==HAL_OK);
  assert(g_mcp3564r_dma_timeout_count==timeouts);
  /* Lost completion spanning tick wrap: buffers stay owned until abort. */
  tick=UINT32_MAX-10;ready=1;wire_word=790022;
  mcp3564r_data_ready_callback(ADC_IRQ_Pin);assert(dma_pending);
  tick+=19;assert(mcp3564r_start_dma()==HAL_BUSY);
  assert(g_mcp3564r_dma_timeout_count==timeouts);
  tick++;abort_fail=1;
  assert(mcp3564r_start_dma()==HAL_ERROR && dma_pending && selected);
  assert(g_mcp3564r_dma_timeout_count==timeouts+1);
  in_isr=1;HAL_SPI_TxRxCpltCallback(&spi);in_isr=0;
  assert(!mcp3564r_pending_samples()); /* Late callback cannot fabricate success. */
  assert(mcp3564r_start_dma()==HAL_OK && !dma_pending && !selected);
  assert(g_mcp3564r_dma_timeout_count==timeouts+1);
  receive(0,790023);(void)take(0,790023);
}

int main(int argc, char **argv) {
  assert(argc == 2);
  if (!strcmp(argv[1], "powerfault")) {
    assert(mcp3564r_init(NULL)==TX_PTR_ERROR && power_writes==0);
    force_power_low=1;
    assert(mcp3564r_init(&spi)==TX_NOT_DONE);
    assert(powered==GPIO_PIN_SET && power_writes==1 && tick==10);
    assert(g_mcp3564r_12v_enable_readback==0 && g_mcp3564r_init_status==4);
    assert(!selected && !start_commands && !timer_running);
    return 0;
  }
  assert(mcp3564r_init(&spi) == TX_SUCCESS);
  assert(powered==GPIO_PIN_SET && power_writes==1 && g_mcp3564r_12v_enable_readback==1);
  if (!strcmp(argv[1], "drdy")) test_drdy_ownership_and_faults();
  else if (!strcmp(argv[1], "config")) test_configuration();
  else if (!strcmp(argv[1], "response")) test_equal_input_response();
  else if (!strcmp(argv[1], "polling")) test_not_ready_and_recovery();
  else if (!strcmp(argv[1], "temperature")) test_temperature_clock_switch();
  else if (!strcmp(argv[1], "auxiliary")) test_auxiliary_pass();
  else if (!strcmp(argv[1], "p6")) test_p6_fast_selection();
  else assert(0);
  assert(cache_calls==0);
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
            '#define ADC_NCS_GPIO_Port ((void *)2)\n#define ADC_NCS_Pin 2\n#define ADC_IRQ_GPIO_Port ((void *)3)\n#define ADC_IRQ_Pin 1024\n')
        harness = folder / 'driver_test.c'
        harness.write_text(HARNESS)
        cls.binary = folder / 'driver_test'
        subprocess.run(['cc', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-DDAQ_CHAMBER_ADC_CHANNEL=-1',
                        '-I', str(folder), '-I', str(ROOT / 'Core/Inc'),
                        str(ROOT / 'Core/Src/mcp3564r.c'), str(harness),
                        '-o', str(cls.binary)], check=True, capture_output=True, text=True)

    def test_falling_edge_dma_ownership_status_fault_and_scan_recovery(self):
        subprocess.run([str(self.binary), "drdy"], check=True)

    def test_12v_enable_readback_failure_stops_before_spi_without_power_cycles(self):
        subprocess.run([str(self.binary), 'powerfault'], check=True)

    def test_spi_configures_both_channels_like_the_working_input(self):
        subprocess.run([str(self.binary), 'config'], check=True)

    def test_both_channels_have_identical_scaling_and_independent_response(self):
        subprocess.run([str(self.binary), 'response'], check=True)

    def test_not_ready_polls_and_failed_dma_do_not_freeze_acquisition(self):
        subprocess.run([str(self.binary), 'polling'], check=True)

    def test_temperature_uses_slow_clock_and_restores_fast_load_scan(self):
        subprocess.run([str(self.binary), "temperature"], check=True)

    def test_auxiliary_scan_preserves_channel_units_and_resumes_loadcells(self):
        subprocess.run([str(self.binary), "auxiliary"], check=True)

    def test_p6_differential_uses_fast_scan_and_switches_only_between_dma_transfers(self):
        subprocess.run([str(self.binary), "p6"], check=True)

    def test_p6_switch_waits_for_chamber_and_loadcell_in_either_conversion_order(self):
        regression = r"""
static void test_complete_scan(void) {
  mcp3564r_request_kg50_input(1);
  assert(mcp3564r_start_dma()==HAL_OK);
  const unsigned pressure=DAQ_CHAMBER_ADC_CHANNEL;
  const unsigned scan=1U|DAQ_CHAMBER_SCAN_MASK;
  assert(registers[7]==scan);
  for(unsigned pass=0;pass<8;++pass) {
    unsigned first=pass%2 ? pressure : 0U, second=pass%2 ? 0U : pressure;
    receive(first,1000);(void)take(first,1000);ready=0;
    assert(mcp3564r_start_dma()==HAL_OK);
    /* Previously the first CH0 conversion immediately switched to P6,
     * resetting the fast scan before CH6 ever produced a conversion. */
    assert(registers[7]==scan && registers[3]==0xcf);
    receive(first,1001);(void)take(first,1001);ready=0;
    assert(mcp3564r_start_dma()==HAL_OK && registers[7]==scan);
    receive(second,2000);(void)take(second,2000);ready=0;
    assert(mcp3564r_start_dma()==HAL_OK && registers[7]==0x200 && registers[3]==0xef);
    receive(9,3000);(void)take(9,3000);ready=0;
    assert(mcp3564r_start_dma()==HAL_OK && registers[7]==scan && registers[3]==0xcf);
  }
}
"""
        folder=pathlib.Path(self.tmp.name)
        for pin in (6,7):
            harness=folder/f'complete-scan-{pin}.c'
            text=HARNESS.replace('int main(int argc, char **argv) {',regression+'\nint main(int argc, char **argv) {')
            text=text.replace('if (!strcmp(argv[1], "config"))','if (!strcmp(argv[1], "complete")) test_complete_scan();\n  else if (!strcmp(argv[1], "config"))')
            harness.write_text(text)
            binary=folder/f'complete-scan-{pin}'
            subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror',f'-DDAQ_CHAMBER_ADC_CHANNEL={pin}',
                '-I',str(folder),'-I',str(ROOT/'Core/Inc'),str(ROOT/'Core/Src/mcp3564r.c'),str(harness),'-lm','-o',str(binary)],check=True,capture_output=True,text=True)
            subprocess.run([str(binary),'complete'],check=True)

    def test_configured_p4_pressure_uses_fast_unity_gain_and_not_p6_gain16(self):
        chamber_test = r"""
static void test_chamber(void) {
  assert(registers[7] == (3U | DAQ_CHAMBER_SCAN_MASK));
  assert(registers[2] == (DAQ_ADC_OSR_BITS<<2) && registers[3] == 0xcf);
  const unsigned channel = DAQ_CHAMBER_ADC_CHANNEL;
  receive(channel,1588752);
  mcp3564r_sample_t sample=take(channel,1588752);
  const float connector=mcp3564r_connector_voltage(channel,sample.voltage_v);
  assert(fabsf(connector-5.0f)<0.001f);
  assert(fabsf(daq_chamber_pressure_psi(connector)-1000.0f)<0.1f);
  receive(channel,0);assert(daq_chamber_pressure_psi(take(channel,0).voltage_v)==0);
  mcp3564r_request_kg50_input(1);
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==(1U|DAQ_CHAMBER_SCAN_MASK));
  receive(channel,1588752);(void)take(channel,1588752);ready=0;
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==(1U|DAQ_CHAMBER_SCAN_MASK));
  receive(0,790001);(void)take(0,790001);
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==0x200 && registers[3]==0xef);
  receive(channel,1588752);assert(!mcp3564r_pending_samples());
  receive(9,123);(void)take(9,123);
  assert(mcp3564r_start_dma()==HAL_OK && registers[7]==(1U|DAQ_CHAMBER_SCAN_MASK) && registers[3]==0xcf);
}
"""
        for pin in (None,6,7):
            with self.subTest(pin=pin):
                folder=pathlib.Path(self.tmp.name)
                harness=folder / f'chamber-{pin}.c'
                text=HARNESS.replace('int main(int argc, char **argv) {',chamber_test+'\nint main(int argc, char **argv) {')
                text=text.replace('if (!strcmp(argv[1], "config"))','if (!strcmp(argv[1], "chamber")) test_chamber();\n  else if (!strcmp(argv[1], "config"))')
                harness.write_text(text)
                binary=folder / f'chamber-{pin}'
                subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror',
                    *([] if pin is None else ['-DDAQ_CHAMBER_ADC_CHANNEL='+str(pin),'-DDAQ_ADC_READ_RATE_HZ=5556','-DDAQ_ADC_OSR=512']),
                    '-I',str(folder),'-I',str(ROOT/'Core/Inc'),str(ROOT/'Core/Src/mcp3564r.c'),str(harness),'-lm','-o',str(binary)],check=True,capture_output=True,text=True)
                subprocess.run([str(binary),'chamber'],check=True)
