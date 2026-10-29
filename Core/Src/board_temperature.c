#include "board_temperature.h"
#include "board_temperature_math.h"
#include "main.h"
#include <stdbool.h>
#include "stm32u5xx_ll_adc.h"
static uint32_t last_attempt;
static uint32_t sample_ms;
static int16_t sample_centi;
static bool valid;
static bool attempted;
static bool due(uint32_t now)
{
  if (attempted && (uint32_t)(now - last_attempt) < 5000U) return false;
  attempted = true; last_attempt = now; return true;
}
static void publish(uint32_t raw, uint32_t ref, uint32_t now)
{
  int16_t c;
  bool ok = board_temperature_convert(raw, ref, *VREFINT_CAL_ADDR,
      *TEMPSENSOR_CAL1_ADDR, *TEMPSENSOR_CAL2_ADDR,
      TEMPSENSOR_CAL1_TEMP, TEMPSENSOR_CAL2_TEMP, &c);
  uint32_t irq = __get_PRIMASK(); __disable_irq();
  if (ok) { sample_centi = c; sample_ms = now; }
  valid = ok;
  __set_PRIMASK(irq);
}
uint32_t board_temperature_health_flags(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t irq = __get_PRIMASK(); __disable_irq();
  uint32_t flags = valid ? board_temperature_pack(sample_centi, now - sample_ms) : 0U;
  __set_PRIMASK(irq);
  return flags;
}

extern bool daq_adc1_temperature_read(uint32_t *raw, uint32_t *ref);
void board_temperature_poll(void)
{
  uint32_t now = HAL_GetTick();
  if (!due(now)) return;
  uint32_t raw = 0U, ref = 0U;
  LL_ADC_SetCommonPathInternalChAdd(ADC12_COMMON, LL_ADC_PATH_INTERNAL_TEMPSENSOR | LL_ADC_PATH_INTERNAL_VREFINT);
  volatile uint32_t delay = (SystemCoreClock / 1000000U + 1U) * LL_ADC_DELAY_TEMPSENSOR_STAB_US;
  while (delay--) __NOP();

  if (!daq_adc1_temperature_read(&raw, &ref)) { publish(0U, 0U, now); return; }
  publish(raw, ref, now);
}
