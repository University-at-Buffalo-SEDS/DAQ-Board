#ifndef SD_FLOAT_FORMAT_H
#define SD_FLOAT_FORMAT_H

#include <stdint.h>
#include <string.h>

/* newlib-nano does not support %llu. Convert without varargs so timestamps
 * remain correct and subsequent CSV arguments cannot be misinterpreted. */
static inline void sd_format_u64(char output[21], uint64_t value)
{
  char reverse[20];
  unsigned count = 0;
  do {
    reverse[count++] = (char)('0' + value % 10U);
    value /= 10U;
  } while (value != 0U);
  unsigned i = 0;
  while (count != 0U) output[i++] = reverse[--count];
  output[i] = '\0';
}

/* Nine significant decimal digits preserve binary32 samples on CSV replay.
 * Avoid newlib's general-purpose double dtoa at the raw ADC sample rate.
 * Normalization uses binary64, so rounding error stays well below binary32
 * precision, including subnormals. The destination must hold 24 bytes. */
static inline void sd_format_float(char output[24], float value)
{
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  const char *sign = (bits >> 31U) ? "-" : "";
  const uint32_t magnitude = bits & 0x7fffffffU;
  if (magnitude >= 0x7f800000U)
  {
    if (*sign) *output++ = '-';
    strcpy(output, magnitude == 0x7f800000U ? "inf" : "nan");
    return;
  }
  if (magnitude == 0U)
  {
    if (*sign) *output++ = '-';
    strcpy(output, "0");
    return;
  }
  double normalized = value;
  if (normalized < 0.0) normalized = -normalized;
  int exponent = 0;
  while (normalized >= 10.0) { normalized /= 10.0; ++exponent; }
  while (normalized < 1.0) { normalized *= 10.0; --exponent; }
  uint32_t digits = (uint32_t)(normalized * 100000000.0 + 0.5);
  if (digits == 1000000000U) { digits = 100000000U; ++exponent; }
  if (*sign) *output++ = '-';
  *output++ = (char)('0' + digits / 100000000U);
  *output++ = '.';
  digits %= 100000000U;
  uint32_t divisor = 10000000U;
  while (divisor != 0U)
  {
    *output++ = (char)('0' + digits / divisor);
    digits %= divisor;
    divisor /= 10U;
  }
  *output++ = 'e';
  *output++ = exponent < 0 ? '-' : '+';
  if (exponent < 0) exponent = -exponent;
  *output++ = (char)('0' + exponent / 10);
  *output++ = (char)('0' + exponent % 10);
  *output = '\0';
}

#endif
