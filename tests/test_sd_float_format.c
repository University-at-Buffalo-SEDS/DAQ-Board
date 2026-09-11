#include "sd_float_format.h"
#include <assert.h>
#include <stdlib.h>

static void check(uint32_t bits)
{
  float value;
  memcpy(&value, &bits, sizeof(value));
  char text[24];
  sd_format_float(text, value);
  char *end;
  float restored = strtof(text, &end);
  uint32_t result;
  memcpy(&result, &restored, sizeof(result));
  assert(*end == '\0');
  if ((bits & 0x7fffffffU) > 0x7f800000U)
    assert((result & 0x7fffffffU) > 0x7f800000U);
  else
    assert(bits == result);
}

int main(void)
{
  const uint64_t timestamps[] = {0, 1, 9, 10, UINT32_MAX, 1789136975096ULL, UINT64_MAX};
  for (size_t i = 0; i < sizeof(timestamps) / sizeof(timestamps[0]); ++i) {
    char text[21], *end;
    sd_format_u64(text, timestamps[i]);
    assert(strtoull(text, &end, 10) == timestamps[i]);
    assert(*end == '\0');
  }
  const uint32_t edges[] = {0, 0x80000000, 1, 0x80000001, 0x007fffff,
    0x00800000, 0x7f7fffff, 0xff7fffff, 0x7f800000, 0xff800000, 0x7fc00000,
    0x3f800000, 0x41200000, 0x3dcccccd};
  for (size_t i = 0; i < sizeof(edges) / sizeof(edges[0]); ++i) check(edges[i]);
  uint32_t state = 42U;
  for (uint32_t i = 0; i < 1000000U; ++i)
  {
    state = state * 1664525U + 1013904223U;
    check(state);
  }
  return 0;
}
