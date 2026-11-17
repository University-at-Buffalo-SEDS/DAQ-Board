#ifndef BOARD_STAMP_H
#define BOARD_STAMP_H
#include <stdint.h>
/* A SEDSnet network stamp (time_epoch, timestamp_ms) as one ordered 64-bit
 * key: the epoch in the top 16 bits, the Unix milliseconds in the low 48.
 * Keys order exactly like seds_stamp_cmp(), so a stored ordering mark (the
 * persisted recovery floor, commissioning history, attempted stamps) keeps its
 * 64-bit layout and plain comparisons while it carries the epoch; a record
 * written before epochs existed reads as epoch 0. 0 means no usable stamp. */
#define BOARD_STAMP_MS_LIMIT (UINT64_C(1) << 48)
static inline uint64_t board_stamp_key(uint16_t epoch, uint64_t ms)
{
  return ms == 0U || ms >= BOARD_STAMP_MS_LIMIT - 1U ? 0U : ((uint64_t)epoch << 48) | ms;
}
static inline uint16_t board_stamp_epoch(uint64_t key) { return (uint16_t)(key >> 48); }
static inline uint64_t board_stamp_ms(uint64_t key) { return key & (BOARD_STAMP_MS_LIMIT - 1U); }
#endif
