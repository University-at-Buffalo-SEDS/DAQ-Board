#ifndef SD_STORAGE_IO_H
#define SD_STORAGE_IO_H
#include "tx_api.h"
#include <stdint.h>
/* Only the SD writer may call read/write. Caller retains its aligned
 * internal-SRAM buffer until this synchronous FileX adapter returns.
 * Storage waiting is asynchronous: no polling HAL completion runs in an ISR. */
UINT sd_storage_io_init(void);
UINT sd_storage_io_read(UCHAR *buffer, ULONG sector, ULONG count);
UINT sd_storage_io_write(const UCHAR *buffer, ULONG sector, ULONG count);
void sd_storage_io_irq(void);
uint8_t sd_storage_io_requires_reinit(void);
/* One IDMA command moves at most this many sectors. */
#define SD_STORAGE_IO_MAX_SECTORS 64U
/* True when IDMA may use this buffer in place: 32-byte aligned internal SRAM.
 * The caller still owns it until the synchronous read/write returns. */
static inline uint8_t sd_storage_io_dma_capable(const void *buffer, ULONG count)
{
  const uintptr_t at = (uintptr_t)buffer;
  if (!buffer || (at & 31U) || count == 0U || count > SD_STORAGE_IO_MAX_SECTORS) return 0U;
#if defined(__arm__) || defined(__thumb__)
  if (at < 0x20000000UL || at > 0x200C0000UL - (uintptr_t)count * 512U) return 0U;
#endif
  return 1U;
}
#endif
