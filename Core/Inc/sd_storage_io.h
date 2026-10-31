#ifndef SD_STORAGE_IO_H
#define SD_STORAGE_IO_H
#include "tx_api.h"
#include <stdint.h>
/* Only the SD writer may call read/write. Caller retains its existing aligned
 * internal-SRAM bounce buffer until this synchronous FileX adapter returns.
 * Storage waiting is asynchronous: no polling HAL completion runs in an ISR. */
UINT sd_storage_io_init(void);
UINT sd_storage_io_read(UCHAR *buffer, ULONG sector, ULONG count);
UINT sd_storage_io_write(const UCHAR *buffer, ULONG sector, ULONG count);
void sd_storage_io_irq(void);
uint8_t sd_storage_io_requires_reinit(void);
#endif
