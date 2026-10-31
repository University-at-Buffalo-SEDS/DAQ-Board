#ifndef SD_STORAGE_CONFIG_H
#define SD_STORAGE_CONFIG_H
/* Contiguous FAT preallocation is experimental. On the qualification card,
 * scanning for a2MiB extent stalled3.7s and increased protected sample loss.
 * Ordinary FileX allocation plus compact queues is the default. */
#ifndef DAQ_SD_PREALLOCATE_ENABLE
#define DAQ_SD_PREALLOCATE_ENABLE 0
#endif
#if DAQ_SD_PREALLOCATE_ENABLE != 0 && DAQ_SD_PREALLOCATE_ENABLE != 1
#error "DAQ_SD_PREALLOCATE_ENABLE must be0or1"
#endif
#endif
