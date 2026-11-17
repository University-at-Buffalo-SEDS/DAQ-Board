#ifndef DAQ_FX_USER_H
#define DAQ_FX_USER_H

/* DAQ log names include their network timestamp. */
#define FX_MAX_LONG_NAME_LEN 64

/* FileX uses one bit per group of FAT sectors when mirroring FAT changes.
 * Its 128-byte default mirrors about 16 sectors per changed sector on our
 * 64 GB / 32 KiB-cluster card. Use sector-granular tracking for FATs up to
 * 32768 sectors (including 64/128 GB cards with this cluster size), avoiding
 * seconds of redundant SD writes at each flush. Both FAT copies and the
 * once-per-second flush policy remain enabled. This changes FX_MEDIA layout:
 * FileX and all application callers must use this same fx_user.h. */
#define FX_FAT_MAP_SIZE 4096

/* The SD writer is the only FileX caller. Removing statistics and notification
 * hooks saves code while the normal error checking remains enabled. */
#define FX_DISABLE_DIRECT_DATA_READ_CACHE_FILL
#define FX_MEDIA_STATISTICS_DISABLE
#define FX_FILE_STATISTICS_DISABLE

/* No FileX clock timer. Its default adds 10 s every 1000 ticks, which assumes
 * 100 ticks/s; at this board's 1000 ticks/s the FAT clock ran 10x fast (the
 * 1980 unknown-time sentinel drifted, stamps ran ahead between sets). The SD
 * writer sets the clock from network time before every create, flush and close. */
#define FX_NO_TIMER

#endif /* DAQ_FX_USER_H */
