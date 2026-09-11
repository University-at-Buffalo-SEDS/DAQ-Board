#ifndef DAQ_FX_USER_H
#define DAQ_FX_USER_H

/* DAQ log names include their network timestamp. */
#define FX_MAX_LONG_NAME_LEN 64

/* The SD writer is the only FileX caller. Removing statistics and notification
 * hooks saves code while the normal error checking remains enabled. */
#define FX_DISABLE_DIRECT_DATA_READ_CACHE_FILL
#define FX_MEDIA_STATISTICS_DISABLE
#define FX_FILE_STATISTICS_DISABLE

#endif /* DAQ_FX_USER_H */
