#ifndef SD_FILE_LIMIT_H
#define SD_FILE_LIMIT_H
#include <stdint.h>
/* Leave headroom below FAT32's 4 GiB file ceiling. Rotate only between complete
 * records/batches; uptime and sequence numbers continue into the next part. */
#define SD_LOG_PART_LIMIT_BYTES (UINT64_C(3) * 1024U * 1024U * 1024U)
static inline int sd_log_part_needs_rotation(uint64_t occupied, uint64_t next)
{
    return occupied >= SD_LOG_PART_LIMIT_BYTES ||
           next > SD_LOG_PART_LIMIT_BYTES - occupied;
}
#endif
