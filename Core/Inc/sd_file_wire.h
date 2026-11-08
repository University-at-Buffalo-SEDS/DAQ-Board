#ifndef SD_FILE_WIRE_H
#define SD_FILE_WIRE_H
/* SD file download protocol v1 on the SEDSnet P2P stream port 4511.
 * GroundStation opens the stream; each request gets one bounded answer:
 *   LIST   [0x21][dir u8][index u16]            -> one regular file entry
 *   READ   [0x22][offset u32][dir u8][len u8][name] -> <=512 bytes in DATA frames
 *   CANCEL [0x23]                               -> drops any pending answer
 * Responses carry opcode|0x80 and a status byte. Every frame stays within the
 * same 117-byte bound already used by the OTA receiver. Little-endian only. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define SD_FILE_STREAM_PORT 4511U
#define SD_FILE_OP_LIST 0x21U
#define SD_FILE_OP_READ 0x22U
#define SD_FILE_OP_CANCEL 0x23U
#define SD_FILE_RESPONSE_FLAG 0x80U

#define SD_FILE_STATUS_OK 0U
#define SD_FILE_STATUS_BAD_MESSAGE 1U
#define SD_FILE_STATUS_BAD_STATE 2U
#define SD_FILE_STATUS_BUSY 3U
#define SD_FILE_STATUS_NOT_FOUND 4U
#define SD_FILE_STATUS_IO_ERROR 5U
#define SD_FILE_STATUS_END 6U
#define SD_FILE_STATUS_BAD_OFFSET 7U

#define SD_FILE_DIR_LAUNCH 0U
#define SD_FILE_DIR_UTC 1U
#define SD_FILE_DIR_NO_UTC 2U
#define SD_FILE_DIR_COUNT 3U

#define SD_FILE_NAME_MAX 63U
#define SD_FILE_FRAME_MAX 117U
/* One SD sector per READ keeps the board buffer bounded and lets FileX read
 * whole sectors directly instead of cycling the writer's small media cache. */
#define SD_FILE_BLOCK_BYTES 512U
#define SD_FILE_DATA_HEADER 11U
#define SD_FILE_DATA_MAX 100U
#define SD_FILE_LIST_HEADER 12U

#define SD_FILE_ENTRY_OPEN 0x01U   /* still open for writing; size may grow */
#define SD_FILE_DATA_LAST 0x01U    /* last frame for this READ */

_Static_assert(SD_FILE_DATA_HEADER + SD_FILE_DATA_MAX <= SD_FILE_FRAME_MAX, "DATA frame bound");
_Static_assert(SD_FILE_LIST_HEADER + SD_FILE_NAME_MAX <= SD_FILE_FRAME_MAX, "LIST frame bound");
_Static_assert(7U + SD_FILE_NAME_MAX <= SD_FILE_FRAME_MAX, "READ request bound");

typedef struct {
    uint8_t op;
    uint8_t dir;
    uint8_t name_len;
    uint16_t index;
    uint32_t offset;
    char name[SD_FILE_NAME_MAX + 1U];
} sd_file_request_t;

static inline void sd_file_put_u16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static inline void sd_file_put_u32(uint8_t *p, uint32_t v)
{ p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24); }
static inline uint16_t sd_file_get_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static inline uint32_t sd_file_get_u32(const uint8_t *p)
{ return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24); }

/* Downloads are permitted only in Idle (1), Landed (13) and Recovery (14).
 * Startup, every Fill phase, Armed, Launch, flight and Aborted are refused. */
static inline bool sd_file_state_allowed(bool has_value, uint8_t flight_state)
{
    return has_value && (flight_state == 1U || flight_state == 13U || flight_state == 14U);
}

static inline const char *sd_file_directory(uint8_t dir)
{
    switch (dir) {
    case SD_FILE_DIR_LAUNCH: return "/LAUNCH";
    case SD_FILE_DIR_UTC: return "/UTC";
    case SD_FILE_DIR_NO_UTC: return "/NO_UTC";
    default: return NULL;
    }
}

/* Plain short leaf names only: no separators, no dot-only names. */
static inline bool sd_file_name_valid(const char *name, size_t len)
{
    if (!name || len == 0U || len > SD_FILE_NAME_MAX) return false;
    bool other = false;
    for (size_t i = 0; i < len; ++i) {
        const char c = name[i];
        const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                        (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
        if (!ok) return false;
        if (c != '.') other = true;
    }
    return other;
}

/* Returns a status; on OK the request is fully validated. */
static inline uint8_t sd_file_parse_request(const uint8_t *p, size_t len, sd_file_request_t *out)
{
    if (!p || !out || len == 0U || len > SD_FILE_FRAME_MAX) return SD_FILE_STATUS_BAD_MESSAGE;
    sd_file_request_t r;
    memset(&r, 0, sizeof r);
    r.op = p[0];
    if (r.op == SD_FILE_OP_CANCEL) {
        if (len != 1U) return SD_FILE_STATUS_BAD_MESSAGE;
    } else if (r.op == SD_FILE_OP_LIST) {
        if (len != 4U) return SD_FILE_STATUS_BAD_MESSAGE;
        r.dir = p[1];
        r.index = sd_file_get_u16(p + 2);
        if (!sd_file_directory(r.dir)) return SD_FILE_STATUS_BAD_MESSAGE;
    } else if (r.op == SD_FILE_OP_READ) {
        if (len < 8U) return SD_FILE_STATUS_BAD_MESSAGE;
        r.offset = sd_file_get_u32(p + 1);
        r.dir = p[5];
        r.name_len = p[6];
        if (!sd_file_directory(r.dir) || (size_t)r.name_len + 7U != len ||
            !sd_file_name_valid((const char *)(p + 7), r.name_len)) return SD_FILE_STATUS_BAD_MESSAGE;
        memcpy(r.name, p + 7, r.name_len);
        r.name[r.name_len] = '\0';
    } else {
        return SD_FILE_STATUS_BAD_MESSAGE;
    }
    *out = r;
    return SD_FILE_STATUS_OK;
}

/* Builds "/DIR/name" into dst; returns length or 0. */
static inline size_t sd_file_path(char *dst, size_t size, uint8_t dir, const char *name, size_t len)
{
    const char *d = sd_file_directory(dir);
    if (!dst || !d || !sd_file_name_valid(name, len)) return 0U;
    const size_t dl = strlen(d);
    if (dl + 1U + len + 1U > size) return 0U;
    memcpy(dst, d, dl); dst[dl] = '/';
    memcpy(dst + dl + 1U, name, len); dst[dl + 1U + len] = '\0';
    return dl + 1U + len;
}

static inline size_t sd_file_encode_status(uint8_t *dst, size_t size, uint8_t op, uint8_t status)
{
    if (!dst || size < 2U) return 0U;
    dst[0] = (uint8_t)(op | SD_FILE_RESPONSE_FLAG); dst[1] = status;
    return 2U;
}

static inline size_t sd_file_encode_list(uint8_t *dst, size_t size, uint8_t status, uint8_t dir,
                                         uint16_t index, uint32_t file_size, uint8_t flags,
                                         const char *name, size_t name_len)
{
    if (!dst || name_len > SD_FILE_NAME_MAX || size < SD_FILE_LIST_HEADER + name_len) return 0U;
    if (status != SD_FILE_STATUS_OK) name_len = 0U;
    dst[0] = SD_FILE_OP_LIST | SD_FILE_RESPONSE_FLAG; dst[1] = status; dst[2] = dir;
    sd_file_put_u16(dst + 3, index); sd_file_put_u32(dst + 5, status == SD_FILE_STATUS_OK ? file_size : 0U);
    dst[9] = flags; dst[10] = (uint8_t)name_len; dst[11] = 0U;
    if (name_len) memcpy(dst + SD_FILE_LIST_HEADER, name, name_len);
    return SD_FILE_LIST_HEADER + name_len;
}

static inline size_t sd_file_encode_data(uint8_t *dst, size_t size, uint8_t status, uint8_t flags,
                                         uint32_t offset, uint32_t file_size,
                                         const uint8_t *data, size_t len)
{
    if (!dst || len > SD_FILE_DATA_MAX || (len && !data) || size < SD_FILE_DATA_HEADER + len) return 0U;
    dst[0] = SD_FILE_OP_READ | SD_FILE_RESPONSE_FLAG; dst[1] = status; dst[2] = flags;
    sd_file_put_u32(dst + 3, offset); sd_file_put_u32(dst + 7, file_size);
    if (len) memcpy(dst + SD_FILE_DATA_HEADER, data, len);
    return SD_FILE_DATA_HEADER + len;
}
#endif
