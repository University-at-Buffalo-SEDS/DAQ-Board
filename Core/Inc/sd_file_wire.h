#ifndef SD_FILE_WIRE_H
#define SD_FILE_WIRE_H
/* SD file transfer protocol v2 on the SEDSnet P2P stream port 4511, the same
 * wire as the FC (GroundStation26 docs/backend/sd-transfer.md):
 *   LIST   [0x21][dir u8][index u16]          -> one regular file entry
 *   CANCEL [0x23]                             -> ends the transfer or hash
 *   OPEN   [0x25][xfer][dir]...[name]         -> streamed DATA (sd_xfer.h)
 *   ACK    [0x26][xfer][una][bitmap][rate][window]
 *   HASH   [0x27][dir u8][len u8][name]       -> size and SHA-256 of the file
 * Answers carry opcode|0x80 and a status byte. Little-endian. DAQ specifics:
 * three directories (below), any plain leaf name. v1 READ (0x22) is gone: it
 * answers BAD_MESSAGE. */
#include "sd_xfer.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define SD_FILE_STREAM_PORT 4511U
#define SD_FILE_OP_LIST 0x21U
#define SD_FILE_OP_CANCEL 0x23U
#define SD_FILE_OP_OPEN SD_XFER_OP_OPEN
#define SD_FILE_OP_ACK SD_XFER_OP_ACK
#define SD_FILE_OP_HASH SD_XFER_OP_HASH
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
/* One SD sector: LIST names and HASH reads go through this block. */
#define SD_FILE_BLOCK_BYTES 512U
#define SD_FILE_REQUEST_MAX (19U + SD_FILE_NAME_MAX)  /* OPEN is the largest request */
#define SD_FILE_LIST_HEADER 12U
#define SD_FILE_HASH_ANSWER 39U
#define SD_FILE_ANSWER_MAX (SD_FILE_LIST_HEADER + SD_FILE_NAME_MAX)

#define SD_FILE_ENTRY_OPEN 0x01U   /* still open for writing; size may grow */

_Static_assert(SD_FILE_ANSWER_MAX >= SD_FILE_HASH_ANSWER && SD_FILE_ANSWER_MAX >= SD_XFER_OPEN_ANSWER, "answer bound");

typedef struct {
    uint8_t op;
    uint8_t dir;
    uint8_t name_len;
    uint16_t index;
    sd_xfer_open_t open;
    char name[SD_FILE_NAME_MAX + 1U];
} sd_file_request_t;

static inline void sd_file_put_u16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static inline void sd_file_put_u32(uint8_t *p, uint32_t v)
{ p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24); }
static inline uint16_t sd_file_get_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static inline uint32_t sd_file_get_u32(const uint8_t *p)
{ return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24); }

/* Transfers are permitted only in Idle (1) and Landed (13). A retired
 * Recovery (14) is mapped to Landed on receipt; accepting it here too keeps a
 * value restored from older firmware readable. Startup, Fill, Armed, Launch,
 * InFlight and Aborted are refused. */
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

/* Returns a status; on OK the request is fully validated. ACK is parsed by
 * the stream layer (sd_xfer_parse_ack) and is not a request. */
static inline uint8_t sd_file_parse_request(const uint8_t *p, size_t len, sd_file_request_t *out)
{
    if (!p || !out || len == 0U || len > SD_FILE_REQUEST_MAX) return SD_FILE_STATUS_BAD_MESSAGE;
    sd_file_request_t r;
    memset(&r, 0, sizeof r);
    r.op = p[0];
    size_t name_at = 0, name_len = 0;
    if (r.op == SD_FILE_OP_CANCEL) {
        if (len != 1U) return SD_FILE_STATUS_BAD_MESSAGE;
    } else if (r.op == SD_FILE_OP_LIST) {
        if (len != 4U) return SD_FILE_STATUS_BAD_MESSAGE;
        r.dir = p[1];
        r.index = sd_file_get_u16(p + 2);
        if (!sd_file_directory(r.dir)) return SD_FILE_STATUS_BAD_MESSAGE;
    } else if (r.op == SD_FILE_OP_OPEN) {
        if (!sd_xfer_parse_open(p, len, &r.dir, &r.open, &name_at, &name_len)) return SD_FILE_STATUS_BAD_MESSAGE;
    } else if (r.op == SD_FILE_OP_HASH) {
        if (len < 4U || (size_t)p[2] + 3U != len) return SD_FILE_STATUS_BAD_MESSAGE;
        r.dir = p[1]; name_at = 3U; name_len = p[2];
    } else {
        return SD_FILE_STATUS_BAD_MESSAGE;
    }
    if (name_at) {
        if (!sd_file_directory(r.dir) || !sd_file_name_valid((const char *)(p + name_at), name_len))
            return SD_FILE_STATUS_BAD_MESSAGE;
        memcpy(r.name, p + name_at, name_len);
        r.name_len = (uint8_t)name_len;
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

/* LIST answer: [0xA1][status][dir][index u16][size u32][flags][name_len][0][name]. */
static inline size_t sd_file_encode_list(uint8_t *dst, uint8_t status, uint8_t dir, uint16_t index,
                                         uint32_t file_size, uint8_t flags, const char *name, size_t name_len)
{
    if (status != SD_FILE_STATUS_OK) name_len = file_size = flags = 0U;
    if (name_len > SD_FILE_NAME_MAX) return 0U;
    dst[0] = SD_FILE_OP_LIST | SD_FILE_RESPONSE_FLAG; dst[1] = status; dst[2] = dir;
    sd_file_put_u16(dst + 3, index); sd_file_put_u32(dst + 5, file_size);
    dst[9] = flags; dst[10] = (uint8_t)name_len; dst[11] = 0U;
    if (name_len) memcpy(dst + SD_FILE_LIST_HEADER, name, name_len);
    return SD_FILE_LIST_HEADER + name_len;
}

/* Status-only answer for op: LIST and OPEN keep their fixed sizes (zeros),
 * HASH its 39 bytes, anything else [op|0x80][status]. dst holds
 * SD_FILE_ANSWER_MAX bytes. */
static inline size_t sd_file_encode_status(uint8_t *dst, uint8_t op, uint8_t status)
{
    const size_t len = op == SD_FILE_OP_LIST ? SD_FILE_LIST_HEADER : op == SD_FILE_OP_OPEN ? SD_XFER_OPEN_ANSWER
                     : op == SD_FILE_OP_HASH ? SD_FILE_HASH_ANSWER : 2U;
    memset(dst, 0, len);
    dst[0] = (uint8_t)(op | SD_FILE_RESPONSE_FLAG); dst[1] = status;
    return len;
}
#endif
