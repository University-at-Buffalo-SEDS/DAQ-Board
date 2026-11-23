#ifndef SD_XFER_H
#define SD_XFER_H
/* SD transfer v2 sender engine: blocks, DATA messages, window, selective
 * retransmission (GroundStation26 docs/backend/sd-transfer.md). Shared
 * verbatim by FlightComputer26 and DAQ-Board. Board glue owns the file, the
 * stream and the locking; this file only cuts, encodes and schedules.
 *
 * Threads: sd_xfer_produce runs where the card is read (the SD thread);
 * sd_xfer_ack runs in the router callback. Both touch the flow fields, so the
 * caller serialises them (short IRQ lock) and passes a snapshot to produce. */
#include "sd_xfer_codec.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SD_XFER_OP_OPEN 0x25U
#define SD_XFER_OP_ACK 0x26U
#define SD_XFER_OP_HASH 0x27U
#define SD_XFER_DATA 0xA6U

#define SD_XFER_OPEN_SDX 0x01U          /* OPEN flags */
#define SD_XFER_FIRST 0x01U             /* DATA flags */
#define SD_XFER_LAST 0x02U
#define SD_XFER_BODY_SDX 0x04U
#define SD_XFER_END 0x10U
#define SD_XFER_GROWING 0x20U
#define SD_XFER_ANSWER_SDX 0x01U        /* OPEN answer flags */
#define SD_XFER_ANSWER_GROWING 0x04U

#define SD_XFER_RAW_WINDOW 1024U        /* raw bytes the encoder sees ahead */
#ifndef SD_XFER_MSG_MAX
#define SD_XFER_MSG_MAX 768U
#endif
#define SD_XFER_MSG_MIN 64U
#define SD_XFER_OPEN_ANSWER 19U
#define SD_XFER_HDR 9U
#define SD_XFER_HDR_LAST 17U
#define SD_XFER_WINDOW_MAX 32U
#define SD_XFER_RING 16U
#define SD_XFER_BLOCK_MAX_KIB 64U
#define SD_XFER_RETX_GUARD_MS 1000U     /* no resend within 1 s of a send */
#define SD_XFER_STALL_MS 3000U          /* no progress: resend the oldest */
#define SD_XFER_ACK_TIMEOUT_MS 15000U   /* no ACK at all: the transfer ends */

/* Reads exactly len file bytes at offset into dst. */
typedef bool (*sd_xfer_read_fn)(void *ctx, uint32_t offset, uint8_t *dst, uint32_t len);

typedef struct {
    uint16_t una;
    uint32_t bitmap;
    uint32_t rate;                       /* DATA bytes per second, 0 = paused */
    uint8_t window;
    uint32_t progress_ms;                /* when una last advanced (or OPEN) */
} sd_xfer_flow_t;

typedef struct {
    uint8_t id;                          /* 0: no transfer */
    uint8_t sdx, growing;
    uint16_t msg;                        /* whole DATA message limit */
    uint32_t block, start, end;          /* raw block target, raw range */
    sd_xfer_flow_t flow;                 /* ACK side; caller-locked */
    uint32_t ack_ms;
    /* Producer (SD thread). */
    uint16_t frontier, cursor;           /* next new seq; next seq the cursor builds */
    uint32_t pos, blk_start, crc;
    uint8_t in_block, done;
    uint32_t raw_base;
    uint16_t raw_len;
    uint8_t ring_n;
    struct { uint32_t off; uint16_t seq; } ring[SD_XFER_RING];
    uint32_t sent_ms[SD_XFER_WINDOW_MAX];
} sd_xfer_t;

typedef struct {
    sdx_enc_t enc;
    uint8_t raw[SD_XFER_RAW_WINDOW];
    uint8_t msg[SD_XFER_MSG_MAX];
} sd_xfer_work_t;

typedef struct {
    uint8_t xfer, flags, window, block_kib;
    uint32_t offset, length, rate;
    uint16_t msg;
} sd_xfer_open_t;

/* OPEN [0x25][xfer][dir][flags][offset u32][length u32][rate u16][msg u16]
 * [window][block][name_len][name]: parses the fixed part; *name_at gets the
 * name offset. False for a malformed request. */
bool sd_xfer_parse_open(const uint8_t *p, size_t len, uint8_t *dir, sd_xfer_open_t *o,
                        size_t *name_at, size_t *name_len);
/* ACK [0x26][xfer][una u16][bitmap u32][rate u16][window]. */
bool sd_xfer_parse_ack(const uint8_t *p, size_t len, uint8_t *xfer, sd_xfer_flow_t *f);

/* Starts a transfer of [offset, offset+length) (length 0: to file_size)
 * after validation; returns a v1 status code (OK, END, BAD_OFFSET) and fills
 * the OPEN answer [0xA5][status][xfer][flags][size u32][offset u32]
 * [length u32][msg u16][block KiB]. */
uint8_t sd_xfer_begin(sd_xfer_t *x, const sd_xfer_open_t *o, uint32_t file_size, bool growing,
                      uint32_t now_ms, uint8_t ans[SD_XFER_OPEN_ANSWER]);
/* Applies an ACK (router thread, caller-locked). False if it is stale. */
bool sd_xfer_ack(sd_xfer_t *x, uint8_t xfer, const sd_xfer_flow_t *f, uint32_t now_ms);
/* Builds the next DATA message to send now, if any (SD thread). Returns its
 * length with *out set, 0 when nothing is due, or SIZE_MAX on a read error
 * (the transfer has ended). flow is a locked snapshot of x->flow. When the
 * transfer completes x->id becomes 0. */
size_t sd_xfer_produce(sd_xfer_t *x, sd_xfer_work_t *w, const sd_xfer_flow_t *flow,
                       sd_xfer_read_fn read, void *ctx, uint32_t now_ms, const uint8_t **out);
#endif
