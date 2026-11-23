#ifndef SD_XFER_CODEC_H
#define SD_XFER_CODEC_H
/* SDX: lossless binary encoding of SD CSV rows for the network transfer only
 * (the card keeps CSV). Shared by FlightComputer26 and DAQ-Board; the
 * GroundStation26 decoder (backend/src/sd_download/sdx.rs) expands it back to
 * the identical bytes. Spec: GroundStation26 docs/backend/sd-transfer.md.
 *
 * A block body is a sequence of items, all state reset at the block start:
 *   0x00 RAW    varint n, n bytes verbatim (comments, partial or odd rows)
 *   0x01 LABEL  varint n, n bytes: defines the next label id (0, 1, ...)
 *   0x02+2*id+s ROW (id < 127): a 13-field CRLF row with label id in field 2.
 *               s=1 reuses this label's previous op list; s=0 sends it:
 *               varint count, then the op nibbles two per byte, low first.
 *               The op payloads follow in op order.
 * Ops: 0 EMPTY, 1 SAME, 2 DELTA, 3 NUM, 4 TEXT, 5 QTEXT, 6 COPY k,
 *      7 QVEC c + c element ops (see the spec). Every replaced token is
 *      re-expanded and compared before use, so the output is exact. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef SDX_CTX_ENTRIES
#define SDX_CTX_ENTRIES 128U         /* token contexts per block; encoder-only choice */
#endif
#define SDX_LABELS 24U
#define SDX_LABEL_MAX 23U
#define SDX_OPS_BYTES 12U            /* op lists up to 24 nibbles are reused */
#define SDX_FIELDS 13U

typedef struct {
    uint8_t key;                     /* field << 4 | element */
    uint8_t kind;                    /* 0 empty, 1 number, 2 short text, 3 long text */
    uint8_t b0, b1, len;
    int16_t exp;
    union { int64_t v; uint8_t text[8]; } u;
} sdx_ctx_t;

typedef struct {
    uint8_t len, ops_len;            /* ops_len 0xff: no reusable list */
    uint8_t ctx_base, ctx_count;     /* this label's run of context slots */
    char name[SDX_LABEL_MAX];
    uint8_t ops[SDX_OPS_BYTES];
} sdx_label_t;

typedef struct {
    uint8_t labels;
    uint16_t ctx_used;
    sdx_label_t label[SDX_LABELS];
    sdx_ctx_t ctx[SDX_CTX_ENTRIES];
} sdx_enc_t;

/* Start a new independent block. */
void sdx_enc_reset(sdx_enc_t *e);

/* Encode the item that starts at in[0] (a line start) into out[0..cap).
 * Consumes one row, or for anything that is not a complete conforming row,
 * bytes up to and including the next LF (at most len bytes; 'final' says no
 * more bytes follow in this block, otherwise an incomplete line consumes 0).
 * Returns the bytes consumed and sets *written; 0 means "need more input"
 * and SDX_FLUSH "the item does not fit in cap: send the output, then retry"
 * (with SDX_FRESH a row that does not fit is carried RAW instead). A RAW item
 * is split when cap is smaller than the line; the rest follows in later
 * items. State changes only when an item is emitted. */
#define SDX_FLUSH ((size_t)-1)
#define SDX_FINAL 1U   /* no more bytes follow in this block: emit a partial line */
#define SDX_FRESH 2U   /* out is an empty message: a row too large for it goes RAW */
size_t sdx_enc_item(sdx_enc_t *e, const uint8_t *in, size_t len, unsigned flags,
                    uint8_t *out, size_t cap, size_t *written);

#endif
