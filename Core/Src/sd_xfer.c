/* SD transfer v2 sender engine (see sd_xfer.h). Shared verbatim by
 * FlightComputer26 and DAQ-Board; host-tested by tests/test_sd_xfer.py. */
#include "sd_xfer.h"
#include "board_crc32.h"
#include <string.h>

#define STATUS_OK 0U
#define STATUS_END 6U
#define STATUS_BAD_OFFSET 7U

static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get32(const uint8_t *p) { return (uint32_t)get16(p) | ((uint32_t)get16(p + 2) << 16); }
static void put16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { put16(p, v); put16(p + 2, v >> 16); }

bool sd_xfer_parse_open(const uint8_t *p, size_t len, uint8_t *dir, sd_xfer_open_t *o,
                        size_t *name_at, size_t *name_len)
{
    if (!p || len < 20U || p[0] != SD_XFER_OP_OPEN || (size_t)p[18] + 19U != len || !p[1]) return false;
    o->xfer = p[1]; *dir = p[2]; o->flags = p[3];
    o->offset = get32(p + 4); o->length = get32(p + 8);
    o->rate = (uint32_t)get16(p + 12) * 16U; o->msg = get16(p + 14);
    o->window = p[16]; o->block_kib = p[17];
    *name_at = 19U; *name_len = p[18];
    return true;
}

bool sd_xfer_parse_ack(const uint8_t *p, size_t len, uint8_t *xfer, sd_xfer_flow_t *f)
{
    if (!p || len != 11U || p[0] != SD_XFER_OP_ACK) return false;
    *xfer = p[1]; f->una = get16(p + 2); f->bitmap = get32(p + 4);
    f->rate = (uint32_t)get16(p + 8) * 16U; f->window = p[10];
    return true;
}

static uint8_t clamp_window(uint8_t w) { return w == 0U ? 1U : w > SD_XFER_WINDOW_MAX ? SD_XFER_WINDOW_MAX : w; }

uint8_t sd_xfer_begin(sd_xfer_t *x, const sd_xfer_open_t *o, uint32_t file_size, bool growing,
                      uint32_t now_ms, uint8_t ans[SD_XFER_OPEN_ANSWER])
{
    memset(x, 0, sizeof *x);
    uint8_t status = STATUS_OK;
    uint32_t length = o->length;
    if (o->offset > file_size) status = STATUS_BAD_OFFSET;
    else if (o->offset == file_size) status = STATUS_END;
    else if (!length || length > file_size - o->offset) length = file_size - o->offset;
    if (status != STATUS_OK) length = 0;
    uint16_t msg = o->msg < SD_XFER_MSG_MIN ? SD_XFER_MSG_MIN : o->msg > SD_XFER_MSG_MAX ? SD_XFER_MSG_MAX : o->msg;
    uint8_t kib = o->block_kib == 0U ? 1U : o->block_kib > SD_XFER_BLOCK_MAX_KIB ? SD_XFER_BLOCK_MAX_KIB : o->block_kib;
    const uint8_t sdx = (o->flags & SD_XFER_OPEN_SDX) ? 1U : 0U;
    ans[0] = SD_XFER_OP_OPEN | 0x80U; ans[1] = status; ans[2] = o->xfer;
    ans[3] = (uint8_t)((sdx ? SD_XFER_ANSWER_SDX : 0U) | (growing ? SD_XFER_ANSWER_GROWING : 0U));
    put32(ans + 4, file_size); put32(ans + 8, o->offset); put32(ans + 12, length);
    put16(ans + 16, msg); ans[18] = kib;
    if (status != STATUS_OK) return status;
    x->sdx = sdx; x->growing = growing; x->msg = msg; x->block = (uint32_t)kib * 1024U;
    x->start = x->pos = o->offset; x->end = o->offset + length;
    x->flow.rate = o->rate; x->flow.window = clamp_window(o->window);
    x->flow.progress_ms = x->ack_ms = now_ms;
    x->raw_base = UINT32_MAX;
    x->id = o->xfer;
    return status;
}

bool sd_xfer_ack(sd_xfer_t *x, uint8_t xfer, const sd_xfer_flow_t *f, uint32_t now_ms)
{
    if (!x->id || xfer != x->id) return false;
    /* una only moves forward within what was sent (modulo 2^16). */
    const uint16_t ahead = (uint16_t)(f->una - x->flow.una);
    const uint16_t sent = (uint16_t)(x->frontier - x->flow.una);
    if (ahead > sent) return false;
    if (ahead) x->flow.progress_ms = now_ms;
    x->flow.una = f->una; x->flow.bitmap = f->bitmap;
    x->flow.rate = f->rate; x->flow.window = clamp_window(f->window);
    x->ack_ms = now_ms;
    return true;
}

static bool received(const sd_xfer_flow_t *f, uint16_t seq)
{
    const uint16_t d = (uint16_t)(seq - f->una);
    if (d >= 0x8000U) return true;               /* below una: acknowledged */
    if (d == 0U || d > 32U) return false;
    return (f->bitmap >> (d - 1U)) & 1U;
}

/* The raw window holds file bytes [raw_base, raw_base + raw_len) with
 * raw_base = pos rounded down to 512, up to SD_XFER_RAW_WINDOW bytes, never
 * past the range end. Contents depend only on pos, so regeneration matches. */
static bool load_window(sd_xfer_t *x, sd_xfer_work_t *w, sd_xfer_read_fn read, void *ctx)
{
    const uint32_t base = x->pos & ~511U;
    uint32_t want = x->end - base;
    if (want > SD_XFER_RAW_WINDOW) want = SD_XFER_RAW_WINDOW;
    if (x->raw_base == base && x->raw_len == want) return true;
    if (x->raw_base + 512U == base && x->raw_len == SD_XFER_RAW_WINDOW) {
        memmove(w->raw, w->raw + 512, 512U);
        if (!read(ctx, base + 512U, w->raw + 512, want - 512U)) { x->raw_base = UINT32_MAX; return false; }
    } else if (!read(ctx, base, w->raw, want)) {
        x->raw_base = UINT32_MAX;
        return false;
    }
    x->raw_base = base; x->raw_len = (uint16_t)want;
    return true;
}

/* One message at the cursor: its body is built after the largest header,
 * then the header is written in front of it. */
static size_t build(sd_xfer_t *x, sd_xfer_work_t *w, sd_xfer_read_fn read, void *ctx, const uint8_t **out)
{
    uint8_t flags = (uint8_t)((x->sdx ? SD_XFER_BODY_SDX : 0U) | (x->growing ? SD_XFER_GROWING : 0U));
    if (!x->in_block) {
        x->in_block = 1U; x->blk_start = x->pos; x->crc = 0U;
        flags |= SD_XFER_FIRST;
        if (x->sdx) sdx_enc_reset(&w->enc);
    }
    uint8_t *const body = w->msg + SD_XFER_HDR_LAST;
    const size_t cap = x->msg - SD_XFER_HDR_LAST;
    size_t used = 0;
    bool last = false;
    for (;;) {
        if (x->pos >= x->end || x->pos - x->blk_start >= x->block) { last = true; break; }
        if (!load_window(x, w, read, ctx)) return SIZE_MAX;
        const uint32_t off = x->pos - x->raw_base;
        size_t avail = x->raw_len - off;
        size_t n, wrote;
        if (x->sdx) {
            const unsigned f = (x->raw_base + x->raw_len >= x->end ? SDX_FINAL : 0U) | (used ? 0U : SDX_FRESH);
            n = sdx_enc_item(&w->enc, w->raw + off, avail, f, body + used, cap - used, &wrote);
            if (n == 0U) n = sdx_enc_item(&w->enc, w->raw + off, avail, f | SDX_FINAL, body + used, cap - used, &wrote);
            if (n == SDX_FLUSH) break;
        } else {
            n = x->blk_start + x->block - x->pos;
            if (n > avail) n = avail;
            if (n > cap - used) n = cap - used;
            if (!n) break;
            memcpy(body + used, w->raw + off, n);
            wrote = n;
        }
        x->crc = board_crc32_software(x->crc, w->raw + off, n);
        x->pos += (uint32_t)n;
        used += wrote;
    }
    const uint16_t seq = x->cursor;
    const size_t hdr = last ? SD_XFER_HDR_LAST : SD_XFER_HDR;
    uint8_t *h = body - hdr;
    if (last) {
        flags |= SD_XFER_LAST;
        if (x->pos >= x->end) flags |= SD_XFER_END;
        put32(h + 9, x->pos - x->blk_start); put32(h + 13, x->crc);
        x->in_block = 0U;
    }
    h[0] = SD_XFER_DATA; h[1] = x->id; h[2] = flags;
    put16(h + 3, seq); put32(h + 5, x->blk_start);
    *out = h;
    return hdr + used;
}

static void rewind_to(sd_xfer_t *x, uint16_t seq)
{
    unsigned i = 0;
    while (i + 1U < x->ring_n && (uint16_t)(seq - x->ring[i + 1U].seq) < 0x8000U) ++i;
    x->cursor = x->ring[i].seq;
    x->pos = x->ring[i].off;
    x->in_block = 0U;
}

size_t sd_xfer_produce(sd_xfer_t *x, sd_xfer_work_t *w, const sd_xfer_flow_t *flow,
                       sd_xfer_read_fn read, void *ctx, uint32_t now_ms, const uint8_t **out)
{
    if (!x->id) return 0;
    if ((uint32_t)(now_ms - x->ack_ms) >= SD_XFER_ACK_TIMEOUT_MS) { x->id = 0; return 0; }
    /* Forget blocks whose successor starts at or below una: fully acked. */
    while (x->ring_n > 1U && (uint16_t)(flow->una - x->ring[1].seq) < 0x8000U) {
        memmove(x->ring, x->ring + 1, (size_t)(x->ring_n - 1U) * sizeof x->ring[0]);
        --x->ring_n;
    }
    if (x->done && flow->una == x->frontier) { x->id = 0; return 0; }
    if (!flow->rate) return 0;
    /* A gap (a later message received) or a stall: regenerate from the
     * block holding una, unless una was sent within the guard time. */
    const uint16_t una = flow->una;
    if (una != x->frontier && (flow->bitmap || (uint32_t)(now_ms - flow->progress_ms) >= SD_XFER_STALL_MS) &&
        (uint32_t)(now_ms - x->sent_ms[una % SD_XFER_WINDOW_MAX]) >= SD_XFER_RETX_GUARD_MS &&
        (uint16_t)(x->cursor - una) <= (uint16_t)(x->frontier - una) && x->cursor != una && x->ring_n)
        rewind_to(x, una);
    for (unsigned budget = 0; budget < 2U * SD_XFER_WINDOW_MAX; ++budget) {
        const bool fresh = x->cursor == x->frontier;
        if (fresh && (x->done || (uint16_t)(x->frontier - una) >= flow->window)) return 0;
        const uint32_t start = x->in_block ? x->blk_start : x->pos;
        const bool first = !x->in_block;
        const uint16_t seq = x->cursor;
        const size_t n = build(x, w, read, ctx, out);
        if (n == SIZE_MAX) { x->id = 0; return SIZE_MAX; }
        ++x->cursor;
        if (fresh) {
            x->frontier = x->cursor;
            if (first) {
                if (x->ring_n == SD_XFER_RING) {   /* cannot happen with window <= 32 */
                    memmove(x->ring, x->ring + 1, (SD_XFER_RING - 1U) * sizeof x->ring[0]);
                    --x->ring_n;
                }
                x->ring[x->ring_n].off = start; x->ring[x->ring_n].seq = seq; ++x->ring_n;
            }
            if ((*out)[2] & SD_XFER_END) x->done = 1U;
        } else if (received(flow, seq) ||
                   (uint32_t)(now_ms - x->sent_ms[seq % SD_XFER_WINDOW_MAX]) < SD_XFER_RETX_GUARD_MS) {
            continue;                              /* the GS has it, or it is in flight */
        }
        x->sent_ms[seq % SD_XFER_WINDOW_MAX] = now_ms;
        return n;
    }
    return 0;
}
