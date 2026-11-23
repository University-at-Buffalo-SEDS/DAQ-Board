/* SDX encoder for the SD transfer (see sd_xfer_codec.h). Shared verbatim by
 * FlightComputer26 and DAQ-Board; host-tested against the GroundStation26
 * decoder's reference (tests/test_sd_xfer_codec.py). */
#include "sd_xfer_codec.h"
#include <string.h>

#define TAG_RAW 0x00U
#define TAG_LABEL 0x01U
#define TAG_ROW 0x02U
enum { OP_EMPTY, OP_SAME, OP_DELTA, OP_NUM, OP_TEXT, OP_QTEXT, OP_COPY, OP_QVEC };
enum { K_NUM = 1, K_TEXT, K_LONG };

typedef struct { uint8_t *p; size_t n, cap; bool commit; } sink_t;
typedef struct { uint8_t b0, b1; int16_t exp; int64_t v; } num_t;
typedef struct { const uint8_t *p; uint16_t len; bool quoted; } field_t;

static void put(sink_t *s, uint8_t b) { if (s->commit && s->n < s->cap) s->p[s->n] = b; ++s->n; }
static void put_bytes(sink_t *s, const uint8_t *b, size_t n) { while (n--) put(s, *b++); }
static void put_varint(sink_t *s, uint64_t v) { while (v >= 0x80U) { put(s, (uint8_t)(v | 0x80U)); v >>= 7; } put(s, (uint8_t)v); }
static uint64_t zz(int64_t v) { return v < 0 ? ((uint64_t)(-(v + 1)) << 1) | 1U : (uint64_t)v << 1; }

static const int64_t pow10_3[4] = {1, 10, 100, 1000};

/* [-]D{1..15}[.D{1..15}][(e|E)[+|-]D{1..3}], <= 18 digits. True only when
 * the decoder's expansion is this exact text: the digit counts, the 'E' and
 * '+' are part of the shape, so the only text it cannot rebuild is a minus
 * sign on a zero mantissa or on a zero exponent. */
static bool num_parse(const uint8_t *t, size_t n, num_t *o)
{
    size_t i = 0, d0;
    uint64_t v = 0;
    const bool neg = n && t[0] == '-';
    if (neg) i = 1;
    d0 = i;
    while (i < n && t[i] >= '0' && t[i] <= '9' && i - d0 < 16U) v = v * 10U + (uint64_t)(t[i++] - '0');
    const size_t nint = i - d0;
    if (nint == 0U || nint > 15U) return false;
    size_t nfrac = 0;
    if (i < n && t[i] == '.') {
        d0 = ++i;
        while (i < n && t[i] >= '0' && t[i] <= '9' && i - d0 < 15U) v = v * 10U + (uint64_t)(t[i++] - '0');
        nfrac = i - d0;
        if (nfrac == 0U || nint + nfrac > 18U) return false;
    }
    o->b0 = 0; o->exp = 0;
    if (i < n && (t[i] == 'e' || t[i] == 'E')) {
        o->b0 = (uint8_t)(1U | (t[i] == 'E' ? 2U : 0U));
        ++i;
        bool eneg = false;
        if (i < n && (t[i] == '+' || t[i] == '-')) { if (t[i] == '+') o->b0 |= 4U; else eneg = true; ++i; }
        d0 = i;
        int e = 0;
        while (i < n && t[i] >= '0' && t[i] <= '9' && i - d0 < 3U) e = e * 10 + (t[i++] - '0');
        if (i == d0 || (eneg && e == 0)) return false;
        o->b0 |= (uint8_t)((i - d0) << 3);
        o->exp = (int16_t)(eneg ? -e : e);
    }
    if (i != n || (neg && v == 0U)) return false;
    o->b1 = (uint8_t)(nint | (nfrac << 4));
    o->v = neg ? -(int64_t)v : (int64_t)v;
    return true;
}

/* One token; returns the op. The label's slot for this token position is
 * used only if it holds this token's key. On commit the slot takes the token
 * and any other slot of the label holding the same key is cleared, so a
 * matching slot always holds the key's latest token, which is what the
 * decoder's map (keyed by label, field and element) holds. */
static uint8_t enc_token(sdx_enc_t *e, const sdx_label_t *l, unsigned ord, uint8_t key,
                         const uint8_t *t, size_t n, sink_t *s)
{
    sdx_ctx_t *c = ord < l->ctx_count ? &e->ctx[l->ctx_base + ord] : NULL;
    const bool have = c && c->kind && c->key == key;
    num_t x;
    const bool num = num_parse(t, n, &x);
    uint8_t op = OP_TEXT;
    if (have && ((num && c->kind == K_NUM && c->b0 == x.b0 && c->b1 == x.b1 && c->exp == x.exp && c->u.v == x.v) ||
                 (!num && c->kind == K_TEXT && c->len == n && !memcmp(c->u.text, t, n)))) {
        op = OP_SAME;
    } else if (num) {
        op = OP_NUM;
        if (have && c->kind == K_NUM && c->b0 == x.b0 && c->b1 == x.b1) {
            const int de = x.exp - c->exp;
            if (!(x.b0 & 1U)) {
                const int64_t d = x.v - c->u.v;
                if ((d < 0 ? -d : d) < ((int64_t)1 << 62)) { op = OP_DELTA; put_varint(s, zz(d)); }
            } else if (de >= -3 && de <= 3 && (x.b1 & 15U) + (x.b1 >> 4) <= 15U) {
                const int m = de < 0 ? de : 0;   /* finer exponent, relative to x */
                const int64_t d = x.v * pow10_3[de - m] - c->u.v * pow10_3[-m];
                if ((d < 0 ? -d : d) < ((int64_t)1 << 59)) {
                    op = OP_DELTA;
                    put_varint(s, (zz(d) << 3) | (uint64_t)(de + 3));
                }
            }
        }
        if (op == OP_NUM) {
            put(s, x.b0); put(s, x.b1);
            if (x.b0 & 1U) put_varint(s, zz(x.exp));
            put_varint(s, zz(x.v));
        }
    } else {
        put_varint(s, n); put_bytes(s, t, n);
    }
    if (s->commit) {
        for (unsigned i = 0; i < l->ctx_count; ++i)
            if (e->ctx[l->ctx_base + i].key == key) e->ctx[l->ctx_base + i].kind = 0;
    }
    if (s->commit && c) {
        c->key = key;
        if (num) { c->kind = K_NUM; c->b0 = x.b0; c->b1 = x.b1; c->exp = x.exp; c->u.v = x.v; }
        else if (n <= sizeof c->u.text) { c->kind = K_TEXT; c->len = (uint8_t)n; memcpy(c->u.text, t, n); }
        else c->kind = K_LONG;
    }
    return op;
}

static void nib(uint8_t *ops, unsigned *count, uint8_t v)
{
    if (*count & 1U) ops[*count >> 1] |= (uint8_t)(v << 4); else ops[*count >> 1] = v;
    ++*count;
}

/* Splits one CSV record; true for a 13-field row ending in CRLF. */
static bool split_row(const uint8_t *in, size_t len, field_t *f, size_t *end)
{
    size_t i = 0;
    for (unsigned k = 0;; ++k) {
        if (k == SDX_FIELDS) return false;
        const size_t start = i;
        if (i < len && in[i] == '"') {
            for (++i;; i += 2) {
                while (i < len && in[i] != '"') ++i;
                if (i >= len) return false;
                if (i + 1 >= len || in[i + 1] != '"') break;
            }
            f[k].p = in + start + 1; f[k].len = (uint16_t)(i - start - 1); f[k].quoted = true; ++i;
        } else {
            while (i < len && in[i] != ',' && in[i] != '\r' && in[i] != '\n' && in[i] != '"') ++i;
            f[k].p = in + start; f[k].len = (uint16_t)(i - start); f[k].quoted = false;
        }
        if (i >= len) return false;
        if (in[i] == ',') { ++i; continue; }
        if (in[i] == '\r' && i + 1 < len && in[i + 1] == '\n' && k == SDX_FIELDS - 1U) { *end = i + 2U; return true; }
        return false;
    }
}

/* Op nibbles and payloads of one row; the payload goes to pay. Each label
 * owns a run of context slots, one per token position (n-th context-using
 * token of the row); a slot also records its (field, element) key, so a row
 * of a different shape simply finds no previous value. Returns the op count;
 * *tokens gets the number of context-using tokens. */
static unsigned row_ops(sdx_enc_t *e, const sdx_label_t *l, const field_t *f, uint8_t *ops,
                        sink_t *pay, unsigned *tokens_used)
{
    unsigned count = 0, ord = 0;
#define TOKEN(KEY, P, N) enc_token(e, l, ord++, (KEY), (P), (N), pay)
    for (unsigned k = 0; k < SDX_FIELDS; ++k) {
        if (k == 2U) continue;
        const field_t *x = &f[k];
        if (x->len || x->quoted) {
            unsigned g = 0;
            while (g < k && (g == 2U || x->len != f[g].len || x->quoted != f[g].quoted || memcmp(x->p, f[g].p, x->len))) ++g;
            if (g < k) { nib(ops, &count, OP_COPY); nib(ops, &count, (uint8_t)g); continue; }
        }
        const uint8_t key = (uint8_t)(k << 4);
        if (!x->quoted) {
            uint8_t op = OP_EMPTY;
            if (x->len) { op = TOKEN(key, x->p, x->len); }
            nib(ops, &count, op);
            continue;
        }
        unsigned tokens = 1U;
        bool vec = x->len && x->p[0] != ' ' && x->p[x->len - 1U] != ' ';
        for (unsigned i = 0; vec && i < x->len; ++i) {
            if (x->p[i] == '"' || (x->p[i] == ' ' && x->p[i - 1U] == ' ')) vec = false;
            else if (x->p[i] == ' ') ++tokens;
        }
        if (vec && tokens <= 15U) {
            nib(ops, &count, OP_QVEC); nib(ops, &count, (uint8_t)tokens);
            const uint8_t *t = x->p;
            for (unsigned el = 0; el < tokens; ++el) {
                size_t n = 0;
                while (t + n < x->p + x->len && t[n] != ' ') ++n;
                const uint8_t op = TOKEN((uint8_t)(key | el), t, n);
                nib(ops, &count, op);
                t += n + 1U;
            }
            continue;
        }
        nib(ops, &count, OP_QTEXT); put_varint(pay, x->len); put_bytes(pay, x->p, x->len);
    }
#undef TOKEN
    *tokens_used = ord;
    return count;
}

/* Row item: sized when !s->commit, emitted when s->commit. */
static size_t enc_row(sdx_enc_t *e, unsigned lid, const field_t *f, sink_t *s)
{
    const bool fresh = lid == e->labels;
    sdx_label_t scratch, *l = fresh ? &scratch : &e->label[lid];
    if (fresh) {
        put(s, TAG_LABEL); put_varint(s, f[2].len); put_bytes(s, f[2].p, f[2].len);
        memset(&scratch, 0, sizeof scratch);   /* no slots yet: nothing to refer to */
    }
    uint8_t ops[(SDX_FIELDS - 1U) * 17U / 2U + 1U];
    sink_t probe = {NULL, 0, 0, false};
    unsigned tokens;
    const unsigned count = row_ops(e, l, f, ops, &probe, &tokens);   /* ops only */
    const unsigned bytes = (count + 1U) / 2U;
    const bool same = !fresh && l->ops_len == count && !memcmp(l->ops, ops, bytes);
    put(s, (uint8_t)(TAG_ROW + 2U * lid + (same ? 1U : 0U)));
    if (!same) { put_varint(s, count); put_bytes(s, ops, bytes); }
    if (s->commit) {
        if (fresh) {
            l = &e->label[e->labels++];
            l->len = (uint8_t)f[2].len; memcpy(l->name, f[2].p, f[2].len);
            const unsigned room = SDX_CTX_ENTRIES - e->ctx_used;
            l->ctx_base = e->ctx_used;
            l->ctx_count = (uint8_t)(tokens < room ? tokens : room);
            e->ctx_used = (uint16_t)(e->ctx_used + l->ctx_count);
        }
        if (!same) {
            if (bytes <= SDX_OPS_BYTES) { l->ops_len = (uint8_t)count; memcpy(l->ops, ops, bytes); }
            else l->ops_len = 0xFFU;
        }
    }
    (void)row_ops(e, l, f, ops, s, &tokens);                   /* payloads */
    return s->n;
}

void sdx_enc_reset(sdx_enc_t *e) { memset(e, 0, sizeof *e); }

static size_t emit_raw(const uint8_t *in, size_t n, uint8_t *out, size_t cap, size_t *written)
{
    if (cap < 5U) return SDX_FLUSH;
    size_t take = n;
    if (take > cap - 4U) take = cap - 4U;     /* tag + varint (<= 3 bytes) */
    sink_t s = {out, 0, cap, true};
    put(&s, TAG_RAW); put_varint(&s, take); put_bytes(&s, in, take);
    *written = s.n;
    return take;
}

size_t sdx_enc_item(sdx_enc_t *e, const uint8_t *in, size_t len, unsigned flags,
                    uint8_t *out, size_t cap, size_t *written)
{
    *written = 0;
    if (!len) return 0;
    size_t line = 0;
    while (line < len && in[line] != '\n') ++line;
    if (line == len && !(flags & SDX_FINAL)) return 0;
    if (line < len) ++line;
    field_t f[SDX_FIELDS];
    size_t end = 0;
    if (in[0] != '#' && split_row(in, len, f, &end) && f[2].len && !f[2].quoted && f[2].len <= SDX_LABEL_MAX) {
        unsigned lid = 0;
        while (lid < e->labels && !(e->label[lid].len == f[2].len && !memcmp(e->label[lid].name, f[2].p, f[2].len))) ++lid;
        if (lid < SDX_LABELS) {
            sink_t s = {out, 0, cap, false};
            if (enc_row(e, lid, f, &s) <= cap) {
                s.n = 0; s.commit = true;
                *written = enc_row(e, lid, f, &s);
                return end;
            }
            if (!(flags & SDX_FRESH)) return SDX_FLUSH;   /* caller flushes, then retries */
        }
    }
    return emit_raw(in, line, out, cap, written);
}
