"""Reference SDX decoder (test oracle for Core/Src/sd_xfer_codec.c).

Mirrors GroundStation26 backend/src/sd_download/sdx.rs; the wire format is
specified in GroundStation26 docs/backend/sd-transfer.md. Kept free of any
encoder heuristics: it only expands what the board sends.
"""

def put_varint(out, v):
    while v >= 0x80:
        out.append((v & 0x7f) | 0x80); v >>= 7
    out.append(v)

def get_varint(buf, pos):
    v = shift = 0
    while True:
        b = buf[pos]; pos += 1
        v |= (b & 0x7f) << shift
        if b < 0x80: return v, pos
        shift += 7
        if shift > 63: raise ValueError('varint')

def zz(v): return v * 2 if v >= 0 else -v * 2 - 1
def unzz(u): return u >> 1 if not u & 1 else -((u + 1) >> 1)

DIGITS = b'0123456789'
U63 = 1 << 63

def parse_num(t):
    """-> ((b0, b1), exp, value) or None. Not checked for round trip here."""
    n = len(t); i = 0
    neg = n > 0 and t[0] == 0x2d
    if neg: i = 1
    s = i
    while i < n and t[i] in DIGITS: i += 1
    nint = i - s
    if nint == 0 or nint > 15: return None
    v = int(t[s:i])
    nfrac = 0
    if i < n and t[i] == 0x2e:
        i += 1; fs = i
        while i < n and t[i] in DIGITS: i += 1
        nfrac = i - fs
        if nfrac == 0 or nfrac > 15 or nint + nfrac > 18: return None
        v = v * 10 ** nfrac + int(t[fs:i])
    b0 = 0; ev = 0
    if i < n and t[i] in b'eE':
        b0 = 1 | ((t[i] == 0x45) << 1); i += 1
        eneg = False
        if i < n and t[i] in b'+-':
            if t[i] == 0x2b: b0 |= 4
            else: eneg = True
            i += 1
        es = i
        while i < n and t[i] in DIGITS: i += 1
        x = i - es
        if x == 0 or x > 3: return None
        b0 |= x << 3
        ev = int(t[es:i])
        if eneg: ev = -ev
    if i != n: return None
    return (b0, nint | (nfrac << 4)), ev, (-v if neg else v)

def fmt_num(shape, ev, v):
    b0, b1 = shape
    nint = b1 & 15; nfrac = b1 >> 4
    d = str(abs(v)).rjust(nint + nfrac, '0')
    if len(d) != nint + nfrac: return None
    s = ('-' if v < 0 else '') + d[:nint] + ('.' + d[nint:] if nfrac else '')
    if b0 & 1:
        if ev > 0 and not b0 & 4: pass
        s += 'E' if b0 & 2 else 'e'
        if ev < 0: s += '-'
        elif b0 & 4: s += '+'
        x = (b0 >> 3) & 3
        e = str(abs(ev)).rjust(x, '0')
        if len(e) != x: return None
        s += e
    elif ev != 0: return None
    return s.encode()

def apply_delta(prev, u):
    shape, pev, pv = prev
    if not shape[0] & 1:
        return shape, pev, pv + unzz(u)
    de = (u & 7) - 3
    if de > 3: raise ValueError('bad exponent delta')
    ev = pev + de; m = min(ev, pev)
    scaled = pv * 10 ** (pev - m) + unzz(u >> 3)
    q, r = divmod(scaled, 10 ** (ev - m))
    if r: raise ValueError('delta not exact')
    return shape, ev, q

def dec_tok(ctx, key, op, buf, pos):
    if op == 1:
        return ctx[key][0], pos
    if op == 2:
        u, pos = get_varint(buf, pos)
        p = apply_delta(ctx[key][1], u)
    elif op == 3:
        shape = (buf[pos], buf[pos + 1]); pos += 2
        ev = 0
        if shape[0] & 1:
            e, pos = get_varint(buf, pos); ev = unzz(e)
        v, pos = get_varint(buf, pos)
        p = (shape, ev, unzz(v))
    elif op == 4:
        L, pos = get_varint(buf, pos)
        t = bytes(buf[pos:pos + L]); pos += L
        ctx[key] = (t, None); return t, pos
    else:
        raise ValueError('bad op %d' % op)
    t = fmt_num(*p)
    if t is None: raise ValueError('bad number')
    ctx[key] = (t, p); return t, pos

def decode_block(buf):
    out = bytearray(); labels = []; ops_last = {}; ctx = {}
    pos = 0; n = len(buf)
    while pos < n:
        tag = buf[pos]; pos += 1
        if tag == 0:
            L, pos = get_varint(buf, pos); out += buf[pos:pos + L]; pos += L; continue
        if tag == 1:
            L, pos = get_varint(buf, pos); labels.append(bytes(buf[pos:pos + L])); pos += L; continue
        lid = (tag - 2) >> 1
        if tag & 1:
            ops = ops_last[lid]
        else:
            cnt, pos = get_varint(buf, pos)
            ops = []
            for k in range((cnt + 1) // 2):
                b = buf[pos]; pos += 1
                ops.append(b & 15); ops.append(b >> 4)
            ops = ops[:cnt]; ops_last[lid] = ops
        fields = [None] * 13; k = 0
        for f in range(13):
            if f == 2: fields[2] = labels[lid]; continue
            op = ops[k]; k += 1
            if op == 0: fields[f] = b''
            elif op == 6: fields[f] = fields[ops[k]]; k += 1
            elif op == 5:
                L, pos = get_varint(buf, pos); fields[f] = b'"' + bytes(buf[pos:pos + L]) + b'"'; pos += L
            elif op == 7:
                cnt = ops[k]; k += 1; toks = []
                for e in range(cnt):
                    t, pos = dec_tok(ctx, (lid, f, e), ops[k], buf, pos); k += 1; toks.append(t)
                fields[f] = b'"' + b' '.join(toks) + b'"'
            else:
                fields[f], pos = dec_tok(ctx, (lid, f, 0), op, buf, pos)
        out += b','.join(fields) + b'\r\n'
    return bytes(out)
