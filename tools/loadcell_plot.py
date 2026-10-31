#!/usr/bin/env python3
"""Live DAQ sensor plots using Python's standard library and OpenOCD.

Run: python3 tools/loadcell_plot.py
Reads the existing ADC ring buffer while the MCU runs; never drains the queue,
halts, resets, flashes, tares, or changes calibration. See --help for options.
"""
from __future__ import annotations

import argparse
from collections import deque
import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import socket
import struct
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse
import webbrowser

ROOT = Path(__file__).resolve().parents[1]
DEPTH = 128
RING_OFFSET = 60
ENTRY_SIZE = 16
CONTEXT_SIZE = RING_OFFSET + DEPTH * ENTRY_SIZE
CALIBRATION_WORDS = 17
CHANNELS = tuple(range(8)) + (9,)
FIELDS = ['sequence', 'host_unix_ms', 'board_ms', 'channel', 'adc_code',
          'voltage_v', 'raw_value', 'converted_value', 'temperature_c']


class Debugger:
    def __init__(self, port):
        self.sock = socket.create_connection(('127.0.0.1', port), timeout=5)
        self.sock.settimeout(10)
        self.pending = b''

    def close(self):
        self.sock.close()

    def command(self, command):
        self.sock.sendall(command.encode() + b'\x1a')
        while b'\x1a' not in self.pending:
            block = self.sock.recv(65536)
            if not block:
                raise ConnectionError('OpenOCD disconnected')
            self.pending += block
        result, self.pending = self.pending.split(b'\x1a', 1)
        return result.decode().strip()

    def read(self, address, count):
        """Read count 32-bit words without halting the target."""
        result = self.command(f'read_memory 0x{address:x} 32 {count}')
        try:
            words = [int(word, 0) for word in result.split()]
        except ValueError as exc:
            raise RuntimeError(f'Cannot read target memory: {result}') from exc
        if len(words) != count:
            raise RuntimeError(f'Short memory read: expected {count}, got {len(words)}')
        return struct.pack('<' + 'I' * count, *words)

    def word(self, address):
        return struct.unpack('<I', self.read(address, 1))[0]


def load_elf(path):
    """Load addresses and immutable flash sections from an ARM ELF32 file."""
    blob = path.read_bytes()
    if blob[:6] != b'\x7fELF\x01\x01':
        raise ValueError('Expected a little-endian ELF32 firmware file')
    header = struct.unpack_from('<16sHHIIIIIHHHHHH', blob)
    if header[2] != 40:
        raise ValueError('ELF is not ARM firmware')
    sections = [struct.unpack_from('<10I', blob, header[6] + i * header[11])
                for i in range(header[12])]

    def section_bytes(section):
        return blob[section[4]:section[4] + section[5]]

    def string(table, offset):
        return table[offset:table.index(b'\0', offset)].decode()

    names = section_bytes(sections[header[13]])
    flash, symbols = [], {}
    for section in sections:
        name = string(names, section[0])
        if name in ('.isr_vector', '.text', '.rodata'):
            flash.append((name, section[3], section_bytes(section)))
        if section[1] == 2:  # SHT_SYMTAB
            strings = section_bytes(sections[section[6]])
            for offset in range(section[4], section[4] + section[5], section[9]):
                n, value, size, _, _, _ = struct.unpack_from('<IIIBBH', blob, offset)
                symbols[string(strings, n)] = (value, size)
    required = {'g_mcp3564r': CONTEXT_SIZE, 'g_calibration': CALIBRATION_WORDS * 4, 'uwTick': 4,
                'g_mcp3564r_init_status': 4, 'g_daq_loadcell_publish_ok_count': 4,
                'g_daq_kg50_publish_ok_count': 4}
    for name, size in required.items():
        if name not in symbols or symbols[name][1] != size:
            raise ValueError(f'Unsupported firmware layout: {name} must be {size} bytes')
    if len(flash) != 3:
        raise ValueError('ELF lacks the firmware sections required for verification')
    return {name: value for name, (value, _) in symbols.items()}, flash


def verify_firmware(debugger, symbols, flash, report):
    if debugger.command('stm32u5x.cpu curstate') != 'running':
        raise RuntimeError('DAQ is not running; this tool will not resume or reset it')
    if (debugger.word(0xE000ED00) >> 4) & 0xFFF != 0xD21:
        raise RuntimeError('Probe is not attached to the expected Cortex-M33 DAQ')
    vector = next(address for name, address, _ in flash if name == '.isr_vector')
    if debugger.word(0xE000ED08) != vector:
        raise RuntimeError('Running vector table differs from this ELF; select the flashed ELF')
    for name, address, expected in flash:
        report(f'Checking flashed firmware {name}…')
        for offset in range(0, len(expected), 4096):
            block = expected[offset:offset + 4096]
            actual = debugger.read(address + offset, (len(block) + 3) // 4)
            if actual[:len(block)] != block:
                raise RuntimeError(f'Flashed firmware differs from the ELF ({name}); '
                                   'use --elf with the exact file that was flashed')
    if debugger.word(symbols['g_mcp3564r_init_status']) != 0:
        raise RuntimeError('Firmware reports ADC initialization failure')


def read_stable_ring(debugger, base):
    """Double-read small blocks: a slow full-ring read can span an entire wrap.

    Entries differing between reads are omitted. The ring is never consumed or
    modified; missed samples are expected in a sampled SWD diagnostic capture.
    """
    stable = bytearray(DEPTH * ENTRY_SIZE)
    for start in range(0, DEPTH, 16):
        address = base + RING_OFFSET + start * ENTRY_SIZE
        first = debugger.read(address, 16 * ENTRY_SIZE // 4)
        second = debugger.read(address, 16 * ENTRY_SIZE // 4)
        for index in range(16):
            offset = index * ENTRY_SIZE
            a, b = first[offset:offset + ENTRY_SIZE], second[offset:offset + ENTRY_SIZE]
            if a == b:
                stable[(start + index) * ENTRY_SIZE:(start + index + 1) * ENTRY_SIZE] = a
    return stable


def decode_ring(payload, head_before, head_after, board_ms, seen):
    """Decode current16-byte ring entries, excluding changing/stale slots."""
    if len(payload) != DEPTH * ENTRY_SIZE:
        raise ValueError('Invalid ADC ring layout')
    changing = set()
    if head_before is not None:
        if not (0 <= head_before < DEPTH and 0 <= head_after < DEPTH):
            raise ValueError('Invalid ADC ring head')
        changing = {(head_before + i) % DEPTH
                    for i in range((head_after - head_before) % DEPTH + 1)}
    result = []
    for slot in range(DEPTH):
        if slot in changing:
            continue
        entry = bytes(payload[slot * ENTRY_SIZE:(slot + 1) * ENTRY_SIZE])
        word, stamp, temperature, _ = struct.unpack('<IIfi', entry)
        channel = word >> 28
        age = (board_ms - stamp) & 0xFFFFFFFF
        if channel not in CHANNELS or age > 1000 or entry == bytes(ENTRY_SIZE):
            continue
        if seen.get(slot) == entry:
            continue
        seen[slot] = entry
        code = word & 0x01FFFFFF
        if code & 0x01000000:
            code -= 0x02000000
        result.append((age, stamp, channel, code, temperature))
    result.sort(key=lambda item: -item[0])
    return result


def f32(value):
    return struct.unpack('<f', struct.pack('<f', value))[0]


def convert(code, channel, calibration, temperature=math.nan):
    """Raw input scales from mcp3564r_board_config.h; never reuse P8 on P6.

    Converted values are unsmoothed. P6 calibration lives in GroundStation and
    is unavailable over this firmware's SWD symbols, so its mass is absent.
    """
    voltage = f32(f32(float(code) * f32(2.4)) / 8388608)
    if channel == 9:
        voltage = f32(voltage / 16)
        return voltage, voltage, None
    if channel >= 2:
        connector = f32(voltage * 11) if channel in (6, 7) else voltage
        return voltage, connector, f32(connector * 200) if channel == 6 else connector
    raw = f32(f32(f32(f32(float(code)) * f32(2.2104)) / 16777216) * 2) / 16
    corrected = raw
    if len(calibration) >= 15:
        reference, slope = calibration[11 + channel * 2:13 + channel * 2]
        if slope:
            corrected = f32(raw - f32(slope * f32(temperature - reference)))
    if channel == 0:
        value = f32(f32(calibration[0] * corrected) + calibration[1])
    else:
        c = calibration[4:11]
        x = f32(corrected - c[5])
        value = c[4]
        for coefficient in reversed(c[:4]):
            value = f32(f32(value * x) + coefficient)
        value = f32(value - c[6])
    return voltage, raw, value


def json_number(value):
    return value if value is None or not isinstance(value, float) or math.isfinite(value) else None


SD_COUNTERS = {
    'ready': 'g_sd_ready', 'init_stage': 'g_sd_init_stage',
    'write_errors': 'g_sd_write_error_count', 'init_failures': 'g_sd_init_failures',
    'mount_status': 'g_sd_mount_status', 'flushes': 'g_sd_flush_count',
    'raw_written': 'g_sd_raw_records_written_count',
    'raw_flushed': 'g_sd_raw_records_flushed_count',
    'csv_written': 'g_sd_csv_rows_written_count',
    'pressure_written': 'g_sd_pressure_records_written',
    'pressure_flushed': 'g_sd_pressure_records_flushed',
    'raw_protected_drops': 'g_sd_raw_essential_records_dropped',
    'csv_protected_drops': 'g_sd_essential_telemetry_dropped',
    'pressure_drops': 'g_sd_pressure_records_dropped',
}


def read_sd_counters(debugger, symbols):
    missing = [symbol for symbol in SD_COUNTERS.values() if symbol not in symbols]
    if missing:
        raise ValueError('SD symbols absent: ' + ', '.join(missing))
    return {key: debugger.word(symbols[symbol]) for key, symbol in SD_COUNTERS.items()}


class SDStatus:
    """Sample counters, not readiness, prove that storage is progressing."""
    def __init__(self):
        self.previous = None
        self.sampled_at = None
        self.progress_at = None
        self.value = {'state': 'UNKNOWN', 'reason': 'No SD counter sample yet'}

    def unavailable(self, reason):
        self.value = {'state': 'UNKNOWN', 'reason': str(reason)}

    def update(self, counters, now):
        missing = set(SD_COUNTERS) - counters.keys()
        if missing:
            self.unavailable('Missing counters: ' + ', '.join(sorted(missing)))
            return
        previous, elapsed = self.previous, None if self.sampled_at is None else now - self.sampled_at
        reset = previous is not None and any(
            counters[key] < previous[key] and previous[key] - counters[key] < 0x80000000
            for key in ('raw_written', 'csv_written', 'flushes'))
        rate = None
        progressed = False
        if reset:
            self.progress_at = None
        elif previous is not None and elapsed is not None and elapsed > 0:
            delta = sum((counters[key] - previous[key]) & 0xFFFFFFFF
                        for key in ('raw_written', 'csv_written'))
            rate = delta / elapsed
            progressed = delta > 0
            if progressed:
                self.progress_at = now
        self.previous, self.sampled_at = dict(counters), now
        self.value = dict(counters, state='WRITING' if progressed else 'READY / IDLE' if counters['ready'] else 'NOT READY',
                          rate_hz=rate, reset_detected=reset,
                          reason='Counters restarted' if reset else '')

    def snapshot(self, now):
        result = dict(self.value)
        result['last_progress_age_s'] = None if self.progress_at is None else max(0, now - self.progress_at)
        if result['state'] != 'UNKNOWN' and self.sampled_at is not None:
            age = max(0, now - self.sampled_at)
            result['sample_age_s'] = age
            if age > 3:
                result.update(state='UNKNOWN', reason='SD counter reads are stale')
            elif result['state'] == 'WRITING' and age > 1.5:
                result['state'] = 'READY / IDLE' if result['ready'] else 'NOT READY'
        return result


class Capture:
    def __init__(self, filename):
        self.lock = threading.Lock()
        self.rows = deque(maxlen=120000)
        self.sequence = 0
        self.status = 'Connecting to DAQ…'
        self.connected = False
        self.calibration = []
        self.counts = [0, 0]
        self.sd = SDStatus()
        self.filename = filename
        filename.parent.mkdir(parents=True, exist_ok=True)
        self.file = filename.open('x', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(FIELDS)
        self.file.flush()

    def report(self, message, connected=False):
        with self.lock:
            self.status, self.connected = message, connected
            if not connected:
                self.sd.unavailable('Probe is not verified and connected')
        print(message, flush=True)

    def add(self, points, calibration, counts):
        now = round(time.time() * 1000)
        with self.lock:
            self.calibration, self.counts = list(calibration), counts
            for age, stamp, channel, code, temperature in points:
                self.sequence += 1
                voltage, raw, kg = convert(code, channel, calibration, temperature)
                row = [self.sequence, now - age, stamp, channel, code, voltage, raw, kg, temperature]
                self.rows.append(row)
                self.writer.writerow(row)
            self.file.flush()

    def update_sd(self, counters=None, error=None):
        with self.lock:
            if error is not None:
                self.sd.unavailable(error)
            else:
                self.sd.update(counters, time.monotonic())

    def snapshot(self, since):
        with self.lock:
            rows = [row for row in self.rows if row[0] > since][-5000:]
            return {'status': self.status, 'connected': self.connected,
                    'rows': [[json_number(v) for v in row] for row in rows], 'sequence': self.sequence,
                    'calibration': [json_number(v) for v in self.calibration], 'published': self.counts,
                    'sd': self.sd.snapshot(time.monotonic()),
                    'csv': str(self.filename), 'server_ms': round(time.time() * 1000)}


def monitor(capture, args, stop):
    try:
        symbols, flash = load_elf(args.elf)
        identity = hashlib.sha256(args.elf.read_bytes()).hexdigest()[:16]
        capture.report(f'ELF {args.elf} · SHA256 {identity}')
    except Exception as exc:
        capture.report(str(exc))
        return
    while not stop.is_set():
        debugger = None
        try:
            debugger = Debugger(args.tcl_port)
            verify_firmware(debugger, symbols, flash, capture.report)
            capture.report(f'Live · verified ELF {identity} · sampled SWD; no CAN', True)
            seen, previous_tick = {}, None
            calibration = None
            next_calibration = 0
            counts = [0, 0]
            while not stop.is_set():
                started = time.monotonic()
                base = symbols['g_mcp3564r']
                payload = read_stable_ring(debugger, base)
                tick = debugger.word(symbols['uwTick'])
                if previous_tick is not None and tick < previous_tick and previous_tick - tick < 0x80000000:
                    raise RuntimeError('DAQ restarted; reconnecting and rechecking firmware')
                previous_tick = tick
                if started >= next_calibration:
                    cal_bytes = debugger.read(symbols['g_calibration'], CALIBRATION_WORDS)
                    if cal_bytes == debugger.read(symbols['g_calibration'], CALIBRATION_WORDS):
                        calibration = struct.unpack('<17f', cal_bytes)
                    counts = [debugger.word(symbols[name]) for name in
                              ('g_daq_loadcell_publish_ok_count', 'g_daq_kg50_publish_ok_count')]
                    if debugger.command('stm32u5x.cpu curstate') != 'running':
                        raise RuntimeError('DAQ stopped running')
                    try:
                        capture.update_sd(read_sd_counters(debugger, symbols))
                    except (OSError, ValueError, RuntimeError, struct.error) as exc:
                        capture.update_sd(error=exc)
                    next_calibration = started + 1
                # Stable per-block reads allow sampling even if a full scan spans a wrap.
                if calibration is not None:
                    points = decode_ring(payload, None, None, tick, seen)
                    capture.add(points, calibration, counts)
                stop.wait(max(0, 1 / args.rate - (time.monotonic() - started)))
        except (OSError, ValueError, RuntimeError, struct.error) as exc:
            capture.report(f'Disconnected: {exc}')
        finally:
            if debugger is not None:
                debugger.close()
        stop.wait(2)


def serve(capture):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path == '/':
                content, kind = HTML.encode(), 'text/html; charset=utf-8'
            elif parsed.path == '/data':
                try:
                    since = int(parse_qs(parsed.query).get('since', ['0'])[0])
                except ValueError:
                    self.send_error(400, 'Invalid sequence')
                    return
                content = json.dumps(capture.snapshot(since), allow_nan=False).encode()
                kind = 'application/json'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(content)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                self.wfile.write(content)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *_):
            pass
    return Handler


HTML = r'''<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>DAQ · Sensor monitor</title>
<style>
:root{color-scheme:dark;font-family:system-ui,sans-serif;background:#10151e;color:#e4eaf3}
body{max-width:1250px;margin:24px auto;padding:0 20px}h1{font-size:25px;margin-bottom:8px}
p{color:#a6b3c8;line-height:1.5}#status{padding:12px 16px;background:#202a3a;border-radius:8px}
.controls{display:flex;flex-wrap:wrap;gap:12px;align-items:center;margin:20px 0}
select,button{font:inherit;color:inherit;background:#263247;border:1px solid #52617a;border-radius:6px;padding:8px 12px;cursor:pointer}
label{display:flex;align-items:center;gap:8px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:18px}
section{background:#182130;border:1px solid #2b394e;border-radius:12px;padding:18px;min-width:0}
h2{margin:0 0 10px;font-size:18px}.value{font-size:26px;font-variant-numeric:tabular-nums}.meta{font-size:13px;line-height:1.8;color:#a6b3c8}
canvas{width:100%;height:300px;display:block;margin-top:12px}small{color:#a6b3c8}#csv{overflow-wrap:anywhere}
@media(max-width:850px){.grid{grid-template-columns:1fr}body{margin-top:15px}}
</style><h1>DAQ sensor monitor</h1>
<p>Apply a load, hold it, then release it. Watch the selected channel change and return toward its starting value.</p>
<div id="status">Connecting…</div>
<div class="controls"><label>Values <select id="units"><option value="4">Raw ADC counts</option><option value="5">ADC voltage (V)</option><option value="6">Raw telemetry value</option><option value="7">Converted (kg / psi / V)</option><option value="8">ADC temperature (°C)</option></select></label>
<label>History <select id="window"><option value="15">15 seconds</option><option value="60" selected>60 seconds</option><option value="120">2 minutes</option></select></label>
<button id="baseline">Set plot baseline</button><button id="absolute">Show absolute values</button><button id="pause">Pause plot</button>
<button id="download">Download visible CSV</button></div>
<small id="mode">Absolute values · each plot scales independently</small>
<div class="grid"><section><h2 style="color:#68b7ff"><select id="channel0"></select></h2><div class="value" id="value0">—</div><div class="meta" id="meta0">Waiting for samples</div><canvas id="plot0"></canvas></section>
<section><h2 style="color:#64e3b0"><select id="channel1"></select></h2><div class="value" id="value1">—</div><div class="meta" id="meta1">Waiting for samples</div><canvas id="plot1"></canvas></section></div>
<section style="margin-top:18px"><h2>SD card · <span id="sdstate">UNKNOWN</span></h2>
<div class="meta" id="sdinfo">Waiting for verified SD counters</div></section>
<p id="csv">CSV recording is starting…</p>
<p><small>Baseline affects this plot only. Firmware calibration is unchanged. Converted values are unsmoothed. P6 shows input volts; its mass calibration belongs to GroundStation and is not guessed. CH6 converted values are psi for the default P4/3 chamber configuration. Identity load-cell coefficients are uncalibrated. This is a sampled debug capture, so it may omit ADC conversions.</small></p>
<script>
const channels=[0,1,2,3,4,5,6,7,9],selected=[0,9],history=Array.from({length:10},()=>[]),colors=['#68b7ff','#64e3b0'];let sequence=0,baseline=null,paused=false,info=null,serverOffset=0;
const $=id=>document.getElementById(id);const number=(v,d=4)=>Number.isFinite(v)?v.toLocaleString(undefined,{maximumFractionDigits:d}):'—';
function draw(){if(paused)return;const now=Date.now()+serverOffset,span=Number($('window').value)*1000,field=Number($('units').value);
for(let ch=0;ch<2;ch++){
const rows=history[selected[ch]].filter(r=>r[1]>=now-span),last=history[selected[ch]].at(-1),base=baseline?.[selected[ch]]?.[field]??0;
const canvas=$('plot'+ch),dpr=window.devicePixelRatio||1,w=canvas.clientWidth,h=300;canvas.width=Math.round(w*dpr);canvas.height=h*dpr;
const ctx=canvas.getContext('2d');ctx.scale(dpr,dpr);ctx.clearRect(0,0,w,h);ctx.font='11px system-ui';
let lo=Infinity,hi=-Infinity;for(const r of rows){const y=r[field]===null?NaN:r[field]-base;if(Number.isFinite(y)){lo=Math.min(lo,y);hi=Math.max(hi,y);}}
const range=Number.isFinite(lo)?hi-lo:0;if(!Number.isFinite(lo)){lo=0;hi=1;}const pad=Math.max((hi-lo)*.15,field===4?2:Math.abs(hi)*1e-6,1e-8);lo-=pad;hi+=pad;
const left=76,right=w-12,top=15,bottom=h-30;
for(let i=0;i<=4;i++){const y=top+(bottom-top)*i/4;ctx.strokeStyle='#2c3a50';ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(right,y);ctx.stroke();ctx.fillStyle='#a6b3c8';ctx.fillText(number(hi-(hi-lo)*i/4,field===4?0:6),2,y+4);ctx.fillText('-'+Math.round(span/1000*(1-i/4))+'s',left+(right-left)*i/4-10,h-7);}
ctx.strokeStyle=colors[ch];ctx.lineWidth=1.4;ctx.beginPath();let previous=null;
for(const r of rows){const y=r[field]===null?NaN:r[field]-base;if(!Number.isFinite(y))continue;const x=left+(r[1]-(now-span))/span*(right-left),py=bottom-(y-lo)/(hi-lo)*(bottom-top);if(previous===null||r[1]-previous>1000)ctx.moveTo(x,py);else ctx.lineTo(x,py);previous=r[1];}ctx.stroke();
if(last){const age=Math.max(0,(now-last[1])/1000),fresh=age<2;$('value'+ch).textContent=number(last[field]===null?NaN:last[field]-base,field===4?0:7)+(fresh?'':' · STALE');
const c=info?.calibration,actual=selected[ch],identity=c&&(actual===0?c[0]===1&&c[1]===0:actual===1&&c.slice(4,11).every((v,i)=>v===(i===1?1:0)));
$('meta'+ch).textContent='ADC '+number(last[4],0)+' · '+number(last[5],6)+' V · age '+age.toFixed(1)+' s\n'+'Visible range '+number(range,field===4?0:7)+' · '+rows.length+' sampled SWD records'+(identity?' · calibration is identity':'');
}else{$('value'+ch).textContent='No samples';}
}}
async function poll(){try{const response=await fetch('/data?since='+sequence);if(!response.ok)throw Error('HTTP '+response.status);info=await response.json();serverOffset=info.server_ms-Date.now();
for(const row of info.rows){history[row[3]].push(row);sequence=Math.max(sequence,row[0]);}
for(let ch=0;ch<10;ch++){const cutoff=info.server_ms-125000;while(history[ch].length&&history[ch][0][1]<cutoff)history[ch].shift();}
const sd=info.sd??{state:'UNKNOWN'},age=sd.last_progress_age_s;
$('sdstate').textContent=sd.state;
$('sdstate').style.color=sd.state==='WRITING'?'#64e3b0':sd.state==='UNKNOWN'?'#a6b3c8':'#ffcd80';
$('sdinfo').textContent=sd.state==='UNKNOWN'?(sd.reason??'SD counters unavailable'):
 'Ready '+sd.ready+' · init stage '+sd.init_stage+' · '+(sd.rate_hz===null?'rate pending':number(sd.rate_hz,1)+' records/s')+
 ' · last progress '+(age===null?'not observed':number(age,1)+' s ago')+' · raw written '+number(sd.raw_written,0)+
 ' / flushed '+number(sd.raw_flushed,0)+' · CSV written '+number(sd.csv_written,0)+' · PT written '+number(sd.pressure_written,0)+
 ' / flushed '+number(sd.pressure_flushed,0)+' · protected drops raw '+number(sd.raw_protected_drops,0)+' / CSV '+number(sd.csv_protected_drops,0)+
 ' / PT '+number(sd.pressure_drops,0)+' · errors write '+number(sd.write_errors,0)+' / init '+number(sd.init_failures,0)+
 ' / mount status '+number(sd.mount_status,0)+(sd.reset_detected?' · counters restarted':'');
$('status').textContent=info.status;$('status').style.color=info.connected?'#64e3b0':'#ffcd80';$('csv').textContent='Recording all captured samples to: '+info.csv;draw();
}catch(error){$('status').textContent='Plot connection lost: '+error.message;draw();}setTimeout(poll,250);}
$('baseline').onclick=()=>{if(selected.some(ch=>!history[ch].length)){$('mode').textContent='Wait for samples from both channels before setting a baseline.';return;}baseline=history.map(rows=>rows.at(-1)?.slice()??null);$('mode').textContent='Change from plot baseline · each plot scales independently';draw();};
$('absolute').onclick=()=>{baseline=null;$('mode').textContent='Absolute values · each plot scales independently';draw();};
$('pause').onclick=()=>{paused=!paused;$('pause').textContent=paused?'Resume plot':'Pause plot';if(!paused)draw();};
const names={0:'1000 kg · CH0 / P7',1:'50 kg AMP2 · CH1 / P8',6:'Chamber PT · CH6 / P4 pin3',7:'HVAIN2 · CH7 / P4 pin4',9:'50 kg P6 differential · CH9'};
for(let panel=0;panel<2;panel++){const select=$('channel'+panel);for(const ch of channels){const option=document.createElement('option');option.value=ch;option.textContent=names[ch]??('Auxiliary ADC · CH'+ch);select.append(option);}select.value=selected[panel];select.onchange=()=>{selected[panel]=Number(select.value);draw();};}
$('units').onchange=draw;$('window').onchange=draw;window.addEventListener('resize',draw);
$('download').onclick=()=>{const cutoff=Date.now()+serverOffset-Number($('window').value)*1000,rows=history.flat().filter(r=>r[1]>=cutoff).sort((a,b)=>a[0]-b[0]);
const text='sequence,host_unix_ms,board_ms,channel,adc_code,voltage_v,raw_value,converted_value,temperature_c\n'+rows.map(r=>r.join(',')).join('\n');const url=URL.createObjectURL(new Blob([text],{type:'text/csv'}));const a=document.createElement('a');a.href=url;a.download='daq-loadcells.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};poll();
</script></html>'''


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--elf', type=Path, default=ROOT / 'build/Release_Script/DAQ-Board.elf',
                        help='Exact ELF used to flash the running DAQ')
    parser.add_argument('--port', type=int, default=8765, help='Local browser GUI port')
    parser.add_argument('--tcl-port', type=int, default=6666, help='Local OpenOCD TCL port')
    parser.add_argument('--attach', action='store_true', help='Use an existing OpenOCD server')
    parser.add_argument('--probe', help='ST-Link serial number (optional with one probe)')
    parser.add_argument('--openocd', default='openocd', help='OpenOCD executable')
    parser.add_argument('--no-browser', action='store_true', help='Print the URL without opening it')
    parser.add_argument('--rate', type=float, default=20, help='Debug-buffer polls per second (1–30)')
    parser.add_argument('--csv', type=Path, help='CSV output path (must not already exist)')
    args = parser.parse_args()
    if not 1 <= args.rate <= 30:
        parser.error('--rate must be between 1 and 30')
    if not args.elf.is_file():
        parser.error(f'Firmware ELF not found: {args.elf}')
    args.elf = args.elf.resolve()
    filename = args.csv or ROOT / 'build/loadcell-plots' / (datetime.now().strftime('%Y%m%d-%H%M%S') + '.csv')
    filename = filename.resolve()
    if filename.exists():
        parser.error(f'CSV already exists: {filename}')
    capture, process, log, server = Capture(filename), None, None, None
    stop = threading.Event()
    try:
        if not args.attach:
            if not shutil.which(args.openocd):
                parser.error('OpenOCD is not installed or is not on PATH')
            log = filename.with_suffix('.openocd.log').open('w')
            command = [args.openocd, '-f', 'interface/stlink.cfg', '-c', 'transport select hla_swd']
            if args.probe:
                if not re.fullmatch(r'[A-Za-z0-9]+', args.probe):
                    parser.error('Probe serial must contain only letters and digits')
                command += ['-c', f'adapter serial {args.probe}']
            command += ['-f', 'target/stm32u5x.cfg', '-c', 'reset_config none',
                        '-c', 'adapter speed 4000', '-c', 'bindto 127.0.0.1',
                        '-c', f'tcl_port {args.tcl_port}', '-c', 'telnet_port disabled',
                        '-c', 'gdb_port disabled', '-c', 'init']
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            time.sleep(1)
            if process.poll() is not None:
                raise RuntimeError(f'OpenOCD failed; see {filename.with_suffix(".openocd.log")} '
                                   '(use --attach if a server is already running)')
        server = ThreadingHTTPServer(('127.0.0.1', args.port), serve(capture))
        worker = threading.Thread(target=monitor, args=(capture, args, stop), daemon=True)
        worker.start()
        url = f'http://127.0.0.1:{args.port}'
        print(f'Load-cell plot: {url}\nCSV: {filename}\nPress Ctrl+C to stop.', flush=True)
        if not args.no_browser:
            webbrowser.open(url)
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError) as exc:
        parser.exit(1, f'{exc}\n')
    finally:
        stop.set()
        if server:
            server.server_close()
        if 'worker' in locals():
            worker.join(timeout=12)
        if process:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if log:
            log.close()
        capture.file.close()


if __name__ == '__main__':
    main()
