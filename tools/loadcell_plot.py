#!/usr/bin/env python3
"""Live DAQ load-cell plots using Python's standard library and OpenOCD.

Run: python3 tools/loadcell_plot.py
Reads the existing ADC ring buffer while the MCU runs; never drains the queue,
halts, resets, flashes, tares, or changes calibration. See --help for options.
"""
from __future__ import annotations

import argparse
from collections import deque
import csv
from datetime import datetime
import json
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
CONTEXT_SIZE = 28 + DEPTH * 8
FIELDS = ['sequence', 'host_unix_ms', 'board_ms', 'channel', 'adc_code',
          'voltage_v', 'raw_value', 'calibrated_kg']


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
    required = {'g_mcp3564r': CONTEXT_SIZE, 'g_calibration': 44, 'uwTick': 4,
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


def decode_ring(payload, head_before, head_after, board_ms, seen):
    """Exclude slots the running producer may have changed during the read.

    The queue consumer does not erase entries, so retained slots are useful
    even when the firmware has already drained its queue. This is a sampled
    diagnostic capture, not a guarantee of lossless acquisition.
    """
    if len(payload) != DEPTH * 8 or not (0 <= head_before < DEPTH and 0 <= head_after < DEPTH):
        raise ValueError('Invalid ADC ring layout')
    changing = {(head_before + i) % DEPTH
                for i in range((head_after - head_before) % DEPTH + 1)}
    result = []
    for slot in range(DEPTH):
        if slot in changing:
            continue
        word, stamp = struct.unpack_from('<II', payload, slot * 8)
        channel = word >> 28
        age = (board_ms - stamp) & 0xFFFFFFFF
        if channel > 1 or age > 1000 or (word == 0 and stamp == 0):
            continue
        identity = (word, stamp)
        if seen.get(slot) == identity:
            continue
        seen[slot] = identity
        code = word & 0x01FFFFFF
        if code & 0x01000000:
            code -= 0x02000000
        result.append((age, stamp, channel, code))
    result.sort(key=lambda item: -item[0])
    return result


def f32(value):
    return struct.unpack('<f', struct.pack('<f', value))[0]


def convert(code, channel, calibration):
    # Match the firmware's historical float32 raw scale, used by GS calibration.
    raw = f32(f32(f32(f32(float(code)) * f32(2.2104)) / 16777216) * 2) / 16
    if channel == 0:
        kg = calibration[0] * raw + calibration[1]
    else:
        c = calibration[4:]
        x = raw - c[5]
        kg = ((((c[4] * x + c[3]) * x + c[2]) * x + c[1]) * x + c[0]) - c[6]
    return code * 2.4 / 8388608, raw, kg


class Capture:
    def __init__(self, filename):
        self.lock = threading.Lock()
        self.rows = deque(maxlen=120000)
        self.sequence = 0
        self.status = 'Connecting to DAQ…'
        self.connected = False
        self.calibration = [1., 0., 1., 0., 0., 1., 0., 0., 0., 0., 0.]
        self.counts = [0, 0]
        self.filename = filename
        filename.parent.mkdir(parents=True, exist_ok=True)
        self.file = filename.open('x', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(FIELDS)
        self.file.flush()

    def report(self, message, connected=False):
        with self.lock:
            self.status, self.connected = message, connected
        print(message, flush=True)

    def add(self, points, calibration, counts):
        now = round(time.time() * 1000)
        with self.lock:
            self.calibration, self.counts = list(calibration), counts
            for age, stamp, channel, code in points:
                self.sequence += 1
                voltage, raw, kg = convert(code, channel, calibration)
                row = [self.sequence, now - age, stamp, channel, code, voltage, raw, kg]
                self.rows.append(row)
                self.writer.writerow(row)
            self.file.flush()

    def snapshot(self, since):
        with self.lock:
            rows = [row for row in self.rows if row[0] > since][-5000:]
            return {'status': self.status, 'connected': self.connected,
                    'rows': rows, 'sequence': self.sequence,
                    'calibration': self.calibration, 'published': self.counts,
                    'csv': str(self.filename), 'server_ms': round(time.time() * 1000)}


def monitor(capture, args, stop):
    try:
        symbols, flash = load_elf(args.elf)
    except Exception as exc:
        capture.report(str(exc))
        return
    while not stop.is_set():
        debugger = None
        try:
            debugger = Debugger(args.tcl_port)
            verify_firmware(debugger, symbols, flash, capture.report)
            capture.report('Live · reading the running DAQ', True)
            seen, previous_tick = {}, None
            calibration = None
            next_calibration = 0
            counts = [0, 0]
            while not stop.is_set():
                started = time.monotonic()
                base = symbols['g_mcp3564r']
                before = debugger.read(base, 7)
                payload = debugger.read(base + 28, DEPTH * 2)
                after = debugger.read(base, 7)
                tick = debugger.word(symbols['uwTick'])
                if previous_tick is not None and tick < previous_tick and previous_tick - tick < 0x80000000:
                    raise RuntimeError('DAQ restarted; reconnecting and rechecking firmware')
                previous_tick = tick
                if started >= next_calibration:
                    cal_bytes = debugger.read(symbols['g_calibration'], 11)
                    if cal_bytes == debugger.read(symbols['g_calibration'], 11):
                        calibration = struct.unpack('<11f', cal_bytes)
                    counts = [debugger.word(symbols[name]) for name in
                              ('g_daq_loadcell_publish_ok_count', 'g_daq_kg50_publish_ok_count')]
                    if debugger.command('stm32u5x.cpu curstate') != 'running':
                        raise RuntimeError('DAQ stopped running')
                    next_calibration = started + 1
                # Avoid accepting a buffer that could have wrapped during a slow read.
                if time.monotonic() - started < 0.08 and calibration is not None:
                    points = decode_ring(payload, before[8], after[8], tick, seen)
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
<title>DAQ · Load-cell monitor</title>
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
</style><h1>DAQ load-cell monitor</h1>
<p>Apply a load, hold it, then release it. Watch the selected channel change and return toward its starting value.</p>
<div id="status">Connecting…</div>
<div class="controls"><label>Values <select id="units"><option value="4">Raw ADC counts</option><option value="5">ADC voltage (V)</option><option value="6">Raw telemetry value</option><option value="7">Calibrated kg</option></select></label>
<label>History <select id="window"><option value="15">15 seconds</option><option value="60" selected>60 seconds</option><option value="120">2 minutes</option></select></label>
<button id="baseline">Set plot baseline</button><button id="absolute">Show absolute values</button><button id="pause">Pause plot</button>
<button id="download">Download visible CSV</button></div>
<small id="mode">Absolute values · each plot scales independently</small>
<div class="grid"><section><h2 style="color:#68b7ff">Working cell · CH0 / P7</h2><div class="value" id="value0">—</div><div class="meta" id="meta0">Waiting for samples</div><canvas id="plot0"></canvas></section>
<section><h2 style="color:#64e3b0">50 kg cell · CH1 / P8</h2><div class="value" id="value1">—</div><div class="meta" id="meta1">Waiting for samples</div><canvas id="plot1"></canvas></section></div>
<p id="csv">CSV recording is starting…</p>
<p><small>Baseline affects this plot only. Firmware calibration is unchanged. Calibrated kg uses the coefficients saved on the DAQ; identity defaults are uncalibrated. This is a sampled debug capture, so it may omit ADC conversions.</small></p>
<script>
const history=[[],[]],colors=['#68b7ff','#64e3b0'];let sequence=0,baseline=null,paused=false,info=null,serverOffset=0;
const $=id=>document.getElementById(id);const number=(v,d=4)=>Number.isFinite(v)?v.toLocaleString(undefined,{maximumFractionDigits:d}):'—';
function draw(){if(paused)return;const now=Date.now()+serverOffset,span=Number($('window').value)*1000,field=Number($('units').value);
for(let ch=0;ch<2;ch++){
const rows=history[ch].filter(r=>r[1]>=now-span),last=history[ch].at(-1),base=baseline?baseline[ch][field]:0;
const canvas=$('plot'+ch),dpr=window.devicePixelRatio||1,w=canvas.clientWidth,h=300;canvas.width=Math.round(w*dpr);canvas.height=h*dpr;
const ctx=canvas.getContext('2d');ctx.scale(dpr,dpr);ctx.clearRect(0,0,w,h);ctx.font='11px system-ui';
let lo=Infinity,hi=-Infinity;for(const r of rows){const y=r[field]-base;if(Number.isFinite(y)){lo=Math.min(lo,y);hi=Math.max(hi,y);}}
const range=Number.isFinite(lo)?hi-lo:0;if(!Number.isFinite(lo)){lo=0;hi=1;}const pad=Math.max((hi-lo)*.15,field===4?2:Math.abs(hi)*1e-6,1e-8);lo-=pad;hi+=pad;
const left=76,right=w-12,top=15,bottom=h-30;
for(let i=0;i<=4;i++){const y=top+(bottom-top)*i/4;ctx.strokeStyle='#2c3a50';ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(right,y);ctx.stroke();ctx.fillStyle='#a6b3c8';ctx.fillText(number(hi-(hi-lo)*i/4,field===4?0:6),2,y+4);ctx.fillText('-'+Math.round(span/1000*(1-i/4))+'s',left+(right-left)*i/4-10,h-7);}
ctx.strokeStyle=colors[ch];ctx.lineWidth=1.4;ctx.beginPath();let previous=null;
for(const r of rows){const y=r[field]-base;if(!Number.isFinite(y))continue;const x=left+(r[1]-(now-span))/span*(right-left),py=bottom-(y-lo)/(hi-lo)*(bottom-top);if(previous===null||r[1]-previous>1000)ctx.moveTo(x,py);else ctx.lineTo(x,py);previous=r[1];}ctx.stroke();
if(last){const age=Math.max(0,(now-last[1])/1000),fresh=age<2;$('value'+ch).textContent=number(last[field]-base,field===4?0:7)+(fresh?'':' · STALE');
const c=info?.calibration,identity=c&&(ch===0?c[0]===1&&c[1]===0:c.slice(4).every((v,i)=>v===(i===1?1:0)));
$('meta'+ch).textContent='ADC '+number(last[4],0)+' · '+number(last[5],6)+' V · age '+age.toFixed(1)+' s\n'+'Visible range '+number(range,field===4?0:7)+' · '+rows.length+' captured samples · published '+number(info?.published[ch],0)+(identity?' · calibration is identity':'');
}else{$('value'+ch).textContent='No samples';}
}}
async function poll(){try{const response=await fetch('/data?since='+sequence);if(!response.ok)throw Error('HTTP '+response.status);info=await response.json();serverOffset=info.server_ms-Date.now();
for(const row of info.rows){history[row[3]].push(row);sequence=Math.max(sequence,row[0]);}
for(let ch=0;ch<2;ch++){const cutoff=info.server_ms-125000;while(history[ch].length&&history[ch][0][1]<cutoff)history[ch].shift();}
$('status').textContent=info.status;$('status').style.color=info.connected?'#64e3b0':'#ffcd80';$('csv').textContent='Recording all captured samples to: '+info.csv;draw();
}catch(error){$('status').textContent='Plot connection lost: '+error.message;draw();}setTimeout(poll,250);}
$('baseline').onclick=()=>{if(!history[0].length||!history[1].length){$('mode').textContent='Wait for samples from both channels before setting a baseline.';return;}baseline=history.map(rows=>rows.at(-1).slice());$('mode').textContent='Change from plot baseline · each plot scales independently';draw();};
$('absolute').onclick=()=>{baseline=null;$('mode').textContent='Absolute values · each plot scales independently';draw();};
$('pause').onclick=()=>{paused=!paused;$('pause').textContent=paused?'Resume plot':'Pause plot';if(!paused)draw();};
$('units').onchange=draw;$('window').onchange=draw;window.addEventListener('resize',draw);
$('download').onclick=()=>{const cutoff=Date.now()+serverOffset-Number($('window').value)*1000,rows=history.flat().filter(r=>r[1]>=cutoff).sort((a,b)=>a[0]-b[0]);
const text='sequence,host_unix_ms,board_ms,channel,adc_code,voltage_v,raw_value,calibrated_kg\n'+rows.map(r=>r.join(',')).join('\n');const url=URL.createObjectURL(new Blob([text],{type:'text/csv'}));const a=document.createElement('a');a.href=url;a.download='daq-loadcells.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};poll();
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
