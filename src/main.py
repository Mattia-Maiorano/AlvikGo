from arduino_alvik import ArduinoAlvik # type: ignore
from machine import SoftI2C, Pin
import network
import uasyncio as asyncio
import time
import json
import math
import gc  # <--- Added for memory optimization and profiling

# ---------------------------------------------------------
# DEBUGGING & PROFILING
# ---------------------------------------------------------

async def heartbeat_loop():
    """A clean pulse to see if the async scheduler itself is freezing."""
    while True:
        t1 = time.ticks_ms()
        await asyncio.sleep(1.0)
        delta = time.ticks_diff(time.ticks_ms(), t1)
        if delta > 1200: # Allow 200ms of natural jitter
            print(f"[\u26a0\ufe0f PROFILER] async scheduler froze! Expected 1000ms, took {delta}ms")

# ---------------------------------------------------------
# HARDWARE INITIALIZATION & SAFE DEBOUNCERS
# ---------------------------------------------------------
alvik = ArduinoAlvik()
alvik.begin()

_is_braked = False
_current_lin = 0.0
_target_lin = 0.0
_ramp_task = None
_current_led = (-1, -1, -1)
VELOCITY_SCALING = 10.0

def safe_brake():
    global _is_braked, _current_lin, _target_lin, _ramp_task
    
    if _ramp_task:
        _ramp_task.cancel() 
        _ramp_task = None
        
    _current_lin = 0.0
    _target_lin = 0.0
        
    if not _is_braked:
        alvik.brake()
        _is_braked = True

def safe_velocity(lin_cm_s):
    global _target_lin, _ramp_task
    target_speed = lin_cm_s * VELOCITY_SCALING
    
    if target_speed != _target_lin:
        _target_lin = target_speed
        
        if _ramp_task:
            _ramp_task.cancel()
            
        _ramp_task = asyncio.create_task(_ramp_velocity_task(target_speed))

async def _ramp_velocity_task(target_speed):
    global _current_lin, _is_braked
    _is_braked = False
    
    steps = 10
    delay = 0.02 
    
    start_speed = _current_lin
    speed_diff = target_speed - start_speed
    
    try:
        for i in range(1, steps + 1):
            new_speed = start_speed + (speed_diff * (i / steps))
            alvik.set_wheels_speed(new_speed, new_speed)
            _current_lin = new_speed
            await asyncio.sleep(delay)
            
        alvik.set_wheels_speed(target_speed, target_speed)
        _current_lin = target_speed
    except asyncio.CancelledError:
        pass

def safe_leds(r, g, b):
    global _current_led
    if _current_led != (r, g, b):
        alvik.left_led.set_color(r, g, b)
        alvik.right_led.set_color(r, g, b)
        _current_led = (r, g, b)

def animate_breathing_led(r_max, g_max, b_max, speed=1000):
    global _current_led
    t = time.ticks_ms()
    on = (t // speed) % 2

    r = int(r_max * on)
    g = int(g_max * on)
    b = int(b_max * on)
    
    alvik.left_led.set_color(r, g, b)
    alvik.right_led.set_color(r, g, b)
    _current_led = (-1, -1, -1)

async def async_rotate(angle):
    safe_brake()
    await asyncio.sleep(0.15) 
    
    sys_log(f"Executing asynchronous rotation: {angle}°", "HW")
    alvik.rotate(angle, 'deg', blocking=False)
    await asyncio.sleep(1.5) 
    
    safe_brake()
    await asyncio.sleep(0.15) 
    
    global _current_led
    _current_led = (-1, -1, -1)

class ToFFilter:
    def __init__(self, hits_required=3):
        self.size = hits_required
        self.history = [[300.0] * self.size for _ in range(3)]
        self.index = 0

    def update(self, l, c, r):
        self.history[0][self.index] = l
        self.history[1][self.index] = c
        self.history[2][self.index] = r
        self.index = (self.index + 1) % self.size
        return tuple(max(self.history[i]) for i in range(3))

sensor_filter = ToFFilter(hits_required=3)

def get_tof():
    try:
        d_l, _, d_c, _, d_r = alvik.get_distance() 
        
        ll = 300.0 if d_l <= 0 else float(d_l)
        c  = 300.0 if d_c <= 0 else float(d_c)
        rr = 300.0 if d_r <= 0 else float(d_r)
        
        return sensor_filter.update(ll, c, rr)
    except Exception as e:
        sys_log(f"ToF Read Error: {e}", "WARN")
        return 300.0, 50.0, 300.0

# ---------------------------------------------------------
# SYSTEM CONFIGURATION & LOGGER
# ---------------------------------------------------------
WIFI_SSID = "iPh0n3"
WIFI_PASS = "Facile10"

SPEED_CM_S = 15.0
ENGAGE_MAX_CM = 20.0     
FOLLOW_MAX_CM = 70.0  
CHECKPOINT_TRIGGER = 12.0
CHECKPOINT_CLEAR = 30.0  
OBSTACLE_TRIGGER = 15.0

STATE_IDLE = 0
STATE_RECORD = 1
STATE_SEEK = 2
current_state = STATE_IDLE

current_tof_data = (300.0, 50.0, 300.0)
current_nfc_status = "INITIALIZING"
current_rec_substate = 'WAIT_TARGET'
current_target_dist = 0.0
current_seek_accum = 0.0

system_logs = []
current_mem_free = 0  # Global tracker for UI metrics
current_mem_alloc = 0 # Global tracker for UI metrics

LOG_COLORS = {
    "INFO": "#ffffff",   "REC": "#3498db",    "SEEK": "#2ecc71",   
    "MEM": "#d33682",    "HW": "#e67e22",     "WARN": "#e74c3c",   "TOF": "#f1c40f",
}

def sys_log(msg, category="INFO"):
    print(f"[{category}] {msg}")
    color = LOG_COLORS.get(category, "#ffffff")
    html_msg = f'<span style="color:{color}">[{category}] {msg}</span><br>'
    system_logs.append(html_msg)
    if len(system_logs) > 100:  # Reduced log buffer size slightly to save heap memory
        system_logs.pop(0)

# ---------------------------------------------------------
# THE MEMORY (PathRecorder)
# ---------------------------------------------------------
class PathRecorder:
    def __init__(self):
        self.memory = []
        self.distance_in_direction = 0.0
        self.heading_deg = 0 

    def add_drive(self, dist):
        self.distance_in_direction += dist
        if self.memory and self.memory[-1]['type'] == 'drive':
            self.memory[-1]['value'] += dist
        else:
            self.memory.append({'type': 'drive', 'value': dist})

    def add_turn(self, angle):
        self.memory.append({'type': 'turn', 'value': angle})
        self.heading_deg = (self.heading_deg + angle) % 360
        self.distance_in_direction = 0.0
        sys_log(f"Logged Turn: {angle}°. New Heading: {self.heading_deg}°", "MEM")

    def add_checkpoint(self, l, c, r):
        self.memory.append({
            'type': 'checkpoint', 'tof': (l, c, r),
            'direction_dist': self.distance_in_direction, 'heading': self.heading_deg
        })
        sys_log(f"Checkpoint Saved! Distance on edge: {self.distance_in_direction:.1f}cm", "MEM")

    def clear(self):
        self.memory = []
        self.distance_in_direction = 0.0
        self.heading_deg = 0
        sys_log("Memory Cleared.", "MEM")

path = PathRecorder()

# ---------------------------------------------------------
# ASYNC NFC DRIVER & LOOP
# ---------------------------------------------------------
class AsyncMiniPN532:
    def __init__(self, scl_pin, sda_pin):
        self.i2c = SoftI2C(scl=Pin(scl_pin, pull=Pin.PULL_UP), sda=Pin(sda_pin, pull=Pin.PULL_UP), freq=100000, timeout=1000)
        self.addr = 0x24

    async def send_cmd(self, cmd, expected_payload_len):
        length = len(cmd) + 1
        buf = bytearray([0x00, 0x00, 0xFF, length, (~length + 1) & 0xFF, 0xD4])
        buf.extend(cmd)
        buf.append((256 - ((0xD4 + sum(cmd)) & 0xFF)) & 0xFF)
        buf.append(0x00)
        self.i2c.writeto(self.addr, buf)
        
        if not await self._wait_ready(): return None
        self.i2c.readfrom(self.addr, 7) 
        
        if not await self._wait_ready(): return None
        total_len = expected_payload_len + 9
        res = self.i2c.readfrom(self.addr, total_len)
        return res[8:8+expected_payload_len]

    async def _wait_ready(self):
        for _ in range(40):
            try:
                if self.i2c.readfrom(self.addr, 1)[0] == 0x01:
                    return True
            except:
                pass
            await asyncio.sleep(0.05) 
        return False

    async def initialize(self):
        try: self.i2c.readfrom(self.addr, 1)
        except: pass
        await asyncio.sleep(0.1)
        await self.send_cmd([0x14, 0x01, 0x14, 0x01], 1)
        await asyncio.sleep(0.05)
        await self.send_cmd([0x32, 0x05, 0xFF, 0x01, 0x02], 1)
        
    async def scan_for_tag(self, debug=False):
        if debug:
            print("[PN532] RF Field ON: InListPassiveTarget (0x4A) active listening...")

        res = await self.send_cmd([0x4A, 0x01, 0x00], 11)
        if res and res[0] == 1: 
            uid_len = res[4]
            uid = res[5:5+uid_len]
            return "".join([f"{x:02X}" for x in uid]) 
        return None

nfc_ready = False
nfc_reader = AsyncMiniPN532(scl_pin=12, sda_pin=11)

async def nfc_loop():
    global current_state, current_rec_substate, nfc_ready, current_nfc_status

    did_log_disconnect = False
    last_rf_log = 0
    backoff = 0.5

    while True:
        is_actively_moving = (current_state == STATE_SEEK) or \
                             (current_state == STATE_RECORD and current_rec_substate == 'FOLLOW')
        
        if is_actively_moving:
            current_nfc_status = "PAUSED (MOVING)"
            await asyncio.sleep(0.5) 
            continue

        show_rf_debug = False
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, last_rf_log) >= 5000:
            show_rf_debug = True
            last_rf_log = current_time

        if not nfc_ready:
            try:
                current_nfc_status = "DISCONNECTED"
                nfc_reader.i2c.readfrom(nfc_reader.addr, 1) 
                await nfc_reader.initialize()
                nfc_ready = True
                backoff = 0.5
                sys_log("NFC Hardware Linked", "HW")
            except OSError as e:
                if not did_log_disconnect:
                    sys_log(f"NFC Module Timeout/Disconnected: {e}", "WARN")
                    did_log_disconnect = True
                await asyncio.sleep(backoff)
                backoff = min(4.0, backoff * 2.0)
                continue
            except Exception as e:
                sys_log(f"NFC Init Error: {e}", "WARN")
                await asyncio.sleep(backoff)
                backoff = min(4.0, backoff * 2.0)
                continue

        current_nfc_status = "SCANNING..."

        try:
            tag_uid = await nfc_reader.scan_for_tag(debug=show_rf_debug)
            if tag_uid:
                sys_log(f"NFC Tap Detected: {tag_uid}", "INFO")
                
                if current_state == STATE_RECORD:
                    filename = f"route_{tag_uid}.json"
                    with open(filename, 'w') as f:
                        json.dump(path.memory, f)
                    sys_log(f"Path saved to card '{tag_uid}'", "MEM")
                    current_state = STATE_IDLE
                    
                elif current_state == STATE_IDLE:
                    filename = f"route_{tag_uid}.json"
                    try:
                        with open(filename, 'r') as f:
                            path.memory = json.load(f)
                        sys_log(f"Path loaded from card '{tag_uid}'", "MEM")
                        current_state = STATE_SEEK
                    except OSError:
                        sys_log(f"Unregistered Tag: No route found for '{tag_uid}'", "WARN")
                        await gesture_not_found()
                        
                await asyncio.sleep(2.0)
            else:
                await asyncio.sleep(1.0)
                
        except OSError as e:
            nfc_ready = False
            sys_log(f"NFC Module Disconnected: {e}", "WARN")
            await asyncio.sleep(1)

# ---------------------------------------------------------
# WEB SERVER
# ---------------------------------------------------------
HTML_PAGE = """<!DOCTYPE html><html><head><title>Alvik UI</title>
<style>
body{font-family:monospace;background:#111;color:#eee;font-size:14px;padding:10px;margin:0}
button{width:100%;padding:15px;background:#2980b9;color:#fff;font-weight:bold;border:none;cursor:pointer;border-radius:5px}
h3{margin:10px 0;color:#3498db}
p{margin:5px 0}
.w{color:#f39c12;font-weight:bold}
#log{height:200px;overflow-y:auto;background:#000;padding:10px;border-radius:5px;font-size:12px;border:1px solid #333}
</style>
<script>
const g=v=>v>=300?"---":v.toFixed(1);
let b=0,hist=[];
const draw=(alloc,free)=>{
    let c=document.getElementById('mc'),ctx=c.getContext('2d'),w=c.width,h=c.height,tot=alloc+free;
    ctx.clearRect(0,0,w,h);
    let pct=tot>0?alloc/tot:0;
    hist.push(pct);if(hist.length>50)hist.shift();
    ctx.strokeStyle='#222';
    for(let i=1;i<4;i++){ctx.beginPath();ctx.moveTo(0,h*i/4);ctx.lineTo(w,h*i/4);ctx.stroke();}
    ctx.fillStyle='rgba(52,152,219,0.15)';ctx.beginPath();ctx.moveTo(0,h);
    hist.forEach((p,i)=>ctx.lineTo((i/(hist.length-1||1))*w,h-(p*(h-8)+4)));
    ctx.lineTo(w,h);ctx.fill();
    ctx.strokeStyle='#3498db';ctx.lineWidth=1.5;ctx.beginPath();
    hist.forEach((p,i)=>{
        let x=(i/(hist.length-1||1))*w,y=h-(p*(h-8)+4);
        if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);
    });
    ctx.stroke();
};
setInterval(()=>{
    if(b)return;b=1;
    fetch('/api/telemetry').then(r=>r.json()).then(d=>{
        document.getElementById('st').innerText=d.state;
        document.getElementById('sb').innerText=d.substate;
        document.getElementById('hz').innerText=d.fps + " Hz";
        document.getElementById('nv').innerText=d.target_dist>0?d.driven_dist.toFixed(1)+'/'+d.target_dist.toFixed(1):'N/A';
        document.getElementById('nd').innerText=d.nodes;
        
        let nfc=document.getElementById('nf');
        nfc.innerText=d.nfc;
        nfc.style.color=d.nfc.includes("SCAN")?"#3498db":d.nfc.includes("DIS")?"#e74c3c":"#aaa";
        
        document.getElementById('tl').innerText=g(d.tof[0]);
        document.getElementById('tc').innerText=g(d.tof[1]);
        document.getElementById('tr').innerText=g(d.tof[2]);
        
        let alloc=d.alloc_mem||0,free=d.free_mem||0,tot=alloc+free;
        document.getElementById('ma').innerText=(alloc/1024).toFixed(1)+" KB";
        document.getElementById('mf').innerText=(free/1024).toFixed(1)+" KB";
        let pct=tot>0?(alloc/tot)*100:0;
        document.getElementById('mb').style.width=pct.toFixed(1)+"%";
        draw(alloc,free);
        
        let l=document.getElementById('log');
        let s=l.scrollHeight-l.clientHeight<=l.scrollTop+10;
        l.innerHTML=d.logs;if(s)l.scrollTop=l.scrollHeight;
        b=0;
    }).catch(()=>b=0);
},1000);
</script></head><body>

<button onclick="fetch('/api/record')">START RECORDING</button>

<h3><span id="st">IDLE</span> | <span id="sb" style="color:#aaa;font-size:16px">WAIT_TARGET</span> <span style="float:right;color:#bdc3c7;font-size:16px"><span id="hz">--</span></span></h3>

<p>ToF (cm) &rarr; Left: <span id="tl" class="w">--</span> | Center: <span id="tc" class="w">--</span> | Right: <span id="tr" class="w">--</span></p>
<p>Seek Nav &rarr; Dist: <span id="nv" class="w">--</span> | Nodes: <span id="nd" class="w">0</span></p>
<p>NFC Stat &rarr; <span id="nf" class="w">--</span></p>
<p>RAM Monitor &rarr; Alloc: <span id="ma" class="w">--</span> | Free: <span id="mf" class="w">--</span></p>
<div style="background:#222;border:1px solid #333;border-radius:3px;height:8px;margin:5px 0;overflow:hidden">
  <div id="mb" style="background:#3498db;height:100%;width:0%;transition:width 0.3s"></div>
</div>
<canvas id="mc" width="320" height="50" style="width:100%;height:50px;background:#050505;border:1px solid #222;border-radius:3px;margin:5px 0 10px 0"></canvas>

<div id="log"></div>

</body></html>"""

async def serve_client(reader, writer):
    global current_state, SPEED_CM_S, ENGAGE_MAX_CM, FOLLOW_MAX_CM, \
           CHECKPOINT_TRIGGER, CHECKPOINT_CLEAR, OBSTACLE_TRIGGER, current_tof_data, current_nfc_status, current_mem_free, current_mem_alloc

    try:
        request_line = await reader.readline()
        if not request_line: return
        
        async def read_headers():
             while True:
                line = await reader.readline()
                if not line or line == b'\r\n': break

        try:
            await asyncio.wait_for(read_headers(), timeout=1.0)
        except asyncio.TimeoutError:
            return

        req = request_line.decode('utf-8').split()
        if len(req) > 1:
            route = req[1]  
            
            if route == '/api/telemetry':
                state_str = "IDLE" if current_state == 0 else "RECORDING" if current_state == 1 else "SEEKING"

                try: 
                    ll, c, rr = current_tof_data
                except: 
                    ll, c, rr = (300.0, 300.0, 300.0)

                telemetry_data = {
                    "state": state_str,
                    "substate": current_rec_substate, 
                    "tof": [ll, c, rr],
                    "target_dist": current_target_dist, 
                    "driven_dist": current_seek_accum,  
                    "nodes": len(path.memory),
                    "nfc": current_nfc_status,
                    "fps": round(current_fps, 1),
                    "free_mem": current_mem_free,
                    "alloc_mem": current_mem_alloc,
                    "logs": "".join(system_logs)
                }
                
                writer.write(b"HTTP/1.0 200 OK\r\nConnection: close\r\nContent-Type: application/json\r\n\r\n")
                await writer.drain() 
                writer.write(json.dumps(telemetry_data).encode('utf-8'))
                await writer.drain() 
                return
                
            elif route == '/api/record':
                path.clear()
                current_state = STATE_RECORD
                sys_log("Switched to RECORD mode.", "REC")
                writer.write(b"HTTP/1.0 200 OK\r\n\r\nOK")
                await writer.drain()
                return
                
            else:
                writer.write(b"HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n")
                writer.write(HTML_PAGE.encode('utf-8'))
                await writer.drain()

    except OSError as e:
        if e.args[0] != 104: sys_log(f"Socket OSError: {e}", "WARN")
    except Exception as e:
        print(f"[WEB CRASH] Server error: {e}")
    finally:
        try:
            writer.close()
            await writer.wait_closed()
        except: pass

async def start_web_server():
    server = await asyncio.start_server(serve_client, "0.0.0.0", 80)
    while True: await asyncio.sleep(3600)

# -------------------------------------------------
# VISUAL FEEDBACK GESTURES
# -------------------------------------------------
async def gesture_nod():
    global _is_braked
    safe_brake()
    alvik.left_led.set_color(100, 100, 100) 
    alvik.right_led.set_color(100, 100, 100)
    
    _is_braked = False
    alvik.set_wheels_speed(15, 15)
    await asyncio.sleep(0.15)
    
    safe_brake()
    await asyncio.sleep(0.05) 
    
    _is_braked = False
    alvik.set_wheels_speed(-15, -15)
    await asyncio.sleep(0.15)
    safe_brake()
    
async def gesture_not_found():
    safe_brake()
    for _ in range(2):
        safe_leds(120, 100, 0)
        await asyncio.sleep(0.15)
        safe_leds(0, 0, 0)
        await asyncio.sleep(0.15)

async def gesture_turn_warning(angle):
    safe_brake()
    if angle > 0: 
        alvik.left_led.set_color(0, 120, 0)
        alvik.right_led.set_color(120, 0, 0)
    else:         
        alvik.left_led.set_color(120, 0, 0)
        alvik.right_led.set_color(0, 120, 0)
    await asyncio.sleep(1.0) 

async def gesture_blocked():
    safe_brake()
    for _ in range(3): 
        safe_leds(120, 0, 0)
        await asyncio.sleep(0.2)
        safe_leds(0, 0, 0)
        await asyncio.sleep(0.2)

async def gesture_arrival():
    safe_brake()
    alvik.rotate(360, 'deg', blocking=False)
    await asyncio.sleep(0.1) 
    for _ in range(8):
        safe_leds(0, 100, 0)
        await asyncio.sleep(0.2)
        safe_leds(0, 0, 0)
        await asyncio.sleep(0.2)
    safe_brake()

# ---------------------------------------------------------
# CORE STATE MACHINE 
# ---------------------------------------------------------

current_fps = 0.0

async def robot_loop():
    global current_state, current_rec_substate, current_target_dist, current_seek_accum, current_fps, current_mem_free, current_mem_alloc
    
    last_ticks = time.ticks_ms()
    last_log_ticks = 0
    rec_substate = 'WAIT_TARGET' 
    seek_index = 0
    seek_accum = 0.0

    frame_count = 0
    last_fps_ticks = time.ticks_ms()

    while True:
        try: 
            current_ticks = time.ticks_ms()
            dt = time.ticks_diff(current_ticks, last_ticks) / 1000.0
            last_ticks = current_ticks

            frame_count += 1
            fps_dt = time.ticks_diff(current_ticks, last_fps_ticks)

            if fps_dt >= 1000:
                current_fps = frame_count / (fps_dt / 1000.0)
                current_mem_free = gc.mem_free()
                current_mem_alloc = gc.mem_alloc()
                
                # Dynamic terminal monitoring logging strain stats
                print(f"[PERF] Loop Frequency: {current_fps:.1f} Hz | Alloc: {current_mem_alloc}B | Free: {current_mem_free}B")
                frame_count = 0
                last_fps_ticks = current_ticks

            global current_tof_data
            current_tof_data = get_tof()

            ll, c, rr = current_tof_data

            do_periodic_log = False
            if time.ticks_diff(current_ticks, last_log_ticks) >= 1000:
                do_periodic_log = True
                last_log_ticks = current_ticks

            if current_state == STATE_IDLE:
                safe_brake()
                animate_breathing_led(0, 100, 100)
                
                rec_substate = 'WAIT_TARGET'
                seek_index = 0
                seek_accum = 0.0

                try:
                    if hasattr(alvik, 'get_touch_center') and alvik.get_touch_center():
                        path.clear()
                        current_state = STATE_RECORD
                        sys_log("Switched to RECORD mode.", "REC")
                except Exception as e:
                    pass
                
            elif current_state == STATE_RECORD:
                if rec_substate == 'WAIT_TARGET':
                    safe_leds(0, 100, 100)
                    
                    if c < ENGAGE_MAX_CM:
                        sys_log(f"ENGAGE | Center ({c:.1f}cm) triggered.", "REC")
                        if len(path.memory) == 0: await gesture_nod()
                        rec_substate = 'FOLLOW'

                    elif ll < ENGAGE_MAX_CM:
                        sys_log(f"ENGAGE | Left ({ll:.1f}cm). Pivoting 90°.", "REC")
                        if len(path.memory) == 0: await gesture_nod()
                        await async_rotate(90)
                        path.add_turn(90)
                        rec_substate = 'FOLLOW'

                    elif rr < ENGAGE_MAX_CM:
                        sys_log(f"ENGAGE | Right ({rr:.1f}cm). Pivoting -90°.", "REC")
                        if len(path.memory) == 0: await gesture_nod()
                        await async_rotate(-90)
                        path.add_turn(-90)
                        rec_substate = 'FOLLOW'

                elif rec_substate == 'FOLLOW':
                    safe_leds(0, 0, 120)
                    if 0 < c < CHECKPOINT_TRIGGER:
                        safe_brake()
                        safe_leds(120, 0, 0)
                        sys_log(f"NODE | Target reached ({c:.1f}cm). Saving...", "REC")
                        await asyncio.sleep(0.5) 
                        snap_ll, snap_c, snap_rr = current_tof_data
                        path.add_checkpoint(snap_ll, snap_c, snap_rr)
                        safe_leds(0, 120, 0) 
                        rec_substate = 'WAIT_CLEAR'
                    else:
                        if c <= FOLLOW_MAX_CM:
                            if do_periodic_log: sys_log(f"FOLLOW | Tracking -> C:{c:.1f}", "TOF")
                            safe_velocity(SPEED_CM_S)
                            path.add_drive(SPEED_CM_S * dt)
                        else:
                            safe_brake() 
                            if do_periodic_log: sys_log("FOLLOW | Target lost.", "WARN")

                elif rec_substate == 'WAIT_CLEAR':
                    safe_brake()
                    min_dist = min(ll, c, rr)
                    if min_dist > CHECKPOINT_CLEAR:
                        sys_log("CLEAR | Area cleared.", "REC")
                        rec_substate = 'WAIT_TARGET'

            elif current_state == STATE_SEEK:
                if seek_index >= len(path.memory):
                    if do_periodic_log: sys_log("Playback complete.", "SEEK")
                    await gesture_arrival() 
                    current_state = STATE_IDLE
                    continue

                node = path.memory[seek_index]

                if node['type'] == 'drive':
                    target_dist = node['value']
                    
                    if c < OBSTACLE_TRIGGER:
                        await gesture_blocked() 
                        if do_periodic_log: sys_log(f"BLOCKED | Obstacle detected!", "WARN")
                    else:
                        safe_leds(0, 120, 0)
                        safe_velocity(SPEED_CM_S) 
                        seek_accum += SPEED_CM_S * dt
                        
                        if seek_accum >= target_dist:
                            safe_brake()
                            seek_accum = 0.0
                            seek_index += 1
                            sys_log(f"NODE | Segment completed.", "SEEK")

                elif node['type'] == 'turn':
                    sys_log(f"TURN | Rotating {node['value']}°", "SEEK")
                    await gesture_turn_warning(node['value']) 
                    await async_rotate(node['value'])
                    seek_index += 1
                    safe_leds(0, 120, 0)

                elif node['type'] == 'checkpoint':
                    safe_brake()
                    safe_leds(0, 0, 120)
                    await asyncio.sleep(1.0) 
                    seek_index += 1
                    safe_leds(0, 120, 0)

        except Exception as e:
            sys_log(f"Robot Loop Crash: {e}", "WARN")
            alvik.brake()
            await asyncio.sleep(1)

        # Proactively flush garbage out of memory at the end of every frame loop iteration
        gc.collect() 
        await asyncio.sleep(0.1) 

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True) 
    try: wlan.disconnect()
    except OSError: pass 
        
    wlan.active(False)
    time.sleep(0.5)
    wlan.active(True)

    try: wlan.config(txpower=10)
    except OSError: print("[SYSTEM] Warning: Could not set Wi-Fi TX power. Continuing with default settings.")

    if not wlan.isconnected():
        print(f"\n[SYSTEM] Connecting to {WIFI_SSID}...")
        wlan.connect(WIFI_SSID, WIFI_PASS)
        timeout = 15
        while not wlan.isconnected() and timeout > 0:
            time.sleep(1)
            timeout -= 1
            print(".", end="")
        print() 
        
    if wlan.isconnected():
        msg = f"Wi-Fi Connected! IP: {wlan.ifconfig()[0]}"
        print(f"[SYSTEM] {msg}")
        sys_log(msg, "INFO")
    else:
        print("[SYSTEM] ERROR: Could not connect to Wi-Fi.")

async def main():
    connect_wifi()

    try:
        if hasattr(alvik, 'dead_reckoning'):
            alvik.dead_reckoning.stop()
            print("[SYSTEM] Optical flow sensor disabled via dead_reckoning.")
        elif hasattr(alvik, 'positioning'):
            alvik.positioning.stop()
            print("[SYSTEM] Optical flow sensor disabled via positioning.")
        else:
            alvik.reset_dead_reckoning() 
            print("[SYSTEM] Resetting/Disabling low-level tracking.")
    except Exception as e:
        print(f"[SYSTEM] Advanced disable failed: {e}. Sensor will remain on defaults.")

    asyncio.create_task(start_web_server())
    asyncio.create_task(nfc_loop())
    asyncio.create_task(heartbeat_loop())
    sys_log("System Booted. Awaiting commands.", "INFO")
    await robot_loop()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        alvik.brake()
        alvik.left_led.set_color(0, 0, 0)
        alvik.right_led.set_color(0, 0, 0)
        print("\nSystem halted by user.")