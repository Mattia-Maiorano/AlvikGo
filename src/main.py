from arduino_alvik import ArduinoAlvik
import network
import uasyncio as asyncio
import time
import json
import math

# ---------------------------------------------------------
# HARDWARE INITIALIZATION & SAFE DEBOUNCERS
# ---------------------------------------------------------
alvik = ArduinoAlvik()
alvik.begin()

_is_braked = False
_current_lin = 0.0
_current_led = (-1, -1, -1)
VELOCITY_SCALING = 10.0

def safe_brake():
    global _is_braked, _current_lin
    if not _is_braked:
        alvik.brake()
        _is_braked = True
        _current_lin = 0.0

def safe_velocity(lin_cm_s):
    global _is_braked, _current_lin
    target_speed = lin_cm_s * VELOCITY_SCALING
    if _is_braked or _current_lin != target_speed:
        alvik.set_wheels_speed(target_speed, target_speed)
        _is_braked = False
        _current_lin = target_speed

def safe_leds(r, g, b):
    global _current_led
    if _current_led != (r, g, b):
        alvik.left_led.set_color(r, g, b)
        alvik.right_led.set_color(r, g, b)
        _current_led = (r, g, b)

def animate_breathing_led(r_max, g_max, b_max, speed=0.002):
    """Creates a smooth pulsing effect without blocking the async loop."""
    global _current_led
    
    # Generate a sine wave oscillating between 0.0 and 1.0
    intensity = (math.sin(time.ticks_ms() * speed) + 1.0) / 2.0 
    
    r = int(r_max * intensity)
    g = int(g_max * intensity)
    b = int(b_max * intensity)
    
    # Push directly to hardware
    alvik.left_led.set_color(r, g, b)
    alvik.right_led.set_color(r, g, b)
    
    # Wipe the safety memory so the next solid color command forces an update
    _current_led = (-1, -1, -1)

async def async_rotate(angle):
    safe_brake()
    sys_log(f"Executing asynchronous rotation: {angle}°", "HW")
    alvik.rotate(angle, 'deg', blocking=False)
    await asyncio.sleep(1.5) 
    safe_brake()

    global _current_led
    _current_led = (-1, -1, -1)

raw_tof_data = []

class ToFFilter:
    def __init__(self, hits_required=5):
        self.size = hits_required
        # Create a 5-element array, where each element is a list of 'size' readings
        self.history = [[300.0] * self.size for _ in range(5)]
        self.index = 0

    def update(self, raw_readings):
        """Adds new readings to the buffer and returns the debounced (max) values."""
        for i in range(5):
            self.history[i][self.index] = raw_readings[i]
        
        # Move the circular buffer index forward
        self.index = (self.index + 1) % self.size
        
        # Return the maximum value of the last N frames for each sensor
        return tuple(max(self.history[i]) for i in range(5))

# Initialize the global filter instance
sensor_filter = ToFFilter(hits_required=3)

def get_tof():
    try:
        distances = alvik.get_distance()
        d = [300.0 if x <= 0 else float(x) for x in distances]
        
        # --- STANDARD MAPPING ---
        raw_ll = d[0] # Extreme Left
        raw_cl = d[1] # Center Left
        raw_c  = d[2] # Strict Center
        raw_cr = d[3] # Center Right
        raw_rr = d[4] # Extreme Right
        
        # --- APPLY THE 3-SHOT FILTER ---
        ll, cl, c, cr, rr = sensor_filter.update((raw_ll, raw_cl, raw_c, raw_cr, raw_rr))
        
        return ll, cl, c, cr, rr
    except Exception as e:
        sys_log(f"ToF Read Error: {e}", "WARN")
        return 300.0, 300.0, 50.0, 300.0, 300.0

# ---------------------------------------------------------
# SYSTEM CONFIGURATION & LOGGER
# ---------------------------------------------------------
WIFI_SSID = "iliadbox 2.4"
WIFI_PASS = "QuiciFerrari21"

SPEED_CM_S = 15.0
ENGAGE_MAX_CM = 20.0     
FOLLOW_MAX_CM = 70.0
MID_FOLLOW_MAX_CM = 50.0     
CHECKPOINT_TRIGGER = 12.0
CHECKPOINT_CLEAR = 30.0  
OBSTACLE_TRIGGER = 15.0

STATE_IDLE = 0
STATE_RECORD = 1
STATE_SEEK = 2
current_state = STATE_IDLE

current_rec_substate = 'WAIT_TARGET'
current_target_dist = 0.0
current_seek_accum = 0.0

system_logs = []

LOG_COLORS = {
    "INFO": "#ffffff",   "REC": "#3498db",    "SEEK": "#2ecc71",   
    "MEM": "#d33682",    "HW": "#e67e22",     "WARN": "#e74c3c",   "TOF": "#f1c40f",
}

def sys_log(msg, category="INFO"):
    print(f"[{category}] {msg}")
    color = LOG_COLORS.get(category, "#ffffff")
    html_msg = f'<span style="color:{color}">[{category}] {msg}</span><br>'
    system_logs.append(html_msg)
    if len(system_logs) > 200:
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
        sys_log(f"Checkpoint Saved! Distance on this edge: {self.distance_in_direction:.1f}cm", "MEM")

    def clear(self):
        self.memory = []
        self.distance_in_direction = 0.0
        self.heading_deg = 0
        sys_log("Memory Cleared.", "MEM")

path = PathRecorder()

# ---------------------------------------------------------
# WEB SERVER
# ---------------------------------------------------------
HTML_PAGE = """<!DOCTYPE html><html><head><title>Alvik Telemetry</title>
<style>
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #121212; color: #e0e0e0; margin: 0; padding: 20px; }
    .container { display: grid; grid-template-columns: 1fr 1fr; grid-template-rows: auto auto auto 1fr; gap: 15px; max-width: 950px; margin: 0 auto; height: 95vh;}
    
    .panel { background: #1e1e1e; border: 1px solid #333; border-radius: 8px; padding: 15px; display: flex; flex-direction: column;}
    h2 { margin-top: 0; font-size: 14px; color: #888; text-transform: uppercase; border-bottom: 1px solid #333; padding-bottom: 5px; margin-bottom: 10px; }
    
    .controls { grid-column: 1 / -1; display: flex; justify-content: space-around; gap: 10px; }
    .controls button { flex: 1; font-size: 16px; padding: 12px; border-radius: 8px; cursor: pointer; border: none; font-weight: bold; color: white; }
    .btn-record { background: #2980b9; } .btn-save { background: #c0392b; } .btn-seek { background: #27ae60; }
    
    .status-sector { grid-column: 1; font-size: 24px; font-weight: bold; align-items: center; justify-content: center;}
    .tof-sector { grid-column: 2; align-items: center; }
    
    /* Settings Grid */
    .settings-sector { grid-column: 1 / -1; display: grid; grid-template-columns: 1fr 1fr; gap: 10px 30px; }
    .data-row { display: flex; justify-content: space-between; align-items: center; font-family: monospace; font-size: 13px; color: #aaa;}
    .highlight { color: #f39c12; font-weight: bold; min-width: 25px; display: inline-block; text-align: right;}
    input[type=range] { flex-grow: 1; margin-left: 15px; cursor: pointer; height: 4px; }
    
    .log-sector { grid-column: 1 / -1; overflow-y: auto; font-family: monospace; font-size: 13px; background: #000; border-radius: 4px; padding: 10px; }
</style>

<script>
    function triggerAction(action) { fetch('/api/' + action); }
    
    function updateParam(name, value) {
        document.getElementById('label-' + name).innerText = value;
        fetch('/api/set_param?name=' + name + '&val=' + value);
    }

    function getConeColor(dist) {
        if (dist >= 300) return "#2a2a2a"; 
        if (dist < 12)   return "#e74c3c"; 
        if (dist < 20)   return "#e67e22"; 
        if (dist <= 70)  return "#3498db"; 
        return "#2ecc71";                  
    }

    setInterval(() => {
        fetch('/api/telemetry').then(r => r.json()).then(data => {
            document.getElementById('sys-state').innerText = data.state;
            document.getElementById('sys-substate').innerText = data.substate;
            const zones = ['ll', 'cl', 'c', 'cr', 'rr'];
            for(let i=0; i<5; i++) {
                let d = data.tof[i];
                document.getElementById('cone-'+zones[i]).style.fill = getConeColor(d);
                document.getElementById('val-'+zones[i]).innerText = (d>=300)?"---":d.toFixed(1);
            }
            document.getElementById('nav-dist').innerText = data.target_dist > 0 ? (data.driven_dist.toFixed(1) + '/' + data.target_dist.toFixed(1)) : 'N/A';
            document.getElementById('nav-nodes').innerText = data.nodes;
            const lb = document.getElementById('log-box');
            const sc = lb.scrollHeight - lb.clientHeight <= lb.scrollTop + 10;
            lb.innerHTML = data.logs;
            if (sc) lb.scrollTop = lb.scrollHeight;
        });
    }, 500);
</script>
</head><body>
    <div class="container">
        <div class="controls">
            <button class="btn-record" onclick="triggerAction('record')">RECORD</button>
            <button class="btn-save" onclick="triggerAction('save')">SAVE</button>
            <button class="btn-seek" onclick="triggerAction('seek')">SEEK</button>
        </div>
        
        <div class="panel status-sector">
            <div id="sys-state" style="color: #3498db; font-size: 32px;">IDLE</div>
            <div id="sys-substate" style="font-size: 16px; color: #aaa;">WAIT_TARGET</div>
            <div style="width: 100%; border-top: 1px dashed #333; margin-top: 15px; padding-top: 10px; font-size: 14px;">
                <div class="data-row"><span>Seek:</span> <span id="nav-dist" class="highlight">--</span></div>
                <div class="data-row"><span>Nodes:</span> <span id="nav-nodes" class="highlight">0</span></div>
            </div>
        </div>

        <div class="panel tof-sector">
            <svg viewBox="0 0 200 130" style="width: 100%; max-width: 180px;">
                <rect x="85" y="110" width="30" height="20" rx="3" fill="#888" />
                <polygon id="cone-ll" points="100,110 10,40 40,10" stroke="#222" />
                <polygon id="cone-cl" points="100,110 40,10 75,0" stroke="#222" />
                <polygon id="cone-c"  points="100,110 75,0 125,0" stroke="#222" />
                <polygon id="cone-cr" points="100,110 125,0 160,10" stroke="#222" />
                <polygon id="cone-rr" points="100,110 160,10 190,40" stroke="#222" />
            </svg>
            <div style="display: flex; justify-content: space-between; width: 100%; font-size: 11px; font-family: monospace; color: #777;">
                <div id="val-ll">--</div><div id="val-cl">--</div><div id="val-c">--</div><div id="val-cr">--</div><div id="val-rr">--</div>
            </div>
        </div>
        
        <div class="panel settings-sector">
            <h2 style="grid-column: 1 / -1;">Hyperparameters</h2>
            <div class="data-row">
                <span>Speed (<span id="label-SPEED_CM_S" class="highlight">15</span>)</span>
                <input type="range" min="5" max="30" value="15" onchange="updateParam('SPEED_CM_S', this.value)">
            </div>
            <div class="data-row">
                <span>Engage (<span id="label-ENGAGE_MAX_CM" class="highlight">20</span>)</span>
                <input type="range" min="5" max="40" value="20" onchange="updateParam('ENGAGE_MAX_CM', this.value)">
            </div>
            <div class="data-row">
                <span>Follow (<span id="label-FOLLOW_MAX_CM" class="highlight">70</span>)</span>
                <input type="range" min="30" max="150" step="5" value="70" onchange="updateParam('FOLLOW_MAX_CM', this.value)">
            </div>
            <div class="data-row">
                <span>Obstacle (<span id="label-OBSTACLE_TRIGGER" class="highlight">15</span>)</span>
                <input type="range" min="5" max="40" value="15" onchange="updateParam('OBSTACLE_TRIGGER', this.value)">
            </div>
            <div class="data-row">
                <span>Node Trig (<span id="label-CHECKPOINT_TRIGGER" class="highlight">12</span>)</span>
                <input type="range" min="5" max="25" value="12" onchange="updateParam('CHECKPOINT_TRIGGER', this.value)">
            </div>
            <div class="data-row">
                <span>Node Clear (<span id="label-CHECKPOINT_CLEAR" class="highlight">30</span>)</span>
                <input type="range" min="10" max="60" value="30" onchange="updateParam('CHECKPOINT_CLEAR', this.value)">
            </div>
        </div>

        <div class="panel log-sector" id="log-box"></div>
    </div>
</body></html>"""

async def serve_client(reader, writer):
    # --- IMPORTANT: DECLARE ALL GLOBALS TO BE MODIFIABLE ---
    global current_state, SPEED_CM_S, ENGAGE_MAX_CM, FOLLOW_MAX_CM, \
           CHECKPOINT_TRIGGER, CHECKPOINT_CLEAR, OBSTACLE_TRIGGER

    try:
        request_line = await reader.readline()
        if not request_line: return
        
        while True:
            h = await reader.readline()
            if not h or h == b'\r\n': break

        req = request_line.decode('utf-8').split()
        if len(req) > 1:
            route = req[1]  
            
            if route.startswith('/api/set_param?'):
                try:
                    query = route.split('?')[1]
                    params = dict(p.split('=') for p in query.split('&'))
                    
                    name = params.get('name')
                    val = float(params.get('val'))
                    
                    # Unified mapping for all 6 sliders
                    if name == 'SPEED_CM_S': SPEED_CM_S = val
                    elif name == 'ENGAGE_MAX_CM': ENGAGE_MAX_CM = val
                    elif name == 'FOLLOW_MAX_CM': FOLLOW_MAX_CM = val
                    elif name == 'OBSTACLE_TRIGGER': OBSTACLE_TRIGGER = val
                    elif name == 'CHECKPOINT_TRIGGER': CHECKPOINT_TRIGGER = val
                    elif name == 'CHECKPOINT_CLEAR': CHECKPOINT_CLEAR = val
                        
                    sys_log(f"SET | {name} = {val}", "INFO")
                except Exception as e:
                    sys_log(f"Param Error: {e}", "WARN")
                
                writer.write("HTTP/1.0 200 OK\r\n\r\nOK".encode('utf-8'))
                await writer.drain()
                return
            
            elif route == '/api/telemetry':
                state_names = {0: "IDLE", 1: "RECORDING", 2: "SEEKING"}
                try: ll, cl, c, cr, rr = get_tof()
                except: ll, cl, c, cr, rr = (0.0, 0.0, 0.0, 0.0, 0.0)

                telemetry_data = {
                    "state": state_names.get(current_state, "UNKNOWN"),
                    "substate": current_rec_substate, 
                    "tof": [ll, cl, c, cr, rr],
                    "target_dist": current_target_dist, 
                    "driven_dist": current_seek_accum,  
                    "nodes": len(path.memory),
                    "logs": "".join(system_logs)
                }
                
                headers = "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\n\r\n"
                writer.write(headers.encode('utf-8'))
                await writer.drain() 
                
                body = json.dumps(telemetry_data)
                writer.write(body.encode('utf-8'))
                await writer.drain() 
                return
                
            elif route == '/api/record':
                path.clear()
                current_state = STATE_RECORD
                sys_log("Switched to RECORD mode.", "REC")
                response = "HTTP/1.0 200 OK\r\n\r\nOK"
                writer.write(response.encode('utf-8'))
                await writer.drain()
                
            elif route == '/api/save':
                current_state = STATE_IDLE
                sys_log("Switched to IDLE mode.", "INFO")
                response = "HTTP/1.0 200 OK\r\n\r\nOK"
                writer.write(response.encode('utf-8'))
                await writer.drain()
                
            elif route == '/api/seek':
                current_state = STATE_SEEK
                sys_log("Switched to SEEK mode.", "SEEK")
                response = "HTTP/1.0 200 OK\r\n\r\nOK"
                writer.write(response.encode('utf-8'))
                await writer.drain()
                
            else:
                response = "HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n" + HTML_PAGE
                writer.write(response.encode('utf-8'))
                await writer.drain()

    except OSError as e:
        if e.args[0] != 104: sys_log(f"Socket OSError: {e}", "WARN")
    except Exception as e:
        print(f"[WEB CRASH] Server error: {e}")
        import sys
        sys.print_exception(e) 
    finally:
        try:
            writer.close()
            await writer.wait_closed()
        except: pass

async def start_web_server():
    server = await asyncio.start_server(serve_client, "0.0.0.0", 80)
    while True: await asyncio.sleep(3600)

# -------------------------------------------------
# FEEDBACK (Moved up to prevent scoping issues)
# -------------------------------------------------
async def gesture_nod():
    """A quick forward-back motion indicating 'I understand / Locked On'"""
    safe_brake()
    alvik.left_led.set_color(255, 255, 255)
    alvik.right_led.set_color(255, 255, 255)
    
    alvik.set_wheels_speed(15, 15)
    await asyncio.sleep(0.15)
    alvik.set_wheels_speed(-15, -15)
    await asyncio.sleep(0.15)
    safe_brake()

async def gesture_shimmy():
    """Flashes green to indicate 'Data Saved Successfully' without moving."""
    safe_brake()
    for _ in range(3):
        alvik.left_led.set_color(0, 255, 0)
        alvik.right_led.set_color(0, 255, 0)
        await asyncio.sleep(0.15)
        
        alvik.left_led.set_color(0, 0, 0)
        alvik.right_led.set_color(0, 0, 0)
        await asyncio.sleep(0.15)

async def gesture_turn_warning(angle):
    """Signals a turn is about to happen using split LED colors."""
    safe_brake()
    if angle > 0: # Turning Left
        alvik.left_led.set_color(0, 255, 0)   # Green on Left
        alvik.right_led.set_color(255, 0, 0)  # Red on Right
    else:         # Turning Right
        alvik.left_led.set_color(255, 0, 0)   # Red on Left
        alvik.right_led.set_color(0, 255, 0)  # Green on Right
    await asyncio.sleep(1.0) 

async def gesture_blocked():
    """Flashes Red to indicate a physical barrier."""
    safe_brake()
    for _ in range(3): 
        safe_leds(255, 0, 0)
        await asyncio.sleep(0.2)
        safe_leds(0, 0, 0)
        await asyncio.sleep(0.2)

async def gesture_arrival():
    """Spins 360 degrees while flashing green."""
    safe_brake()
    alvik.rotate(360, 'deg', blocking=False)
    for _ in range(8):
        safe_leds(0, 255, 0)
        await asyncio.sleep(0.2)
        safe_leds(0, 0, 0)
        await asyncio.sleep(0.2)
    safe_brake() 

# ---------------------------------------------------------
# CORE STATE MACHINE 
# ---------------------------------------------------------
async def robot_loop():
    global current_state, current_rec_substate, current_target_dist, current_seek_accum
    
    last_ticks = time.ticks_ms()
    last_log_ticks = 0
    rec_substate = 'WAIT_TARGET' 
    seek_index = 0
    seek_accum = 0.0

    checkpoint_hits = 0
    obstacle_hits = 0
    REQUIRED_HITS = 5

    while True:
        try: 
            current_ticks = time.ticks_ms()
            dt = time.ticks_diff(current_ticks, last_ticks) / 1000.0
            last_ticks = current_ticks

            do_periodic_log = False
            if time.ticks_diff(current_ticks, last_log_ticks) >= 1000:
                do_periodic_log = True
                last_log_ticks = current_ticks

            if current_state == STATE_IDLE:
                safe_brake()
                
                # Smooth breathing white light (calm, resting state)
                animate_breathing_led(255, 255, 255, speed=0.002) 
                
                rec_substate = 'WAIT_TARGET'
                seek_index = 0
                seek_accum = 0.0
                
            elif current_state == STATE_RECORD:
                ll, cl, c, cr, rr = get_tof()

                if rec_substate == 'WAIT_TARGET':
                    safe_leds(255, 255, 255)
                    
                    if c < ENGAGE_MAX_CM:
                        sys_log(f"ENGAGE | Center ({c:.1f}cm) triggered. Locking Straight.", "REC")
                        if len(path.memory) == 0:
                            await gesture_nod()
                        rec_substate = 'FOLLOW'

                    elif ll < ENGAGE_MAX_CM:
                        sys_log(f"ENGAGE | Extreme Left ({ll:.1f}cm) triggered. Pivoting 90°.", "REC")
                        if len(path.memory) == 0:
                            await gesture_nod()
                        await async_rotate(90)
                        path.add_turn(90)
                        rec_substate = 'FOLLOW'

                    elif rr < ENGAGE_MAX_CM:
                        sys_log(f"ENGAGE | Extreme Right ({rr:.1f}cm) triggered. Pivoting -90°.", "REC")
                        if len(path.memory) == 0:
                            await gesture_nod()
                        await async_rotate(-90)
                        path.add_turn(-90)
                        rec_substate = 'FOLLOW'

                elif rec_substate == 'FOLLOW':
                    safe_leds(0, 0, 255)
                    
                    # Because get_tof() is filtered, if c < TRIGGER, it HAS been 3 frames.
                    if 0 < c < CHECKPOINT_TRIGGER:
                        safe_brake()
                        safe_leds(255, 0, 0)
                        sys_log(f"NODE | Target reached ({c:.1f}cm). Saving...", "REC")
                        await gesture_shimmy()
                        await asyncio.sleep(0.5) 
                        snap_ll, snap_cl, snap_c, snap_cr, snap_rr = get_tof()
                        path.add_checkpoint(snap_ll, snap_c, snap_rr)
                        safe_leds(0, 255, 0) 
                        rec_substate = 'WAIT_CLEAR'
                    else:
                        if c <= FOLLOW_MAX_CM:
                            if do_periodic_log:
                                sys_log(f"FOLLOW | Tracking Center -> C:{c:.1f}", "TOF")
                            safe_velocity(SPEED_CM_S)
                            path.add_drive(SPEED_CM_S * dt)
                        else:
                            safe_brake() 
                            if do_periodic_log:
                                sys_log("FOLLOW | Braking. Target lost.", "WARN")

                elif rec_substate == 'WAIT_CLEAR':
                    safe_brake()
                    # Only check the 3 primary sensors
                    min_dist = min(ll, c, rr)
                    if do_periodic_log:
                        sys_log(f"CLEAR | Closest object: {min_dist:.1f}cm (Need > {CHECKPOINT_CLEAR})", "TOF")

                    if min_dist > CHECKPOINT_CLEAR:
                        sys_log("CLEAR | Area cleared. Awaiting new direction.", "REC")
                        rec_substate = 'WAIT_TARGET'

            elif current_state == STATE_SEEK:
                if seek_index >= len(path.memory):
                    if do_periodic_log: sys_log("Playback complete.", "SEEK")
                    
                    # --- WIRED: Trigger Arrival Gesture ---
                    await gesture_arrival() 
                    current_state = STATE_IDLE
                    continue

                node = path.memory[seek_index]

                if node['type'] == 'drive':
                    target_dist = node['value']
                    ll, cl, c, cr, rr = get_tof()
                    
                    if c < OBSTACLE_TRIGGER:
                        await gesture_blocked() 
                        if do_periodic_log: sys_log(f"BLOCKED | Obstacle detected ahead!", "WARN")
                    else:
                        safe_leds(0, 255, 0)
                        safe_velocity(SPEED_CM_S) 
                        seek_accum += SPEED_CM_S * dt
                        
                        if seek_accum >= target_dist:
                            safe_brake()
                            seek_accum = 0.0
                            seek_index += 1
                            sys_log(f"NODE | Segment completed.", "SEEK")

                elif node['type'] == 'turn':
                    sys_log(f"TURN | Rotating {node['value']}°", "SEEK")
                    
                    # --- WIRED: Trigger Turn Warning ---
                    await gesture_turn_warning(node['value']) 
                    await async_rotate(node['value'])
                    seek_index += 1

                    safe_leds(0, 255, 0)

                elif node['type'] == 'checkpoint':
                    sys_log("CHECKPOINT | Verifying location...", "SEEK")
                    safe_brake()
                    safe_leds(0, 0, 255)
                    await asyncio.sleep(1.0) 
                    seek_index += 1
                    safe_leds(0, 255, 0)

        except Exception as e:
            sys_log(f"Robot Loop Crash: {e}", "WARN")
            alvik.brake()
            await asyncio.sleep(1)

        await asyncio.sleep(0.1) 

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True) 
    try:
        wlan.disconnect()
    except OSError:
        pass 
        
    wlan.active(False)
    time.sleep(0.5)
    
    wlan.active(True)
    if not wlan.isconnected():
        print(f"[SYSTEM] Connecting to {WIFI_SSID}...")
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
    asyncio.create_task(start_web_server())
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