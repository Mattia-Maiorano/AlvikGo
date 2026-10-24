import time
import network
import bluetooth
import uasyncio as asyncio
from arduino_alvik import ArduinoAlvik
import math

# ========================================================================
# SYSTEM LOGGER (Memory Buffer for Web UI)
# ========================================================================
LOG_BUFFER = []
MAX_LOG_LINES = 200 

def sys_log(msg):
    """Prints to Serial AND saves to a rolling buffer for the Web UI."""
    print(msg) # Keep serial output for debugging via mpremote
    LOG_BUFFER.append(msg)
    if len(LOG_BUFFER) > MAX_LOG_LINES:
        LOG_BUFFER.pop(0)

# ========================================================================
# REAL ALVIK HARDWARE INTERFACE
# ========================================================================

# Initialize the actual robot hardware
alvik = ArduinoAlvik()
alvik.begin()

class AlvikRobot:
    def stop(self):
        sys_log("[HARDWARE] Motors BRAKING.")
        alvik.brake()  # Official API to stop motors
        
    def drive(self, distance_cm):
        sys_log(f"[HARDWARE] Driving forward {distance_cm}cm...")
        # Official API: move(distance, unit, blocking)
        alvik.move(distance_cm, 'cm', blocking=True) 
        
    def rotate(self, degrees):
        sys_log(f"[HARDWARE] Rotating {degrees} degrees...")
        # Official API: rotate(angle, unit, blocking)
        # Note: Alvik's positive rotation might be counter-clockwise depending on the firmware, 
        # but this will safely execute the blocking turn.
        alvik.rotate(degrees, 'deg', blocking=True)
        
    def get_tof(self):
        # The API returns 5 zones: (left, center_left, center, center_right, right)
        try:
            distances = alvik.get_distance()
            l_dist = distances[0]
            c_dist = distances[2]
            r_dist = distances[4]
            
            # Treat 0 or negative as "out of range" (clear path)
            l_dist = 300.0 if l_dist <= 0 else float(l_dist)
            c_dist = 300.0 if c_dist <= 0 else float(c_dist)
            r_dist = 300.0 if r_dist <= 0 else float(r_dist)
            
            return l_dist, c_dist, r_dist
        except Exception as e:
            sys_log(f"[WARN] ToF Read Error: {e}")
            return 300.0, 50.0, 300.0 # Fallback
        
    def set_led(self, color):
        sys_log(f"[HARDWARE] LED state requested: {color.upper()}")
        try:
            # Define standard RGB values
            r, g, b = 0, 0, 0
            if color == 'blue':
                r, g, b = 0, 0, 255  # Recording Mode
            elif color == 'green':
                r, g, b = 0, 255, 0  # Success / Idle
            elif color == 'red':
                r, g, b = 255, 0, 0  # Obstacle / Detour
                
            # The official Alvik API addresses the top LEDs individually
            alvik.left_led.set_color(r, g, b)
            alvik.right_led.set_color(r, g, b)
        except Exception as e:
            sys_log(f"[WARN] LED Error: {e}")
             
    def play_melody(self, melody_type):
        sys_log(f"[HARDWARE] Melody requested: {melody_type}")
        try:
            if melody_type == 'success':
                # Map notes to frequencies (Hz) for maximum compatibility
                # C4=262, E4=330, G4=392, C5=523
                notes = [(262, 100), (330, 100), (392, 100), (523, 200)]
                for freq, duration in notes:
                    # The Alvik API usually uses alvik.tone(frequency, duration)
                    if hasattr(alvik, 'tone'):
                        alvik.tone(freq, duration)
                    elif hasattr(alvik, 'play_note'):
                        # Fallback for older/newer experimental firmwares
                        alvik.play_note(freq, duration)
                    
                    time.sleep_ms(duration + 10) # Pause so notes don't overlap
        except Exception as e:
            sys_log(f"[WARN] Melody Error: {e}")
        
    def beep(self):
        sys_log("[HARDWARE] BEEP!")
        try:
            # High pitched warning tone (880Hz = A5)
            if hasattr(alvik, 'tone'):
                alvik.tone(880, 200)
            elif hasattr(alvik, 'play_note'):
                alvik.play_note(880, 200)
            time.sleep_ms(210)
        except Exception as e:
            sys_log(f"[WARN] Buzzer Error: {e}")

robot = AlvikRobot()

# ========================================================================
# CONSTANTS & CONFIGURATION
# ========================================================================
# --- ENTER YOUR WIFI CREDENTIALS HERE ---
WIFI_SSID = "sky_ext"
WIFI_PASSWORD = "QHbv6IDXxceT"
# ----------------------------------------

SEGMENT_CM = 50.0
CHUNK_CM = 10.0
OBSTACLE_CM = CHUNK_CM + 5.0
ARRIVAL_THRESHOLD = 500.0

REMAINING_SEGMENT_CM = 50.0

STATE_IDLE = 0
STATE_RECORD = 1
STATE_ORIENTATION = 2
STATE_SEEK = 3
STATE_DETOUR = 4
STATE_ARRIVAL = 5

# --- Intersection State Machine ---
LEFT_WALL_STATE = "closed"
RIGHT_WALL_STATE = "closed"
LEFT_OPEN_STREAK = 0
RIGHT_OPEN_STREAK = 0
LEFT_CLOSE_STREAK = 0
RIGHT_CLOSE_STREAK = 0

PATH_CONFIDENCE = 100.0 
LAST_ANCHOR_RSSI = -100.0
LAST_TREND = 'stable'

TARGET_FINGERPRINT = {}
CURRENT_STATE = STATE_IDLE

# Networking: Disable AP, Enable Station mode
ap = network.WLAN(network.AP_IF)
ap.active(False) 

wlan = network.WLAN(network.STA_IF)
wlan.active(True)

ble = bluetooth.BLE()
ble.active(True)

_ble_scan_results = {}

_ble_scan_results = {}
_ble_scan_counts = {} # NEW: Track packet counts

def bt_irq(event, data):
    if event == 5: # _IRQ_SCAN_RESULT
        addr_type, addr, adv_type, rssi, adv_data = data
        mac = bytes(addr)
        if mac in _ble_scan_results:
            _ble_scan_results[mac] += float(rssi)
            _ble_scan_counts[mac] += 1
        else:
            _ble_scan_results[mac] = float(rssi)
            _ble_scan_counts[mac] = 1

ble.irq(bt_irq)

# ========================================================================
# WEB UI & HTTP SERVER
# ========================================================================
WEB_PAGE = """<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { background: #121212; color: #fff; font-family: sans-serif; text-align: center; padding: 10px; margin: 0; }
        .btn { padding: 15px; font-size: 20px; width: 95%; margin: 8px 0; border-radius: 8px; border: none; font-weight: bold; cursor: pointer; }
        .btn-rec { background: #007bff; color: white; }
        .btn-seek { background: #28a745; color: white; }
        .btn:active { opacity: 0.7; }
        #log-box { 
            text-align: left; background: #000; color: #0f0; 
            padding: 10px; height: 350px; overflow-y: scroll; 
            font-family: monospace; font-size: 13px; border-radius: 5px; 
            margin-top: 15px; border: 1px solid #333;
        }
    </style>
    <script>
        function sendCmd(cmd) {
            fetch('/api/' + cmd).then(() => console.log('Command ' + cmd + ' sent'));
        }
        function fetchLogs() {
            fetch('/logs')
                .then(r => r.text())
                .then(txt => {
                    let box = document.getElementById('log-box');
                    let formatted = txt.replace(/\\n/g, '<br>');
                    if(box.innerHTML !== formatted && formatted.trim() !== '') {
                        let isScrolledToBottom = box.scrollHeight - box.clientHeight <= box.scrollTop + 10;
                        box.innerHTML = formatted;
                        if(isScrolledToBottom) {
                            box.scrollTop = box.scrollHeight; // Auto-scroll
                        }
                    }
                }).catch(e => console.log("Log fetch error"));
        }
        setInterval(fetchLogs, 1000); // Check for new logs every 1 second
    </script>
</head>
<body>
    <h2 style="margin-top:5px;">Alvik Navigation</h2>
    <button class="btn btn-rec" onclick="sendCmd('record')">1. RECORD TARGET</button>
    <button class="btn btn-seek" onclick="sendCmd('seek')">2. SEEK TARGET</button>
    <div id="log-box">Awaiting connection...</div>
</body>
</html>
"""

async def handle_web_request(reader, writer):
    global CURRENT_STATE
    try:
        request_line = await reader.readline()
        if not request_line:
            return
            
        req_str = request_line.decode('utf-8').strip()
        parts = req_str.split(' ')
        if len(parts) < 2: return
        
        path = parts[1]
        
        # Consume the rest of the HTTP headers so the socket is clean
        while True:
            line = await reader.readline()
            if not line or line == b'\r\n':
                break
                
        # ROUTE: API Commands (Triggered by JS fetch)
        if path == '/api/record':
            sys_log(">>> WEB UI: Triggered RECORD mode")
            CURRENT_STATE = STATE_RECORD
            writer.write(b"HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nOK")
            
        elif path == '/api/seek':
            sys_log(">>> WEB UI: Triggered SEEK mode")
            CURRENT_STATE = STATE_ORIENTATION
            writer.write(b"HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nOK")
            
        # ROUTE: Log Fetcher
        elif path == '/logs':
            log_text = "\n".join(LOG_BUFFER)
            writer.write(b"HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\n")
            writer.write(log_text.encode('utf-8'))
            
        # ROUTE: Main Dashboard (Everything else)
        else:
            writer.write(b"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n")
            writer.write(WEB_PAGE.encode('utf-8'))
            
        await writer.drain()
    except Exception as e:
        print(f"Web Server Error: {e}")
    finally:
        writer.close()
        await writer.wait_closed()


# ========================================================================
# HELPER FUNCTIONS
# ========================================================================
def execute_wlan_scan():
    sys_log("  -> WLAN Scan Running...")
    networks = wlan.scan()
    return {net[1]: float(net[3]) for net in networks}

def execute_ble_scan(duration_ms):
    sys_log(f"  -> BLE Scan ({duration_ms}ms)...")
    global _ble_scan_results, _ble_scan_counts
    _ble_scan_results = {}
    _ble_scan_counts = {}
    
    # 50% Duty cycle
    ble.gap_scan(duration_ms, 30000, 15000) 
    time.sleep_ms(duration_ms + 100) 
    
    # Calculate true average for each MAC
    final_results = {}
    for mac in list(_ble_scan_results.keys()):
        final_results[mac] = _ble_scan_results[mac] / _ble_scan_counts[mac]
        
    return final_results

def calculate_similarity(current_scan):
    if not TARGET_FINGERPRINT or not current_scan:
        return 0.0

    # THE FIX: Strip out all random noise/phones before we sort anything.
    # We ONLY care about MACs we recorded in Phase 1.
    filtered_current = {mac: rssi for mac, rssi in current_scan.items() if mac in TARGET_FINGERPRINT}
    
    if not filtered_current:
        return 0.0 # Saw absolutely nothing we recognize

    # Now we sort, and the ranks will be pure.
    target_sorted = sorted(TARGET_FINGERPRINT.keys(), key=lambda k: TARGET_FINGERPRINT[k], reverse=True)
    current_sorted = sorted(filtered_current.keys(), key=lambda k: filtered_current[k], reverse=True)

    top_k = min(10, len(target_sorted))
    target_top = target_sorted[:top_k]

    score = 0.0
    max_possible_score = 0.0

    for i, mac in enumerate(target_top):
        weight = top_k - i 
        max_possible_score += weight

        if mac in current_sorted:
            current_rank = current_sorted.index(mac)
            # Gentler penalty for slight multipath shuffling
            rank_diff = abs(i - current_rank)
            match_score = max(0.0, weight - (rank_diff * 0.75))
            score += match_score
        # If the MAC is completely missing, it adds 0 to the score, 
        # naturally penalizing the direction for missing beacons.

    return score / max_possible_score if max_possible_score > 0 else 0.0

# ========================================================================
# PHASE HANDLERS
# ========================================================================
async def phase_1_record():
    global TARGET_FINGERPRINT
    sys_log("\n[PHASE 1] Strict Anchor Vetting...")
    
    robot.stop()
    robot.set_led('blue')
    await asyncio.sleep(0.02)
    
    history = {}
    scan_count = 5
    
    # Take 5 distinct snapshots to analyze stability over time
    for i in range(scan_count):
        sys_log(f"  -> Baseline Scan {i+1}/{scan_count} (800ms)...")
        ble_res = execute_ble_scan(800)
        
        for mac, rssi in ble_res.items():
            if mac not in history:
                history[mac] = []
            history[mac].append(rssi)
            
        await asyncio.sleep(0.02)
        
    TARGET_FINGERPRINT.clear()
    
    # The Vetting Process
    for mac, rssi_list in history.items():
        # Rule 1: Must be present in ALL 5 scans (No dropped packets allowed)
        if len(rssi_list) == scan_count:
            max_rssi = max(rssi_list)
            min_rssi = min(rssi_list)
            variance = max_rssi - min_rssi
            
            # Rule 2: Signal cannot bounce more than 6 dBm.
            # Rule 3: Signal must be decently strong (better than -85 dBm)
            avg_rssi = sum(rssi_list) / scan_count
            
            if variance <= 6.0 and avg_rssi > -85.0:
                TARGET_FINGERPRINT[mac] = avg_rssi
                sys_log(f"  [VETTED] MAC {mac.hex()[-4:]} | Avg: {avg_rssi:.1f} | Var: {variance:.1f}")
                
    if not TARGET_FINGERPRINT:
        sys_log("[ERROR] No stable anchors found! Cannot navigate.")
        robot.set_led('red')
        robot.beep()
        return STATE_IDLE
        
    sys_log(f"[PHASE 1] Vetting Complete. Locked onto {len(TARGET_FINGERPRINT)} stable anchors.")
    
    robot.play_melody('success')
    robot.set_led('green')
    return STATE_IDLE

async def phase_2_orientation():
    sys_log("\n[PHASE 2] Manhattan Grid & Power Sweep...")
    
    if not TARGET_FINGERPRINT:
        sys_log("[ERROR] No target fingerprint found!")
        return STATE_IDLE
        
    target_sorted = sorted(TARGET_FINGERPRINT.keys(), key=lambda k: TARGET_FINGERPRINT[k], reverse=True)
    top_macs = target_sorted[:5] 
    
    power_scores = []
    
    # ==========================================
    # HARDWARE TUNING
    # Adjust this value based on your floor type (carpet needs more, tile needs less)
    # ==========================================
    FRICTION_OFFSET = 2.0 
    PHYSICAL_TURN = 45.0 + FRICTION_OFFSET
    
    for angle_step in range(8):
        current_angle = angle_step * 45 # The MATH stays perfect
        ble_res = execute_ble_scan(1500) 
        
        step_score = 0.0
        
        for mac in top_macs:
            if mac in ble_res:
                points = max(0.0, 100.0 + ble_res[mac])
                step_score += points
                
        power_scores.append(step_score)
        sys_log(f"  [Sweep {angle_step+1}/8] {current_angle} deg: Power Score {step_score:.1f}")
        
        # Execute the overshot physical turn to guarantee 45 degrees of real movement
        robot.rotate(PHYSICAL_TURN)
        time.sleep_ms(150)
        await asyncio.sleep(0.02)
        
    if sum(power_scores) == 0.0:
        sys_log("[PHASE 2] BLIND TRAP: No target beacons found!")
        sys_log("[PHASE 2] Signal lost. Aborting sweep.")
        robot.beep()
        robot.beep()
        return STATE_IDLE 
    
    sum_x = 0.0
    sum_y = 0.0
    
    for i in range(8):
        angle_rad = math.radians(i * 45) # The MATH stays perfect
        sum_x += power_scores[i] * math.cos(angle_rad)
        sum_y += power_scores[i] * math.sin(angle_rad)
        
    raw_target_angle_deg = math.degrees(math.atan2(sum_y, sum_x)) % 360
    
    # ==========================================
    # MANHATTAN GRID SNAP
    # Forces the final vector to 0, 90, 180, or 270
    # ==========================================
    cardinal_target_deg = round(raw_target_angle_deg / 90.0) * 90.0 % 360
    
    rotation_deg = cardinal_target_deg
    if rotation_deg > 180:
        rotation_deg -= 360
        
    sys_log(f"[PHASE 2] Raw Vector: {raw_target_angle_deg:.1f} deg.")
    sys_log(f"[PHASE 2] Manhattan Snap: {cardinal_target_deg:.1f} deg.")
    sys_log(f"[PHASE 2] Executing rotation: {rotation_deg:.1f} deg.")
    
    # Add friction compensation to the final turn as well (if it's not going straight)
    if rotation_deg != 0:
        final_physical_turn = rotation_deg + (FRICTION_OFFSET if rotation_deg > 0 else -FRICTION_OFFSET)
        robot.rotate(final_physical_turn)
        
    await asyncio.sleep(0.02)
    return STATE_SEEK

async def drive_sentry_segment(target_cm):
    global LEFT_WALL_STATE, RIGHT_WALL_STATE
    global LEFT_OPEN_STREAK, RIGHT_OPEN_STREAK, LEFT_CLOSE_STREAK, RIGHT_CLOSE_STREAK
    
    sys_log(f"[SENTRY] Driving {target_cm}cm...")
    driven = 0.0
    
    while driven < target_cm:
        l_tof, c_tof, r_tof = robot.get_tof()
        
        # 1. Obstacle Check
        if c_tof < OBSTACLE_CM:
            sys_log(f"!! OBSTACLE at {c_tof:.1f}cm !!")
            return "obstacle", driven 
            
        # 2. Intersection State Machine
        triggered = False
        
        # --- LEFT WALL LOGIC ---
        if l_tof > 250.0:
            LEFT_OPEN_STREAK += 1
            LEFT_CLOSE_STREAK = 0
            if LEFT_OPEN_STREAK >= 3 and LEFT_WALL_STATE == "closed":
                LEFT_WALL_STATE = "open"
                sys_log("[SENTRY] Left intersection opening detected!")
                triggered = True
        else:
            LEFT_CLOSE_STREAK += 1
            LEFT_OPEN_STREAK = 0
            if LEFT_CLOSE_STREAK >= 2 and LEFT_WALL_STATE == "open":
                LEFT_WALL_STATE = "closed"
                sys_log("  -> Left wall re-acquired.")
                
        # --- RIGHT WALL LOGIC ---
        if r_tof > 250.0:
            RIGHT_OPEN_STREAK += 1
            RIGHT_CLOSE_STREAK = 0
            if RIGHT_OPEN_STREAK >= 3 and RIGHT_WALL_STATE == "closed":
                RIGHT_WALL_STATE = "open"
                sys_log("[SENTRY] Right intersection opening detected!")
                triggered = True
        else:
            RIGHT_CLOSE_STREAK += 1
            RIGHT_OPEN_STREAK = 0
            if RIGHT_CLOSE_STREAK >= 2 and RIGHT_WALL_STATE == "open":
                RIGHT_WALL_STATE = "closed"
                sys_log("  -> Right wall re-acquired.")

        if triggered:
            return "intersection", driven

        robot.drive(CHUNK_CM)
        driven += CHUNK_CM
        await asyncio.sleep(0.02)
        
    return "success", driven

def get_positive_power():
    """Scans and converts RSSI to positive points for threshold math."""
    ble_res = execute_ble_scan(1500)
    target_sorted = sorted(TARGET_FINGERPRINT.keys(), key=lambda k: TARGET_FINGERPRINT[k], reverse=True)
    top_macs = target_sorted[:5]
    
    score = 0.0
    for mac in top_macs:
        if mac in ble_res:
            score += max(0.0, 100.0 + ble_res[mac])
    return score

async def execute_proportional_sweep(sweep_type):
    global PATH_CONFIDENCE, LAST_ANCHOR_RSSI, SEGMENTS_SINCE_SWEEP
    sys_log(f"\n[SWEEP] Proportional Dominance ({sweep_type.upper()})")
    robot.stop()
    
    sys_log("  -> Scanning FRONT...")
    front_power = get_positive_power()
    left_power = 0.0
    right_power = 0.0
    
    # Optimized Rotation Sequence
    if sweep_type == "left":
        robot.rotate(-90)
        sys_log("  -> Scanning LEFT...")
        left_power = get_positive_power()
        robot.rotate(90) # Return to center
    elif sweep_type == "right":
        robot.rotate(90)
        sys_log("  -> Scanning RIGHT...")
        right_power = get_positive_power()
        robot.rotate(-90) # Return to center
    elif sweep_type == "both":
        robot.rotate(-90)
        sys_log("  -> Scanning LEFT...")
        left_power = get_positive_power()
        robot.rotate(180) # Sweep all the way to right
        sys_log("  -> Scanning RIGHT...")
        right_power = get_positive_power()
        robot.rotate(-90) # Return to center
        
    total_power = front_power + left_power + right_power
    if total_power == 0:
        sys_log("[SWEEP] Zero signal. Holding course.")
        return
        
    f_pct = front_power / total_power
    l_pct = left_power / total_power
    r_pct = right_power / total_power
    
    sys_log(f"  [POWER] F:{f_pct*100:.1f}% | L:{l_pct*100:.1f}% | R:{r_pct*100:.1f}%")
    
    best_dir = "front"
    
    # THE RULE: Side path must hold > 55% of power, OR be 1.5x stronger than front.
    if l_pct > 0.55 or l_pct > (f_pct * 1.5):
        if l_pct > r_pct: best_dir = "left"
        
    if r_pct > 0.55 or r_pct > (f_pct * 1.5):
        if r_pct > l_pct: best_dir = "right"
        
    if best_dir == "left":
        sys_log("[SWEEP] Threshold crossed. Committing LEFT.")
        robot.rotate(-90)
        PATH_CONFIDENCE = 100.0
        LAST_ANCHOR_RSSI = -100.0
        SEGMENTS_SINCE_SWEEP = 0
        LEFT_WALL_STATE = "closed"  # RESET TOPOLOGY
        RIGHT_WALL_STATE = "closed" 
    elif best_dir == "right":
        sys_log("[SWEEP] Threshold crossed. Committing RIGHT.")
        robot.rotate(90)
        PATH_CONFIDENCE = 100.0
        LAST_ANCHOR_RSSI = -100.0
        SEGMENTS_SINCE_SWEEP = 0
        LEFT_WALL_STATE = "closed"  # RESET TOPOLOGY
        RIGHT_WALL_STATE = "closed"
    else:
        sys_log("[SWEEP] Threshold failed. Holding forward course.")

async def phase_3_seek():
    global REMAINING_SEGMENT_CM, PATH_CONFIDENCE, LAST_ANCHOR_RSSI, LAST_TREND
    global LEFT_OPEN_COUNT, RIGHT_OPEN_COUNT, SEGMENTS_SINCE_SWEEP
    
    sys_log(f"\n[PHASE 3] Bayesian Drive (Confidence: {PATH_CONFIDENCE:.1f}%)")
    
    status, driven = await drive_sentry_segment(REMAINING_SEGMENT_CM)
    
    # 1. Update Remaining Segment Distance
    if status == "success":
        REMAINING_SEGMENT_CM = SEGMENT_CM
        SEGMENTS_SINCE_SWEEP += 1
    else:
        REMAINING_SEGMENT_CM -= driven
        
    # 2. Execute Bayesian Fusion (Only if we actually moved)
    if driven > 0:
        current_scan = execute_ble_scan(1000)
        current_anchor_rssi = -100.0
        found_anchors = 0
        
        for mac in TARGET_FINGERPRINT:
            if mac in current_scan:
                current_anchor_rssi += current_scan[mac]
                found_anchors += 1
                
        if found_anchors > 0:
            current_anchor_rssi = (current_anchor_rssi + 100.0) / found_anchors
            
        if LAST_ANCHOR_RSSI != -100.0:
            delta = current_anchor_rssi - LAST_ANCHOR_RSSI
            if delta > 1.0: 
                reward = min(40.0, delta * 2.5) 
                PATH_CONFIDENCE = min(100.0, PATH_CONFIDENCE + reward)
                LAST_TREND = 'improved'
                sys_log(f"  -> Signal Improved (+{delta:.1f} dBm). Confidence +{reward:.1f}%")
            elif delta < -1.5: 
                penalty = min(60.0, abs(delta) * 4.0) 
                PATH_CONFIDENCE -= penalty
                LAST_TREND = 'dropped'
                sys_log(f"  -> Signal Dropped ({delta:.1f} dBm). Confidence -{penalty:.1f}%")
            else: 
                if LAST_TREND == 'dropped':
                    decay = 15.0
                    PATH_CONFIDENCE -= decay
                    sys_log(f"  -> Stuck in Drop ({current_anchor_rssi:.1f} dBm). Decay -{decay:.1f}%")
                else:
                    LAST_TREND = 'stable'
                    sys_log(f"  -> Signal Coasting ({current_anchor_rssi:.1f} dBm). Confidence stable.")
                    
        LAST_ANCHOR_RSSI = current_anchor_rssi
        
        if current_anchor_rssi > -60.0 and PATH_CONFIDENCE > 80.0:
            return STATE_ARRIVAL

    # 3. Handle Obstacles
    if status == "obstacle":
        return STATE_DETOUR
        
    # 4. Handle Complete Confidence Loss
    if PATH_CONFIDENCE <= 20.0:
        sys_log("[PHASE 3] Confidence lost. Forcing Re-Orientation.")
        PATH_CONFIDENCE = 100.0 
        LAST_ANCHOR_RSSI = -100.0
        SEGMENTS_SINCE_SWEEP = 0
        return STATE_ORIENTATION

    # 5. Handle Intersections & Failsafe Wiggles
    needs_sweep = False
    sweep_type = "none"
    
    if status == "intersection":
        needs_sweep = True
        # Read the global states to know which direction opened up
        if LEFT_WALL_STATE == "open" and RIGHT_WALL_STATE == "open":
            sweep_type = "both"
        elif LEFT_WALL_STATE == "open":
            sweep_type = "left"
        elif RIGHT_WALL_STATE == "open":
            sweep_type = "right"
            
    elif SEGMENTS_SINCE_SWEEP >= 3:
        sys_log("[PHASE 3] 3-Segment Failsafe triggered.")
        needs_sweep = True
        sweep_type = "both"
        SEGMENTS_SINCE_SWEEP = 0
        
    if needs_sweep:
        await execute_proportional_sweep(sweep_type)
        
    return STATE_SEEK

async def phase_4_detour():
    sys_log("\n[PHASE 4] SMART DETOUR PROTOCOL")
    robot.stop()
    robot.set_led('red')
    robot.beep()
    await asyncio.sleep(0.02)
    
    MAX_DETOUR_CM = 100.0
    CLEARANCE_CM = 15.0
    SIDE_OBSTACLE_CM = 25.0
    
    # --- ATTEMPT 1: Go Left (Wall on Right) ---
    sys_log("[DETOUR] Attempt 1: Turning Left.")
    robot.rotate(-90)
    await asyncio.sleep(0.02)
    
    driven_left = 0.0
    success_left = False
    
    while driven_left < MAX_DETOUR_CM:
        l_tof, c_tof, r_tof = robot.get_tof()
        
        # Scenario 1: New obstacle blocks the front
        if c_tof < OBSTACLE_CM:
            sys_log("[DETOUR] Front blocked. Left attempt failed.")
            break
            
        # Scenario 2: Right sensor loses the wall (Obstacle ended)
        if r_tof > SIDE_OBSTACLE_CM:
            sys_log("[DETOUR] Obstacle cleared on the right!")
            success_left = True
            break
            
        robot.drive(CHUNK_CM)
        driven_left += CHUNK_CM
        await asyncio.sleep(0.02)
        
    if success_left:
        sys_log("[DETOUR] Moving clearance distance.")
        robot.drive(CLEARANCE_CM)
        robot.rotate(90) # Turn back to original heading
        sys_log("[PHASE 4] Detour complete. Resuming segment.")
        return STATE_SEEK
        
    # --- ATTEMPT 2: Go Right (Wall on Left) ---
    sys_log("[DETOUR] Attempt 2: Turning 180 to try Right side.")
    robot.rotate(180)
    await asyncio.sleep(0.02)
    
    driven_right = 0.0
    success_right = False
    
    # Allow it to drive back past where it started, plus the max limit
    while driven_right < (MAX_DETOUR_CM + driven_left):
        l_tof, c_tof, r_tof = robot.get_tof()
        
        if c_tof < OBSTACLE_CM:
            sys_log("[DETOUR] Front blocked. Right attempt failed.")
            break
            
        # Only check for clear AFTER we've driven back past the original starting point
        if driven_right > driven_left and l_tof > SIDE_OBSTACLE_CM:
            sys_log("[DETOUR] Obstacle cleared on the left!")
            success_right = True
            break
            
        robot.drive(CHUNK_CM)
        driven_right += CHUNK_CM
        await asyncio.sleep(0.02)
        
    if success_right:
        sys_log("[DETOUR] Moving clearance distance.")
        robot.drive(CLEARANCE_CM)
        robot.rotate(-90) # Turn back to original heading
        sys_log("[PHASE 4] Detour complete. Resuming segment.")
        return STATE_SEEK
        
    # --- FALLBACK ---
    # If both sides are completely blocked, the robot is stuck in a corner or facing a massive wall.
    sys_log("[DETOUR] Trapped! Triggering 3D Sweep to find new path.")
    return STATE_ORIENTATION

async def phase_5_arrival():
    sys_log("\n[PHASE 5] Isotropy Verification Spin...")
    sims = []
    max_absolute_rssi = -100.0
    
    # Perform a 4-way spin (Front, Right, Back, Left)
    for i in range(4):
        scan = execute_ble_scan(1000)
        sim_score = calculate_similarity(scan)
        sims.append(sim_score)
        
        # Check the strongest signal in the room (Power Gate)
        if scan:
            best_signal_in_scan = max(scan.values())
            max_absolute_rssi = max(max_absolute_rssi, best_signal_in_scan)
            
        sys_log(f"  [Verify {i+1}/4] Similarity: {(sim_score*100):.1f}% | Max RSSI: {max_absolute_rssi:.1f}dBm")
        
        robot.rotate(90)
        await asyncio.sleep(0.02)
        
    avg_sim = sum(sims) / 4.0
    
    # Calculate Standard Deviation to check for Isotropy (uniformity)
    variance = sum((s - avg_sim) ** 2 for s in sims) / 4.0
    std_dev = math.sqrt(variance)
    
    sys_log(f"[PHASE 5] Avg Sim: {(avg_sim*100):.1f}% | StdDev: {std_dev:.3f}")
    
    # THE GATES:
    # 1. Similarity is universally high (> 75%)
    # 2. It is isotropic (Deviation is very low, < 0.10)
    # 3. We are physically close to a beacon (e.g., > -65 dBm)
    if avg_sim > 0.75 and std_dev < 0.10 and max_absolute_rssi > -65.0:
        sys_log("\n===========================================")
        sys_log(" TARGET CONFIRMED. NAVIGATION COMPLETE.")
        sys_log("===========================================")
        robot.play_melody('success')
        robot.set_led('green')
        return STATE_IDLE
    else:
        sys_log("[PHASE 5] Environment not isotropic. False peak. Resuming.")
        return STATE_SEEK
    
# ========================================================================
# CORE SUPER-LOOP
# ========================================================================
async def super_loop():
    global CURRENT_STATE
    
    # 1. Force a clean Network Reset
    sys_log("[NETWORK] Resetting Wi-Fi interface...")
    wlan.active(False)
    await asyncio.sleep(1) 
    wlan.active(True)
    
    sys_log(f"[NETWORK] Connecting to '{WIFI_SSID}'...")
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    
    # Wait for a valid IP address
    timeout = 30 
    while timeout > 0:
        if wlan.isconnected() and wlan.ifconfig()[0] != '0.0.0.0':
            break
        await asyncio.sleep(0.5)
        timeout -= 1
            
    if wlan.isconnected() and wlan.ifconfig()[0] != '0.0.0.0':
        ip = wlan.ifconfig()[0]
        sys_log("===========================")
        sys_log(" SYSTEM ONLINE")
        sys_log(f" URL: http://{ip}")
        sys_log("===========================")
        
        # Start Web Server
        try:
            asyncio.create_task(asyncio.start_server(handle_web_request, '0.0.0.0', 80))
            sys_log("[SYSTEM] Web Dashboard running.")
        except Exception as e:
            sys_log(f"[SYSTEM] Failed to start UI: {e}")
    else:
        sys_log(f"[NETWORK ERROR] Could not connect.")
        
    sys_log("[SYSTEM] Ready. Waiting for UI commands.")
    
    last_state = -1
    while True:
        await asyncio.sleep(0.02)
        
        # We handle logging the state change right before transitioning
        if CURRENT_STATE != last_state:
            last_state = CURRENT_STATE
            
        if CURRENT_STATE == STATE_IDLE:
            pass 
        elif CURRENT_STATE == STATE_RECORD:
            CURRENT_STATE = await phase_1_record()
        elif CURRENT_STATE == STATE_ORIENTATION:
            CURRENT_STATE = await phase_2_orientation()
        elif CURRENT_STATE == STATE_SEEK:
            CURRENT_STATE = await phase_3_seek()
        elif CURRENT_STATE == STATE_DETOUR:
            CURRENT_STATE = await phase_4_detour()
        elif CURRENT_STATE == STATE_ARRIVAL:
            CURRENT_STATE = await phase_5_arrival()

if __name__ == "__main__":
    try:
        asyncio.run(super_loop())
    except KeyboardInterrupt:
        robot.stop()
        print("\nNavigation offline.")