import math
import time
import uasyncio as asyncio
import builtins

# --- INJECT WEB LOGGER ---
from web_logger import web_log, start_server, check_virtual_record, check_virtual_seek

def custom_print(*args, **kwargs):
    """Overrides default print to output to both terminal and web UI."""
    msg = " ".join(str(a) for a in args)
    builtins.print(msg)  
    web_log(msg)         

print = custom_print     
# -------------------------

try:
    import bluetooth
except ImportError:
    bluetooth = None

try:
    import network
except ImportError:
    network = None

# Navigation tuning
SAMPLE_DURATION_MS = 2000
FAST_SAMPLE_DURATION_MS = 500
SAMPLES_PER_SCAN = 3
MISSING_RSSI = -100
ARRIVAL_THRESHOLD = 120.0

# Motion tuning (calibrate for your robot)
FORWARD_SPEED = 0.5  # normalized speed 0..1
TURN_SPEED = 0.45
SEGMENT_CM = 100
MICRO_SCAN_DEG = 30
DETOUR_CM = 40
OBSTACLE_CM = 10
CHUNK_CM = 5         # <-- NEW: Distance to drive before pausing to check ToF

# Button handling
LONG_PRESS_MS = 2000
DOUBLE_CLICK_MS = 1000

# Globals
TARGET_FINGERPRINT = None


# -----------------------------
# Hardware Abstraction Layer
# -----------------------------
class AlvikHAL:
    def __init__(self):
        print("[INIT] Booting Alvik Hardware Abstraction Layer...")
        self.robot = self._init_robot()
        self.motors = getattr(self.robot, "motors", None)
        self.buzzer = getattr(self.robot, "buzzer", None)
        self.tof_sensors = self._init_tof_sensors()
        self._tof_error_count = 0
        print(f"[INIT] Found {len(self.tof_sensors)} ToF sensors.")

        if network is None:
            raise RuntimeError("network module not available.")
        self.wlan = network.WLAN(network.STA_IF)

        # Force cycle the radio to clear any stuck states
        self.wlan.active(False)
        time.sleep(0.1)
        self.wlan.active(True)
        self.wlan.disconnect()
        print("[WIFI] Radio initialized and disconnected for pure scanning.")

    def _init_robot(self):
        try:
            from arduino_alvik import ArduinoAlvik
        except ImportError:
            raise RuntimeError("Alvik library not found.")
        robot = ArduinoAlvik()
        robot.begin() 
        return robot

    def _init_tof_sensors(self):
        return [1, 2, 3, 4, 5]

    def read_tof_cm(self):
        """Synchronous, safe ToF reading."""
        try:
            distances = self.robot.get_distance()            
            if distances:
                valid_distances = [d for d in distances if isinstance(d, (int, float)) and d > 0]
                if valid_distances:
                    self._tof_error_count = 0 
                    return min(valid_distances) 
                
            raise ValueError("Invalid ToF Data - Empty or Zero")
        except Exception as e:
            self._tof_error_count += 1
            if self._tof_error_count % 5 == 0:
                print(f"[TOF I2C ERROR] {e} (Consecutive: {self._tof_error_count})")
            
            if self._tof_error_count > 10:
                print("[TOF RECOVERY] Restarting Alvik I2C Hardware...")
                try:
                    self.robot.begin()
                    self._tof_error_count = 0
                except:
                    pass
        return None

    def motors_stop(self):
        try:
            self.robot.stop()
        except Exception:
            pass

    async def drive_cm(self, cm, speed, abort_event=None):
        """Step-and-Scan Driving: Drives a chunk, stops, checks ToF, repeats."""
        print(f"[MOTION] Step-and-Scanning {cm}cm")
        direction = 1 if cm >= 0 else -1
        abs_cm = abs(cm)
        driven = 0

        while driven < abs_cm:
            if abort_event and abort_event.is_set():
                print("[MOTION] Drive aborted due to active obstacle event.")
                break

            step = min(CHUNK_CM, abs_cm - driven)
            
            # 1. Drive the physical chunk using the official Alvik API
            try:
                if hasattr(self.robot, "move"):
                    # The official API blocks by default, which safely pauses the loop!
                    self.robot.move(step * direction, "cm")
                elif self.motors and hasattr(self.motors, "drive_cm"):
                    # Fallback just in case you are using a custom wrapper
                    self.motors.drive_cm(step * direction, speed)
                else:
                    print("[ERROR] No valid movement API found on robot!")
                    break
            except Exception as e:
                print(f"[MOTION ERROR] {e}")
            
            driven += step

            # 2. Check the ToF safely while motors are stopped
            dist = self.read_tof_cm()
            if dist is not None and dist < OBSTACLE_CM:
                print(f"[SAFETY] Obstacle detected at {dist}cm! Stopping.")
                self.set_leds(255, 0, 0)
                self.beep(300, 120)
                if abort_event:
                    abort_event.set()
                break
            
            # Keep the web logger and buttons responsive
            await asyncio.sleep_ms(20)
            
        self.motors_stop()
        self.set_leds(0, 0, 0)

    async def rotate_deg(self, deg, speed, abort_event=None):
        start_rot = time.ticks_ms()
        print(f"[MOTION] >>> Starting Rotation: {deg}deg")
        
        CHUNK_DEG = 15 
        direction = 1 if deg >= 0 else -1
        abs_deg = abs(deg)
        rotated = 0
        
        while rotated < abs_deg:
            step = min(CHUNK_DEG, abs_deg - rotated)
            
            # --- HIGH DEPTH LOGGING ---
            t_call = time.ticks_ms()
            print(f"[HW_DEBUG] Calling self.robot.rotate({step * direction}) | Total so far: {rotated}")
            
            try:
                # This is the line where we suspect the hang
                self.robot.rotate(step * direction, "deg")
                
                t_resp = time.ticks_ms()
                print(f"[HW_DEBUG] Hardware returned in {time.ticks_diff(t_resp, t_call)}ms")
            except Exception as e:
                print(f"[HW_DEBUG] Hardware CRASHED: {e}")
                break
            
            rotated += step
            await asyncio.sleep_ms(20) # Give Wi-Fi a chance to flush the logs
            
        print(f"[MOTION] <<< Rotation Complete. Took {time.ticks_diff(time.ticks_ms(), start_rot)}ms")

    def button_pressed(self):
        try:
            if hasattr(self.robot, "get_touch_ok"):
                return bool(self.robot.get_touch_ok())
            if hasattr(self.robot, "touch_ok"):
                return bool(self.robot.touch_ok.is_pressed())
        except Exception:
            pass
        return False

    def set_leds(self, r, g, b):
        try:
            if hasattr(self.robot, "left_led") and hasattr(self.robot, "right_led"):
                self.robot.left_led.set_color(r, g, b)
                self.robot.right_led.set_color(r, g, b)
        except Exception:
            pass

    def show_idle(self):
        self.set_leds(0, 0, 50)  
    def show_recorded(self):
        self.set_leds(0, 255, 0)  
    def show_navigating(self):
        self.set_leds(100, 0, 100)  
    def show_check(self):
        self.set_leds(0, 255, 0)  
    def show_warning(self):
        self.set_leds(255, 0, 0)  

    def beep(self, freq=1200, duration_ms=120):
        if self.buzzer and hasattr(self.buzzer, "beep"):
            self.buzzer.beep(freq, duration_ms)

    def melody_success(self):
        for freq in (880, 1175, 1568):
            self.beep(freq, 120)
            time.sleep_ms(80)


# -----------------------------
# BLE / Wi-Fi Fingerprinting
# -----------------------------
_IRQ_SCAN_RESULT = 5
_IRQ_SCAN_DONE = 6

class BLEScanner:
    def __init__(self):
        if bluetooth is None:
            raise RuntimeError("bluetooth module not available.")
        self.ble = bluetooth.BLE()
        self.ble.active(True)
        self._results = {}
        self._scan_done = False
        self.ble.irq(self._irq)

    def _irq(self, event, data):
        try:
            if event == _IRQ_SCAN_RESULT:
                addr_type, addr, adv_type, rssi, adv_data = data
                raw_mac = bytes(addr) 
                self._results.setdefault(raw_mac, []).append(rssi)
            elif event == _IRQ_SCAN_DONE:
                self._scan_done = True
        except Exception as e:
            # CRITICAL: Prevent silent crashes inside the hardware interrupt!
            print(f"[BLE IRQ FATAL ERROR] {e}")

    async def scan(self, duration_ms):
        print(f"[DEBUG BLE] I. Starting scan for {duration_ms}ms")
        self._results = {}
        self._scan_done = False
        
        try:
            self.ble.gap_scan(None)
            await asyncio.sleep_ms(50)  
        except Exception:
            pass
            
        try:
            print("[DEBUG BLE] III. Triggering passive gap_scan...")
            self.ble.gap_scan(duration_ms, 30000, 15000, False) 
            print("[DEBUG BLE] IV. gap_scan triggered successfully.")
        except OSError as e:
            if e.args[0] == 120:  
                pass
            else:
                raise  
                
        print("[DEBUG BLE] V. Entering wait loop...")
        start_time = time.ticks_ms()
        
        # CRITICAL FIX: Safer timeout logic that won't break if the clock rolls over
        while not self._scan_done:
            if time.ticks_diff(time.ticks_ms(), start_time) > (duration_ms + 1000):
                print("[DEBUG BLE] ... Timeout triggered in wait loop!")
                break
            await asyncio.sleep_ms(20)
            
        print(f"[DEBUG BLE] VI. Wait loop exited. scan_done={self._scan_done}, found {len(self._results)} devices.")
        
        if not self._scan_done:
            print("[RADIO WARNING] BLE scan timed out! Preventing freeze.")
            try:
                self.ble.gap_scan(None) 
            except:
                pass

        processed_results = {}
        for mac_bytes, rssi_list in self._results.items():
            # Convert bytes to string only now, in a safe memory context
            mac_str = ":".join("%02x" % b for b in mac_bytes)
            processed_results[mac_str] = _average(rssi_list)
            
        return processed_results

    @staticmethod
    def _addr_to_str(addr):
        return ":".join("%02x" % b for b in addr)

class WLANScanner:
    def __init__(self):
        if network is None:
            raise RuntimeError("network module not available.")
        self.wlan = network.WLAN(network.STA_IF)
        if not self.wlan.active():
            self.wlan.active(True)

    def scan(self):
        results = {}
        try:
            for ssid, bssid, channel, rssi, authmode, hidden in self.wlan.scan():
                mac = ":".join("%02x" % b for b in bssid)
                results[mac] = rssi
        except Exception as e:
            print(f"[RADIO WARNING] WLAN scan failed: {e}")
        return results

# --- NEW: Global Singletons ---
# This ensures the radio is only initialized once, preventing hardware lockups.
_global_ble_scanner = None
_global_wlan_scanner = None

def get_ble_scanner():
    global _global_ble_scanner
    if bluetooth and _global_ble_scanner is None:
        _global_ble_scanner = BLEScanner()
    return _global_ble_scanner

def get_wlan_scanner():
    global _global_wlan_scanner
    if network and _global_wlan_scanner is None:
        _global_wlan_scanner = WLANScanner()
    return _global_wlan_scanner


async def scan_environment(samples=SAMPLES_PER_SCAN, duration_ms=SAMPLE_DURATION_MS, fast_mode=False):
    print(f"[DEBUG SCAN_ENV] A. Started. fast_mode={fast_mode}")
    if fast_mode:
        samples = 1
        duration_ms = FAST_SAMPLE_DURATION_MS
    
    ble_scanner = get_ble_scanner()
    wlan_scanner = get_wlan_scanner()

    aggregations = {}
    for i in range(samples):
        print(f"[DEBUG SCAN_ENV] B. Starting sample {i+1}/{samples}")
        ble_task = None
        if ble_scanner:
            ble_task = asyncio.create_task(ble_scanner.scan(duration_ms))
        
        wifi_results = {}
        if not fast_mode and wlan_scanner:
            print("[DEBUG SCAN_ENV] C. Executing WLAN scan...")
            wifi_results = wlan_scanner.scan()
            print("[DEBUG SCAN_ENV] D. WLAN scan finished.")
            
        ble_results = {}
        if ble_task:
            print("[DEBUG SCAN_ENV] E. Awaiting BLE task...")
            ble_results = await ble_task
            print("[DEBUG SCAN_ENV] F. BLE task finished.")

        _merge_results(aggregations, ble_results, "BLE:")
        _merge_results(aggregations, wifi_results, "WIFI:")
        
        if not fast_mode:
            await asyncio.sleep_ms(50)

    print("[DEBUG SCAN_ENV] G. Returning results.")
    final_results = {k: _average(v) for k, v in aggregations.items()}
    return final_results

def _merge_results(aggregations, results, prefix):
    for mac, rssi in results.items():
        key = prefix + mac
        aggregations.setdefault(key, []).append(rssi)

def _average(values):
    return sum(values) / max(1, len(values))

def calculate_vector_distance(current_dict, target_dict):
    if not target_dict:
        return float("inf")
    keys = set(target_dict.keys()) | set(current_dict.keys())
    total = 0.0
    for key in keys:
        cur = current_dict.get(key, MISSING_RSSI)
        tgt = target_dict.get(key, MISSING_RSSI)
        diff = cur - tgt
        total += diff * diff
    mse = total / max(1, len(keys))
    return mse


# -----------------------------
# Navigation State Machine
# -----------------------------
STATE_IDLE = 0
STATE_NAVIGATING = 1

class Navigator:
    def __init__(self, hal):
        self.hal = hal
        self.state = STATE_IDLE
        self.target = None
        self.obstacle_event = asyncio.Event()
        self._request_record = False
        self._request_seek = False

    async def run(self):
        self.hal.show_idle()
        print("[STATE] Idle. Waiting for input...")
        asyncio.create_task(self._button_task())
        # Removed the background _tof_task to prevent hardware collisions
        await self._state_task()

    async def _state_task(self):
        global TARGET_FINGERPRINT
        while True:
            try:
                if self._request_record:
                    self._request_record = False
                    print("[STATE] Transitioning to RECORD MODE")
                    self.hal.motors_stop()
                    print("[STATE] Acquiring target map...")
                    self.target = await scan_environment(fast_mode=False)
                    TARGET_FINGERPRINT = self.target
                    if self.target:
                        print(f"[STATE] Map Acquired! {len(self.target)} unique signals.")
                        self.hal.beep(1200, 100)
                        self.hal.show_recorded()
                    else:
                        print("[STATE] Failed to record any signals.")
                        self.hal.beep(200, 200)
                        self.hal.show_warning()
                    await asyncio.sleep_ms(400)
                    self.hal.show_idle()

                if self._request_seek:
                    self._request_seek = False
                    if not self.target:
                        print("[ERROR] Cannot seek. No target fingerprint recorded.")
                        self.hal.beep(200, 200)
                        self.hal.show_warning()
                        await asyncio.sleep_ms(500)
                        self.hal.show_idle()
                    else:
                        print("[STATE] Transitioning to SEEK MODE")
                        self.state = STATE_NAVIGATING
                        self.hal.show_navigating()
                        await self._navigate_to_target()
                        self.state = STATE_IDLE
                        self.hal.show_idle()
                        
            except Exception as e:
                print(f"[STATE ERROR] Crash caught in main loop: {e}")
                self.state = STATE_IDLE
                self.hal.show_idle()
                
            await asyncio.sleep_ms(50)

    async def _button_task(self):
        heartbeat_counter = 0
        while True:
            try:
                heartbeat_counter += 1
                if heartbeat_counter >= 100:
                    heartbeat_counter = 0

                if self.state == STATE_IDLE:
                    if check_virtual_record():
                        print("[INPUT] Web UI Action: RECORD")
                        self._request_record = True
                    elif check_virtual_seek():
                        print("[INPUT] Web UI Action: SEEK")
                        self._request_seek = True

                if self.state == STATE_IDLE and self.hal.button_pressed():
                    press_start = time.ticks_ms()
                    while self.hal.button_pressed():
                        await asyncio.sleep_ms(20)

                    duration = time.ticks_diff(time.ticks_ms(), press_start)
                    if duration >= LONG_PRESS_MS:
                        self._request_record = True
                    elif duration > 50:
                        self._request_seek = True
                    await asyncio.sleep_ms(200)

            except Exception as e:
                print(f"[BUTTON TASK ERROR] {e}")
            await asyncio.sleep_ms(50)

    async def _navigate_to_target(self):
        await self._radar_orientation()
        while self.state == STATE_NAVIGATING:
            if await self._handle_obstacle_if_needed():
                continue
            arrived = await self._drive_segment()
            if arrived:
                return

    async def _drive_segment(self):
        current_scan = await scan_environment(fast_mode=True)
        score = calculate_vector_distance(current_scan, self.target)
        print(f"[NAV] Current vector error score: {score:.2f}")
        
        if score < ARRIVAL_THRESHOLD:
            print("[NAV] Threshold reached! Confirming with precision scan...")
            confirm_scan = await scan_environment(fast_mode=False)
            confirm_score = calculate_vector_distance(confirm_scan, self.target)
            
            if confirm_score < ARRIVAL_THRESHOLD:
                print(f"[NAV] Destination confirmed! (Score: {confirm_score:.2f})")
                self.hal.motors_stop()
                self.hal.melody_success()
                self.hal.show_check()
                await asyncio.sleep_ms(800)
                return True
            else:
                print(f"[NAV] False alarm. Score is actually {confirm_score:.2f}. Continuing...")
                score = confirm_score 

        drive_dist = SEGMENT_CM
        if score < 180:
            drive_dist = 20  
            print(f"[NAV] Approaching target. Shrinking step to {drive_dist}cm")
        elif score < 350:
            drive_dist = 50  
            print(f"[NAV] Closing in. Shrinking step to {drive_dist}cm")

        await self.hal.drive_cm(drive_dist, FORWARD_SPEED, abort_event=self.obstacle_event)
        if self.obstacle_event.is_set():
            return False

        await self._micro_scan_adjust()
        return False

    async def _radar_orientation(self):
        print("[RADAR] Initiating Weighted Slope Analysis...")
        steps = 8  
        angle_per_step = 360 // steps
        current_angle = 0
        raw_scores = {} 

        for step in range(steps):
            print(f"\n[DEBUG RADAR] --- Starting Step {step} ---")
            if step > 0:
                print("[DEBUG RADAR] 1. Calling rotate_deg...")
                await self.hal.rotate_deg(angle_per_step, TURN_SPEED, abort_event=self.obstacle_event)
                current_angle += angle_per_step
                print("[DEBUG RADAR] 2. Finished rotate_deg.")

            print("[DEBUG RADAR] 3. Calling scan_environment...")
            scan = await scan_environment(fast_mode=True)
            print("[DEBUG RADAR] 4. Finished scan_environment.")
            
            score = calculate_vector_distance(scan, self.target)
            raw_scores[current_angle % 360] = score
            print(f"[RADAR] Sample at {current_angle % 360} deg: {score:.2f}")

        slope_results = {}
        for i in range(steps):
            theta = (i * angle_per_step) % 360
            s_target = raw_scores[theta]
            s_opp  = raw_scores[(theta + 180) % 360]
            s_bl   = raw_scores[(theta + 135) % 360] 
            s_br   = raw_scores[(theta + 225) % 360] 
            s_fl   = raw_scores[(theta + 45) % 360]  
            s_fr   = raw_scores[(theta + 315) % 360] 
            
            intensity = (
                2.0 * (s_opp - s_target) + 
                1.5 * (s_bl - s_target) + 
                1.5 * (s_br - s_target) + 
                1.0 * (s_fl - s_target) + 
                1.0 * (s_fr - s_target)
            )
            slope_results[theta] = intensity

        best_heading = max(slope_results, key=slope_results.get)
        correction = best_heading - (current_angle % 360)
        if correction > 180: correction -= 360
        elif correction < -180: correction += 360

        print(f"[RADAR] Slope winner: {best_heading} deg. Final adjustment: {correction} deg.")
        await self.hal.rotate_deg(correction, TURN_SPEED, abort_event=self.obstacle_event)

    async def _micro_scan_adjust(self):
        print("[MICRO SCAN] Sampling Center trajectory...")
        center_scan = await scan_environment(fast_mode=True)
        center_score = calculate_vector_distance(center_scan, self.target)

        print("[MICRO SCAN] Sampling Left trajectory...")
        await self.hal.rotate_deg(-MICRO_SCAN_DEG, TURN_SPEED, abort_event=self.obstacle_event)
        left_scan = await scan_environment(fast_mode=True)
        left_score = calculate_vector_distance(left_scan, self.target)

        print("[MICRO SCAN] Sampling Right trajectory...")
        await self.hal.rotate_deg(MICRO_SCAN_DEG * 2, TURN_SPEED, abort_event=self.obstacle_event)
        right_scan = await scan_environment(fast_mode=True)
        right_score = calculate_vector_distance(right_scan, self.target)

        denominator = 2 * (left_score + right_score - 2 * center_score)
        
        if denominator > 0:
            optimal_offset = MICRO_SCAN_DEG * (left_score - right_score) / denominator
            optimal_offset = max(-MICRO_SCAN_DEG * 2, min(MICRO_SCAN_DEG * 2, optimal_offset))
            print(f"[MICRO SCAN] Peak heading is {optimal_offset:.1f} deg from center.")
        else:
            if left_score < right_score:
                optimal_offset = -MICRO_SCAN_DEG
            else:
                optimal_offset = MICRO_SCAN_DEG
            print(f"[MICRO SCAN] Linear fallback. Peak heading mapped to {optimal_offset:.1f} deg.")

        turn_needed = optimal_offset - MICRO_SCAN_DEG
        if abs(turn_needed) > 2.0: 
            await self.hal.rotate_deg(turn_needed, TURN_SPEED, abort_event=self.obstacle_event)

        return min(left_score, center_score, right_score)

    async def _handle_obstacle_if_needed(self):
        if not self.obstacle_event.is_set():
            return False

        print("[SAFETY] Executing Detour Maneuver...")
        self.obstacle_event.clear()
        
        await self.hal.rotate_deg(90, TURN_SPEED)
        await self.hal.drive_cm(DETOUR_CM, FORWARD_SPEED)
        await self.hal.rotate_deg(-90, TURN_SPEED)
        await self.hal.drive_cm(DETOUR_CM, FORWARD_SPEED)

        print("[SAFETY] Detour complete. Re-orienting to target...")
        await self._radar_orientation()
        return True


async def main():
    WIFI_SSID = "sky_ext"
    WIFI_PASSWORD = "QHbv6IDXxceT"
    
    try:
        asyncio.create_task(start_server(WIFI_SSID, WIFI_PASSWORD))
        hal = AlvikHAL()
        navigator = Navigator(hal)
        await navigator.run()
    except Exception as e:
        print(f"[FATAL ERROR] {e}")


try:
    asyncio.run(main())
finally:
    asyncio.new_event_loop()