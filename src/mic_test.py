import machine
import time

# --- CONFIGURATION ---
# Match the physical labels on your Arduino Nano ESP32 board
ANALOG_PINS = ["A0", "A1", "A2", "A3", "A4", "A5", "A6"]
DIGITAL_PINS = ["D2", "D3", "D4", "D5", "D6", "D7", "D8"] # Map to your digital connections

MAX_WIDTH = 20
LERP_FACTOR = 0.2

# Initialize Analog Inputs
adcs = [machine.ADC(machine.Pin(p), atten=machine.ADC.ATTN_11DB) for p in ANALOG_PINS]

# Initialize Digital Inputs with internal pull-downs to prevent floating when unlinked
digitals = [machine.Pin(p, machine.Pin.IN, machine.Pin.PULL_DOWN) for p in DIGITAL_PINS]

# State tracker for each sensor
states = [{"val": 0} for _ in ANALOG_PINS]

# ANSI Colors
RED = "\033[91m"
GREEN = "\033[92m"
BLUE = "\033[94m"
RESET = "\033[0m"

def get_bar(value, width):
    length = int((value / 65535) * width)
    length = max(0, min(length, width))
    return "#" * length + " " * (width - length)

print("Monitoring Analog + Digital Sensors...")

try:
    while True:
        print("\033[H", end='') # Home cursor to top-left to avoid flicker
        
        for i in range(len(ANALOG_PINS)):
            # Poll both hardware layers
            raw_analog = adcs[i].read_u16()
            raw_digital = digitals[i].value() # Returns 0 or 1
            
            # Smooth the analog value
            states[i]["val"] += (raw_analog - states[i]["val"]) * LERP_FACTOR
            
            # Build the bar representation
            bar = get_bar(states[i]["val"], MAX_WIDTH)
            
            # Formatting the digital trigger message
            # KY-037 typically outputs 1 (HIGH) when a sound threshold is crossed
            if raw_digital == 1:
                digi_status = f"{RED}[TRIGGERED]{RESET}"
            else:
                digi_status = f"{GREEN}[ QUIET ]{RESET}"
            
            print(f"Sensor {i} -> Analog: |{bar}| {int(states[i]['val']):05d} | Digital: {digi_status}")
            
        time.sleep(0.02)
except KeyboardInterrupt:
    print("\n\nStopped.")