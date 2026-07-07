from machine import SoftI2C, Pin
import time

print("--- STARTING SOFT-I2C SNIPER SCAN ---")

# The most common SDA / SCL pin pairs for the Nano ESP32 and Alvik routing
pin_pairs = [
    (11, 12),  # Standard Nano ESP32 Arduino pins (Most likely)
    (8, 9),    # Alternative raw ESP32 mapping
    (4, 5),    # Common generic Grove routing
    (18, 19),  # Default hardware I2C pins
    (1, 2)     # Secondary mapping
]

found_anything = False

for sda_pin, scl_pin in pin_pairs:
    try:
        print(f"Testing SDA={sda_pin}, SCL={scl_pin}...", end=" ")
        
        # SoftI2C forces these specific pins to act as I2C
        i2c = SoftI2C(scl=Pin(scl_pin), sda=Pin(sda_pin), freq=100000)
        devices = i2c.scan()
        
        if devices:
            print(f"SUCCESS! Found device at: {[hex(d) for d in devices]}")
            found_anything = True
        else:
            print("Nothing.")
    except Exception as e:
        print(f"Invalid pin combo: {e}")
        
    time.sleep(0.1) # Tiny pause to let the bus clear

if not found_anything:
    print("\n--- SNIPER SCAN FAILED ---")
    print("Conclusion: The ESP32 physically cannot see the data lines.")