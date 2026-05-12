from arduino_alvik import ArduinoAlvik
import time

print("--- ALVIK PULSE CHECK ---")
print("Attempting to connect to hardware...")

try:
    # Initialize using the correct class name
    alvik = ArduinoAlvik()
    alvik.begin()
    print("Hardware initialized successfully!")
    
    # Loop 5 times: Flash Green, Print to terminal
    for i in range(5):
        print(f"Pulse {i + 1}/5")
        
        # Turn LEDs bright Green (using 1 for ON, 0 for OFF)
        alvik.left_led.set_color(0, 1, 0)
        alvik.right_led.set_color(0, 1, 0)
        
        time.sleep(0.5)
        
        # Turn LEDs off
        alvik.left_led.set_color(0, 0, 0)
        alvik.right_led.set_color(0, 0, 0)
        
        time.sleep(0.5)

    print("Pulse check complete. Alvik is healthy.")

except Exception as e:
    print(f"CRITICAL ERROR: {e}")