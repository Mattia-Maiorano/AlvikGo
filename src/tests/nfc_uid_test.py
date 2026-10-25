from machine import SoftI2C, Pin
import time

class MiniPN532:
    def __init__(self, scl_pin, sda_pin):
        # Using the specific pins discovered by the sniper script
        self.i2c = SoftI2C(scl=Pin(scl_pin), sda=Pin(sda_pin), freq=100000)
        self.addr = 0x24

    def send_cmd(self, cmd, expected_payload_len):
        length = len(cmd) + 1
        buf = bytearray([0x00, 0x00, 0xFF, length, (~length + 1) & 0xFF, 0xD4])
        buf.extend(cmd)
        buf.append((256 - ((0xD4 + sum(cmd)) & 0xFF)) & 0xFF)
        buf.append(0x00)
        
        # This line will now throw an OSError if wires are disconnected
        self.i2c.writeto(self.addr, buf)
        
        if not self._wait_ready(): return None
        self.i2c.readfrom(self.addr, 7) 
        
        if not self._wait_ready(): return None
        total_len = expected_payload_len + 9
        res = self.i2c.readfrom(self.addr, total_len)
        return res[8:8+expected_payload_len]

    def _wait_ready(self):
        # Poll the board until it sets the "Ready" bit to 1
        for _ in range(20):
            try:
                if self.i2c.readfrom(self.addr, 1)[0] == 0x01:
                    return True
            except:
                pass
            time.sleep(0.05)
        return False

    def initialize(self):
        # Send a tiny electrical pulse to wake the chip up
        try: self.i2c.readfrom(self.addr, 1)
        except: pass
        time.sleep(0.1)
        
        # Configure the Security Access Module (SAM) to Normal Mode
        self.send_cmd([0x14, 0x01, 0x14, 0x01], 1)
        
    def scan_for_tag(self):
        # Ask the chip to look for passive tags (like your keychain)
        res = self.send_cmd([0x4A, 0x01, 0x00], 11)
        
        # If a tag is found, extract its Unique ID (UID)
        if res and res[0] == 1: 
            uid_len = res[4]
            uid = res[5:5+uid_len]
            # Format the hex numbers cleanly (e.g., A1-B2-C3-D4)
            return "-".join([f"{x:02X}" for x in uid])
        return None

# --- MAIN EXECUTABLE ---
import sys

print("Initializing PN532 on Pins SDA=11, SCL=12...")
nfc = MiniPN532(scl_pin=12, sda_pin=11)

# --- THE STRICT HARDWARE CHECK ---
print("Pinging I2C Bus...")
devices = nfc.i2c.scan()

if nfc.addr not in devices:
    print("\n❌ CRITICAL ERROR: PN532 is completely dead or disconnected!")
    print("Check your naked wires inside the white Grove socket.")
    sys.exit() # Stop the script completely
# ---------------------------------

nfc.initialize()

print("✅ Hardware Ready!")
print("Tap your blue keychain or white card to the red board...")
print("(Press Ctrl+C to stop)\n")

while True:
    try:
        tag_uid = nfc.scan_for_tag()
        
        if tag_uid:
            print(f"\n✅ TAG DETECTED: {tag_uid}")
            time.sleep(2) 
        else:
            print(".", end="") # Normal speed dots
            time.sleep(0.2)
            
    except OSError:
        # This triggers the moment the SDA/SCL wires lose contact
        print("\n❌ HARDWARE DISCONNECTED! (The I2C bus is empty)")
        
        # Try to heal
        connected = False
        while not connected:
            print("Searching for PN532...", end="\r")
            try:
                if 0x24 in nfc.i2c.scan():
                    nfc.initialize()
                    print("\n✅ HARDWARE RECONNECTED!")
                    connected = True
                else:
                    time.sleep(1)
            except:
                time.sleep(1)