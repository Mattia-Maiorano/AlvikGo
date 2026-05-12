import network
import uasyncio as asyncio
import json
import time

LOG_BUFFER_SIZE = 100
_logs = []

# Virtual Button States
virtual_request_record = False
virtual_request_seek = False

def web_log(msg):
    timestamp = time.ticks_ms() / 1000.0
    _logs.append(f"[{timestamp:.1f}] {msg}")
    if len(_logs) > LOG_BUFFER_SIZE:
        _logs.pop(0)

# Provide functions for main.py to read and clear the virtual flags
def check_virtual_record():
    global virtual_request_record
    if virtual_request_record:
        virtual_request_record = False
        return True
    return False

def check_virtual_seek():
    global virtual_request_seek
    if virtual_request_seek:
        virtual_request_seek = False
        return True
    return False

async def connect_wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(False)
    await asyncio.sleep(0.5)
    wlan.active(True)
    await asyncio.sleep(0.5)

    if not wlan.isconnected():
        print(f"[WIFI] Connecting to '{ssid}'...")
        try:
            wlan.disconnect()
            wlan.connect(ssid, password)
        except OSError as e:
            print(f"[WIFI] Radio Crash: {e}.")
            return None
        
        attempts = 0
        while not wlan.isconnected() and attempts < 15:
            print(f"[WIFI] Trying to connect... ({attempts+1}/15)")
            await asyncio.sleep(1)
            attempts += 1
            
        if not wlan.isconnected():
            print("[WIFI] FATAL: Could not connect.")
            return None
            
    ip = wlan.ifconfig()[0]
    print(f"[WIFI] Connected! UI at: http://{ip}")
    return ip

# Updated UI with buttons that send POST requests
HTML_PAGE = """<!DOCTYPE html>
<html><head><title>Alvik Telemetry</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
    body { background: #1e1e1e; color: #00ff00; font-family: monospace; padding: 20px; }
    h2 { color: #fff; border-bottom: 1px solid #444; padding-bottom: 10px; }
    #logs { white-space: pre-wrap; font-size: 14px; line-height: 1.4; margin-top: 20px; border: 1px solid #333; padding: 10px; height: 300px; overflow-y: scroll; }
    .btn { background: #444; color: white; border: none; padding: 15px 20px; margin: 5px; cursor: pointer; font-size: 16px; border-radius: 5px; }
    .btn:hover { background: #666; }
</style>
<script>
function sendCommand(cmd) {
    fetch('/command', { method: 'POST', body: cmd });
}
setInterval(() => {
    fetch('/logs').then(r => r.json()).then(data => {
        const logDiv = document.getElementById('logs');
        const newText = data.join('\\n');
        if (logDiv.innerText !== newText) {
            logDiv.innerText = newText;
            logDiv.scrollTop = logDiv.scrollHeight;
        }
    }).catch(e => console.log("Connection paused..."));
}, 1000);
</script>
</head><body>
    <h2>🤖 Alvik Live Telemetry</h2>
    <div>
        <button class="btn" onclick="sendCommand('RECORD')">🔴 RECORD TARGET</button>
        <button class="btn" onclick="sendCommand('SEEK')">🟢 SEEK TARGET</button>
    </div>
    <div id="logs">Waiting for logs...</div>
</body></html>
"""

async def handle_client(reader, writer):
    global virtual_request_record, virtual_request_seek
    try:
        request_line = await reader.readline()
        req = request_line.decode().split()
        
        # Determine if there is a body to read (for POST requests)
        content_length = 0
        while True:
            line = await reader.readline()
            if line == b"\r\n" or not line:
                break
            line_str = line.decode().lower()
            if line_str.startswith("content-length:"):
                content_length = int(line_str.split(":")[1].strip())

        if len(req) > 1:
            method = req[0]
            path = req[1]
            
            if method == "POST" and path == '/command':
                body = await reader.read(content_length)
                cmd = body.decode()
                if cmd == 'RECORD':
                    virtual_request_record = True
                elif cmd == 'SEEK':
                    virtual_request_seek = True
                writer.write('HTTP/1.1 200 OK\r\n\r\n')
                
            elif path == '/logs':
                response = json.dumps(_logs)
                writer.write('HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n')
                writer.write(response)
            else:
                writer.write('HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n')
                writer.write(HTML_PAGE)

        await writer.drain()
    except Exception as e:
        pass
    finally:
        writer.close()
        await writer.wait_closed()

async def start_server(ssid, password):
    await connect_wifi(ssid, password)
    await asyncio.start_server(handle_client, "0.0.0.0", 80)