# Arduino Alvik Robot Project

This project controls an Arduino Alvik robot using an Arduino Nano ESP32 running MicroPython. It includes features for pathfinding, obstacle detection, NFC scanning, and remote telemetry logging.

## Features

*   **Asynchronous Task Scheduling**: Uses `uasyncio` for non-blocking concurrent execution of robotic tasks, like driving, sensor monitoring, and telemetry logging.
*   **Hardware Control**: Controls the Arduino Alvik base (motors and LEDs) seamlessly using the `arduino-alvik` library.
*   **NFC Scanning**: Integrates an asynchronous PN532 module for scanning RFID/NFC tags for interactions (like checkpoints or routing logic).
*   **Time of Flight (ToF) Sensors**: Uses obstacle detection logic (filtering noise) and safely brakes the robot if it comes too close to an obstruction.
*   **Web-based Telemetry Logging**: Uses `uasyncio` to host a simple web server, broadcasting live debug messages to a web UI and accepting virtual interaction commands.

## Repository Structure

*   `src/`: Main source directory for MicroPython code.
    *   `main.py`: The entry point and main logic loop for the robot, coordinating sensors, async tasks, and motors.
    *   `web_logger.py`: Module for managing the asynchronous web server for telemetry and remote interaction.
*   `src/tests/`: Hardware testing scripts and small utilities used during development (e.g., checking microphone, NFC, and general pulse).
*   `src/archive/`: Old or experimental iterations of the main script (`BROKEN_main.py`, `PARTIAL_main.py`).
*   `firmware/`: Holds MicroPython firmware builds (like `ARDUINO_NANO_ESP32*.bin`) specific to this project context.

## Hardware Requirements

*   Arduino Nano ESP32
*   Arduino Alvik Robot Carrier
*   NFC/RFID Module (PN532) connected via I2C.

## Software Requirements & Setup

You will need `mpremote` and `esptool` installed on your host machine to interact with the ESP32. You can install them via pip:
```bash
pip install mpremote
```

Ensure you install these required MicroPython packages via `mpremote mip` on the device:
*   `arduino-alvik`
*   `micropython-uasyncio`

### Flashing Firmware
Use `esptool.py` to flash the MicroPython binary provided in the `firmware/` directory to your Arduino Nano ESP32. Or, if using Arduino IDE/dfu-util, push the .bin file via DFU mode.

### Running the Code
1. Upload the `src/main.py` and `src/web_logger.py` to the board using `mpremote` or VSCode with the MicroPico extension.
2. The `main.py` will execute on boot (if renamed to `main.py` or configured via `boot.py`).
3. View the serial console using `mpremote repl`.
