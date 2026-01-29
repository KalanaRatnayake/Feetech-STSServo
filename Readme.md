# Feetech STS Servo — Linux Sweep Example

This repo demonstrates controlling a Feetech STS servo (e.g., STS3215) via the FE-URT-1 interface board from Linux using /dev/ttyUSB0.

## What’s Included
- Header-only driver: STSServoDriver.hpp — Linux-friendly version of the original Arduino driver.
- Sweep example: sweep_motion_with_interface_board.cpp — moves the servo to ~0°, ~180°, ~360° repeatedly.

## How It Works
- LinuxSerial wrapper: Minimal serial class that opens /dev/ttyUSB0 using POSIX (open, read, write) and configures the port to 1 Mbps via termios (B1000000).
- Arduino shims: The driver provides small replacements for byte, delay(), delayMicroseconds(), and no-op pinMode() / digitalWrite() so the same protocol logic compiles on Linux.
- Protocol: Packets are built as FF FF | ID | LEN | INSTR | PARAMS | ~CHECKSUM and parsed similarly.
- Servo types: The driver auto-detects STS vs SCS per servo ID and applies the correct endianness and sign handling.

## Build
From the project root, build the examples (header-only driver, no extra sources needed):

```bash
g++ -std=c++17 -O2 -o sweep_motion_with_interface_board sweep_motion_with_interface_board.cpp -lpthread
g++ -std=c++17 -O2 -o simple_motion simple_motion.cpp -lpthread
```

## Run
Run the executables (adjust device path and servo ID in the source if needed):

```bash
./sweep_motion_with_interface_board
./simple_motion
```

If you see a permission error on /dev/ttyUSB0, either run with sudo or add your user to the dialout group:

```bash
sudo usermod -a -G dialout "$USER"
# Log out/in or reboot for the group change to take effect
```

## Configure
- Serial device: Edit serial_dev in sweep_motion_with_interface_board.cpp and simple_motion.cpp (default: /dev/ttyUSB0).
- Servo ID: Edit SERVO_ID in sweep_motion_with_interface_board.cpp (example uses 2) and simple_motion.cpp (example uses 1).
- Baud rate: The FE-URT-1 board and Feetech STS servos typically use 1 Mbps; the examples set B1000000.

## Motion Sequence
The sweep program:
- Pings the target servo to confirm communication.
- Moves to 0 (≈0°), waits until motion completes.
- Moves to 2048 (≈180°), waits.
- Moves to 4095 (≈360°) at a slower speed, waits.
- Repeats the cycle.

## Files Overview
- STSServoDriver.hpp: Header-only driver, API-compatible with the Arduino version, adapted for Linux.
- sweep_motion_with_interface_board.cpp: Sweep program that aliases HardwareSerial to LinuxSerial and includes the driver header.
- simple_motion.cpp: Minimal motion program (0 → 180° → 360° once).

## Notes
- Keep the interface board powered with a suitable supply; USB power alone is often insufficient for servo loads.
- Ensure ground is common between your host and the interface board.
- For multiple servos, use setTargetPositions() to send synchronized commands.
- If you see timeouts, check wiring (TX/RX on FE-URT-1), baud rate (1 Mbps), and that the servo ID matches your device.
