# EPP 2 - Checkpoint 5: TLS & Offloaded SLAM

This project implements a secure, distributed teleoperation and SLAM mapping system for an Arduino-based robot affectionately called Alex.

The architecture spans three physical systems: **Arduino** (Firmware), **Raspberry Pi** (Coordinator/Server), and **Desktop** (Client terminals and visualization).

## How the Files Work Together

### 1. Core Robot Operation (Arduino <-> Raspberry Pi)

The Raspberry Pi acts as the main hub, while the Arduino acts as the low-level hardware executor.

- **`code1/pi_sensor.py`**: The main coordinator script on the Raspberry Pi. It opens the serial port to the Arduino, parses movement commands (WASD / speed) from the primary operator, limits camera frame outputs, and manages hardware/software E-Stop states.
- **`code1/sensor_miniproject_template/sensor_miniproject_template.ino`**: The primary C++ firmware flashed to the Arduino. It uses ISR-driven circular buffers for non-blocking I/O to read serial commands, actuates motors and servos (`robotlib.ino`), reads the TCS3200 color sensor (Timer 2), and executes the physical hardware E-Stop (INT1).
- **`code1/packets.py` & `packets.h`**: The shared protocol dictionaries across the system. They ensure that Python and C++ both agree on the exact 103-byte `TPacket` framing layout (MAGIC bytes -> TPacket -> XOR Checksum).

### 2. TLS Secure Teleoperation (Desktop <-> Raspberry Pi)

A second terminal is used by a secondary operator on a desktop computer to remotely control the robotic arm.

- **`code1/second_terminal/second_terminal.py`**: The client-side terminal running on the Desktop. It uses TLS 1.2+ encryption using a self-signed certificate (`certs/server.crt`) to securely transmit arm joint positions to the Pi on TCP port 65432.
- **`code1/second_terminal/relay.py`**: A module imported by `pi_sensor.py` (not a separate process). It starts a TLS TCP server on port 65432, accepts a connection from `second_terminal.py`, and bi-directionally relays TPacket frames between the second terminal and the Arduino serial port.
- **`code1/second_terminal/net_utils.py`**: Wraps the network traffic in a 4-byte length-prefixed frame to prevent partial-packet TCP transmission errors.

### 3. Offloaded SLAM Data Pipeline (LiDAR -> Raspberry Pi -> Desktop)

To avoid overloading the Raspberry Pi with heavy algorithms, the SLAM map generation is offloaded to the desktop.

- **`code1/lidar/alex_lidar.py` and `code1/pyrplidar/`**: Low-level wrappers that talk to the physical RPLidar A1M8 hardware over USB.
- **`code1/slam/lidar.py`**: The driver that manages the hardware lifecycle (connecting, checking scan modes, and yielding complete 360-degree rotation 'rounds' of angles/distances).
- **`code1/slam/lidar_forwarder.py`**: Runs on the Raspberry Pi. It imports `lidar.py` to fetch the raw rounds, resamples the uneven data into exactly 360 equal-angle bins, packages it as JSON, and streams it continuously over TCP port 5002.
- **`code1/slam/slam_client.py`**: Runs on the Desktop. Connects to port 5002, feeds the incoming JSON arrays into the BreezySLAM algorithm locally, and utilizes `matplotlib` to render a highly accurate, continuous-coordinate map of the room.

### 4. Vision

- **`code1/alex_camera.py`**: Connects to the Pi Camera module, conditionally capturing pictures based on an allowed budget limit per mission, and condensing them into an 80x44 greyscale ASCII block output directly in the terminal interface.
