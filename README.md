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

## Pin Mapping (Arduino Mega 2560)

### Color Sensor (TCS3200)

| Arduino Digital Pin | ATmega2560 Port/Pin | Function |
|---|---|---|
| D22 | PA0 | S0 (Frequency Scaling) |
| D23 | PA1 | S1 (Frequency Scaling) |
| D24 | PA2 | S2 (Photodiode Select) |
| D25 | PA3 | S3 (Photodiode Select) |
| D26 | PA4 | Sensor OUT (Frequency Output) |

### E-Stop Button

| Arduino Digital Pin | ATmega2560 Port/Pin | Function |
|---|---|---|
| D20 (INT1) | PD1 | E-Stop Push Button (External Interrupt 1) |

### Robot Arm Servos

| Arduino Digital Pin | ATmega2560 Port/Pin | Function |
|---|---|---|
| D37 | PC0 | Base Servo Signal |
| D36 | PC1 | Shoulder Servo Signal |
| D35 | PC2 | Gripper Servo Signal |
| D33 | PC4 | Elbow Servo Signal |

### Motor Shield (Adafruit Motor Shield v1)

| Arduino Digital Pin | Function |
|---|---|
| D4, D7, D8, D12 | 74HC595 Shift Register (Motor Direction Control) |
| D3, D11 | PWM Speed Control (DC Motors) |

### Serial

| Arduino Digital Pin | ATmega2560 Port/Pin | Function |
|---|---|---|
| D0 (RX0) | PE0 | USART0 RX (Serial to Raspberry Pi) |
| D1 (TX0) | PE1 | USART0 TX (Serial to Raspberry Pi) |

## Interrupts & Timers

| Resource | Type | Purpose | Configuration |
|---|---|---|---|
| INT1 | External Interrupt | E-Stop button debounce & state machine | Triggers on any logical change (`ISC10`); reads PD1 pin state |
| Timer 2 | 8-bit Timer (CTC) | Color sensor measurement time-base | Prescaler 8, OCR2A = 199 → 100 µs tick; `TIMER2_COMPA_vect` increments `_timerTicks` |
| Timer 5 | 16-bit Timer (CTC) | Robot arm servo PWM (4-channel staggered) | Prescaler 8, OCR5A = 39999 → 20 ms period; `TIMER5_COMPA_vect` runs servo lerp, `TIMER5_COMPB_vect` generates staggered pulses |
| USART0 RX | Peripheral Interrupt | Bare-metal serial receive | `USART0_RX_vect` ISR enqueues bytes into circular `rx_buf` |
| USART0 UDRE | Peripheral Interrupt | Bare-metal serial transmit | `USART0_UDRE_vect` ISR dequeues bytes from circular `tx_buf`; auto-disables when empty |
| Timers 1, 3, 4 | Hardware Timers | AFMotor library (motor PWM) | Reserved by the Adafruit Motor Shield v1 library for DC motor speed control; not available for user code |
