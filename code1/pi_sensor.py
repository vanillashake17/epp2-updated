#!/usr/bin/env python3
"""
Studio 13: Sensor Mini-Project
Raspberry Pi Sensor Interface — pi_sensor.py

Extends the communication framework from Studio 12 with:
  - Magic number + checksum framing for reliable packet delivery
  - A command-line interface that reads user commands while displaying
    live Arduino data

Packet framing format (103 bytes total):
  MAGIC (2 B) | TPacket (100 B) | CHECKSUM (1 B)

Usage:
  source env/bin/activate
  python3 pi_sensor.py
"""

import struct
import serial
import time
import sys
import select
import tty
import termios
from second_terminal import relay
from packets import *

# ----------------------------------------------------------------
# FEATURE FLAGS — set to False if a component is not connected
# ----------------------------------------------------------------

CAMERA_ENABLED = True
COLOR_ENABLED  = True
CAMERA_ROTATE_CCW_90 = True

# ----------------------------------------------------------------
# SERIAL PORT SETUP
# ----------------------------------------------------------------

PORT     = "/dev/ttyACM0"
BAUDRATE = 9600

_ser = None


def openSerial():
    """Open the serial port and wait for the Arduino to boot."""
    global _ser
    _ser = serial.Serial(PORT, BAUDRATE, timeout=5)
    print(f"Opened {PORT} at {BAUDRATE} baud. Waiting for Arduino...")
    time.sleep(2)
    print("Ready.\n")


def closeSerial():
    """Close the serial port."""
    global _ser
    if _ser and _ser.is_open:
        _ser.close()


# ----------------------------------------------------------------
# TPACKET CONSTANTS
# (must match sensor_miniproject_template.ino)
# ----------------------------------------------------------------


TPACKET_SIZE = 1 + 1 + 2 + MAX_STR_LEN + (PARAMS_COUNT * 4)  # = 100
TPACKET_FMT  = f'<BB2x{MAX_STR_LEN}s{PARAMS_COUNT}I'

# ----------------------------------------------------------------
# RELIABLE FRAMING: magic number + XOR checksum
# ----------------------------------------------------------------

MAGIC = b'\xDE\xAD'          # 2-byte magic number (0xDEAD)
FRAME_SIZE = len(MAGIC) + TPACKET_SIZE + 1   # 2 + 100 + 1 = 103


def computeChecksum(data: bytes) -> int:
    """Return the XOR of all bytes in data."""
    result = 0
    for b in data:
        result ^= b
    return result


def packFrame(packetType, command, data=b'', params=None):
    """
    Build a framed packet: MAGIC | TPacket bytes | checksum.

    Args:
        packetType: integer packet type constant
        command:    integer command / response constant
        data:       bytes for the data field (padded/truncated to MAX_STR_LEN)
        params:     list of PARAMS_COUNT uint32 values (default: all zeros)

    Returns:
        bytes of length FRAME_SIZE (103)
    """
    if params is None:
        params = [0] * PARAMS_COUNT
    data_padded = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command,
                               data_padded, *params)
    checksum = computeChecksum(packet_bytes)
    return MAGIC + packet_bytes + bytes([checksum])


def unpackTPacket(raw):
    """Deserialise a 100-byte TPacket into a dict."""
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command':    fields[1],
        'data':       fields[2],
        'params':     list(fields[3:]),
    }


def receiveFrame():
    """
    Read bytes from the serial port until a valid framed packet is found.

    Synchronises to the magic number, reads the TPacket body, then
    validates the checksum.  Discards corrupt or out-of-sync bytes.

    Returns:
        A packet dict (see unpackTPacket), or None on timeout / error.
    """
    MAGIC_HI = MAGIC[0]
    MAGIC_LO = MAGIC[1]

    while True:
        b = _ser.read(1)
        if not b:
            return None
        if b[0] != MAGIC_HI:
            continue

        b = _ser.read(1)
        if not b:
            return None
        if b[0] != MAGIC_LO:
            continue

        raw = b''
        while len(raw) < TPACKET_SIZE:
            chunk = _ser.read(TPACKET_SIZE - len(raw))
            if not chunk:
                return None
            raw += chunk

        cs_byte = _ser.read(1)
        if not cs_byte:
            return None
        expected = computeChecksum(raw)
        if cs_byte[0] != expected:
            continue

        return unpackTPacket(raw)


def sendCommand(commandType, data=b'', params=None):
    """Send a framed COMMAND packet to the Arduino."""
    frame = packFrame(PACKET_TYPE_COMMAND, commandType, data=data, params=params)
    _ser.write(frame)


# ----------------------------------------------------------------
# E-STOP STATE
# ----------------------------------------------------------------

_estop_state = STATE_RUNNING


def isEstopActive():
    """Return True if the E-Stop is currently active (system stopped)."""
    return _estop_state == STATE_STOPPED


# ----------------------------------------------------------------
# PACKET DISPLAY
# ----------------------------------------------------------------

def printPacket(pkt):
    global _estop_state
    ptype = pkt['packetType']
    cmd   = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            print("Response: OK")
        elif cmd == RESP_STATUS:
            state = pkt['params'][0]
            _estop_state = state
            if state == STATE_RUNNING:
                print("Status: RUNNING")
            else:
                print("Status: STOPPED")

        elif cmd == RESP_COLOR:
            r = pkt['params'][0]
            g = pkt['params'][1]
            b = pkt['params'][2]
            print(f"R: {r} Hz, G: {g} Hz, B: {b} Hz")

        elif cmd == RESP_ARM:
            labels = ['Base', 'Shoulder', 'Elbow', 'Gripper']
            angles = [pkt['params'][i] for i in range(4)]
            print("Arm: " + "  ".join(f"{l}={a}°" for l, a in zip(labels, angles)))

        else:
            print(f"Response: unknown command {cmd}")

        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug:
            print(f"Arduino debug: {debug}")

    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"Arduino: {msg}")
    else:
        print(f"Packet: type={ptype}, cmd={cmd}")


# ----------------------------------------------------------------
# ACTIVITY 2: COLOR SENSOR
# ----------------------------------------------------------------

def handleColorCommand():
    if not COLOR_ENABLED:
        print("Color sensor not enabled.")
        return
    if isEstopActive():
        print("System stopped. Cannot read color sensor.")
        return

    print("Requesting color sensor reading...")
    sendCommand(COMMAND_COLOR)


# ----------------------------------------------------------------
# ACTIVITY 3: CAMERA
# ----------------------------------------------------------------

# TODO (Activity 3): import the camera library provided (alex_camera.py).
if CAMERA_ENABLED:
    import alex_camera
    _camera = alex_camera.cameraOpen()
_frames_remaining = 30  # frames remaining before further captures are refused


def handleCameraCommand():
    """
    TODO (Activity 3): capture and display a greyscale frame.

    Gate on E-Stop state and the remaining frame count.
    Use captureGreyscaleFrame() and renderGreyscaleFrame() from alex_camera.
    """
    global _frames_remaining

    if not CAMERA_ENABLED:
        print("Camera not enabled.")
        return

    if not isEstopActive(): # estop not active, capture and display max 10 images.
        if _frames_remaining > 0:
            newImage = alex_camera.captureGreyscaleFrame(
                _camera,
                rotate_ccw_90=CAMERA_ROTATE_CCW_90,
            )
            alex_camera.renderGreyscaleFrame(newImage)
            _frames_remaining -= 1
            print(f"Frames remaining: {_frames_remaining}")
        else: # no frames remaining
            print("No frames remaining. Please wait for the system to reset.")
    else: # estop active
        print("E-Stop is active. Cannot capture camera frame.")



# ----------------------------------------------------------------
# COMMAND-LINE INTERFACE
# ----------------------------------------------------------------

_motor_speed = 200   # current speed (0-255)
SPEED_STEP   = 25    # how much +/- changes the speed
RAW_MOVE_MS  = 100   # duration per keypress in raw drive mode


def handleMovementCommand(direction, duration_ms):
    """Send a COMMAND_MOVE with direction in data, speed in params[0], duration in params[1]."""
    if isEstopActive():
        print("System stopped. Cannot drive motors.")
        return

    labels = {'w': 'FORWARD', 's': 'BACKWARD', 'a': 'LEFT (CCW)', 'd': 'RIGHT (CW)'}
    print(f"Sending {labels.get(direction, direction)} at speed {_motor_speed} for {duration_ms} ms...")
    sendCommand(COMMAND_MOVE, data=direction.encode(), params=[_motor_speed, duration_ms] + [0] * 14)


def handleSpeedChange(delta):
    """Adjust motor speed by delta, clamped to 0-255."""
    global _motor_speed
    _motor_speed = max(0, min(255, _motor_speed + delta))
    print(f"Motor speed set to {_motor_speed}")


def handleArmCommand(line):
    """Parse and send arm commands: rb/rs/re/rg <angle>, rv <delay>, rh (home)."""
    parts = line.split()
    cmd_char = parts[0]
    char_map = {'rb': 'B', 'rs': 'S', 're': 'E', 'rg': 'G', 'rv': 'V', 'rh': 'H'}
    arm_char = char_map[cmd_char]

    if arm_char == 'H':
        sendCommand(COMMAND_ARM, data=b'H')
        print("Homing arm...")
        return

    if len(parts) < 2:
        print(f"Usage: {cmd_char} <value>")
        return
    try:
        val = int(parts[1])
    except ValueError:
        print(f"Invalid value: '{parts[1]}'")
        return

    sendCommand(COMMAND_ARM, data=arm_char.encode(), params=[val] + [0] * 15)
    label = {'B': 'Base', 'S': 'Shoulder', 'E': 'Elbow', 'G': 'Gripper', 'V': 'Speed'}
    print(f"Arm {label[arm_char]} -> {val}")


def handleUserInput(line):

    if line == 'e':
        print("Sending E-Stop command...")
        sendCommand(COMMAND_ESTOP, data=b'This is a debug message')

    elif line == 'c':
        handleColorCommand()

    elif line == 'p':
        handleCameraCommand()


    elif line and line[0] in ('w', 's', 'a', 'd'):
        parts = line.split()
        direction = parts[0]
        if len(parts) >= 2:
            try:
                duration_ms = int(parts[1])
            except ValueError:
                print(f"Invalid duration: '{parts[1]}'. Use e.g. 'w 500'")
                return
        else:
            duration_ms = 2000  # default
        handleMovementCommand(direction, duration_ms)

    elif line == 'x':
        print("Stopping robot...")
        sendCommand(COMMAND_MOVE, data=b'x', params=[0, 0] + [0] * 14)

    elif line.split()[0] in ('rb', 'rs', 're', 'rg', 'rv', 'rh'):
        handleArmCommand(line)

    elif line == '+':
        handleSpeedChange(SPEED_STEP)

    elif line == '-':
        handleSpeedChange(-SPEED_STEP)

    else:
        print(f"Unknown input: '{line}'. Valid: e, c, p, w/s/a/d <ms>, x, +/-, rb/rs/re/rg/rv/rh")


def _processSerial():
    """Check for incoming serial packets and relay data."""
    if _ser.in_waiting >= FRAME_SIZE:
        pkt = receiveFrame()
        if pkt:
            printPacket(pkt)
            relay.onPacketReceived(packFrame(pkt['packetType'], pkt['command'],
pkt['data'], pkt['params']))
    relay.checkSecondTerminal(_ser)


def runRawDriveMode():
    """Raw drive mode: WASD keys move instantly, no Enter required."""
    print("\n-- RAW DRIVE MODE --")
    print("  WASD = drive    +/- = speed    e = E-Stop    x = stop")
    print("  Press 'm' to return to normal mode\n")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    _last_color_time = 0.0
    try:
        tty.setcbreak(fd)
        while True:
            _processSerial()

            now = time.time()
            if COLOR_ENABLED and (now - _last_color_time) >= 6.0:
                handleColorCommand()
                _last_color_time = now

            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not rlist:
                continue

            ch = sys.stdin.read(1).lower()

            if ch == 'm':
                print("\n-- NORMAL MODE --")
                return

            if ch in ('w', 'a', 's', 'd'):
                handleMovementCommand(ch, RAW_MOVE_MS)
            elif ch == 'x':
                print("Stopping robot...")
                sendCommand(COMMAND_MOVE, data=b'x', params=[0, 0] + [0] * 14)
            elif ch == 'e':
                print("Sending E-Stop command...")
                sendCommand(COMMAND_ESTOP, data=b'This is a debug message')
            elif ch == '+':
                handleSpeedChange(SPEED_STEP)
            elif ch == '-':
                handleSpeedChange(-SPEED_STEP)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def runCommandInterface():

    print("Sensor interface ready. Commands:")
    print(f"  e = E-Stop    c = Color sensor {'(disabled)' if not COLOR_ENABLED else ''}")
    print(f"  p = Camera {'(disabled)' if not CAMERA_ENABLED else ''}")
    print("  w <ms> = Forward   s <ms> = Backward")
    print("  a <ms> = Turn left d <ms> = Turn right")
    print("  (duration in ms, default 2000. e.g. 'w 500')")
    print("  x = Stop robot")
    print(f"  +/- = Speed up/down  (current: {_motor_speed})")
    print("  m = Raw drive mode (WASD instant control)")
    print("  rb <0-175> = Base   rs <140-180> = Shoulder")
    print("  re <0-180> = Elbow  rg <5-40> = Gripper")
    print("  rv <1-999> = Arm speed (ms/step)  rh = Home arm")
    print("Press Ctrl+C to exit.\n")



    while True:
        _processSerial()


        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            line = sys.stdin.readline().strip().lower()
            if not line:
                time.sleep(0.05)
                continue
            if line == 'm':
                runRawDriveMode()
                continue
            handleUserInput(line)

        time.sleep(0.05)


# ----------------------------------------------------------------
# MAIN
# ----------------------------------------------------------------

if __name__ == '__main__':
    openSerial()
    relay.start()
    try:
        runCommandInterface()
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        if CAMERA_ENABLED:
            alex_camera.cameraClose(_camera)
        # TODO Disconnect Lidar if used (unsure)
        relay.shutdown()
        closeSerial()
        
