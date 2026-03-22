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
from second_terminal import relay
from packets import *

# ----------------------------------------------------------------
# FEATURE FLAGS — set to False if a component is not connected
# ----------------------------------------------------------------

CAMERA_ENABLED = False
COLOR_ENABLED  = False

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
_frames_remaining = 5   # frames remaining before further captures are refused


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

    if not isEstopActive(): # estop not active, capture and display max 5 images.
        if _frames_remaining > 0:
            newImage = alex_camera.captureGreyscaleFrame(_camera)
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

_motor_speed = 175   # current speed (0-255)
SPEED_STEP   = 25    # how much +/- changes the speed


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

    elif line == '+':
        handleSpeedChange(SPEED_STEP)

    elif line == '-':
        handleSpeedChange(-SPEED_STEP)

    else:
        print(f"Unknown input: '{line}'. Valid: e, c, p, l, w/s/a/d <ms>, x, +, -")


def runCommandInterface():

    print("Sensor interface ready. Commands:")
    print(f"  e = E-Stop    c = Color sensor {'(disabled)' if not COLOR_ENABLED else ''}")
    print(f"  p = Camera {'(disabled)' if not CAMERA_ENABLED else ''}")
    print("  w <ms> = Forward   s <ms> = Backward")
    print("  a <ms> = Turn left d <ms> = Turn right")
    print("  (duration in ms, default 2000. e.g. 'w 500')")
    print("  x = Stop robot")
    print("  +/- = Speed up/down  (current: 200)")
    print("Press Ctrl+C to exit.\n")

    while True:
        if _ser.in_waiting >= FRAME_SIZE:
            pkt = receiveFrame()
            if pkt:
                printPacket(pkt)
                relay.onPacketReceived(packFrame(pkt['packetType'], pkt['command'],
pkt['data'], pkt['params']))

        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            line = sys.stdin.readline().strip().lower()
            relay.checkSecondTerminal(_ser)
            if not line:
                time.sleep(0.05)
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
        