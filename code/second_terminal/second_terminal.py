#!/usr/bin/env python3
"""
Studio 16: Robot Integration
second_terminal.py  -  Second operator terminal.

This terminal connects to pi_sensor.py over TCP.  It:
  - Displays every TPacket forwarded from the robot (via pi_sensor.py).
  - Sends a software E-Stop command when you type 'e'.

Architecture
------------
   [Arduino] <--USB serial--> [pi_sensor.py] <--TCP--> [second_terminal.py]
                                (TCP server,               (TCP client,
                                 port 65432)                localhost:65432)

Run pi_sensor.py FIRST (it starts the TCP server), then run this script.
Both scripts run on the same Raspberry Pi.

Commands
--------
  e   Send a software E-Stop to the robot (same as pressing the button).
  q   Quit.

Usage
-----
    source env/bin/activate
    python3 second_terminal/second_terminal.py

Press Ctrl+C to exit.
"""

import os
import select
import ssl
import struct
import sys
import time

# Add parent directory (code/) to path so we can import the shared packets module.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from packets import *

# net_utils is imported with an absolute import because this script is designed
# to be run directly (python3 second_terminal/second_terminal.py), which adds
# this file's directory to sys.path automatically.
from net_utils import TCPClient, sendTPacketFrame, recvTPacketFrame


# ---------------------------------------------------------------------------
# Connection settings
# ---------------------------------------------------------------------------
# Both scripts run on the same Pi, so the host is 'localhost'.
# Change PI_HOST to the Pi's IP address if you run this from a different machine.
PI_HOST = '100.79.89.12'
PI_PORT = 65432

# ---------------------------------------------------------------------------
# TLS settings (must match relay.py)
# ---------------------------------------------------------------------------
TLS_ENABLED   = True
TLS_CERT_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             'certs', 'server.crt')


# ---------------------------------------------------------------------------
# TPacket constants
# ---------------------------------------------------------------------------
# IMPORTANT: keep these in sync with your pi_sensor.py and the Arduino sketch.

TPACKET_SIZE = 1 + 1 + 2 + MAX_STR_LEN + (PARAMS_COUNT * 4)   # = 100
TPACKET_FMT  = f'<BB2x{MAX_STR_LEN}s{PARAMS_COUNT}I'

MAGIC         = b'\xDE\xAD'
FRAME_SIZE    = len(MAGIC) + TPACKET_SIZE + 1   # = 103
_MIN_PULSE_US = 600    # servo min pulse width (sync with .ino)
_MAX_PULSE_US = 2400   # servo max pulse width (sync with .ino)


# ---------------------------------------------------------------------------
# TPacket helpers
# ---------------------------------------------------------------------------

def _computeChecksum(data: bytes) -> int:
    result = 0
    for b in data:
        result ^= b
    return result


def _packFrame(packetType, command, data=b'', params=None):
    """Pack a TPacket into a 103-byte framed byte string."""
    if params is None:
        params = [0] * PARAMS_COUNT
    else:
        params = (params + [0] * PARAMS_COUNT)[:PARAMS_COUNT]
    data_padded  = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command,
                               data_padded, *params)
    return MAGIC + packet_bytes + bytes([_computeChecksum(packet_bytes)])


def _unpackFrame(frame: bytes):
    """Validate checksum and unpack a 103-byte frame.  Returns None if corrupt."""
    if len(frame) != FRAME_SIZE or frame[:2] != MAGIC:
        return None
    raw = frame[2:2 + TPACKET_SIZE]
    if frame[-1] != _computeChecksum(raw):
        return None
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command':    fields[1],
        'data':       fields[2],
        'params':     list(fields[3:]),
    }


# ---------------------------------------------------------------------------
# Packet display
# ---------------------------------------------------------------------------

_estop_active = False
_print_color  = True   # print colour responses (on by default on second terminal)


def _armParamToAngle(value: int) -> int:
    """Convert ARM response value to degrees.

    Newer firmware may report timer ticks (0.5us each), while older
    firmware can already report angles. This helper accepts either.
    """
    if 0 <= value <= 180:
        return value

    us = value / 2.0
    angle = round((us - _MIN_PULSE_US) * 180.0 / (_MAX_PULSE_US - _MIN_PULSE_US))
    return max(0, min(180, int(angle)))


def _printPacket(pkt):
    """Pretty-print a TPacket forwarded from the robot."""
    global _estop_active

    ptype = pkt['packetType']
    cmd   = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            pass
        elif cmd == RESP_STATUS:
            state         = pkt['params'][0]
            _estop_active = (state == STATE_STOPPED)
            print(f"[robot] Status: {'STOPPED' if _estop_active else 'RUNNING'}")
        elif cmd == RESP_COLOR:
            if _print_color:
                r = pkt['params'][0]
                g = pkt['params'][1]
                b = pkt['params'][2]
                print(f"[robot] R: {r} Hz, G: {g} Hz, B: {b} Hz")
                if r > 5000 and g < 3000 and b < 3000:
                    label = "RED"
                elif g > 4000 and r < 4000 and b < 4000:
                    label = "GREEN"
                elif b > 7000 and r < 4000 and g < 6000:
                    label = "BLUE"
                else:
                    label = "FLOOR"
                print(f"[robot] Colour: {label}")
        elif cmd == RESP_ARM:
            b = _armParamToAngle(pkt['params'][0])
            s = _armParamToAngle(pkt['params'][1])
            e = _armParamToAngle(pkt['params'][2])
            g = _armParamToAngle(pkt['params'][3])
            print(f"[robot] Arm angles: B={b} S={s} E={e} G={g}")
        else:
            print(f"[robot] Response: unknown command {cmd}")
        # Print any debug string embedded in the data field.
        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug:
            print(f"[robot] Debug: {debug}")

    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"[robot] Message: {msg}")

    else:
        print(f"[robot] Packet: type={ptype}, cmd={cmd}")


# ---------------------------------------------------------------------------
# Input handling
# ---------------------------------------------------------------------------

def _handleInput(line: str, client: TCPClient):
    """Handle one line of keyboard input."""
    line = line.strip().lower()
    if not line:
        return

    if line == 'e':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ESTOP)
        sendTPacketFrame(client.sock, frame)
        print("[second_terminal] Sent: E-STOP")

    elif line == 'n':
        global _print_color
        _print_color = not _print_color
        print(f"[second_terminal] Print colour: {'ON' if _print_color else 'OFF'}")

    elif len(line) >= 2 and line[0] in 'bsegv' and line[1:].isdigit():
        cmd_char = line[0].upper()
        val = int(line[1:])
        data = cmd_char.encode('ascii')
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM,
                           data=data, params=[val])
        sendTPacketFrame(client.sock, frame)
        print(f"[second_terminal] Sent: ARM {cmd_char}{val:03d}")

    elif line == 'h':
        data = b'H'
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM, data=data)
        sendTPacketFrame(client.sock, frame)
        print("[second_terminal] Sent: ARM HOME")

    elif line == 'q':
        print("[second_terminal] Quitting.")
        raise KeyboardInterrupt

    else:
        print(f"[second_terminal] Unknown: '{line}'.  "
              f"Valid: e (E-Stop)  n (toggle print)  "
              f"b/s/e/g/vNNN (arm)  h (home arm)  q (quit)")


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def run():
    ssl_context = None
    if TLS_ENABLED:
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ssl_context.minimum_version = ssl.TLSVersion.TLSv1_2
        ssl_context.load_verify_locations(TLS_CERT_PATH)
    client = TCPClient(host=PI_HOST, port=PI_PORT,
                       ssl_context=ssl_context, server_hostname='mango')
    print(f"[second_terminal] Connecting to pi_sensor.py at {PI_HOST}:{PI_PORT} "
          f"({'TLS' if TLS_ENABLED else 'plain TCP'})...")

    if not client.connect(timeout=10.0):
        print("[second_terminal] Could not connect.")
        print("  Make sure pi_sensor.py is running and waiting for a"
              " second terminal connection.")
        sys.exit(1)

    print("[second_terminal] Connected!")
    print("[second_terminal] Commands:  e = E-Stop  n = Toggle colour printing")
    print("[second_terminal]           b/s/e/g/vNNN = arm  h = home arm  q = quit")
    print("[second_terminal] Servo limits:")
    print("  Base (b):     0 - 180")
    print("  Shoulder (s): 110 (up) - 180 (down)")
    print("  Elbow (e):    0 (down/in) - 180 (up/out)")
    print("  Gripper (g):  5 (open) - 35 (closed)")
    print("[second_terminal] Incoming robot packets will be printed below.\n")

    try:
        while True:
            # Check for forwarded TPackets from pi_sensor.py (non-blocking).
            if client.hasData():
                frame = recvTPacketFrame(client.sock)
                if frame is None:
                    print("[second_terminal] Connection to pi_sensor.py closed.")
                    break
                pkt = _unpackFrame(frame)
                if pkt:
                    _printPacket(pkt)

            # Check for keyboard input (non-blocking via select).
            rlist, _, _ = select.select([sys.stdin], [], [], 0)
            if rlist:
                line = sys.stdin.readline()
                _handleInput(line, client)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[second_terminal] Exiting.")
    finally:
        client.close()


if __name__ == '__main__':
    run()
