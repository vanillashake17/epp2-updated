#!/usr/bin/env python3
"""
lidar_forwarder.py - TCP server that streams raw LIDAR scan data to clients.

Run this on the Raspberry Pi instead of (or alongside) slam.py when you want
to offload SLAM processing to a more powerful client machine.

It reads scans from the RPLidar, resamples them to 360 equal-angle bins, and
sends each scan as a newline-delimited JSON object over TCP port 5002 to a
connected client.

Usage (from inside the slam/ directory on the Pi):
    source ../env/bin/activate
    python3 lidar_forwarder.py

The client (slam_client.py) connects to this server to receive scan data and
run BreezySLAM locally with a matplotlib display.
"""

import json
import os
import socket
import statistics
import sys

_slam_dir = os.path.dirname(os.path.abspath(__file__))
_parent_dir = os.path.dirname(_slam_dir)
sys.path.insert(0, _slam_dir)
if _parent_dir not in sys.path:
    sys.path.insert(1, _parent_dir)

from settings import (
    LIDAR_OFFSET_DEG, MAX_DISTANCE_MM, SCAN_SIZE,
    INITIAL_ROUNDS_SKIP,
)
import lidar as lidar_driver

HOST = '0.0.0.0'
PORT = 5002


_OUTLIER_THRESHOLD = 0.30   # reject readings that deviate > 30 % from bin median


def _resample(raw_angles, raw_distances):
    """Resample a raw LIDAR scan into SCAN_SIZE equal-angle bins.

    Converts from RPLidar clockwise convention to BreezySLAM counter-clockwise
    convention, applies LIDAR_OFFSET_DEG, then bins readings that fall into the
    same degree bucket.

    Each bin is reduced to a single distance value using median + outlier
    rejection:
      - Bins with 1–2 readings: median of those readings (== min for 1 reading).
      - Bins with 3+ readings: readings that deviate more than _OUTLIER_THRESHOLD
        (30 %) from the bin median are discarded; the median of the survivors is
        used.  This eliminates spurious spikes while keeping genuine close
        obstacles.
    Empty bins are filled with MAX_DISTANCE_MM (treated as "no obstacle").

    Returns (angles, distances) as two parallel lists of SCAN_SIZE floats.
    angles[i] = i * (360 / SCAN_SIZE) degrees
    distances[i] = filtered distance in mm for that bin
    """
    bins = [[] for _ in range(SCAN_SIZE)]

    for angle, dist in zip(raw_angles, raw_distances):
        if dist <= 0:
            continue
        ccw_angle = (-angle + LIDAR_OFFSET_DEG) % 360
        idx = int(round(ccw_angle)) % SCAN_SIZE
        bins[idx].append(dist)

    angles = []
    distances = []
    for i in range(SCAN_SIZE):
        angles.append(float(i * 360 / SCAN_SIZE))
        readings = bins[i]
        if not readings:
            distances.append(float(MAX_DISTANCE_MM))
            continue
        med = statistics.median(readings)
        if len(readings) >= 3:
            # Discard readings that deviate too far from the median.
            threshold = med * _OUTLIER_THRESHOLD
            survivors = [d for d in readings if abs(d - med) <= threshold]
            if survivors:
                med = statistics.median(survivors)
        distances.append(min(med, float(MAX_DISTANCE_MM)))

    return angles, distances


def serve():
    lidar = lidar_driver.connect()
    if lidar is None:
        print('[forwarder] Failed to connect to LIDAR', file=sys.stderr)
        return

    scan_mode = lidar_driver.get_scan_mode(lidar)
    print(f'[forwarder] LIDAR connected (mode {scan_mode})')

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)
    server.settimeout(1.0)
    print(f'[forwarder] Listening on {HOST}:{PORT}')

    try:
        while True:
            print('[forwarder] Waiting for client...')
            conn = None
            while conn is None:
                try:
                    conn, addr = server.accept()
                except socket.timeout:
                    continue

            print(f'[forwarder] Client connected from {addr}')
            round_num = 0

            try:
                for raw_angles, raw_dists in lidar_driver.scan_rounds(lidar, scan_mode):
                    round_num += 1
                    if round_num <= INITIAL_ROUNDS_SKIP:
                        continue

                    angles, distances = _resample(raw_angles, raw_dists)
                    payload = json.dumps({'angles': angles, 'distances': distances}) + '\n'
                    try:
                        conn.sendall(payload.encode('utf-8'))
                    except (BrokenPipeError, OSError):
                        print('[forwarder] Client disconnected')
                        break
            except Exception as exc:
                print(f'[forwarder] Scan error: {exc}')
            finally:
                conn.close()

    except KeyboardInterrupt:
        print('\n[forwarder] Shutting down')
    finally:
        lidar_driver.disconnect(lidar)
        server.close()


if __name__ == '__main__':
    serve()
