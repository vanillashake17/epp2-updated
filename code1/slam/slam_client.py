#!/usr/bin/env python3
"""
slam_client.py - Matplotlib SLAM visualiser (client side).

Connects to lidar_forwarder.py running on the Raspberry Pi, receives LiDAR
scans as newline-delimited JSON, runs BreezySLAM locally, and displays:
  - Left subplot : occupancy-grid map (grayscale) with robot position and
                   heading arrow
  - Right subplot: LiDAR point cloud relative to the robot

This approach offloads the SLAM computation and rendering from the Pi to a
more powerful machine, which keeps the Pi free for teleoperation and camera.

Usage:
    python3 slam_client.py <PI_IP_ADDRESS>
    python3 slam_client.py <PI_IP_ADDRESS> <PORT>   # default port is 5002

Example:
    python3 slam_client.py 192.168.1.141
    python3 slam_client.py mango.local 5002

Requirements:
    pip install breezyslam matplotlib numpy
"""

import json
import math
import socket
import sys

# ---------------------------------------------------------------------------
# Matplotlib backend selection (try GUI backends in preference order so the
# script works on both native Linux and WSL2/WSLg environments).
# ---------------------------------------------------------------------------
import matplotlib
for _backend in ['TkAgg', 'Qt5Agg', 'GTK3Agg', 'WXAgg', 'Agg']:
    try:
        matplotlib.use(_backend)
        break
    except Exception:
        continue

import matplotlib.pyplot as plt
import numpy as np

# ---------------------------------------------------------------------------
# BreezySLAM
# ---------------------------------------------------------------------------
try:
    from breezyslam.algorithms import RMHC_SLAM
    from breezyslam.sensors import Laser
except ImportError:
    print('[slam_client] ERROR: breezyslam is not installed.')
    print('  Run:  bash install_slam.sh  or  pip install breezyslam')
    sys.exit(1)

# ---------------------------------------------------------------------------
# SLAM / map parameters (tuned for a classroom-sized environment)
# ---------------------------------------------------------------------------

# Number of equal-angle bins the LIDAR scan is resampled to.
SCAN_SIZE = 360
# Approximate rotation rate of the RPLidar A1M8.
SCAN_RATE_HZ = 5
# Full 360-degree field of view.
SCAN_FOV_DEG = 360
# Distances beyond this are treated as "no obstacle" (12 m).
MAX_DIST_MM = 4000

# SLAM map dimensions.
MAP_PIXELS = 1200        # pixels per side of the square map
MAP_METERS = 7          # real-world metres covered by the map
MAP_MM = MAP_METERS * 1000

# BreezySLAM quality parameter (1 = cautious, 10 = aggressive).
MAP_QUALITY = 8

# Gap size (mm) that SLAM treats as a continuous wall.
HOLE_WIDTH_MM = 100

# Minimum valid scan points required to do a full SLAM update; otherwise the
# previous good scan is reused.
MIN_VALID_POINTS = 150

# ---------------------------------------------------------------------------
# Visualisation constants
# ---------------------------------------------------------------------------
UPDATE_INTERVAL_MS = 100   # matplotlib animation interval (ms)
POINT_SIZE = 2             # LiDAR point cloud dot size

# ---------------------------------------------------------------------------
# Networking defaults
# ---------------------------------------------------------------------------
DEFAULT_PORT = 5002


# ---------------------------------------------------------------------------
# Helper: map bytes → 2-D numpy array
# ---------------------------------------------------------------------------

def _map_to_grid(mapbytes):
    """Convert the flat BreezySLAM map bytearray to a 2-D uint8 numpy array."""
    return np.frombuffer(mapbytes, dtype=np.uint8).reshape(MAP_PIXELS, MAP_PIXELS)


# ---------------------------------------------------------------------------
# Helper: LiDAR scan → Cartesian point cloud (robot-relative, metres)
# ---------------------------------------------------------------------------

def _scan_to_xy(distances, angles):
    """Convert a LiDAR scan to Cartesian coordinates in metres.

    The scan is in robot-relative coordinates (robot at origin, forward = +y).
    Points at or beyond MAX_DIST_MM are discarded.
    """
    d = np.asarray(distances, dtype=np.float64)
    a = np.asarray(angles,    dtype=np.float64)
    mask = (d > 0) & (d < MAX_DIST_MM)
    d, a = d[mask], a[mask]
    rad = np.radians((a + 90) % 360)
    d_m = d / 1000.0
    return d_m * np.cos(rad), d_m * np.sin(rad)


# ---------------------------------------------------------------------------
# Main client
# ---------------------------------------------------------------------------

def run(host: str, port: int = DEFAULT_PORT):
    # -- SLAM setup ----------------------------------------------------------
    laser = Laser(SCAN_SIZE, SCAN_RATE_HZ, SCAN_FOV_DEG, MAX_DIST_MM)
    slam = RMHC_SLAM(
        laser,
        MAP_PIXELS,
        MAP_METERS,
        hole_width_mm=HOLE_WIDTH_MM,
        map_quality=MAP_QUALITY,
    )
    mapbytes = bytearray(MAP_PIXELS * MAP_PIXELS)
    prev_distances = None

    # -- Matplotlib setup ----------------------------------------------------
    plt.ion()
    fig, (ax_map, ax_pts) = plt.subplots(1, 2, figsize=(14, 6))

    # Left subplot: occupancy-grid map
    ax_map.set_title('SLAM Map')
    ax_map.set_xlabel('X (m)')
    ax_map.set_ylabel('Y (m)')
    map_img = ax_map.imshow(
        np.full((MAP_PIXELS, MAP_PIXELS), 127, dtype=np.uint8),
        cmap='gray', origin='lower',
        extent=[0, MAP_METERS, 0, MAP_METERS],
        vmin=0, vmax=255,
    )
    robot_dot,  = ax_map.plot([], [], 'bo', markersize=8, label='robot')
    # Quiver arrow for robot heading — updated in-place each frame.
    robot_quiver = ax_map.quiver(
        [0], [0], [0], [0.15],
        color='green', scale=1, scale_units='xy', angles='xy',
    )

    # Right subplot: LiDAR point cloud
    ax_pts.set_title('LiDAR Point Cloud')
    ax_pts.set_xlabel('X (m)')
    ax_pts.set_ylabel('Y (m)')
    half = MAP_METERS / 2.0
    ax_pts.set_xlim(-half, half)
    ax_pts.set_ylim(-half, half)
    ax_pts.set_aspect('equal')
    ax_pts.grid(True)
    pts_plot, = ax_pts.plot([], [], 'r.', markersize=POINT_SIZE)
    pos_text = ax_pts.text(
        -half + 0.2, half - 0.4, '',
        fontsize=9, color='black',
        bbox=dict(facecolor='white', alpha=0.7),
    )

    fig.tight_layout()
    fig.canvas.draw()
    plt.pause(0.1)

    # -- TCP connection -------------------------------------------------------
    print(f'[slam_client] Connecting to {host}:{port} ...')
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((host, port))
        sock.settimeout(2.0)
    except Exception as exc:
        print(f'[slam_client] Connection failed: {exc}')
        plt.close('all')
        return

    print('[slam_client] Connected. Starting SLAM ...')

    buf = ''

    try:
        while plt.fignum_exists(fig.number):
            # -- Receive one newline-delimited JSON scan ----------------------
            try:
                chunk = sock.recv(65536).decode('utf-8')
                if not chunk:
                    print('[slam_client] Server closed connection')
                    break
                buf += chunk
            except socket.timeout:
                plt.pause(0.05)
                continue
            except OSError as exc:
                print(f'[slam_client] Socket error: {exc}')
                break

            # Process all complete JSON objects in the buffer.
            # SLAM is updated for every scan, but display is only
            # refreshed once after draining the buffer.
            last_distances = None
            last_angles = None
            last_valid = 0

            while '\n' in buf:
                line, buf = buf.split('\n', 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    scan = json.loads(line)
                except json.JSONDecodeError:
                    continue

                angles    = scan['angles']
                distances = scan['distances']

                # -- SLAM update ---------------------------------------------
                d_arr = np.asarray(distances, dtype=np.float64)
                valid = int(np.count_nonzero((d_arr > 0) & (d_arr < MAX_DIST_MM)))
                if valid >= MIN_VALID_POINTS:
                    slam.update(distances, scan_angles_degrees=angles)
                    prev_distances = list(distances)
                elif prev_distances is not None:
                    slam.update(prev_distances, scan_angles_degrees=angles)

                last_distances = distances
                last_angles = angles
                last_valid = valid

            # -- Update display once per draw frame --------------------------
            if last_distances is not None:
                x_mm, y_mm, theta_deg = slam.getpos()
                x_mm = max(0.0, min(x_mm, float(MAP_MM)))
                y_mm = max(0.0, min(y_mm, float(MAP_MM)))

                slam.getmap(mapbytes)
                map_img.set_data(np.rot90(_map_to_grid(mapbytes), 3))

                x_m = x_mm / 1000.0
                y_m = y_mm / 1000.0
                robot_dot.set_data([MAP_METERS - y_m], [x_m])

                theta_rad = math.radians(theta_deg)
                arrow_len = 0.15
                robot_quiver.set_offsets([[MAP_METERS - y_m, x_m]])
                robot_quiver.set_UVC(
                    [-arrow_len * math.sin(theta_rad)],
                    [arrow_len * math.cos(theta_rad)],
                )

                px, py = _scan_to_xy(last_distances, last_angles)
                pts_plot.set_data(px, py)
                pos_text.set_text(
                    f'x={x_m:.2f}m  y={y_m:.2f}m\nθ={theta_deg:.1f}°  valid={last_valid}'
                )

                fig.canvas.draw_idle()

            plt.pause(0.001)

    except KeyboardInterrupt:
        print('\n[slam_client] Interrupted')
    finally:
        sock.close()
        plt.close('all')
        print('[slam_client] Done')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f'Usage: python3 {sys.argv[0]} <PI_IP> [PORT]')
        print(f'Example: python3 {sys.argv[0]} 192.168.1.141')
        sys.exit(1)

    _host = sys.argv[1]
    _port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_PORT
    run(_host, _port)
