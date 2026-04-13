#!/usr/bin/env python3
"""
settings.py - User-configurable settings for the SLAM data pipeline.

These settings are used by lidar.py and lidar_forwarder.py on the Raspberry Pi.
The desktop SLAM client (slam_client.py) has its own independent parameters.

The most common settings to change are LIDAR_PORT and LIDAR_OFFSET_DEG.
"""

# ===========================================================================
# LIDAR hardware settings
# ===========================================================================

# Serial port the RPLidar is plugged into.
# On the Raspberry Pi with a USB adapter this is usually /dev/ttyUSB0.
# If you have other USB devices connected it might be /dev/ttyUSB1, etc.
LIDAR_PORT = '/dev/ttyUSB0'

# Baud rate for the RPLidar A1M8. Do not change this.
LIDAR_BAUD = 115200

# ===========================================================================
# Scan settings
# ===========================================================================

# Number of angle bins the forwarder resamples each 360-degree rotation into.
SCAN_SIZE = 360

# Readings beyond this distance (in mm) are treated as misses (no return).
# Must match MAX_DIST_MM in slam_client.py so the forwarder and SLAM client
# agree on what counts as "no obstacle".
MAX_DISTANCE_MM = 3500

# Number of scans to skip at startup.  The LIDAR motor needs several rotations
# to reach full speed; early scans are noisier than steady-state scans.
INITIAL_ROUNDS_SKIP = 10

# ===========================================================================
# Scan quality filtering
# ===========================================================================

# Minimum per-measurement quality value to keep.  The RPLidar reports quality
# in the range 0–15; 0 is explicitly invalid, and 1–4 are marginal readings.
# Raise this if you still see spikes; lower it if too many points are dropped.
MIN_QUALITY = 5

# ===========================================================================
# LIDAR mounting offset
# ===========================================================================

# Degrees to rotate all LIDAR readings so that 0 deg = robot forward.
# Stand behind the robot and measure the CCW angle from robot forward
# to the LIDAR's forward direction.  Default 0 = already aligned.
LIDAR_OFFSET_DEG = 0
