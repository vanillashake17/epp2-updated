import sys, os
# Add this directory to sys.path so internal absolute imports work
_pkg_dir = os.path.dirname(os.path.abspath(__file__))
if _pkg_dir not in sys.path:
    sys.path.insert(0, _pkg_dir)

from .pyrplidar import PyRPlidar
from .pyrplidar_serial import PyRPlidarSerial
