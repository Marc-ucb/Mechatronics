# -*- coding: utf-8 -*-
import time
import numpy as np
import board
import busio
from digitalio import DigitalInOut, Direction
from adafruit_vl53l0x import VL53L0X
from scipy.signal import medfilt
import sys
import os

# ==========================================================================================================
# Pixy2 Python module - FIXED IMPORT =====================================================================
# ==========================================================================================================

# CRITICAL: Must insert the correct path BEFORE any import attempt
# This ensures we get the Pixy2 camera module, not the terminal color library
PIXY2_PATH = "/home/smashbot/pixy2/build/python_demos"

# Remove the wrong pixy from sys.modules if it was already imported
if 'pixy' in sys.modules:
    del sys.modules['pixy']

# Insert correct path at the BEGINNING
if PIXY2_PATH not in sys.path:
    sys.path.insert(0, PIXY2_PATH)

try:
    # Now import from the correct location
    import pixy
    from pixy import BlockArray, VectorArray
    from ctypes import *
    
    # Verify we got the right module
    if not hasattr(pixy, 'init'):
        raise ImportError("Wrong pixy module - missing 'init' function")
    
    pixy_blocks = BlockArray(50)
    pixy_vectors = VectorArray(10)
    _pixy_available = True
    print(f"[OK] Pixy2 module loaded from: {pixy.__file__}")
    
except Exception as e:
    print(f"[WARN] Pixy2 Python module not available: {e}")
    pixy = None
    BlockArray = VectorArray = None
    pixy_blocks = pixy_vectors = None
    _pixy_available = False

# ==========================================================================================================
# General Setup ============================================================================================
# ==========================================================================================================

ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 115200

# Pixy2 Color Signatures
SIG_BLUE = 1
SIG_ORANGE = 2
SIG_PURPLE = 3

# Rest of your code continues here...
