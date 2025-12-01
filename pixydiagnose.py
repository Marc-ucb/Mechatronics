#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pixy2 Import Diagnostics Script
Run this to understand why the pixy module isn't loading
"""

import sys
import os

print("=" * 60)
print("PIXY2 MODULE DIAGNOSTICS")
print("=" * 60)

# 1. Check Python version and executable
print(f"\n1. Python Version: {sys.version}")
print(f"   Python Executable: {sys.executable}")

# 2. Check current sys.path
print(f"\n2. Current sys.path:")
for i, path in enumerate(sys.path):
    print(f"   [{i}] {path}")

# 3. Try to find pixy module files
print(f"\n3. Searching for pixy module files...")
potential_paths = [
    "/home/pi/pixy2/build/python_demos",
    "/home/pi/pixy2/scripts",
    "/usr/local/lib/python3.9/dist-packages",
    "/usr/local/lib/python3.11/dist-packages",
    os.path.expanduser("~/pixy2/build/python_demos"),
]

found_files = []
for search_path in potential_paths:
    if os.path.exists(search_path):
        print(f"   ✓ Path exists: {search_path}")
        try:
            files = os.listdir(search_path)
            pixy_files = [f for f in files if 'pixy' in f.lower()]
            if pixy_files:
                print(f"     Found files: {pixy_files}")
                found_files.append((search_path, pixy_files))
        except Exception as e:
            print(f"     Error listing: {e}")
    else:
        print(f"   ✗ Path not found: {search_path}")

# 4. Try importing with detailed error info
print(f"\n4. Attempting to import pixy module...")

# First try without modifying path
try:
    import pixy

    print(f"   ✓ SUCCESS: pixy imported directly")
    print(f"     Module location: {pixy.__file__}")
    print(f"     Module dir: {dir(pixy)[:10]}...")  # First 10 attributes
except ImportError as e:
    print(f"   ✗ Direct import failed: {e}")
except Exception as e:
    print(f"   ✗ Unexpected error: {type(e).__name__}: {e}")

# Try with each found path
for path, files in found_files:
    print(f"\n   Trying with path: {path}")
    if path not in sys.path:
        sys.path.insert(0, path)

    try:
        import pixy

        print(f"   ✓ SUCCESS with this path!")
        print(f"     Module location: {pixy.__file__}")
        break
    except ImportError as e:
        print(f"   ✗ Still failed: {e}")
        sys.path.remove(path)
    except Exception as e:
        print(f"   ✗ Unexpected error: {type(e).__name__}: {e}")
        if path in sys.path:
            sys.path.remove(path)

# 5. Check for _pixy.so (SWIG wrapper)
print(f"\n5. Checking for SWIG wrapper file (_pixy.so)...")
for path, files in found_files:
    so_files = [f for f in files if f.endswith('.so')]
    if so_files:
        print(f"   Found in {path}:")
        for so_file in so_files:
            full_path = os.path.join(path, so_file)
            print(f"     - {so_file}")
            if os.path.exists(full_path):
                size = os.path.getsize(full_path)
                print(f"       Size: {size} bytes")

# 6. Try importing specific classes
print(f"\n6. Testing import of specific Pixy classes...")
try:
    from pixy import BlockArray, VectorArray

    print(f"   ✓ BlockArray and VectorArray imported successfully")
except ImportError as e:
    print(f"   ✗ Failed to import classes: {e}")
except Exception as e:
    print(f"   ✗ Unexpected error: {type(e).__name__}: {e}")

# 7. Check USB connection
print(f"\n7. Checking USB devices...")
if os.path.exists('/dev'):
    usb_devices = [d for d in os.listdir('/dev') if 'ttyACM' in d or 'usb' in d.lower()]
    if usb_devices:
        print(f"   Found USB devices: {usb_devices}")
    else:
        print(f"   No obvious USB devices found in /dev")

print("\n" + "=" * 60)
print("DIAGNOSTICS COMPLETE")
print("=" * 60)
