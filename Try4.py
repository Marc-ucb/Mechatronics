# -*- coding: utf-8 -*-
import time
import os
import sys
import numpy as np
import board  # Blinka pin names for Raspberry Pi
import busio
from digitalio import DigitalInOut, Direction
from adafruit_vl53l0x import VL53L0X
from scipy.signal import medfilt

# ==========================================================================================================
# Pixy2 Python module path + imports (USB via SWIG) ========================================================
# ==========================================================================================================

# Try a set of likely locations
_candidate_paths = [
    "/src/host/libpixyusb2_examples/python_demos",
    "/home/pi/pixy2/build/python_demos",
    "/home/smashbot/pixy2/build/python_demos",
    "/usr/local/lib/python3.11/dist-packages",
    os.path.join(os.path.dirname(__file__), "..", "pixy2", "build", "python_demos"),
]

# Append the first existing candidate path(s) to sys.path (avoid duplicates)
for p in _candidate_paths:
    try:
        if p and os.path.exists(p) and p not in sys.path:
            sys.path.insert(0, p)
    except Exception:
        pass

# Try safe imports with fallbacks
pixy = None
pixy_mod = None
BlockArray = None
pixy_blocks = None
pixy_blocks_capacity = 0
_pixy_available = False

try:
    import importlib

    # Import top-level package first
    try:
        pixy = importlib.import_module("pixy")
        pixy_mod = pixy
    except Exception as e:
        pixy = None
        pixy_mod = None

    # Attempt to import SWIG-compiled internals (common name: _pixy)
    _swig_mod = None
    try:
        _swig_mod = importlib.import_module("pixy._pixy")
    except Exception:
        try:
            _swig_mod = importlib.import_module("_pixy")
        except Exception:
            _swig_mod = None

    # Resolve classes / functions from available modules
    sources = [m for m in (pixy, _swig_mod) if m is not None]

    for m in sources:
        if BlockArray is None and hasattr(m, "BlockArray"):
            BlockArray = getattr(m, "BlockArray")
        if pixy_mod is None and hasattr(m, "init"):
            pixy_mod = m

    # final attempt: try to import the compiled wrapper
    if BlockArray is None:
        try:
            _maybe = importlib.import_module("pixy")
            if hasattr(_maybe, "BlockArray"):
                BlockArray = getattr(_maybe, "BlockArray")
        except Exception:
            pass

    # Construct arrays if classes available
    if BlockArray is not None:
        try:
            pixy_blocks = BlockArray(50)
            pixy_blocks_capacity = 50  # Store capacity since len() doesn't work on SWIG arrays
        except Exception:
            pixy_blocks = None
            pixy_blocks_capacity = 0

    # Determine availability
    if pixy_mod is not None and pixy_blocks is not None:
        _pixy_available = True
    else:
        _pixy_available = False

except Exception as _e:
    print(f"[WARN] Pixy import fallback failed: {_e}")
    pixy = None
    BlockArray = None
    pixy_blocks = None
    pixy_blocks_capacity = 0
    _pixy_available = False

# If we still couldn't find the SWIG symbols, warn but allow program to continue without Pixy
if not _pixy_available:
    print("[WARN] Pixy2 Python module not available or missing BlockArray. Pixy functionality will be skipped.")
else:
    print("[OK] Pixy module appears available (pixy_mod=%s, BlockArray=%s)" % (
        getattr(pixy_mod, "__name__", str(pixy_mod)),
        "Yes" if BlockArray is not None else "No"
    ))

# ==========================================================================================================
# General Setup ============================================================================================
# ==========================================================================================================

ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 115200

# Pixy2 Color Signatures
SIG_BLUE = 1   # Signature 1 = Blue
SIG_ORANGE = 2 # Signature 2 = Orange
SIG_PURPLE = 3 # Signature 3 = Purple

# Try to open serial; fall back to print-only if not available
try:
    import serial

    _ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.02)
    _serial_ok = True
except Exception as _e:
    print(f"[warn] Serial not available ({_e}). Will print commands instead.")
    _ser = None
    _serial_ok = False


def _send_line(line: str):
    msg = (line.rstrip() + "\n").encode("ascii", errors="ignore")
    if _serial_ok and _ser is not None:
        try:
            _ser.write(msg)
        except Exception as e:
            print(f"[serial err] {e}. Falling back to print.")
            print(line.rstrip())
    else:
        print(line.rstrip())


# -------- Addresses (Right, Left, Front1, Front2) --------
LOX1_ADDRESS = 0x30  # Right
LOX2_ADDRESS = 0x31  # Left
LOX3_ADDRESS = 0x32  # Front1
LOX4_ADDRESS = 0x33  # Front2

# -------- XSHUT pins --------
SHT_LOX1_PIN = board.D5   # Right XSHUT
SHT_LOX2_PIN = board.D17  # Left  XSHUT
SHT_LOX3_PIN = board.D6   # Front1 XSHUT
SHT_LOX4_PIN = board.D27  # Front2 XSHUT

# ---- hardware bring-up (VL53L0X) ----
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)

# XSHUT controls
xshut1 = DigitalInOut(SHT_LOX1_PIN)
xshut1.direction = Direction.OUTPUT
xshut2 = DigitalInOut(SHT_LOX2_PIN)
xshut2.direction = Direction.OUTPUT
xshut3 = DigitalInOut(SHT_LOX3_PIN)
xshut3.direction = Direction.OUTPUT
xshut4 = DigitalInOut(SHT_LOX4_PIN)
xshut4.direction = Direction.OUTPUT

# Sensor objects
lox1 = lox2 = lox3 = lox4 = None


def setID():
    """Bring up and assign unique I2C addresses to all four VL53L0X sensors.
       Returns True if all sensors initialized successfully, False otherwise.
    """
    global lox1, lox2, lox3, lox4

    # Clear any old objects
    lox1 = lox2 = lox3 = lox4 = None

    # all reset (XSHUT LOW)
    for x in (xshut1, xshut2, xshut3, xshut4):
        x.value = False
    time.sleep(0.01)

    # ---- Right -> 0x30 ----
    xshut1.value = True
    time.sleep(0.01)
    try:
        lox1 = VL53L0X(i2c)
        time.sleep(0.01)
        lox1.set_address(LOX1_ADDRESS)
        time.sleep(0.01)
        print("[OK] Right sensor initialized at 0x30")
    except Exception as e:
        print(f"[ERROR] Right sensor (0x30) failed to initialize: {e}")
        lox1 = None

    # ---- Left -> 0x31 ----
    xshut2.value = True
    time.sleep(0.01)
    try:
        lox2 = VL53L0X(i2c)
        time.sleep(0.01)
        lox2.set_address(LOX2_ADDRESS)
        time.sleep(0.01)
        print("[OK] Left sensor initialized at 0x31")
    except Exception as e:
        print(f"[ERROR] Left sensor (0x31) failed to initialize: {e}")
        lox2 = None

    # ---- Front1 -> 0x32 ----
    xshut3.value = True
    time.sleep(0.01)
    try:
        lox3 = VL53L0X(i2c)
        time.sleep(0.01)
        lox3.set_address(LOX3_ADDRESS)
        time.sleep(0.01)
        print("[OK] Front1 sensor initialized at 0x32")
    except Exception as e:
        print(f"[ERROR] Front1 sensor (0x32) failed to initialize: {e}")
        lox3 = None

    # ---- Front2 -> 0x33 ----
    xshut4.value = True
    time.sleep(0.01)
    try:
        lox4 = VL53L0X(i2c)
        time.sleep(0.01)
        lox4.set_address(LOX4_ADDRESS)
        time.sleep(0.01)
        print("[OK] Front2 sensor initialized at 0x33")
    except Exception as e:
        print(f"[ERROR] Front2 sensor (0x33) failed to initialize: {e}")
        lox4 = None

    all_ok = all(s is not None for s in (lox1, lox2, lox3, lox4))
    if not all_ok:
        print("[FATAL] One or more VL53L0X sensors failed to initialize. Driving will NOT start.")
    return all_ok


# =============================================================================================================
# Motor constants =============================================================================================
# =============================================================================================================

full_Speed = 1
half_Speed = 0.5
quarter_Speed = 0.25
three_Quarter_Speed = 0.75


def send_set_vel_pwm(left_pwm, right_pwm):
    speed = half_Speed
    L = round(left_pwm * speed)
    R = round(right_pwm * speed)
    _send_line(f"SET_VEL L={L} R={R}")


# ==========================================================================================================
# Servo helpers ============================================================================================
# ==========================================================================================================
def send_servo_raise(deg: int):
    """
    Counterclockwise relative move by 'deg' (0..180).
    """
    d = max(0, min(180, int(deg)))
    _send_line(f"SERVO A={d}")


def send_servo_lower():
    """
    Clockwise return to the original 'home' angle set before the last raise.
    """
    _send_line("SERVO REV")


# ==========================================================================================================
# =============================== LOOP STATE / HISTORY =====================================================
# ==========================================================================================================

previous_states = []
last_detected_color = None
color_detection_timestamp = 0
prev_color_for_pair = None


def state_history(state: int):
    previous_states.append(state)
    if len(previous_states) > 10:
        previous_states.pop(0)
    print("History:", previous_states)


def robotState(state: int):
    full = 255

    match state:
        case 1:
            # Obstacle
            send_set_vel_pwm(100, 100)
            time.sleep(0.1)
            send_set_vel_pwm(0, 0)
            time.sleep(0.1)
            send_set_vel_pwm(-100, -100)
            time.sleep(.5)
            send_set_vel_pwm(100, 100)
            time.sleep(0.1)
            send_set_vel_pwm(0, 0)
            state_history(1)

        case 2:
            # Right90
            send_set_vel_pwm(0, 0)
            time.sleep(0.1)
            send_set_vel_pwm(-full, full)
            time.sleep(.5)
            send_set_vel_pwm(0, 0)
            send_set_vel_pwm(64, 64)
            state_history(2)
            print("Right 90 turn")

        case 3:
            # Left90
            send_set_vel_pwm(0, 0)
            time.sleep(0.1)
            send_set_vel_pwm(full, -full)
            time.sleep(0.4)
            send_set_vel_pwm(0, 0)
            send_set_vel_pwm(64, 64)
            state_history(3)
            print("Left 90 turn")

        case 4:
            # Bridge
            print("[BRIDGE] Executing bridge sequence")
            send_servo_raise(180)
            send_set_vel_pwm(150, 150)
            time.sleep(2)
            send_servo_lower()
            state_history(4)

        case 5:
            # Ramp
            print("[RAMP] Executing ramp sequence")
            send_servo_raise(180)
            send_set_vel_pwm(150, 150)
            time.sleep(2)
            send_servo_lower()
            state_history(5)

        case 6:
            # Gravel
            print("[GRAVEL] Executing gravel sequence")
            send_servo_raise(180)
            send_set_vel_pwm(full, full)
            time.sleep(2)
            send_set_vel_pwm(64, 64)
            time.sleep(0.1)
            send_set_vel_pwm(0, 0)
            send_servo_lower()
            state_history(6)


# ==========================================================================================================
# Pixy2 USB (SWIG) Code - COLOR DETECTION ONLY ============================================================
# ==========================================================================================================

def init_pixy2():
    """Initialize Pixy2 camera over USB for COLOR BLOCK DETECTION ONLY."""
    global pixy_blocks, pixy_mod, pixy_blocks_capacity
    if not _pixy_available:
        print("[WARN] Pixy2 Python module not loaded; skipping Pixy init.")
        return False

    # use pixy_mod for procedural calls if available; fallback to pixy
    mod = pixy_mod or pixy

    try:
        if hasattr(mod, "init"):
            mod.init()
        else:
            if hasattr(pixy, "init"):
                pixy.init()

        # CRITICAL: Switch program to CCC (Color Connected Components) mode
        try:
            if hasattr(mod, "change_prog"):
                print("[PIXY] Switching to 'ccc' (color detection) mode...")
                mod.change_prog("ccc")
            elif hasattr(pixy, "change_prog"):
                print("[PIXY] Switching to 'ccc' (color detection) mode...")
                pixy.change_prog("ccc")
        except Exception as e:
            print(f"[WARN] Could not switch to ccc mode: {e}")

        # Turn on lamps
        try:
            if hasattr(mod, "set_lamp"):
                mod.set_lamp(1, 1)
            elif hasattr(pixy, "set_lamp"):
                pixy.set_lamp(1, 1)
        except Exception:
            pass

        # Ensure BlockArray exists
        if pixy_blocks is None and BlockArray is not None:
            try:
                pixy_blocks = BlockArray(50)
                pixy_blocks_capacity = 50
            except Exception:
                pixy_blocks = None
                pixy_blocks_capacity = 0

        print(f"[OK] Pixy2 initialized over USB in CCC (color detection) mode")
        print(f"[OK] BlockArray capacity: {pixy_blocks_capacity}")
        print(f"[OK] Detecting signatures: 1=BLUE, 2=ORANGE, 3=PURPLE")
        return True
    except Exception as e:
        print(f"[WARN] Pixy2 not available: {e}")
        return False


def detect_color_blocks():
    """
    Detect color blocks using trained signatures via SWIG API.

    Returns: detected_signature or None

    Signature mapping:
    - 1 = Blue
    - 2 = Orange
    - 3 = Purple
    """
    global last_detected_color, color_detection_timestamp, pixy_blocks, pixy_mod, pixy_blocks_capacity

    if not _pixy_available or pixy_blocks is None:
        return None

    # Use stored capacity instead of len() which doesn't work on SWIG arrays
    try:
        if hasattr(pixy_mod, "ccc_get_blocks"):
            count = pixy_mod.ccc_get_blocks(pixy_blocks_capacity, pixy_blocks)
        elif hasattr(pixy, "ccc_get_blocks"):
            count = pixy.ccc_get_blocks(pixy_blocks_capacity, pixy_blocks)
        else:
            # try _pixy function directly
            import importlib
            _sw = None
            try:
                _sw = importlib.import_module("pixy._pixy")
            except Exception:
                try:
                    _sw = importlib.import_module("_pixy")
                except Exception:
                    _sw = None
            if _sw and hasattr(_sw, "ccc_get_blocks"):
                count = _sw.ccc_get_blocks(pixy_blocks_capacity, pixy_blocks)
            else:
                return None
    except Exception as e:
        print(f"[Pixy2 Color ERROR] {e}")
        return None

    if count <= 0:
        return None

    # Select largest area block
    largest = None
    largest_area = 0
    for i in range(count):
        try:
            b = pixy_blocks[i]
            area = b.m_width * b.m_height
            if area > largest_area:
                largest_area = area
                largest = b
        except Exception:
            continue

    if largest is None or largest_area < 100:  # noise threshold
        return None

    signature = largest.m_signature

    # Map signature to color name for logging
    color_names = {1: "BLUE", 2: "ORANGE", 3: "PURPLE"}
    color_name = color_names.get(signature, "UNKNOWN")

    # Debounce
    current_time = time.time()
    if signature != last_detected_color or (current_time - color_detection_timestamp) > 2.0:
        print(f"[PIXY] Detected {color_name} (sig={signature}), size={largest_area}, pos=({largest.m_x},{largest.m_y})")
        last_detected_color = signature
        color_detection_timestamp = current_time

    return signature


def handle_color_detection(signature):
    """
    Execute actions based on detected color signature.
    Color combinations:
    - Blue + Orange = Bridge
    - Orange + Purple = Gravel
    - Blue + Purple = Ramp
    """
    global prev_color_for_pair

    color_names = {SIG_BLUE: "BLUE", SIG_ORANGE: "ORANGE", SIG_PURPLE: "PURPLE"}
    this_name = color_names.get(signature, f"UNKNOWN({signature})")

    if prev_color_for_pair is None:
        prev_color_for_pair = signature
        print(f"[PIXY] First color in pair: {this_name} (sig={signature})")
        return

    first_sig = prev_color_for_pair
    first_name = color_names.get(first_sig, f"UNKNOWN({first_sig})")
    colors = {first_sig, signature}

    print(f"[PIXY] Color pair detected: {first_name} + {this_name} -> {colors}")

    if colors == {SIG_BLUE, SIG_ORANGE}:
        print("[ACTION] Blue + Orange detected -> Bridge")
        robotState(4)

    elif colors == {SIG_ORANGE, SIG_PURPLE}:
        print("[ACTION] Orange + Purple detected -> Gravel")
        robotState(6)

    elif colors == {SIG_BLUE, SIG_PURPLE}:
        print("[ACTION] Blue + Purple detected -> Ramp")
        robotState(5)

    else:
        print(f"[WARN] Invalid color combination: {first_name} + {this_name}")

    prev_color_for_pair = None


# =============================================================================================================
# TOF Sensor Logic ============================================================================================
# =============================================================================================================

MIN_VALID_MM = 50
MAX_VALID_MM = 1500


def tof_valid(d):
    return MIN_VALID_MM <= d <= MAX_VALID_MM


def reset_all_sensors():
    print("\n[RESET] Power-cycling ALL VL53L0X sensors...")

    send_set_vel_pwm(0, 0)

    for x in (xshut1, xshut2, xshut3, xshut4):
        x.value = False
    time.sleep(0.1)

    ok = setID()

    if ok:
        print("[RESET OK] All sensors restored — continuing operation.\n")
    else:
        print("[RESET FAIL] One or more sensors did not restart.\n")

    return ok


def safe_read(sensor, name):
    try:
        return sensor.range
    except Exception as e:
        print(f"[I2C ERROR] {name}: {e} — resetting ALL sensors")
        reset_all_sensors()
        return 9999  # fail-safe distance


def interpret_data(r, l, fr, fl):
    """
    COLOR DETECTION ONLY VERSION
    - Primary: Color block detection and response
    - Secondary: ToF-based navigation
    - No line following
    """
    full = 255

    # PRIMARY: Check for color blocks
    detected_sig = detect_color_blocks()
    if detected_sig is not None:
        handle_color_detection(detected_sig)
        return

    # Front both < 230mm - obstacle avoidance
    fronts_near = ((fr < 230) and (fl < 230))

    # Front Slant
    slant = abs(fr - fl) >= 400

    # 90 degree turns based on side sensors
    if abs(r - l) > 200 and fronts_near:
        if previous_states and previous_states[-1] in (2, 3):
            return

        if r > l:
            robotState(2)
            return
        elif r < l:
            robotState(3)
            return

    # ToF-based steering (no line following)
    if abs(r - l) >= 50 or slant:
        if r > l:
            send_set_vel_pwm(10, 200)
            state_history(10)
        elif r < l:
            send_set_vel_pwm(200, 10)
            state_history(11)
        elif fr > fl:
            send_set_vel_pwm(10, 200)
            state_history(12)
        elif fr < fl:
            send_set_vel_pwm(200, 10)
            state_history(13)
    else:
        send_set_vel_pwm(150, 150)
        state_history(99)


def driving():
    """Main driving loop - COLOR DETECTION ONLY - 60 SECOND TEST"""
    for s in (lox1, lox2, lox3, lox4):
        s.measurement_timing_budget = 20000
        s.continuous_mode()

    time.sleep(0.1)

    arrR, arrL, arrFR, arrFL = [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]

    print("\n[DRIVING] Starting COLOR DETECTION mode...")
    print("[INFO] Priority: Color detection -> ToF navigation")
    print("[INFO] Test will run for 60 seconds\n")

    # Statistics tracking
    start_time = time.time()
    end_time = start_time + 60.0
    loop_count = 0
    color_detected_count = 0
    obstacle_count = 0
    turn_count = 0
    colors_detected = {SIG_BLUE: 0, SIG_ORANGE: 0, SIG_PURPLE: 0}

    while time.time() < end_time:
        loop_count += 1
        
        mR = safe_read(lox1, "Right")
        mL = safe_read(lox2, "Left")
        mFR = safe_read(lox3, "Front1")
        mFL = safe_read(lox4, "Front2")

        if mFR < 170 or mFL < 170 or mR < 40 or mL < 40:
            obstacle_count += 1
            robotState(1)

        # sliding window of length 3
        arrR = np.append(arrR[1:], mR)
        arrL = np.append(arrL[1:], mL)
        arrFR = np.append(arrFR[1:], mFR)
        arrFL = np.append(arrFL[1:], mFL)

        # median filter over the 3 samples
        fR = medfilt(arrR, kernel_size=3)[-1]
        fL = medfilt(arrL, kernel_size=3)[-1]
        fFR = medfilt(arrFR, kernel_size=3)[-1]
        fFL = medfilt(arrFL, kernel_size=3)[-1]

        # Track color detections
        sig = detect_color_blocks()
        if sig is not None:
            color_detected_count += 1
            if sig in colors_detected:
                colors_detected[sig] += 1

        # Count turns
        if previous_states and len(previous_states) > 0:
            turn_count = len([s for s in previous_states if s in (2, 3)])

        interpret_data(fR, fL, fFR, fFL)
    
    # Stop the robot
    send_set_vel_pwm(0, 0)
    time.sleep(0.1)
    
    # Calculate statistics
    elapsed = time.time() - start_time
    
    print("\n")
    print("=" * 60)
    print("60 SECOND TEST COMPLETE - READOUT")
    print("=" * 60)
    print(f"\nTest Duration: {elapsed:.1f} seconds")
    print(f"Total Loop Iterations: {loop_count}")
    print(f"Average Loop Rate: {loop_count/elapsed:.1f} Hz")
    
    print("\n--- COLOR DETECTION STATS ---")
    print(f"Total Color Detections: {color_detected_count} loops ({100*color_detected_count/loop_count:.1f}%)")
    color_names = {SIG_BLUE: "BLUE", SIG_ORANGE: "ORANGE", SIG_PURPLE: "PURPLE"}
    for sig, name in color_names.items():
        if colors_detected[sig] > 0:
            print(f"  {name}: {colors_detected[sig]} detections")
    
    print("\n--- STATE HISTORY ---")
    print(f"State History (last 10): {previous_states}")
    print(f"Total Obstacles Detected: {obstacle_count}")
    print(f"Total Turns Executed: {turn_count}")
    
    # Count state occurrences
    if previous_states:
        from collections import Counter
        state_counts = Counter(previous_states)
        print("\n--- STATE BREAKDOWN ---")
        state_names = {
            1: "Obstacle Avoidance",
            2: "Right 90° Turn",
            3: "Left 90° Turn",
            4: "Bridge (Blue+Orange)",
            5: "Ramp (Blue+Purple)",
            6: "Gravel (Orange+Purple)",
            10: "ToF Right Steer",
            11: "ToF Left Steer",
            12: "ToF Front-Right Steer",
            13: "ToF Front-Left Steer",
            99: "Straight (ToF Centered)"
        }
        for state, count in sorted(state_counts.items()):
            name = state_names.get(state, f"Unknown State {state}")
            print(f"  State {state:2d} ({name}): {count} times")
    
    print("\n" + "=" * 60)
    print("Test complete. Motors stopped.")
    print("=" * 60 + "\n")


# ==========================================================================================================
# Main =====================================================================================================
# ==========================================================================================================
def main():
    print("=" * 60)
    print("COLOR BLOCK DETECTION ONLY - DIAGNOSTIC VERSION")
    print("=" * 60)
    print("\nStarting sensor initialization...")
    
    ok = setID()
    if not ok:
        print("Exiting due to sensor initialization failure.")
        return

    print("\nInitializing Pixy2 in CCC (color) mode...")
    init_pixy2()

    print("\nWaiting 20 seconds before starting...")
    time.sleep(20)
    
    driving()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n[EXIT] Keyboard interrupt detected")
        send_set_vel_pwm(0, 0)
        print("Motors stopped. Exiting.")
