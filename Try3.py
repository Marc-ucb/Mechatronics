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
VectorArray = None
line_get_main_features = None
line_get_vectors = None
get_frame_width = None
pixy_blocks = None
pixy_vectors = None
pixy_vectors_capacity = 0  # SWIG arrays don't support len(), store capacity
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
        if VectorArray is None and hasattr(m, "VectorArray"):
            VectorArray = getattr(m, "VectorArray")
        if line_get_main_features is None and hasattr(m, "line_get_main_features"):
            line_get_main_features = getattr(m, "line_get_main_features")
        if line_get_vectors is None and hasattr(m, "line_get_vectors"):
            line_get_vectors = getattr(m, "line_get_vectors")
        if get_frame_width is None and hasattr(m, "get_frame_width"):
            get_frame_width = getattr(m, "get_frame_width")
        if pixy_mod is None and hasattr(m, "init"):
            pixy_mod = m

    # final attempt: try to import the compiled wrapper from the build dir name seen in diagnostics
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
        except Exception:
            pixy_blocks = None

    if VectorArray is not None:
        try:
            pixy_vectors = VectorArray(10)
            pixy_vectors_capacity = 10  # Store capacity since len() doesn't work on SWIG arrays
        except Exception:
            pixy_vectors = None
            pixy_vectors_capacity = 0

    # Determine availability
    if pixy_mod is not None and (pixy_blocks is not None or BlockArray is None):
        _pixy_available = True
    else:
        _pixy_available = False

except Exception as _e:
    print(f"[WARN] Pixy import fallback failed: {_e}")
    pixy = None
    BlockArray = VectorArray = None
    pixy_blocks = pixy_vectors = None
    _pixy_available = False

# If we still couldn't find the SWIG symbols, warn but allow program to continue without Pixy
if not _pixy_available:
    print("[WARN] Pixy2 Python module not available or missing expected symbols. Pixy functionality will be skipped.")
else:
    print("[OK] Pixy module appears available (pixy_mod=%s, VectorArray=%s)" % (
        getattr(pixy_mod, "__name__", str(pixy_mod)),
        "Yes" if VectorArray is not None else "No"
    ))

# ==========================================================================================================
# General Setup ============================================================================================
# ==========================================================================================================

ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 115200

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
# =============================== LOOP STATE / HISTORY =====================================================
# ==========================================================================================================

previous_states = []


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
            time.sleep(.5)  # dial 90 degree turns
            send_set_vel_pwm(0, 0)
            send_set_vel_pwm(64, 64)
            state_history(2)
            print("Right 90 turn")

        case 3:
            # Left90
            send_set_vel_pwm(0, 0)
            time.sleep(0.1)
            send_set_vel_pwm(full, -full)
            time.sleep(0.4)  # dial 90 degree turns
            send_set_vel_pwm(0, 0)
            send_set_vel_pwm(64, 64)
            state_history(3)
            print("Left 90 turn")


# ==========================================================================================================
# Pixy2 USB (SWIG) Code - LINE FOLLOWING ONLY =============================================================
# ==========================================================================================================

def init_pixy2():
    """Initialize Pixy2 camera over USB for LINE DETECTION ONLY."""
    global pixy_vectors, pixy_mod, pixy_vectors_capacity
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

        # switch program to line
        try:
            if hasattr(mod, "change_prog"):
                mod.change_prog("line")
            elif hasattr(pixy, "change_prog"):
                pixy.change_prog("line")
        except Exception:
            pass

        try:
            if hasattr(mod, "set_lamp"):
                mod.set_lamp(1, 1)
            elif hasattr(pixy, "set_lamp"):
                pixy.set_lamp(1, 1)
        except Exception:
            pass

        # ensure arrays exist
        if pixy_vectors is None and VectorArray is not None:
            try:
                pixy_vectors = VectorArray(10)
                pixy_vectors_capacity = 10
            except Exception:
                pixy_vectors = None
                pixy_vectors_capacity = 0

        fw = None
        if get_frame_width is not None:
            try:
                fw = get_frame_width()
            except Exception:
                fw = None

        print(f"[OK] Pixy2 initialized over USB in LINE mode (frame width = {fw})")
        print(f"[OK] VectorArray capacity: {pixy_vectors_capacity}")
        return True
    except Exception as e:
        print(f"[WARN] Pixy2 not available: {e}")
        return False


def get_pixy_steering():
    """
    Read Pixy2 line vectors and return steering info.

    Returns:
      None if no lines,
      or (correction, mode, edge_side) where:
        mode = "two"  -> using two edges (center between them)
        mode = "one"  -> single line; edge_side in {"left","right"}
    """
    global pixy_vectors, pixy_mod, pixy_vectors_capacity

    if not _pixy_available or pixy_vectors is None:
        return None

    try:
        # Update line features then grab vectors
        if line_get_main_features is not None:
            line_get_main_features()
        elif hasattr(pixy_mod, "line_get_main_features"):
            pixy_mod.line_get_main_features()
        elif hasattr(pixy, "line_get_main_features"):
            pixy.line_get_main_features()

        # Use the stored capacity instead of len() which doesn't work on SWIG arrays
        if line_get_vectors is not None:
            count = line_get_vectors(pixy_vectors_capacity, pixy_vectors)
        elif hasattr(pixy_mod, "line_get_vectors"):
            count = pixy_mod.line_get_vectors(pixy_vectors_capacity, pixy_vectors)
        elif hasattr(pixy, "line_get_vectors"):
            count = pixy.line_get_vectors(pixy_vectors_capacity, pixy_vectors)
        else:
            # try from swig module
            import importlib
            _sw = None
            try:
                _sw = importlib.import_module("pixy._pixy")
            except Exception:
                try:
                    _sw = importlib.import_module("_pixy")
                except Exception:
                    _sw = None
            if _sw and hasattr(_sw, "line_get_vectors"):
                count = _sw.line_get_vectors(pixy_vectors_capacity, pixy_vectors)
            else:
                return None
    except Exception as e:
        print(f"[Pixy2 Line ERROR] {e}")
        return None

    if count <= 0:
        return None

    # Compute midpoint X of each vector
    line_xs = []
    for i in range(count):
        try:
            v = pixy_vectors[i]
            mid_x = 0.5 * (v.m_x0 + v.m_x1)
            line_xs.append(mid_x)
        except Exception:
            continue

    if not line_xs:
        return None

    line_xs.sort()

    # Frame width from Pixy, fallback if needed
    try:
        if get_frame_width is not None:
            frame_width = float(get_frame_width())
        elif hasattr(pixy_mod, "get_frame_width"):
            frame_width = float(pixy_mod.get_frame_width())
        elif hasattr(pixy, "get_frame_width"):
            frame_width = float(pixy.get_frame_width())
        else:
            frame_width = 79.0
    except Exception:
        frame_width = 79.0

    robot_center = frame_width / 2.0
    gain = 2.5

    # Two (or more) lines: treat as left/right edges
    if len(line_xs) >= 2:
        left_edge = line_xs[0]
        right_edge = line_xs[-1]
        track_center = 0.5 * (left_edge + right_edge)
        error = track_center - robot_center
        correction = error * gain
        return correction, "two", None

    # Exactly one line
    line_x = line_xs[0]
    edge_side = "left" if line_x < robot_center else "right"
    error = line_x - robot_center
    correction = error * gain
    return correction, "one", edge_side


# =============================================================================================================
# TOF Sensor Logic ============================================================================================
# =============================================================================================================

MIN_VALID_MM = 50
MAX_VALID_MM = 1500
TARGET_SIDE_DIST_MM = 300   # desired distance to the "far" wall
K_TOF_STEER = 0.05          # how strongly ToF affects steering


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
    LINE FOLLOWING ONLY VERSION
    - Removed color detection
    - Uses Pixy2 line following as primary navigation
    - ToF fusion when only one line detected
    - ToF-only fallback when no lines detected
    """
    full = 255

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

    # PRIMARY: Pixy2 line steering (with ToF fusion when only one line)
    pixy_result = get_pixy_steering()

    if pixy_result is not None:
        pixy_correction, mode, edge_side = pixy_result

        base_speed = 150

        if mode == "two":
            # Two lines detected - pure Pixy steering
            left_speed = base_speed + pixy_correction
            right_speed = base_speed - pixy_correction
            print(f"[LINE] Two edges: correction={pixy_correction:.1f}")

        elif mode == "one":
            # One line detected - fuse with ToF
            tof_extra = 0.0

            # line on the right -> use LEFT ToF as far wall
            if edge_side == "right":
                if tof_valid(l):
                    tof_error = l - TARGET_SIDE_DIST_MM
                    tof_extra = K_TOF_STEER * tof_error
                    print(f"[FUSION] Right line + Left ToF: pixy={pixy_correction:.1f}, tof={tof_extra:.1f}")
                else:
                    print("[FUSION] Left ToF invalid - Pixy-only steering")

            # line on the left -> use RIGHT ToF as far wall
            elif edge_side == "left":
                if tof_valid(r):
                    tof_error = r - TARGET_SIDE_DIST_MM
                    tof_extra = -K_TOF_STEER * tof_error
                    print(f"[FUSION] Left line + Right ToF: pixy={pixy_correction:.1f}, tof={tof_extra:.1f}")
                else:
                    print("[FUSION] Right ToF invalid - Pixy-only steering")

            combined = pixy_correction + tof_extra

            left_speed = base_speed + combined
            right_speed = base_speed - combined

        else:
            left_speed = base_speed
            right_speed = base_speed

        left_speed = max(50, min(200, left_speed))
        right_speed = max(50, min(200, right_speed))

        send_set_vel_pwm(int(left_speed), int(right_speed))
        state_history(20)  # Pixy2 steering state
        return

    # FALLBACK: ToF-only steering when no lines detected
    print("[FALLBACK] No lines detected - using ToF-only steering")
    
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
    """Main driving loop - LINE FOLLOWING ONLY - 60 SECOND TEST"""
    for s in (lox1, lox2, lox3, lox4):
        s.measurement_timing_budget = 20000
        s.continuous_mode()

    time.sleep(0.1)  # delay for readings to begin giving data

    arrR, arrL, arrFR, arrFL = [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]

    print("\n[DRIVING] Starting LINE FOLLOWING mode...")
    print("[INFO] Priority: Pixy2 line -> ToF fusion (1-line) -> ToF-only fallback")
    print("[INFO] Test will run for 60 seconds\n")

    # Statistics tracking
    start_time = time.time()
    end_time = start_time + 60.0  # 60 second test
    loop_count = 0
    pixy_line_count = 0
    tof_fallback_count = 0
    obstacle_count = 0
    turn_count = 0

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

        # Track what navigation mode was used
        pixy_result = get_pixy_steering()
        if pixy_result is not None:
            pixy_line_count += 1
        else:
            tof_fallback_count += 1
        
        # Count turns
        if previous_states and len(previous_states) > 0:
            last_state = previous_states[-1]
            if last_state in (2, 3):
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
    print("\n--- NAVIGATION MODE USAGE ---")
    print(f"Pixy Line Following: {pixy_line_count} loops ({100*pixy_line_count/loop_count:.1f}%)")
    print(f"ToF Fallback: {tof_fallback_count} loops ({100*tof_fallback_count/loop_count:.1f}%)")
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
            10: "ToF Right Steer",
            11: "ToF Left Steer",
            12: "ToF Front-Right Steer",
            13: "ToF Front-Left Steer",
            20: "Pixy Line Steering",
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
    print("LINE FOLLOWING ONLY - DIAGNOSTIC VERSION")
    print("=" * 60)
    print("\nStarting sensor initialization...")
    
    ok = setID()
    if not ok:
        print("Exiting due to sensor initialization failure.")
        return

    print("\nInitializing Pixy2 in LINE mode...")
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
