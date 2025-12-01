# -*- coding: utf-8 -*-
import time
import numpy as np
import board  # Blinka pin names for Raspberry Pi
import busio
from digitalio import DigitalInOut, Direction
from adafruit_vl53l0x import VL53L0X
from scipy.signal import medfilt
import sys
import pixy
from pixy import *
from ctypes import *

# ==========================================================================================================
# Pixy2 Python module path + imports (USB via SWIG) ========================================================
# ==========================================================================================================

# Adjust this path if your pixy2 repo is in a different location
#sys.path.append("/home/pi/pixy2/build/python_demos")

try:
    pixy_blocks = BlockArray(50)   # up to 50 blocks
    pixy_vectors = VectorArray(10) # up to 10 line vectors
    _pixy_available = True
except Exception as _e:
    print(f"[WARN] Pixy2 Python module not available: {_e}")
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
    Matches Arduino behavior: SERVO A=<deg> moves +<deg> from current and stays.
    """
    d = max(0, min(180, int(deg)))
    _send_line(f"SERVO A={d}")


def send_servo_lower():
    """
    Clockwise return to the original 'home' angle set before the last raise.
    Toggles back to the offset if called again (Arduino maintains the toggle).
    """
    _send_line("SERVO REV")


def send_servo_pos():
    """Ask Arduino to print current servo angle (useful for debugging)."""
    _send_line("SERVO POS")


# ==========================================================================================================
# =============================== LOOP STATE / HISTORY =====================================================
# ==========================================================================================================

previous_states = []
last_detected_color = None
color_detection_timestamp = 0

# For color-pair logic (separate from debouncing)
prev_color_for_pair = None


def state_history(state: int):
    previous_states.append(state)
    if len(previous_states) > 10:
        previous_states.pop(0)
    print("History:", previous_states)


def robotState(state: int):
    full = 255
    half = 255 / 2

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

        case 4:
            # Bridge
            send_servo_raise(180)
            # TODO: add detailed bridge behavior here
            send_servo_lower()
            state_history(4)

        case 5:
            # Ramp
            send_servo_raise(180)
            # TODO: add detailed ramp behavior here
            send_servo_lower()
            state_history(5)

        case 6:
            # Gravel
            send_servo_raise(180)
            send_set_vel_pwm(full, full)
            time.sleep(2)  # dial gravel time
            send_set_vel_pwm(64, 64)
            time.sleep(0.1)
            send_set_vel_pwm(0, 0)
            send_servo_lower()
            state_history(6)


# ==========================================================================================================
# Pixy2 USB (SWIG) Code ====================================================================================
# ==========================================================================================================

def init_pixy2():
    """Initialize Pixy2 camera over USB for color + line detection."""
    global pixy_blocks, pixy_vectors
    if not _pixy_available:
        print("[WARN] Pixy2 Python module not loaded; skipping Pixy init.")
        return False

    try:
        pixy.init()
        pixy.change_prog("line")  # use line-tracking program

        try:
            pixy.set_lamp(1, 1)  # turn on both lamps if supported
        except AttributeError:
            pass

        if pixy_blocks is None and BlockArray is not None:
            pixy_blocks = BlockArray(50)
        if pixy_vectors is None and VectorArray is not None:
            pixy_vectors = VectorArray(10)

        fw = get_frame_width()
        print(f"[OK] Pixy2 initialized over USB (frame width = {fw})")
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
    global last_detected_color, color_detection_timestamp

    if not _pixy_available or pixy_blocks is None:
        return None

    try:
        count = pixy.ccc_get_blocks(50, pixy_blocks)
    except Exception as e:
        print(f"[Pixy2 Color ERROR] {e}")
        return None

    if count <= 0:
        return None

    # Select largest area block
    largest = None
    largest_area = 0
    for i in range(count):
        b = pixy_blocks[i]
        area = b.m_width * b.m_height
        if area > largest_area:
            largest_area = area
            largest = b

    if largest is None or largest_area < 100:  # noise threshold
        return None

    signature = largest.m_signature

    # Map signature to color name for logging
    color_names = {1: "BLUE", 2: "ORANGE", 3: "PURPLE"}
    color_name = color_names.get(signature, "UNKNOWN")

    # Debounce
    current_time = time.time()
    if signature != last_detected_color or (current_time - color_detection_timestamp) > 2.0:
        print(f"[PIXY] Detected {color_name} (sig={signature}), size={largest_area}")
        last_detected_color = signature
        color_detection_timestamp = current_time

    return signature


def get_pixy_steering():
    """
    Read Pixy2 line vectors and return steering info.

    Returns:
      None if no lines,
      or (correction, mode, edge_side) where:
        mode = "two"  -> using two edges (center between them)
        mode = "one"  -> single line; edge_side in {"left","right"}
    """
    if not _pixy_available or pixy_vectors is None:
        return None

    try:
        # Update line features then grab vectors
        line_get_main_features()
        count = line_get_vectors(len(pixy_vectors), pixy_vectors)
    except Exception as e:
        print(f"[Pixy2 Line ERROR] {e}")
        return None

    if count <= 0:
        return None

    # Compute midpoint X of each vector
    line_xs = []
    for i in range(count):
        v = pixy_vectors[i]
        mid_x = 0.5 * (v.m_x0 + v.m_x1)
        line_xs.append(mid_x)

    line_xs.sort()

    # Frame width from Pixy, fallback if needed
    try:
        frame_width = float(get_frame_width())
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

    print(f"[PIXY] Color pair: {first_name} + {this_name} -> {colors}")

    if colors == {SIG_BLUE, SIG_ORANGE}:
        print("[ACTION] Blue + Orange detected - Bridge")
        robotState(4)

    elif colors == {SIG_ORANGE, SIG_PURPLE}:
        print("[ACTION] Orange + Purple detected - Gravel")
        robotState(6)

    elif colors == {SIG_BLUE, SIG_PURPLE}:
        print("[ACTION] Blue + Purple detected - Ramp")
        robotState(5)

    else:
        print("[WARN] Invalid color combination detected")

    prev_color_for_pair = None


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
    full = 255

    # Check for color blocks first
    detected_sig = detect_color_blocks()
    if detected_sig is not None:
        handle_color_detection(detected_sig)
        return

    # Front both < 200mm
    fronts_near = ((fr < 230) and (fl < 230))

    # Front Slant
    slant = abs(fr - fl) >= 400  # adjust slant threshold as needed

    # 90 degree turns
    if abs(r - l) > 200 and fronts_near:
        if previous_states and previous_states[-1] in (2, 3):
            return

        if r > l:
            robotState(2)
            return

        elif r < l:
            robotState(3)
            return

    # Try Pixy2 steering first (with ToF fusion when only one line)
    pixy_result = get_pixy_steering()

    if pixy_result is not None:
        pixy_correction, mode, edge_side = pixy_result

        base_speed = 150

        if mode == "two":
            left_speed = base_speed + pixy_correction
            right_speed = base_speed - pixy_correction

        elif mode == "one":
            tof_extra = 0.0

            # line on the right -> use LEFT ToF as far wall
            if edge_side == "right":
                if tof_valid(l):
                    tof_error = l - TARGET_SIDE_DIST_MM
                    tof_extra = K_TOF_STEER * tof_error
                else:
                    print("[FUSION] Left ToF invalid in 1-line RIGHT mode — Pixy-only steering")

            # line on the left -> use RIGHT ToF as far wall
            elif edge_side == "left":
                if tof_valid(r):
                    tof_error = r - TARGET_SIDE_DIST_MM
                    tof_extra = -K_TOF_STEER * tof_error
                else:
                    print("[FUSION] Right ToF invalid in 1-line LEFT mode — Pixy-only steering")

            combined = pixy_correction + tof_extra

            left_speed = base_speed + combined
            right_speed = base_speed - combined

        else:
            left_speed = base_speed
            right_speed = base_speed

        left_speed = max(50, min(200, left_speed))
        right_speed = max(50, min(200, right_speed))

        send_set_vel_pwm(int(left_speed), int(right_speed))
        state_history(20)  # Pixy2 steering (possibly fused with ToF)
        return

    # Fallback to original TOF-based steering
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
    for s in (lox1, lox2, lox3, lox4):
        s.measurement_timing_budget = 20000
        s.continuous_mode()

    time.sleep(0.1)  # delay for readings to begin giving data

    arrR, arrL, arrFR, arrFL = [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]

    while True:
        mR = safe_read(lox1, "Right")
        mL = safe_read(lox2, "Left")
        mFR = safe_read(lox3, "Front1")
        mFL = safe_read(lox4, "Front2")

        if mFR < 170 or mFL < 170 or mR < 40 or mL < 40:
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

        interpret_data(fR, fL, fFR, fFL)


# ===========================================================================================================
# MOTOR TEST SEQUENCE
# ==========================================================================================================
def motor_test_sequence():
    send_set_vel_pwm(200, 200)
    time.sleep(2)

    print("\n[TEST] Stop")
    send_set_vel_pwm(64, 64)
    time.sleep(0.1)
    send_set_vel_pwm(0, 0)
    time.sleep(1.0)

    print("\n[TEST] Motor test sequence complete.")


# ==========================================================================================================
# Main =====================================================================================================
# ==========================================================================================================
def main():
    print("Starting")
    ok = setID()
    if not ok:
        print("Exiting due to sensor initialization failure.")
        return

    init_pixy2()  # USB Pixy2

    time.sleep(20)
    driving()
    # motor_test_sequence()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting.")
