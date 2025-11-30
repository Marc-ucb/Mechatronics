# -*- coding: utf-8 -*-
import time
import numpy as np
import board  # Blinka pin names for Raspberry Pi
import busio
from digitalio import DigitalInOut, Direction
from adafruit_vl53l0x import VL53L0X
from scipy.signal import medfilt

# ==========================================================================================================
# General Setup ============================================================================================
# ==========================================================================================================

ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 115200

# Pixy2 Setup
PIXY2_PORT = "/dev/ttyACM1"  # Adjust if Pixy2 is on different port
pixy2 = None

# Pixy2 Color Signatures
SIG_BLUE = 1  # Signature 1 = Blue
SIG_ORANGE = 2  # Signature 2 = Orange
SIG_PURPLE = 3  # Signature 3 = Purple

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
SHT_LOX1_PIN = board.D5  # Right XSHUT
SHT_LOX2_PIN = board.D17  # Left  XSHUT
SHT_LOX3_PIN = board.D6  # Front1 XSHUT
SHT_LOX4_PIN = board.D27  # Front2 XSHUT

# ---- hardware bring-up (VL53L0X) ----
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)

# XSHUT controls
xshut1 = DigitalInOut(SHT_LOX1_PIN);
xshut1.direction = Direction.OUTPUT
xshut2 = DigitalInOut(SHT_LOX2_PIN);
xshut2.direction = Direction.OUTPUT
xshut3 = DigitalInOut(SHT_LOX3_PIN);
xshut3.direction = Direction.OUTPUT
xshut4 = DigitalInOut(SHT_LOX4_PIN);
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
# =============================== LOOP START ?? ============================================================
# ==========================================================================================================

previous_states = []
last_detected_color = None
color_detection_timestamp = 0


# history of states - no double 90 in the same direction
def state_history(state: int):
    previous_states.append(state)
    if len(previous_states) > 10:
        previous_states.pop(0)
    print("History:", previous_states)


# state of machine --> motor commands   In loop or out of loop?
def robotState(state: int):
    full = 255
    half = 255 / 2

    match state:
        case 1:
            # Obstacle
            # Stop
            # Backup
            # reassess
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
            time.sleep(
                .5)  # ===================== use this to dial 90 degree turns =====================================
            send_set_vel_pwm(0, 0)
            send_set_vel_pwm(64, 64)
            state_history(2)
            print("Right 90 turn")

        case 3:
            # Left90
            send_set_vel_pwm(0, 0)
            time.sleep(0.1)
            send_set_vel_pwm(full, -full)
            time.sleep(
                0.4)  # ===================== use this to dial 90 degree turns =====================================
            send_set_vel_pwm(0, 0)
            send_set_vel_pwm(64, 64)
            state_history(3)
            print("Left 90 turn")

        case 4:
            # Bridge
            send_servo_raise(180)
            # motor commands
            # ===========================
            # LINE UP WITH OBSTACLE
            # ===========================
            send_servo_lower()
            state_history(4)

        case 5:
            # Ramp
            send_servo_raise(180)
            # motor commands
            # ===========================
            # LINE UP WITH OBSTACLE
            # ===========================
            send_servo_lower()
            state_history(5)

        case 6:
            # gravel
            send_servo_raise(180)
            # motor commands
            # ===========================
            # LINE UP WITH OBSTACLE
            # ===========================
            send_set_vel_pwm(full, full)
            time.sleep(2)  #================= Use to Dial gravel time ======================
            send_set_vel_pwm(64, 64)
            time.sleep(0.1)
            send_set_vel_pwm(0, 0)
            # ===========================
            # AM I IN THE RIGHT SPOT?
            # ==========================
            send_servo_lower()
            state_history(6)


# ==========================================================================================================
# Pixy Code  ===============================================================================================
# ==========================================================================================================

def init_pixy2():
    """Initialize Pixy2 camera for color and line detection."""
    global pixy2
    try:
        from pixy2 import Pixy2
        pixy2 = Pixy2(port=PIXY2_PORT)
        pixy2.set_lamp(1, 1)  # Turn on lamps
        print("[OK] Pixy2 initialized")
        return True
    except Exception as e:
        print(f"[WARN] Pixy2 not available: {e}")
        pixy2 = None
        return False


def detect_color_blocks():
    """
    Detect color blocks using trained signatures.
    Returns: detected_signature or None

    Signature mapping:
    - 1 = Blue
    - 2 = Orange
    - 3 = Purple
    """
    global last_detected_color, color_detection_timestamp

    if pixy2 is None:
        return None

    try:
        # Get color blocks from Pixy2
        blocks = pixy2.get_blocks(sigmap=7, maxblocks=10)  # sigmap=7 means sigs 1,2,3

        if not blocks or len(blocks) == 0:
            return None

        # Find the largest block (most likely to be relevant)
        largest_block = max(blocks, key=lambda b: b['width'] * b['height'])

        signature = largest_block['signature']

        # Only report if block is large enough (filter noise)
        area = largest_block['width'] * largest_block['height']
        if area < 100:  # Adjust threshold as needed
            return None

        # Map signature to color name for logging
        color_names = {1: "BLUE", 2: "ORANGE", 3: "PURPLE"}
        color_name = color_names.get(signature, "UNKNOWN")

        # Debounce: only report new color if it's different or been a while
        current_time = time.time()
        if signature != last_detected_color or (current_time - color_detection_timestamp) > 2.0:
            print(f"[PIXY] Detected {color_name} (sig={signature}), size={area}")
            last_detected_color = signature
            color_detection_timestamp = current_time

        return signature

    except Exception as e:
        print(f"[Pixy2 Color ERROR] {e}")
        return None


def get_pixy_steering():
    """
    Read Pixy2 line vectors and return steering adjustment.
    Returns: correction value to apply to motors, or None if no lines detected.

    Positive correction = turn left (increase left motor, decrease right motor)
    Negative correction = turn right
    """
    if pixy2 is None:
        return None

    try:
        # Get line vectors from Pixy2
        vectors = pixy2.get_main_features()

        if not vectors or len(vectors) < 2:
            return None

        # Find left and right edge lines
        line_xs = [(vec['x0'] + vec['x1']) / 2 for vec in vectors]
        line_xs.sort()

        left_edge = line_xs[0]
        right_edge = line_xs[-1]

        # Calculate track center and robot position
        track_center = (left_edge + right_edge) / 2
        robot_center = 79 / 2  # Pixy2 frame width / 2

        # Error = how far off-center we are
        error = track_center - robot_center

        # Simple proportional control
        correction = error * 2.5  # Adjust this gain as needed

        return correction

    except Exception as e:
        print(f"[Pixy2 Line ERROR] {e}")
        return None


def handle_color_detection(signature):
    """
    Execute actions based on detected color signature.
    Color combinations:
    - Blue + Orange = Bridge
    - Orange + Purple = Gravel
    - Blue + Purple = Ramp
    """
    global last_detected_color

    # Check if we have a previous color to make a pair
    if last_detected_color is None:
        # First color detected, just store it
        return

    # We have two colors, check the combination
    colors = {last_detected_color, signature}

    if colors == {SIG_BLUE, SIG_ORANGE}:
        print("[ACTION] Blue + Orange detected - Bridge")
        robotState(4)  # Bridge routine
        last_detected_color = None  # Reset after action

    elif colors == {SIG_ORANGE, SIG_PURPLE}:
        print("[ACTION] Orange + Purple detected - Gravel")
        robotState(6)  # Gravel routine
        last_detected_color = None  # Reset after action

    elif colors == {SIG_BLUE, SIG_PURPLE}:
        print("[ACTION] Blue + Purple detected - Ramp")
        robotState(5)  # Ramp routine
        last_detected_color = None  # Reset after action
    else:
        # Invalid combination, reset
        print(f"[WARN] Invalid color combination detected")
        last_detected_color = None


# =============================================================================================================
# TOF Sensor Logic ============================================================================================
# =============================================================================================================
def reset_all_sensors():
    print("\n[RESET] Power-cycling ALL VL53L0X sensors...")

    # Stop movement just in case
    send_set_vel_pwm(0, 0)

    # Kill all sensors via XSHUT
    for x in (xshut1, xshut2, xshut3, xshut4):
        x.value = False
    time.sleep(0.1)

    # Re-initialize using existing logic
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


# decision-making based on filtered sensor data
# Interpret data / assign triggers
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
    slant = abs(fr - fl) >= 400  # ======== Adjust slant values here =========

    # ------------------------------------------------------------------

    # 90 degree turns
    if abs(r - l) > 200 and fronts_near:
        # If last state was 2 (Right90) or 3 (Left90) → skip this block
        if previous_states and previous_states[-1] in (2, 3):
            return

        if r > l:
            robotState(2)
            return

        elif r < l:
            robotState(3)
            return

    # Try Pixy2 steering first
    pixy_correction = get_pixy_steering()

    if pixy_correction is not None:
        # Pixy2 is tracking both lines - use it for steering
        base_speed = 150
        left_speed = base_speed + pixy_correction
        right_speed = base_speed - pixy_correction

        # Constrain speeds
        left_speed = max(50, min(200, left_speed))
        right_speed = max(50, min(200, right_speed))

        send_set_vel_pwm(int(left_speed), int(right_speed))
        state_history(20)  # Pixy2 steering
        return

    # Fallback to original TOF-based steering
    if abs(r - l) >= 50 or slant:  # ======== Adjust turn values here ===========
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
        send_set_vel_pwm(150, 150)  # ======= ADJUST STANDARD SPEED HERE ===============
        state_history(99)
    #-------------------------------------------------------------------


def driving():
    for s in (lox1, lox2, lox3, lox4):
        s.measurement_timing_budget = 20000
        s.continuous_mode()

    time.sleep(0.1)  # delay for readings to begin giving data

    arrR, arrL, arrFR, arrFL = [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]

    while True:
        # read raw sensor data / collect 3 readings per sensor in array
        mR = safe_read(lox1, "Right")
        mL = safe_read(lox2, "Left")
        mFR = safe_read(lox3, "Front1")
        mFL = safe_read(lox4, "Front2")

        if mFR < 170 or mFL < 170 or mR < 40 or mL < 40:  # ===========  EMERGENCY STOP CONDITIONS  ============
            robotState(1)

        # ----- sliding window of length 3 for each sensor -----
        np.delete(arrR, 0);
        np.delete(arrL, 0);
        np.delete(arrFR, 0);
        np.delete(arrFL, 0);

        # convert to NumPy arrays
        arrR = np.append(arrR[1:], mR)
        arrL = np.append(arrL[1:], mL)
        arrFR = np.append(arrFR[1:], mFR)
        arrFL = np.append(arrFL[1:], mFL)

        # median filter over the 3 samples
        fR = medfilt(arrR, kernel_size=3)[-1]
        fL = medfilt(arrL, kernel_size=3)[-1]
        fFR = medfilt(arrFR, kernel_size=3)[-1]
        fFL = medfilt(arrFL, kernel_size=3)[-1]

        # decision logic (now with Pixy2)
        interpret_data(fR, fL, fFR, fFL)


#===========================================================================================================
# MOTOR TEST SEQUENCE
#==========================================================================================================
def motor_test_sequence():
    """
    Simple motor test that ignores all sensors and just exercises the robotState
    commands in a fixed sequence:

      1) Straight for 3 seconds
      2) Adjust right
      3) Adjust left
      4) Slope right for 3 seconds
      5) Slope left for 3 seconds
      6) 90 degree right
      7) 90 degree left
      8) Stop
    """

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

    # Try to initialize Pixy2 (optional)
    init_pixy2()

    time.sleep(20)
    driving()
    #motor_test_sequence()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting.")