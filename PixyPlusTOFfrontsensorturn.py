# -*- coding: utf-8 -*-
import time
import numpy as np
import board
import busio
from digitalio import DigitalInOut, Direction
from adafruit_vl53l0x import VL53L0X
from scipy.signal import medfilt
from pixy2 import Pixy2

# ==========================================================================================================
# General Setup ============================================================================================
# ==========================================================================================================

ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 115200

# Pixy2 Setup
PIXY2_PORT = "/dev/ttyACM1"  # Adjust if Pixy2 is on different port
pixy2 = None

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
    # Global scaling for all speeds
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
# Robot State Machine ======================================================================================
# ==========================================================================================================

previous_states = []


def state_history(state: int):
    previous_states.append(state)
    if len(previous_states) > 10:
        previous_states.pop(0)
    print("History:", previous_states)


def read_front_distance():
    """Return the minimum of the two front ToF sensors (mm)."""
    return min(
        safe_read(lox3, "Front1"),
        safe_read(lox4, "Front2")
    )


FRONT_CLEAR_MM = 600   # ~2 ft (tune as needed)
TURN_TIMEOUT   = 3.0   # safety timeout in seconds


def turn_until_front_clear(direction: str):
    """
    Turn in place until the front is clear (front distance > FRONT_CLEAR_MM),
    or until TURN_TIMEOUT is reached.
    """
    if direction == "right":
        left_cmd, right_cmd = -255, 255
    elif direction == "left":
        left_cmd, right_cmd = 255, -255
    else:
        return

    send_set_vel_pwm(left_cmd, right_cmd)
    start_time = time.time()

    while True:
        if read_front_distance() > FRONT_CLEAR_MM:
            break
        if time.time() - start_time > TURN_TIMEOUT:
            break
        time.sleep(0.02)

    send_set_vel_pwm(0, 0)
    time.sleep(0.1)
    send_set_vel_pwm(64, 64)
    time.sleep(0.2)
    send_set_vel_pwm(0, 0)


def robotState(state: int):
    """
    Discrete routines for obstacle / 90° turns / bridge / ramp / gravel.
    90° turns (2 / 3) use front ToF 'clear' logic.
    """
    full = 255
    half = 255 / 2

    match state:
        case 1:
            # Obstacle routine
            send_set_vel_pwm(100, 100)
            time.sleep(0.1)
            send_set_vel_pwm(0, 0)
            time.sleep(0.1)
            send_set_vel_pwm(-100, -100)
            time.sleep(0.25)
            send_set_vel_pwm(0, 0)
            state_history(1)
            print("Obstacle routine")

        case 2:
            # Right 90 using front-clear logic
            turn_until_front_clear("right")
            state_history(2)

        case 3:
            # Left 90 using front-clear logic
            turn_until_front_clear("left")
            state_history(3)

        case 4:
            # Bridge
            send_servo_raise(180)
            # TODO: add motor sequence for bridge
            send_servo_lower()
            state_history(4)
            print("Bridge routine")

        case 5:
            # Ramp
            send_servo_raise(180)
            # TODO: add motor sequence for ramp
            send_servo_lower()
            state_history(5)
            print("Ramp routine")

        case 6:
            # Gravel
            send_servo_raise(180)
            send_set_vel_pwm(full, full)
            time.sleep(2)  # tune gravel time
            send_set_vel_pwm(64, 64)
            time.sleep(0.1)
            send_set_vel_pwm(0, 0)
            send_servo_lower()
            state_history(6)
            print("Gravel routine")


# ==========================================================================================================
# Pixy2 Code ===============================================================================================
# ==========================================================================================================

def init_pixy2():
    """Initialize Pixy2 camera for line following."""
    global pixy2
    try:
        pixy2 = Pixy2(port=PIXY2_PORT)
        pixy2.set_lamp(1, 1)  # Turn on lamps
        print("[OK] Pixy2 initialized")
        return True
    except Exception as e:
        print(f"[WARN] Pixy2 not available: {e}")
        pixy2 = None
        return False


def get_pixy_steering():
    """
    Read Pixy2 and return steering adjustment.
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

        # We assume vectors is a list of dict-like objects with x0/x1
        line_xs = [(vec['x0'] + vec['x1']) / 2 for vec in vectors]
        line_xs.sort()

        left_edge = line_xs[0]
        right_edge = line_xs[-1]

        # Calculate track center and robot position
        track_center = (left_edge + right_edge) / 2
        robot_center = 79 / 2  # Pixy2 frame width / 2 (approx)

        # Error = how far off-center we are
        error = track_center - robot_center

        # Simple proportional control
        correction = error * 2.5  # Adjust gain as needed

        return correction

    except Exception as e:
        print(f"[Pixy2 ERROR] {e}")
        return None


# ==========================================================================================================
# TOF Sensor Logic =========================================================================================
# ==========================================================================================================

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
        return 9999


# ==========================================================================================================
# Combined Decision Logic (ToF + Pixy2) ====================================================================
# ==========================================================================================================

def interpret_data(r, l, fr, fl):
    """
    Combined ToF + Pixy2 decision:
      - ToF = coarse structure (90° turns) and guardrails
      - Pixy2 = fine steering when line is visible
    """
    fronts_near = (fr < 230 and fl < 230)
    slant = abs(fr - fl) >= 400

    # ---------- 1) Look for 90°-turn situations from ToF ----------
    if abs(r - l) > 200 and fronts_near:
        # Avoid immediately re-triggering after a turn
        if previous_states and previous_states[-1] in (2, 3):
            return

        if r > l:
            robotState(2)  # Right 90
        else:
            robotState(3)  # Left 90
        return  # Turn overrides line following for this cycle

    # ---------- 2) Get Pixy steering if available ----------
    pixy_correction = get_pixy_steering()

    base_speed = 150
    left_speed = base_speed
    right_speed = base_speed

    if pixy_correction is not None:
        # Use Pixy for fine steering
        left_speed += pixy_correction
        right_speed -= pixy_correction

        # ---------- 3) Use ToF to fence Pixy steering ----------
        # Example: if right side is very close, don't allow strong right turns
        if r < 100:
            right_speed = max(right_speed, base_speed - 20)

        if l < 100:
            left_speed = max(left_speed, base_speed - 20)

        # Clip speeds
        left_speed = max(50, min(200, left_speed))
        right_speed = max(50, min(200, right_speed))

        send_set_vel_pwm(int(left_speed), int(right_speed))
        state_history(20)  # Pixy-based steering with ToF guardrails
        return

    # ---------- 4) Fallback: ToF-only steering ----------
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


# ==========================================================================================================
# Main Driving Loop ========================================================================================
# ==========================================================================================================

def driving():
    # Configure all sensors for continuous ranging (~20ms per reading)
    for s in (lox1, lox2, lox3, lox4):
        s.measurement_timing_budget = 20000
        s.continuous_mode()

    time.sleep(0.1)

    # Sliding window buffers for median filter
    arrR, arrL, arrFR, arrFL = [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]

    while True:
        # Read raw sensor data
        mR = safe_read(lox1, "Right")
        mL = safe_read(lox2, "Left")
        mFR = safe_read(lox3, "Front1")
        mFL = safe_read(lox4, "Front2")

        # ===========  EMERGENCY STOP CONDITIONS  ============
        if mFR < 170 or mFL < 170 or mR < 40 or mL < 40:
            robotState(1)

        # Sliding window (3 samples each)
        arrR = np.append(arrR[1:], mR)
        arrL = np.append(arrL[1:], mL)
        arrFR = np.append(arrFR[1:], mFR)
        arrFL = np.append(arrFL[1:], mFL)

        # Median filter; take latest filtered value
        fR = medfilt(arrR, kernel_size=3)[-1]
        fL = medfilt(arrL, kernel_size=3)[-1]
        fFR = medfilt(arrFR, kernel_size=3)[-1]
        fFL = medfilt(arrFL, kernel_size=3)[-1]

        # Combined decision logic (ToF + Pixy2)
        interpret_data(fR, fL, fFR, fFL)
        # Optional: small pause if you want to reduce CPU usage
        # time.sleep(0.005)


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

    time.sleep(2)
    driving()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting.")
        send_set_vel_pwm(0, 0)
