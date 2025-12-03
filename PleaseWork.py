# -*- coding: utf-8 -*-
import time
import numpy as np
import board  # Blinka pin names for Raspberry Pi
import busio
from digitalio import DigitalInOut, Direction
from adafruit_vl53l0x import VL53L0X
from scipy.signal import medfilt

# ==========================================================================================================
# General Setup ===========================================================================================
# ==========================================================================================================

ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 115200

# Try to open serial; fall back to print-only if not available
try:
    import serial

    _ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.02)
    _serial_ok = True
    print(f"[INFO] Serial OK on {ARDUINO_PORT} @ {ARDUINO_BAUD}")
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


def _read_serial_line():
    """Read a line from Arduino serial if available."""
    if _serial_ok and _ser is not None:
        try:
            if _ser.in_waiting > 0:
                line = _ser.readline().decode('ascii', errors='ignore').strip()
                return line
        except Exception:
            pass
    return None


def _parse_ir_from_serial():
    """
    Parse IR sensor readings from Arduino serial.
    Expected format: "IR L=1.23 R=4.56"
    Updates global _ir_analog_left and _ir_analog_right
    """
    global _ir_analog_left, _ir_analog_right

    line = _read_serial_line()
    if line and line.startswith("IR "):
        try:
            # Parse "IR L=1.23 R=4.56"
            parts = line.split()
            for part in parts:
                if part.startswith("L="):
                    _ir_analog_left = float(part[2:])
                elif part.startswith("R="):
                    _ir_analog_right = float(part[2:])
        except Exception:
            # Silently ignore parse errors
            pass


# ==========================================================================================================
# IR Sensor Setup (Backup/Line Detection) =================================================================
# ==========================================================================================================

# DIGITAL IR:
# True  = dark surface (track)
# False = light surface (bumper/off track)

IR_LEFT_DIGITAL_PIN = board.D23
IR_RIGHT_DIGITAL_PIN = board.D24

ir_left_digital = DigitalInOut(IR_LEFT_DIGITAL_PIN)
ir_left_digital.direction = Direction.INPUT

ir_right_digital = DigitalInOut(IR_RIGHT_DIGITAL_PIN)
ir_right_digital.direction = Direction.INPUT

# Analog readings come from Arduino via serial
# Arduino sends: "IR L=<left_voltage> R=<right_voltage>"
_ir_analog_left = None
_ir_analog_right = None
_analog_available = True  # We get analog via Arduino

# Voltage thresholds for analog IR sensors (for potential future use)
ANALOG_DARK_THRESHOLD = 1.0  # Voltage < 1.0V = dark/on track
ANALOG_LIGHT_THRESHOLD = 4.0  # Voltage > 4.0V = light/off track


def read_ir_analog():
    """
    Read analog voltage from both IR sensors (via Arduino serial).
    Returns: (left_voltage, right_voltage) in volts (0.0 to 5.0V)
    Returns (None, None) if no recent data from Arduino.
    """
    _parse_ir_from_serial()
    return (_ir_analog_left, _ir_analog_right)


def read_ir_analog_boolean():
    """
    Analog → boolean based on thresholds.
    Not currently used in decision logic, but kept for tuning/debug.
    """
    left_v, right_v = read_ir_analog()

    if left_v is None or right_v is None:
        return (None, None)

    # Left sensor boolean
    if left_v < ANALOG_DARK_THRESHOLD:
        left_bool = False  # dark/on track
    elif left_v > ANALOG_LIGHT_THRESHOLD:
        left_bool = True   # light/off track
    else:
        left_bool = None   # transition

    # Right sensor boolean
    if right_v < ANALOG_DARK_THRESHOLD:
        right_bool = False
    elif right_v > ANALOG_LIGHT_THRESHOLD:
        right_bool = True
    else:
        right_bool = None

    return (left_bool, right_bool)


def read_ir_digital():
    """
    Read digital state from both IR sensors.

    With your current wiring / behavior:
    - True  = dark surface (ON TRACK)
    - False = light surface (OFF TRACK / bumper)
    """
    return (ir_left_digital.value, ir_right_digital.value)


def read_ir_sensors():
    """
    Read both analog and digital IR sensors.
    Returns: dict with all readings
    """
    left_v, right_v = read_ir_analog()
    left_analog_bool, right_analog_bool = read_ir_analog_boolean()
    left_d, right_d = read_ir_digital()

    return {
        'left_voltage': left_v,
        'right_voltage': right_v,
        'left_analog_bool': left_analog_bool,
        'right_analog_bool': right_analog_bool,
        'left_digital': left_d,
        'right_digital': right_d
    }


def check_track_status():
    """
    Digital-only track status (not currently used in driving logic).
    """
    left_digital, right_digital = read_ir_digital()

    left_on_track = (left_digital is True)
    right_on_track = (right_digital is True)

    if left_on_track and right_on_track:
        return "ON_TRACK"
    elif not left_on_track and not right_on_track:
        return "COMPLETELY_OFF_TRACK"
    elif not left_on_track and right_on_track:
        return "DRIFTING_LEFT"
    elif left_on_track and not right_on_track:
        return "DRIFTING_RIGHT"
    else:
        return "ON_TRACK"


# ==========================================================================================================
# VL53L0X ToF Sensor Setup ================================================================================
# ==========================================================================================================

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
xshut1 = DigitalInOut(SHT_LOX1_PIN); xshut1.direction = Direction.OUTPUT
xshut2 = DigitalInOut(SHT_LOX2_PIN); xshut2.direction = Direction.OUTPUT
xshut3 = DigitalInOut(SHT_LOX3_PIN); xshut3.direction = Direction.OUTPUT
xshut4 = DigitalInOut(SHT_LOX4_PIN); xshut4.direction = Direction.OUTPUT

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
# Motor commands =============================================================================================
# =============================================================================================================

def send_set_vel_pwm(left_pwm, right_pwm):
    L = round(left_pwm)
    R = round(right_pwm)
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


def send_servo_pos():
    """Ask Arduino to print current servo angle (useful for debugging)."""
    _send_line("SERVO POS")


# ==========================================================================================================
# State Management =========================================================================================
# ==========================================================================================================

previous_states = []


def state_history(state: int):
    previous_states.append(state)
    if len(previous_states) > 10:
        previous_states.pop(0)


def robotState(state: int):
    full = 255
    half = 255 / 2

    match state:
        case 1:
            # Obstacle - Stop, Backup, Reassess
            send_set_vel_pwm(100, 100)
            time.sleep(0.1)
            send_set_vel_pwm(0, 0)
            time.sleep(0.1)
            send_set_vel_pwm(-100, -100)
            time.sleep(.5)
            send_set_vel_pwm(0, 0)
            state_history(1)

        case 2:
            # Right 90
            send_set_vel_pwm(0, 0)
            time.sleep(0.1)
            send_set_vel_pwm(-full, full)
            time.sleep(.45)
            send_set_vel_pwm(0, 0)
            state_history(2)
            print("Right 90 turn")

        case 3:
            # Left 90
            send_set_vel_pwm(0, 0)
            time.sleep(0.1)
            send_set_vel_pwm(full, -full)
            time.sleep(0.4)
            send_set_vel_pwm(0, 0)
            state_history(3)
            print("Left 90 turn")

        case 4:
            # Bridge
            send_servo_raise(180)
            # Add bridge crossing logic here
            send_servo_lower()
            state_history(4)

        case 5:
            # Ramp
            send_servo_raise(180)
            # Add ramp climbing logic here
            send_servo_lower()
            state_history(5)

        case 6:
            # Gravel
            send_servo_raise(180)
            send_set_vel_pwm(full, full)
            time.sleep(2)
            send_set_vel_pwm(64, 64)
            time.sleep(0.1)
            send_set_vel_pwm(0, 0)
            send_servo_lower()
            state_history(6)

        case 7:
            # OFF TRACK - Emergency stop and correction
            print("[EMERGENCY] Robot off track!")
            send_set_vel_pwm(0, 0)
            time.sleep(0.2)
            send_set_vel_pwm(-100, -100)
            time.sleep(0.3)
            send_set_vel_pwm(0, 0)
            state_history(7)


# =============================================================================================================
# IR-based Track Correction (MATCHES DIAGNOSTIC LOGIC, BUT LESS AGGRESSIVE) ==================================
# =============================================================================================================

def handle_ir_correction():
    """
    IR correction using the same DIGITAL semantics as diagnostic:

      DIGITAL:
        True  = dark / track
        False = light / bumper

      Cases:
        L=True,  R=True  -> ON_TRACK      -> no correction
        L=True,  R=False -> too far RIGHT -> nudge LEFT  (small pivot)
        L=False, R=True  -> too far LEFT  -> nudge RIGHT (small pivot)
        L=False, R=False -> BOTH_LIGHT    -> OFF TRACK (emergency)

    Return:
        True  -> EMERGENCY (both light) so ToF should be skipped this cycle
        False -> non-emergency (or no correction), ToF still runs
    """
    left_digital, right_digital = read_ir_digital()

    # Both on track -> no correction
    if left_digital and right_digital:
        return False

    # L=True, R=False  -> robot too far RIGHT -> nudge LEFT
    if left_digital and not right_digital:
        send_set_vel_pwm(-80, 80)   # small left pivot
        time.sleep(0.05)
        state_history(71)
        return False  # non-emergency, let ToF run too

    # L=False, R=True -> robot too far LEFT -> nudge RIGHT
    if right_digital and not left_digital:
        send_set_vel_pwm(80, -80)   # small right pivot
        time.sleep(0.05)
        state_history(72)
        return False  # non-emergency, let ToF run too

    # L=False, R=False -> both light -> OFF TRACK emergency
    robotState(7)
    state_history(73)
    return True  # EMERGENCY: skip ToF this cycle


# =============================================================================================================
# TOF Sensor Logic ============================================================================================
# =============================================================================================================

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
    Decision-making based on filtered ToF sensor data,
    with IR correction given PRIORITY but NOT dominating.
    """

    # PRIORITY 1: IR-based track correction
    ir_emergency = handle_ir_correction()
    if ir_emergency:
        # Off-track emergency handled by IR logic; skip ToF this cycle
        return

    # PRIORITY 2: Emergency stop conditions (ToF)
    if fr < 100 or fl < 100 or r < 20 or l < 20:
        robotState(1)
        return

    # Front both < ~350mm
    fronts_near = ((fr < 350) or (fl < 350))

    # Front slant
    slant = abs(fr - fl) >= 30

    # 90-degree turns using side sensors
    if abs(r - l) > 400 and fronts_near:
        if previous_states and previous_states[-1] in (2, 3):
            return

        if r > l:
            robotState(2)
            return
        elif r < l:
            robotState(3)
            return

    # Keep robot straight using ToF
    if abs(r - l) >= 30 or slant:
        if r > l:
            send_set_vel_pwm(10, 200)
            state_history(10)
            return
        elif r < l:
            send_set_vel_pwm(200, 10)
            state_history(11)
            return
        elif fr > fl:
            send_set_vel_pwm(10, 200)
            state_history(12)
            return
        elif fr < fl:
            send_set_vel_pwm(200, 10)
            state_history(13)
            return
    else:
        send_set_vel_pwm(200, 200)
        state_history(99)
        return


def driving():
    """Main driving loop with ToF and IR sensor integration"""

    for s in (lox1, lox2, lox3, lox4):
        s.measurement_timing_budget = 20000
        s.continuous_mode()

    time.sleep(0.1)

    arrR, arrL, arrFR, arrFL = [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]

    while True:
        # Read ToF sensors with median filtering
        mR = safe_read(lox1, "Right")
        mL = safe_read(lox2, "Left")
        mFR = safe_read(lox3, "Front1")
        mFL = safe_read(lox4, "Front2")

        # Sliding window of length 3 for each sensor
        arrR = np.append(arrR[1:], mR)
        arrL = np.append(arrL[1:], mL)
        arrFR = np.append(arrFR[1:], mFR)
        arrFL = np.append(arrFL[1:], mFL)

        # Median filter over the 3 samples
        fR = medfilt(arrR, kernel_size=3)[-1]
        fL = medfilt(arrL, kernel_size=3)[-1]
        fFR = medfilt(arrFR, kernel_size=3)[-1]
        fFL = medfilt(arrFL, kernel_size=3)[-1]

        # Decision logic (includes IR checking)
        interpret_data(fR, fL, fFR, fFL)


# ==========================================================================================================
# Main =====================================================================================================
# ==========================================================================================================
def main():
    print("Starting")

    # Initialize ToF sensors
    ok = setID()
    if not ok:
        print("Exiting due to sensor initialization failure.")
        return

    print("[OK] Backup IR Left sensor initialized")
    print("[OK] Backup IR Right sensor initialized")

    time.sleep(10)
    driving()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        send_set_vel_pwm(0, 0)
        print("\nExiting.")
