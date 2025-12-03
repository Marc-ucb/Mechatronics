# -*- coding: utf-8 -*-
import time
import numpy as np
import board
import busio
from digitalio import DigitalInOut, Direction
from adafruit_vl53l0x import VL53L0X
from scipy.signal import medfilt

# ==========================================================================================================
# General Setup ===========================================================================================
# ==========================================================================================================

ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 115200

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


# ==========================================================================================================
# Motor commands ===========================================================================================
# ==========================================================================================================

def send_set_vel_pwm(left_pwm, right_pwm):
    L = round(left_pwm)
    R = round(right_pwm)
    _send_line(f"SET_VEL L={L} R={R}")


# ==========================================================================================================
# Servo helpers ============================================================================================
# ==========================================================================================================

def send_servo_raise(deg: int):
    d = max(0, min(180, int(deg)))
    _send_line(f"SERVO A={d}")


def send_servo_lower():
    _send_line("SERVO REV")


# ==========================================================================================================
# VL53L0X ToF Sensor Setup ================================================================================
# ==========================================================================================================

# Right, Left, Front1, Front2
LOX1_ADDRESS = 0x30  # Right
LOX2_ADDRESS = 0x31  # Left
LOX3_ADDRESS = 0x32  # Front1
LOX4_ADDRESS = 0x33  # Front2

SHT_LOX1_PIN = board.D5
SHT_LOX2_PIN = board.D17
SHT_LOX3_PIN = board.D6
SHT_LOX4_PIN = board.D27

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)

xshut1 = DigitalInOut(SHT_LOX1_PIN); xshut1.direction = Direction.OUTPUT
xshut2 = DigitalInOut(SHT_LOX2_PIN); xshut2.direction = Direction.OUTPUT
xshut3 = DigitalInOut(SHT_LOX3_PIN); xshut3.direction = Direction.OUTPUT
xshut4 = DigitalInOut(SHT_LOX4_PIN); xshut4.direction = Direction.OUTPUT

lox1 = lox2 = lox3 = lox4 = None


def setID():
    global lox1, lox2, lox3, lox4

    lox1 = lox2 = lox3 = lox4 = None

    # Reset all
    for x in (xshut1, xshut2, xshut3, xshut4):
        x.value = False
    time.sleep(0.01)

    # Right
    xshut1.value = True
    time.sleep(0.01)
    try:
        lox1 = VL53L0X(i2c)
        lox1.set_address(LOX1_ADDRESS)
        print("[OK] Right sensor (0x30)")
    except Exception as e:
        print(f"[ERROR] Right sensor failed: {e}")

    # Left
    xshut2.value = True
    time.sleep(0.01)
    try:
        lox2 = VL53L0X(i2c)
        lox2.set_address(LOX2_ADDRESS)
        print("[OK] Left sensor (0x31)")
    except Exception as e:
        print(f"[ERROR] Left sensor failed: {e}")

    # Front1
    xshut3.value = True
    time.sleep(0.01)
    try:
        lox3 = VL53L0X(i2c)
        lox3.set_address(LOX3_ADDRESS)
        print("[OK] Front1 sensor (0x32)")
    except Exception as e:
        print(f"[ERROR] Front1 failed: {e}")

    # Front2
    xshut4.value = True
    time.sleep(0.01)
    try:
        lox4 = VL53L0X(i2c)
        lox4.set_address(LOX4_ADDRESS)
        print("[OK] Front2 sensor (0x33)")
    except Exception as e:
        print(f"[ERROR] Front2 failed: {e}")

    return all(s is not None for s in (lox1, lox2, lox3, lox4))


# ==========================================================================================================
# State Management ==========================================================================================
# ==========================================================================================================

previous_states = []

def state_history(state: int):
    previous_states.append(state)
    if len(previous_states) > 10:
        previous_states.pop(0)


def robotState(state: int):
    full = 255

    match state:
        case 1:  # Obstacle
            send_set_vel_pwm(0, 0)
            time.sleep(0.1)
            send_set_vel_pwm(-100, -100)
            time.sleep(.5)
            send_set_vel_pwm(0, 0)
            state_history(1)

        case 2:  # Right 90
            send_set_vel_pwm(0, 0)
            time.sleep(0.1)
            send_set_vel_pwm(-full, full)
            time.sleep(.45)
            send_set_vel_pwm(0, 0)
            state_history(2)

        case 3:  # Left 90
            send_set_vel_pwm(0, 0)
            time.sleep(0.1)
            send_set_vel_pwm(full, -full)
            time.sleep(0.4)
            send_set_vel_pwm(0, 0)
            state_history(3)

        case 7:  # Off Track (no longer used)
            send_set_vel_pwm(0, 0)
            state_history(7)


# ==========================================================================================================
# TOF Sensor Logic (ONLY LEFT/RIGHT FOR STEERING) ==========================================================
# ==========================================================================================================

def safe_read(sensor, name):
    try:
        return sensor.range
    except Exception as e:
        print(f"[I2C ERROR] {name}: {e}")
        return 9999


def interpret_data(r, l, fr, fl):
    """
    ONLY left/right ToF steering
    front sensors only for obstacle detection
    """
    full = 255

    # Emergency stop
    if fr < 100 or fl < 100 or r < 20 or l < 20:
        robotState(1)
        return

    # 90-degree turns
    if abs(r - l) > 400 and (fr < 350 or fl < 350):
        if previous_states and previous_states[-1] in (2, 3):
            return
        if r > l:
            robotState(2)
        else:
            robotState(3)
        return

    # Steering correction
    if abs(r - l) >= 30:
        if r > l:
            send_set_vel_pwm(10, 200)
            state_history(10)
        else:
            send_set_vel_pwm(200, 10)
            state_history(11)
        return

    # Drive straight
    send_set_vel_pwm(200, 200)
    state_history(99)


def driving():
    """Main driving loop"""

    for s in (lox1, lox2, lox3, lox4):
        s.measurement_timing_budget = 20000
        s.continuous_mode()

    time.sleep(0.1)

    arrR = [0, 0, 0]
    arrL = [0, 0, 0]
    arrFR = [0, 0, 0]
    arrFL = [0, 0, 0]

    while True:
        mR = safe_read(lox1, "Right")
        mL = safe_read(lox2, "Left")
        mFR = safe_read(lox3, "Front1")
        mFL = safe_read(lox4, "Front2")

        arrR = np.append(arrR[1:], mR)
        arrL = np.append(arrL[1:], mL)
        arrFR = np.append(arrFR[1:], mFR)
        arrFL = np.append(arrFL[1:], mFL)

        fR = medfilt(arrR, kernel_size=3)[-1]
        fL = medfilt(arrL, kernel_size=3)[-1]
        fFR = medfilt(arrFR, kernel_size=3)[-1]
        fFL = medfilt(arrFL, kernel_size=3)[-1]

        print("running: ", fR, fL, fFR, fFL)
        interpret_data(fR, fL, fFR, fFL)


# ==========================================================================================================
# Main =====================================================================================================
# ==========================================================================================================

def main():
    print("Starting")

    ok = setID()
    if not ok:
        print("Exiting due to sensor initialization failure.")
        return

    time.sleep(2)
    driving()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        send_set_vel_pwm(0, 0)
        print("\nExiting.")
