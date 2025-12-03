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
            _ser.flush()  # Added flush to ensure command is sent
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
    time.sleep(0.1)  # Increased delay for proper reset

    # Right
    xshut1.value = True
    time.sleep(0.05)
    try:
        lox1 = VL53L0X(i2c)
        lox1.set_address(LOX1_ADDRESS)
        print("[OK] Right sensor (0x30)")
    except Exception as e:
        print(f"[ERROR] Right sensor failed: {e}")

    # Left
    xshut2.value = True
    time.sleep(0.05)
    try:
        lox2 = VL53L0X(i2c)
        lox2.set_address(LOX2_ADDRESS)
        print("[OK] Left sensor (0x31)")
    except Exception as e:
        print(f"[ERROR] Left sensor failed: {e}")

    # Front1
    xshut3.value = True
    time.sleep(0.05)
    try:
        lox3 = VL53L0X(i2c)
        lox3.set_address(LOX3_ADDRESS)
        print("[OK] Front1 sensor (0x32)")
    except Exception as e:
        print(f"[ERROR] Front1 failed: {e}")

    # Front2
    xshut4.value = True
    time.sleep(0.05)
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
last_turn_time = 0  # Prevent rapid repeated turns

def state_history(state: int):
    previous_states.append(state)
    if len(previous_states) > 10:
        previous_states.pop(0)


def robotState(state: int):
    global last_turn_time
    full = 255

    match state:
        case 1:  # Obstacle - backup
            send_set_vel_pwm(0, 0)
            time.sleep(0.15)
            send_set_vel_pwm(-120, -120)
            time.sleep(0.6)
            send_set_vel_pwm(0, 0)
            time.sleep(0.1)
            state_history(1)

        case 2:  # Right 90
            send_set_vel_pwm(0, 0)
            time.sleep(0.15)
            send_set_vel_pwm(-full, full)
            time.sleep(0.5)  # Increased for full 90 degree turn
            send_set_vel_pwm(0, 0)
            time.sleep(0.2)
            last_turn_time = time.time()
            state_history(2)

        case 3:  # Left 90
            send_set_vel_pwm(0, 0)
            time.sleep(0.15)
            send_set_vel_pwm(full, -full)
            time.sleep(0.5)  # Increased for full 90 degree turn
            send_set_vel_pwm(0, 0)
            time.sleep(0.2)
            last_turn_time = time.time()
            state_history(3)


# ==========================================================================================================
# TOF Sensor Logic (UPDATED WITH ERROR RECOVERY) ===========================================================
# ==========================================================================================================

i2c_error_count = 0
MAX_I2C_ERRORS = 10

def safe_read(sensor, name):
    global i2c_error_count
    try:
        val = sensor.range
        i2c_error_count = 0  # Reset error count on success
        return val
    except Exception as e:
        i2c_error_count += 1
        print(f"[I2C ERROR] {name}: {e} (count: {i2c_error_count})")
        
        # If too many errors, try to reinitialize sensors
        if i2c_error_count >= MAX_I2C_ERRORS:
            print("[CRITICAL] Too many I2C errors, attempting sensor reset...")
            time.sleep(0.5)
            reinit_sensors()
            i2c_error_count = 0
        
        return 9999


def reinit_sensors():
    """Attempt to reinitialize sensors after I2C errors"""
    global lox1, lox2, lox3, lox4
    
    send_set_vel_pwm(0, 0)  # Stop robot
    print("Reinitializing sensors...")
    
    try:
        setID()
        time.sleep(0.5)
        for s in (lox1, lox2, lox3, lox4):
            if s is not None:
                s.measurement_timing_budget = 20000
                s.continuous_mode()
        time.sleep(0.2)
        print("Sensor reinitialization complete")
    except Exception as e:
        print(f"Reinitialization failed: {e}")


def interpret_data(r, l, fr, fl):
    """
    Improved ToF navigation with better turn detection
    """
    global last_turn_time

    full = 255
    current_time = time.time()

    # Prevent turns too close together
    TURN_COOLDOWN = 1.5  # seconds
    if current_time - last_turn_time < TURN_COOLDOWN:
        # Just drive forward during cooldown
        send_set_vel_pwm(180, 180)
        return

    # =============== EMERGENCY STOP ==================
    if min(fr, fl) < 100:
        print(f"EMERGENCY STOP: fr={fr}, fl={fl}")
        robotState(1)
        return

    # =============== TURN LOGIC ======================
    TURN_THRESHOLD = 400     # side must be this open to turn
    FRONT_BLOCK = 350        # front blocked at this distance

    front_blocked = min(fr, fl) < FRONT_BLOCK
    
    if front_blocked:
        print(f"Front blocked: fr={fr}, fl={fl}, r={r}, l={l}")
        
        # Left open → turn left
        if l > TURN_THRESHOLD and r < TURN_THRESHOLD:
            print("→ Turn Left triggered")
            robotState(3)
            return

        # Right open → turn right
        if r > TURN_THRESHOLD and l < TURN_THRESHOLD:
            print("→ Turn Right triggered")
            robotState(2)
            return

        # Both open → choose larger gap
        if r > TURN_THRESHOLD and l > TURN_THRESHOLD:
            if r > l:
                print("→ Choosing Right turn (both open)")
                robotState(2)
            else:
                print("→ Choosing Left turn (both open)")
                robotState(3)
            return
        
        # Both closed → backup and reassess
        print("→ Both sides closed, backing up")
        robotState(1)
        return

    # =============== CORRIDOR CENTERING ==================
    diff = r - l

    if abs(diff) > 50:
        if diff > 0:
            send_set_vel_pwm(150, 210)  # drift left
            print(f"Centering: drift left (r={r}, l={l})")
        else:
            send_set_vel_pwm(210, 150)  # drift right
            print(f"Centering: drift right (r={r}, l={l})")
        return

    # =============== DRIVE STRAIGHT ==================
    send_set_vel_pwm(190, 190)
    state_history(99)


def driving():
    """Main driving loop with error handling"""

    # Set up continuous mode
    for s in (lox1, lox2, lox3, lox4):
        if s is not None:
            s.measurement_timing_budget = 20000
            s.continuous_mode()

    time.sleep(0.2)

    # Initialize filter arrays
    arrR = [1000, 1000, 1000]
    arrL = [1000, 1000, 1000]
    arrFR = [1000, 1000, 1000]
    arrFL = [1000, 1000, 1000]

    loop_count = 0

    while True:
        try:
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

            if loop_count % 10 == 0:  # Print every 10th loop to reduce spam
                print(f"Sensors: R={fR:.0f}, L={fL:.0f}, FR={fFR:.0f}, FL={fFL:.0f}")
            
            interpret_data(fR, fL, fFR, fFL)
            
            loop_count += 1
            time.sleep(0.05)  # Small delay to prevent CPU overload

        except KeyboardInterrupt:
            raise
        except Exception as e:
            print(f"[LOOP ERROR] {e}")
            send_set_vel_pwm(0, 0)
            time.sleep(0.5)


# ==========================================================================================================
# Main =====================================================================================================
# ==========================================================================================================

def main():
    print("Starting Robot Navigation System...")

    ok = setID()
    if not ok:
        print("ERROR: Sensor initialization failure.")
        print("Check I2C connections and sensor power.")
        return

    print("Sensors initialized successfully!")
    time.sleep(2)
    
    print("Starting navigation...")
    driving()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        send_set_vel_pwm(0, 0)
        print("\nStopping robot and exiting...")
    except Exception as e:
        send_set_vel_pwm(0, 0)
        print(f"\n[CRITICAL ERROR] {e}")
        print("Robot stopped.")
