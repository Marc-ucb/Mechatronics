# -*- coding: utf-8 -*-
import time
import numpy as np
import board            # Blinka pin names for Raspberry Pi
import busio
from digitalio import DigitalInOut, Direction
from adafruit_vl53l0x import VL53L0X
from scipy.signal import medfilt

# ==========================================================================================================
# General Setup ============================================================================================
# ==========================================================================================================


# ===== Arduino serial link (edit these) =====
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
LOX1_ADDRESS = 0x30   # Right
LOX2_ADDRESS = 0x31   # Left
LOX3_ADDRESS = 0x32   # Front1
LOX4_ADDRESS = 0x33   # Front2

# -------- XSHUT pins --------
SHT_LOX1_PIN = board.D5    # Right XSHUT
SHT_LOX2_PIN = board.D17   # Left  XSHUT
SHT_LOX3_PIN = board.D6    # Front1 XSHUT
SHT_LOX4_PIN = board.D27   # Front2 XSHUT

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
# Motor constants ======================================================================================
# =============================================================================================================

full_Speed = 1
half_Speed = 0.5
quarter_Speed = 0.25
three_Quarter_Speed = 0.75


def send_set_vel_pwm(left_pwm, right_pwm):
    speed = half_Speed
    L = round(left_pwm*speed)
    R = round(right_pwm*speed)
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


# state of machine --> motor commands   In loop or out of loop?
def robotState(state: int, l, r):
    steady = 255
    half = 255/2
    quarter = 255/4

    match state:
        case 1:
            # Obstacle
            #try adjusting left and analyze
            #try adjusting right and analyze
            #reassess center

            # motors stop
            # 90 degree left
            # forward half the distance of l
            # 90 degree right
            # analyze

            # motors stop
            # 90 degree right
            # forward half the distance of l + half distance of r
            # 90 degree left
            # analyze

            # motors stop
            # 90 degree left
            # forward half the distance of r
            # 90 degree right
            # analyze
            send_set_vel_pwm(0,0)
            time.sleep(0.1)
            send_set_vel_pwm(-half, -half)
            time.sleep(1.5)
            state_history(1)


        case 2:
            # Straight
            # even motor commands
            send_set_vel_pwm(steady,steady)
            state_history(2)
            print("straight")

        case 3:
            # Left slope
            # adjust left

            # right motor same speed
            # left motor decrease by half
            send_set_vel_pwm(quarter, steady)
            state_history(3)
            print("Left slope")

        case 4:
            # Right slope
            # adjust right

            # left motor same speed
            # right motor decrease by half
            send_set_vel_pwm(steady, quarter)
            state_history(4)
            print("Right slope")

        case 5:
            # Adjust right
            # turn right then left to straighten

            # left motor same speed
            # right motor decrease by 1/4
            # left motor decrease by half
            # both motors stable
            send_set_vel_pwm(steady, quarter)
            time.sleep(0.75)
            send_set_vel_pwm(quarter, steady)
            time.sleep(0.75)
            send_set_vel_pwm(steady, steady)
            state_history(5)
            print("Adjust right")

        case 6:
            # Adjust left
            # turn left then right to straighten

            # right motor same speed
            # left motor decrease by 1/4
            # right motor decrease by half
            # both motors stable
            send_set_vel_pwm(quarter, steady)
            time.sleep(0.75)
            send_set_vel_pwm(steady, quarter)
            time.sleep(0.75)
            send_set_vel_pwm(steady, steady)
            state_history(6)
            print("Adjust left")

        case 7:
            # Right90
            # Stop -- 90 degree to the right - go straight

            # motors stop
            # right motor pulse forward
            # left motor pulse backwards
            # both motors forward
            send_set_vel_pwm(half, half)
            time.sleep(0.1)
            send_set_vel_pwm(0, 0)
            time.sleep(0.1)
            send_set_vel_pwm(-steady, steady) #arduino code needs to be updated if speed is negative = digital right reverse at same speed as other side
            time.sleep(1.5) # ===================== use this to dial 90 degree turns =====================================
            send_set_vel_pwm(0, 0)
            time.sleep(0.1)
            send_set_vel_pwm(steady, steady)
            state_history(7)
            print("Right 90 turn")

        case 8:
            # Left90
            # stop -- 90 degrees to the left - go straight

            # motors stop
            # left motor pulse forward
            # right motor pulse backwards
            # both motors forward
            send_set_vel_pwm(half, half)
            time.sleep(0.1)
            send_set_vel_pwm(0, 0)
            time.sleep(0.1)
            send_set_vel_pwm(steady,-steady)  # arduino code needs to be updated if speed is negative = digital right reverse at same speed as other side
            time.sleep(1.5)  # ===================== use this to dial 90 degree turns =====================================
            send_set_vel_pwm(0,0)
            time.sleep(0.1)
            send_set_vel_pwm(steady, steady)
            state_history(8)
            print("Left 90 turn")

        case 9:
            # Obstacle: fr > 400 or OOR, fl < 400
            send_set_vel_pwm(half, half)
            state_history(9)
            pass

        case 10:
            # Obstacle: fl > 400 or OOR, fr < 400
            send_set_vel_pwm(half, half)
            state_history(10)
            pass

        case 11:
            # Bridge
            send_servo_raise(180)

            # motor commands

            send_servo_lower()
            state_history(11)
            pass

        case 12:
            # Ramp
            send_servo_raise(180)

            # motor commands

            send_servo_lower()
            state_history(12)
            pass

        case 13:
            # gravel
            send_servo_raise(180)

            # motor commands

            send_servo_lower()
            state_history(13)
            pass

        case _:
            # Unknown / default
            pass


previous_states = []
# history of states - no double 90 in the same direction
def state_history(state: int):
    previous_states.append(state)
    if len(previous_states) > 10:
        previous_states.pop(0)
    print("History:", previous_states)



# ==========================================================================================================
# Backup IR  ===============================================================================================
# ==========================================================================================================


# ==========================================================================================================
# Pixy Code  ===============================================================================================
# ==========================================================================================================


# =============================================================================================================
# TOF Sensor Logic ============================================================================================
# =============================================================================================================
class Measurement:
    __slots__ = ("RangeMM", "RangeStatus")
    def __init__(self, mm=0, status=4):
        self.RangeMM = mm
        self.RangeStatus = status  # 0=ok; 4=OOR (emulated)

def _apply_speed_scale_pwm(pwm: float, scale: float) -> int:
    return max(0, min(255, int(round(pwm * scale))))


def safe_read_sensor(sensor: VL53L0X) -> Measurement:

    try:
        mm = int(sensor.range)
        if 20 <= mm <= 1500:
            return Measurement(mm=mm, status=0)
        else:
            return Measurement(mm=9000, status=4)
    except Exception:
        return Measurement(mm=9000, status=4)

# helper: collect 3 valid readings from ONE sensor
def collect_sensor_data(sensor, delay_s=0.02):
    """
    Use safe_read_sensor(sensor) to collect 3 valid readings (RangeMM)
    and return them as a 1D NumPy array.
    """
    values = []
    while len(values) < 3:
        m = safe_read_sensor(sensor)  # your existing function
        if m.RangeStatus == 0:  # only use valid readings
            values.append(m.RangeMM)
        time.sleep(delay_s)
    return np.array(values, dtype=float)



# decision-making based on filtered sensor data
# Interpret data / assign triggers
def interpret_data(r, l, fr, fl):
    # r  = right sensor (mm or 9000 for OOR)
    # l  = left sensor  (mm or 9000 for OOR)
    # fr = front-right  (mm or 9000 for OOR)
    # fl = front-left   (mm or 9000 for OOR)

    # Right & left roughly equal (within 100mm)
    sides_equal = (r != 9000 and l != 9000 and abs(r - l) <= 150)

    # Front roughly equal (within 100mm)
    fronts_equal = ((fr != 9000 and fl != 9000 and abs(fr - fl) <= 300) or (fr == 9000 and fl == 9000))

    # Front both > 400mm
    fronts_far = ((fr == 9000 or fr > 400) and (fl == 9000 or fl > 400))

    # Front both < 400mm
    fronts_near = ((fr != 9000 and fr < 400) and (fl != 9000 and fl < 400))

    # Front Slant
    slant = ((fr != 9000 and fl != 9000) and abs(fr - fl) >= 300)

    # ------------------------------------------------------------------
    # right and left sensors roughly equal (within 100mm)
    # front sensors roughly equal and < 400mm
    #if sides_equal and fronts_equal and (fr < 250 and fl < 250):
        # Stop - Obstacle
        # state = Obstacle
        #robotState(1, l, r)

    # right and left sensors roughly equal (within 100mm)
    # front sensors not equal and < 400mm
    if sides_equal and ((fr > 400 or fr == 9000) and fl < 400):
         # Stop - Obstacle
        # state = Obstacle
         robotState(9, l, r)

    # right and left sensors roughly equal (within 100mm)
    # front sensors not equal and < 400mm
    elif sides_equal and ((fl > 400 or fl == 9000) and fr < 400):
         # Stop - Obstacle
         # state = Obstacle
         robotState(10, l, r)

    # right and left sensors roughly equal (within 100mm)
    # front sensors roughly equal and > 400mm
    elif sides_equal and fronts_equal and fronts_far:
        # Robot stays straight
        # state = straight
        robotState(2, l, r)

    # right and left sensors roughly equal (within 100mm)
    # front left > front right and > 400mm
    elif sides_equal and slant and (fl > fr) and (fl > 400):
        # Approaching left hand slope turn
        # state = left slope
        robotState(3, l, r)

    # right and left sensors roughly equal (within 100mm)
    # front left < front right and > 400mm
    elif sides_equal and slant and (fl < fr) and (fr > 400):
        # Approaching right hand slope turn
        # state =  right slope
        robotState(4, l, r)

    # right sensor > left sensor (greater than 100mm but less than 400mm)
    # front sensors roughly equal and > 400mm
    elif (r != 9000 and l != 9000 and (r - l) > 100 and (r - l) < 400) and fronts_equal and fronts_far:
        # Adjust to the right until roughly equal then straighten
        # state = adjust right
        robotState(5, l, r)

    # right sensor < left sensor (greater than 100mm)
    # front sensors roughly equal and > 400mm
    elif (r != 9000 and l != 9000 and (l - r) > 100 and (l - r) < 400) and fronts_equal and fronts_far:
        # Adjust to the left until roughly equal then straighten
        # state = adjust left
        robotState(6, l, r)

    # ------------------------------------------------------------------
    # right sensor > left sensor (right sensor greater than 400mm or out of range)
    # front sensors roughly equal and < 400mm
    elif fronts_equal and fronts_near and ((r == 9000 and l != 9000) or (r != 9000 and l != 9000 and r > l and r > 400)):
        # stop - 90 degree turn to the right - continue straight
        # state = Right90
        robotState(7, l, r)

    # right sensor > left sensor ( right sensor greater than 400mm or out of range)
    # front sensors roughly equal and > 400mm
    elif fronts_equal and fronts_far and ((r == 9000 and l != 9000) or (r != 9000 and l != 9000 and r > l and r > 400)):
        # continue straight
        # state = straight
        robotState(2, l, r)

    # right sensor < left sensor (left sensor greater than 400mm or out of range)
    # front sensors roughly equal and < 400mm
    elif fronts_equal and fronts_near and ((l == 9000 and r != 9000) or (l != 9000 and r != 9000 and l > r and l > 400)):
        # stop - 90 degree turn to the left - continue straight
        # state = left90
        robotState(8, l, r)

    # right sensor < left sensor (left sensor greater than 400mm or out of range)
    # front sensors roughly equal and > 400mm
    elif fronts_equal and fronts_far and ((l == 9000 and r != 9000) or (l != 9000 and r != 9000 and l > r and l > 400)):
        # continue straight
        # state = straight
        robotState(2, l, r)

    # right sensor and left sensor greater than 400mm or out of range
    # front sensors roughly equal and > 400mm
    elif ( (r == 9000 or r > 400) and (l == 9000 or l > 400) ) and fronts_equal and fronts_far:
        # stay straight
        # state = straight
        robotState(2, l, r)

    # right sensor and left sensor greater than 400mm or out of range
    # front sensors roughly equal and < 400mm
    elif ( (r == 9000 or r > 400) and (l == 9000 or l > 400) ) and fronts_equal and fronts_near:
        # turn right
        # state = right90
        robotState(7, l, r)


def driving():

    while True:
        # call backup IR

        # call pixy

        # read raw sensor data / collect 3 readings per sensor in array
        arrR, arrL, arrFR, arrFL = [], [], [], []

        for _ in range(3):
            mR = safe_read_sensor(lox1)  # Right
            mL = safe_read_sensor(lox2)  # Left
            mFR = safe_read_sensor(lox3)  # Front1
            mFL = safe_read_sensor(lox4)  # Front2

            # append all readings, including OOR (9000)
            arrR.append(mR.RangeMM)
            arrL.append(mL.RangeMM)
            arrFR.append(mFR.RangeMM)
            arrFL.append(mFL.RangeMM)


        # convert to NumPy arrays
        arrR = np.array(arrR, dtype=float)
        arrL = np.array(arrL, dtype=float)
        arrFR = np.array(arrFR, dtype=float)
        arrFL = np.array(arrFL, dtype=float)

        # median filter over the 3 samples
        fR = medfilt(arrR, kernel_size=3)[-1]
        fL = medfilt(arrL, kernel_size=3)[-1]
        fFR = medfilt(arrFR, kernel_size=3)[-1]
        fFL = medfilt(arrFL, kernel_size=3)[-1]

        print("right sensor: ", fR)
        print("left sensor: ", fL)
        print("Front front sensor: ", fFR)
        print("Front left sensor: ", fFL)

        if fFR or fFL < 300:
            robotState(1, fR, fL)

        # decision logic
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
    print("\n[TEST] speed values slowly increasing")
    send_set_vel_pwm(25,25)
    time.sleep(2)
    send_set_vel_pwm(50, 50)
    time.sleep(2)
    send_set_vel_pwm(75, 75)
    time.sleep(2)
    send_set_vel_pwm(100, 100)
    time.sleep(2)
    send_set_vel_pwm(125, 125)
    time.sleep(2)
    send_set_vel_pwm(150, 150)
    time.sleep(2)
    send_set_vel_pwm(175, 175)
    time.sleep(2)
    send_set_vel_pwm(200, 200)
    time.sleep(2)
    send_set_vel_pwm(225, 225)
    time.sleep(2)
    send_set_vel_pwm(250, 250)
    time.sleep(2)
    send_set_vel_pwm(255, 255)
    time.sleep(10)

    print("\n[TEST] Straight for 3 seconds")
    robotState(2, 0, 0)     # straight
    time.sleep(3.0)

    print("\n[TEST] Adjust right (tighten to the right wall)")
    robotState(5, 0, 0)     # adjust right (has its own internal sleeps)
    time.sleep(1.0)

    print("\n[TEST] Adjust left (tighten to the left wall)")
    robotState(6, 0, 0)     # adjust left (has its own internal sleeps)
    time.sleep(1.0)

    print("\n[TEST] Slope right for 3 seconds")
    robotState(4, 0, 0)     # right slope: left steady, right slower
    time.sleep(3.0)

    print("\n[TEST] Slope left for 3 seconds")
    robotState(3, 0, 0)     # left slope: right steady, left slower
    time.sleep(3.0)

    print("\n[TEST] 90 degree right turn")
    robotState(7, 0, 0)     # right 90 (includes its own timing)
    time.sleep(1.0)

    print("\n[TEST] 90 degree left turn")
    robotState(8, 0, 0)     # left 90 (includes its own timing)
    time.sleep(1.0)

    print("\n[TEST] Stop")
    send_set_vel_pwm(64,64)
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
    time.sleep(10)
    #driving()
    motor_test_sequence()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting.")
