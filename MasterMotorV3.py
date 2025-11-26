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

# E_Stop = False
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
# Motor constants =============================================================================================
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
def robotState(state: int):
    full = 255
    half = 255/2


    match state:
        case 1:
            # Obstacle
            # Stop
            # Backup
            # reassess
            send_set_vel_pwm(100, 100)
            time.sleep(0.1)
            send_set_vel_pwm(0,0)
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
            time.sleep(.5) # ===================== use this to dial 90 degree turns =====================================
            send_set_vel_pwm(0, 0)
            send_set_vel_pwm(64,64)
            state_history(2)
            print("Right 90 turn")

        case 3:
            # Left90

            send_set_vel_pwm(0, 0)
            time.sleep(0.1)
            send_set_vel_pwm(full,-full)
            time.sleep(0.4)  # ===================== use this to dial 90 degree turns =====================================
            send_set_vel_pwm(0,0)
            send_set_vel_pwm(64,64)
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
            time.sleep(2)                  #================= Use to Dial gravel time ======================
            send_set_vel_pwm(64, 64)
            time.sleep(0.1)
            send_set_vel_pwm(0, 0)
            # ===========================
            # AM I IN THE RIGHT SPOT?
            # ==========================
            send_servo_lower()
            state_history(6)



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

    # Front both < 200mm
    fronts_near = ((fr < 230) and (fl < 230))

    # Front Slant
    slant = abs(fr - fl) >= 400                      # ======== Adjust slant values here =========

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

    # keep robot straight

    if abs(r - l) >= 50 or slant:                    # ======== Adjust turn values here ===========
        if r > l:
            send_set_vel_pwm(10,200)
            state_history(10)

        elif r < l:
            send_set_vel_pwm(200, 10)
            state_history(11)

        elif fr > fl:
            send_set_vel_pwm(10,200)
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

    time.sleep(0.1) # delay for readings to begin giving data

    arrR, arrL, arrFR, arrFL = [0,0,0], [0, 0, 0], [0, 0, 0], [0, 0, 0]

    while True:
        # call backup IR
            # If emergency stop detected -- robotState(1)
        # call pixy
            # If Pixy sees QR code
            # Decide code -- correct robot state

        # read raw sensor data / collect 3 readings per sensor in array

        mR  = safe_read(lox1, "Right")
        mL  = safe_read(lox2, "Left")
        mFR = safe_read(lox3, "Front1")
        mFL = safe_read(lox4, "Front2")

        if mFR < 170 or mFL < 170 or mR < 40 or mL < 40:   # ===========  EMERGENCY STOP CONDITIONS  ============
            robotState(1)
            # E_stop = True

        # ----- sliding window of length 3 for each sensor -----
        np.delete(arrR,0);
        np.delete(arrL,0); 
        np.delete(arrFR,0); 
        np.delete(arrFL,0); 


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

    #print("\n[TEST] speed values slowly increasing")
    #send_set_vel_pwm(25,25)
    #time.sleep(2)
    #$send_set_vel_pwm(50, 50)
    #time.sleep(2)
    #send_set_vel_pwm(75, 75)
    #time.sleep(2)
    #send_set_vel_pwm(100, 100)
    #time.sleep(2)
    send_set_vel_pwm(200, 200)
    time.sleep(2)
    #send_set_vel_pwm(150, 150)
    #time.sleep(2)
    #send_set_vel_pwm(175, 175)
    #time.sleep(2)
    #send_set_vel_pwm(200, 200)
    #time.sleep(2)
    #send_set_vel_pwm(225, 225)
    #time.sleep(2)
    #send_set_vel_pwm(250, 250)
    #time.sleep(2)
    #send_set_vel_pwm(255, 255)
    #time.sleep(10)

    #print("\n[TEST] Straight for 3 seconds")
    #send_set_vel_pwm(100,100)
    #time.sleep(1)
    #send_set_vel_pwm(128,128)
    #time.sleep(0.1)
    #send_set_vel_pwm(255,255)     # straight
    #time.sleep(1)

    #print("\n[TEST] Obstacle")
    #robotState(1)
    #time.sleep(1.0)

    #print("\n[TEST] 90 degree right turn")
    #robotState(2)     # right 90 (includes its own timing)
    #time.sleep(1.0)

    #print("\n[TEST] 90 degree left turn")
    #robotState(3)     # left 90 (includes its own timing)
    #time.sleep(1.0)

    #print("\n[TEST] Adjust right")
    #send_set_vel_pwm(128, 255)  # motor commands for adjust right
    #time.sleep(1.0)

    #print("\n[TEST] Adjust left")
    #send_set_vel_pwm(255, 75)  # motor commands for adjust left
    #time.sleep(1.0)

    #print("\n[TEST] slant right")
    #send_set_vel_pwm(255, 10)  # motor commands for slant right
    #time.sleep(1.0)

    #print("\n[TEST] slant left")
    #send_set_vel_pwm(10, 255)  # motor commands for slant left
    #time.sleep(1.0)

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
    time.sleep(20)
    driving()
    #motor_test_sequence()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting.")
