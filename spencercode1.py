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
    print("[OK] Serial connection established")
except Exception as _e:
    print(f"[WARN] Serial not available ({_e}). Will print commands instead.")
    _ser = None
    _serial_ok = False


def _send_line(line: str):
    msg = (line.rstrip() + "\n").encode("ascii", errors="ignore")
    if _serial_ok and _ser is not None:
        try:
            _ser.write(msg)
            _ser.flush()
            print(f"[CMD SENT] {line.rstrip()}")
        except Exception as e:
            print(f"[SERIAL ERR] {e}")
            print(f"[FALLBACK] {line.rstrip()}")
    else:
        print(f"[NO SERIAL] {line.rstrip()}")


# ==========================================================================================================
# Motor commands ===========================================================================================
# ==========================================================================================================

def send_set_vel_pwm(left_pwm, right_pwm):
    L = round(left_pwm)
    R = round(right_pwm)
    _send_line(f"SET_VEL L={L} R={R}")


def stop_motors():
    send_set_vel_pwm(0, 0)
    time.sleep(0.1)


# ==========================================================================================================
# VL53L0X ToF Sensor Setup ================================================================================
# ==========================================================================================================

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

    print("\n=== Initializing ToF Sensors ===")
    lox1 = lox2 = lox3 = lox4 = None

    # Reset all
    for x in (xshut1, xshut2, xshut3, xshut4):
        x.value = False
    time.sleep(0.1)

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

    success = all(s is not None for s in (lox1, lox2, lox3, lox4))
    print(f"=== Sensor Init {'SUCCESS' if success else 'FAILED'} ===\n")
    return success


# ==========================================================================================================
# State Management ==========================================================================================
# ==========================================================================================================

def turn_right():
    print("\n*** EXECUTING RIGHT TURN ***")
    stop_motors()
    send_set_vel_pwm(-220, 220)  # Stronger turn
    time.sleep(0.7)  # Longer turn time
    stop_motors()
    time.sleep(0.2)
    # Drive forward after turn to clear the obstacle
    print("*** Moving forward after turn ***")
    send_set_vel_pwm(190, 190)
    time.sleep(0.5)
    print("*** RIGHT TURN COMPLETE ***\n")


def turn_left():
    print("\n*** EXECUTING LEFT TURN ***")
    stop_motors()
    send_set_vel_pwm(220, -220)  # Stronger turn
    time.sleep(0.7)  # Longer turn time
    stop_motors()
    time.sleep(0.2)
    # Drive forward after turn to clear the obstacle
    print("*** Moving forward after turn ***")
    send_set_vel_pwm(190, 190)
    time.sleep(0.5)
    print("*** LEFT TURN COMPLETE ***\n")


def backup():
    print("\n*** BACKING UP ***")
    stop_motors()
    send_set_vel_pwm(-150, -150)
    time.sleep(0.7)
    stop_motors()
    time.sleep(0.2)
    print("*** BACKUP COMPLETE ***\n")


# ==========================================================================================================
# TOF Sensor Logic ========================================================================================
# ==========================================================================================================

i2c_error_count = 0
consecutive_stalls = 0

def safe_read(sensor, name):
    global i2c_error_count
    max_retries = 3
    
    for attempt in range(max_retries):
        try:
            val = sensor.range
            i2c_error_count = 0
            return val
        except Exception as e:
            if attempt == max_retries - 1:
                i2c_error_count += 1
                print(f"[I2C ERROR] {name} failed after {max_retries} attempts: {e}")
                
                if i2c_error_count >= 5:
                    print("\n[CRITICAL] Too many I2C errors - STOPPING ROBOT")
                    stop_motors()
                    raise Exception("I2C bus failure")
                
                return 8000
            time.sleep(0.01)
    
    return 8000


def interpret_data(r, l, fr, fl):
    """
    Simple and aggressive turn logic
    """
    global consecutive_stalls
    
    # Print sensor values every time
    print(f"Sensors → R:{r:4.0f} L:{l:4.0f} FR:{fr:4.0f} FL:{fl:4.0f}", end=" | ")

    # Get minimum front distance
    front_min = min(fr, fl)

    # ============== CRITICAL OBSTACLE ==============
    if front_min < 120:
        print("⚠️ CRITICAL OBSTACLE! Backing up...")
        backup()
        time.sleep(0.3)  # Wait for sensors to update
        # After backup, immediately check sides and turn
        if r > l + 50:
            print("  → After backup: Turning RIGHT")
            turn_right()
        else:
            print("  → After backup: Turning LEFT")
            turn_left()
        consecutive_stalls = 0
        return

    # ============== TURN DECISION ==============
    # If front is getting close, make turn decision
    if front_min < 400:
        print(f"Front blocked ({front_min:.0f}mm) - deciding turn...")
        
        # Check if either side is clearly open
        right_open = r > 300
        left_open = l > 300
        
        right_status = 'OPEN' if right_open else 'CLOSED'
        left_status = 'OPEN' if left_open else 'CLOSED'
        print(f"  → Right: {right_status} ({r:.0f} mm)")
        print(f"  → Left: {left_status} ({l:.0f} mm)")
        
        # Add delay before checking to ensure fresh sensor data
        time.sleep(0.1)
        
        if right_open and not left_open:
            print("  → DECISION: Turn RIGHT")
            turn_right()
            consecutive_stalls = 0
            return
        
        elif left_open and not right_open:
            print("  → DECISION: Turn LEFT")
            turn_left()
            consecutive_stalls = 0
            return
        
        elif right_open and left_open:
            # Both open - choose the more open side
            if r > l + 50:
                print(f"  → DECISION: Turn RIGHT (larger gap: {r:.0f} vs {l:.0f})")
                turn_right()
            else:
                print(f"  → DECISION: Turn LEFT (larger gap: {l:.0f} vs {r:.0f})")
                turn_left()
            consecutive_stalls = 0
            return
        
        else:
            # Neither side clearly open - just pick the larger side
            consecutive_stalls += 1
            print(f"  → Both sides tight - picking larger side (stall count: {consecutive_stalls})")
            
            if r > l + 30:
                print("  → DECISION: Turn RIGHT (r > l)")
                turn_right()
            else:
                print("  → DECISION: Turn LEFT (l >= r)")
                turn_left()
            
            consecutive_stalls = 0
            return

    # ============== CORRIDOR CENTERING ==============
    diff = r - l
    
    if abs(diff) > 80:
        if diff > 0:
            print("Drift LEFT (R>L)")
            send_set_vel_pwm(150, 200)
        else:
            print("Drift RIGHT (L>R)")
            send_set_vel_pwm(200, 150)
    else:
        print("STRAIGHT")
        send_set_vel_pwm(190, 190)
    
    consecutive_stalls = 0


def driving():
    """Main driving loop"""
    
    print("\n=== Configuring sensors for continuous mode ===")
    for s in (lox1, lox2, lox3, lox4):
        if s is not None:
            s.measurement_timing_budget = 33000
            s.continuous_mode()
    
    time.sleep(0.3)
    print("=== Starting navigation ===\n")

    # Initialize filter arrays
    arrR = [1000, 1000, 1000]
    arrL = [1000, 1000, 1000]
    arrFR = [1000, 1000, 1000]
    arrFL = [1000, 1000, 1000]

    loop_count = 0

    while True:
        try:
            # Read all sensors
            mR = safe_read(lox1, "Right")
            mL = safe_read(lox2, "Left")
            mFR = safe_read(lox3, "Front1")
            mFL = safe_read(lox4, "Front2")

            # Update filter arrays
            arrR = np.append(arrR[1:], mR)
            arrL = np.append(arrL[1:], mL)
            arrFR = np.append(arrFR[1:], mFR)
            arrFL = np.append(arrFL[1:], mFL)

            # Apply median filter
            fR = medfilt(arrR, kernel_size=3)[-1]
            fL = medfilt(arrL, kernel_size=3)[-1]
            fFR = medfilt(arrFR, kernel_size=3)[-1]
            fFL = medfilt(arrFL, kernel_size=3)[-1]

            # Navigation logic
            interpret_data(fR, fL, fFR, fFL)
            
            loop_count += 1
            time.sleep(0.08)  # ~12Hz update rate

        except KeyboardInterrupt:
            raise
        except Exception as e:
            print(f"\n[CRITICAL ERROR in main loop] {e}")
            stop_motors()
            raise


# ==========================================================================================================
# Main =====================================================================================================
# ==========================================================================================================

def main():
    print("\n" + "="*60)
    print("ROBOT NAVIGATION SYSTEM - DEBUG MODE")
    print("="*60 + "\n")

    ok = setID()
    if not ok:
        print("\n❌ FATAL: Sensor initialization failed")
        print("Check connections and try again\n")
        return

    print("✓ All systems ready")
    print("Starting in 2 seconds...\n")
    time.sleep(2)
    
    driving()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        stop_motors()
        print("\n\n🛑 User stopped robot\n")
    except Exception as e:
        stop_motors()
        print(f"\n\n💥 FATAL ERROR: {e}\n")
        import traceback
        traceback.print_exc()
