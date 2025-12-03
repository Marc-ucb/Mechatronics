# -*- coding: utf-8 -*-
import time
import board
from digitalio import DigitalInOut, Direction

# ==========================================================================================================
# Serial Setup for Arduino Motor Control ===================================================================
# ==========================================================================================================

ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 115200

try:
    import serial
    _ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.02)
    _serial_ok = True
    print(f"[INFO] Serial OK on {ARDUINO_PORT}")
except Exception as e:
    print(f"[ERROR] Serial not available: {e}")
    _ser = None
    _serial_ok = False


def send_motor_command(left_pwm, right_pwm):
    """Send motor command to Arduino."""
    L = round(left_pwm)
    R = round(right_pwm)
    msg = f"SET_VEL L={L} R={R}\n".encode("ascii")
    
    if _serial_ok and _ser is not None:
        try:
            _ser.write(msg)
        except Exception as e:
            print(f"[ERROR] Serial write failed: {e}")
    else:
        print(f"SET_VEL L={L} R={R}")


# ==========================================================================================================
# IR Sensor Setup ==========================================================================================
# ==========================================================================================================

IR_LEFT_DIGITAL_PIN = board.D23
IR_RIGHT_DIGITAL_PIN = board.D24

ir_left = DigitalInOut(IR_LEFT_DIGITAL_PIN)
ir_left.direction = Direction.INPUT

ir_right = DigitalInOut(IR_RIGHT_DIGITAL_PIN)
ir_right.direction = Direction.INPUT

print("[OK] IR sensors initialized")


def read_ir_sensors():
    """
    Read IR sensors.
    True  = dark surface (ON TRACK)
    False = light surface (OFF TRACK / bumper)
    """
    return (ir_left.value, ir_right.value)


# ==========================================================================================================
# Main Control Loop ========================================================================================
# ==========================================================================================================

def main():
    print("\n=== IR-Only Robot Control ===")
    print("Starting in 3 seconds...\n")
    time.sleep(3)
    
    try:
        while True:
            left_on_track, right_on_track = read_ir_sensors()
            
            # Both sensors on track -> GO STRAIGHT
            if left_on_track and right_on_track:
                print("Both ON track -> STRAIGHT (200, 200)")
                send_motor_command(200, 200)
            
            # Left on track, Right off track -> TOO FAR RIGHT -> NUDGE LEFT
            elif left_on_track and not right_on_track:
                print("Right OFF track -> NUDGE LEFT (-80, 80)")
                send_motor_command(-80, 80)
            
            # Right on track, Left off track -> TOO FAR LEFT -> NUDGE RIGHT
            elif right_on_track and not left_on_track:
                print("Left OFF track -> NUDGE RIGHT (80, -80)")
                send_motor_command(80, -80)
            
            # Both sensors off track -> EMERGENCY BACKUP
            else:
                print("BOTH OFF track -> BACKUP (-150, -150)")
                send_motor_command(-150, -150)
                time.sleep(0.5)
                send_motor_command(0, 0)
                time.sleep(0.1)
            
            time.sleep(0.05)  # Small delay between readings
            
    except KeyboardInterrupt:
        send_motor_command(0, 0)
        print("\n\nStopped.")


if __name__ == "__main__":
    main()
