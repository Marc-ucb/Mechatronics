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
except Exception as e:
    print(f"[warn] Serial not available ({e}). Will print commands instead.")
    _ser = None
    _serial_ok = False


def _send_line(line: str):
    """Send a line to Arduino serial."""
    msg = (line.rstrip() + "\n").encode("ascii", errors="ignore")
    if _serial_ok and _ser is not None:
        try:
            _ser.write(msg)
        except Exception as e:
            print(f"[serial err] {e}. Falling back to print.")
            print(line.rstrip())
    else:
        print(line.rstrip())


def send_motor_command(left_pwm, right_pwm):
    """Send motor command to Arduino (matches main code format)."""
    L = round(left_pwm)
    R = round(right_pwm)
    _send_line(f"SET_VEL L={L} R={R}")


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
    
    error_count = 0
    max_errors = 5
    
    while True:
        try:
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
            
            # Reset error count on successful iteration
            error_count = 0
            time.sleep(0.05)  # Small delay between readings
            
        except KeyboardInterrupt:
            send_motor_command(0, 0)
            print("\n\nStopped.")
            break
            
        except Exception as e:
            error_count += 1
            print(f"\n[ERROR] Exception caught: {e}")
            print(f"[ERROR] Error count: {error_count}/{max_errors}")
            
            # Stop motors on error
            try:
                send_motor_command(0, 0)
            except:
                pass
            
            if error_count >= max_errors:
                print(f"[FATAL] Too many errors ({max_errors}), shutting down.")
                send_motor_command(0, 0)
                break
            
            # Wait a bit before retrying
            print("[RECOVERY] Waiting 1 second before continuing...")
            time.sleep(1)
            print("[RECOVERY] Resuming...\n")


if __name__ == "__main__":
    main()
