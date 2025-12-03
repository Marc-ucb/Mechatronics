# -*- coding: utf-8 -*-
import time
import board
from digitalio import DigitalInOut, Direction

# ==========================================================================================================
# Serial Setup for Arduino Motor Control (with "good" error handling) ======================================
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
    """
    Send a single line to the Arduino over serial.
    Falls back to printing if serial fails or is unavailable.
    """
    msg = (line.rstrip() + "\n").encode("ascii", errors="ignore")
    if _serial_ok and _ser is not None:
        try:
            _ser.write(msg)
        except Exception as e:
            print(f"[serial err] {e}. Falling back to print.")
            print(line.rstrip())
    else:
        # Debug / fallback: just print the command
        print(line.rstrip())


def send_motor_command(left_pwm, right_pwm):
    """
    Send motor command to Arduino (matches SET_VEL protocol):
        SET_VEL L=<L> R=<R>
    """
    L = round(left_pwm)
    R = round(right_pwm)
    _send_line(f"SET_VEL L={L} R={R}")


# ==========================================================================================================
# IR Sensor Setup ==========================================================================================
# ==========================================================================================================

IR_LEFT_DIGITAL_PIN = board.D23
IR_RIGHT_DIGITAL_PIN = board.D24

try:
    ir_left = DigitalInOut(IR_LEFT_DIGITAL_PIN)
    ir_left.direction = Direction.INPUT

    ir_right = DigitalInOut(IR_RIGHT_DIGITAL_PIN)
    ir_right.direction = Direction.INPUT

    print("[OK] IR sensors initialized (D23=LEFT, D24=RIGHT)")
except Exception as e:
    # This would be unusual, but if it happens you'll see the reason
    print(f"[ERROR] Failed to initialize IR sensors: {e}")
    ir_left = None
    ir_right = None


def read_ir_sensors():
    """
    Read IR sensors.
    With your wiring:
      True  = dark surface (ON TRACK)
      False = light surface (OFF TRACK / bumper)

    Returns:
        (left_on_track, right_on_track)
        or (None, None) if sensors not initialized.
    """
    if ir_left is None or ir_right is None:
        return (None, None)

    try:
        return (ir_left.value, ir_right.value)
    except Exception as e:
        # Very rare, but if the GPIO library throws, don't kill the loop
        print(f"[IR ERROR] Failed to read IR sensors: {e}")
        return (None, None)


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

            # If we failed to read sensors, play safe: stop
            if left_on_track is None or right_on_track is None:
                print("[WARN] IR read failed; stopping motors for safety.")
                send_motor_command(0, 0)
                time.sleep(0.1)
                continue

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

            time.sleep(0.05)  # Small delay between readings (~20 Hz)

    except KeyboardInterrupt:
        send_motor_command(0, 0)
        print("\n\nStopped.")


if __name__ == "__main__":
    main()
