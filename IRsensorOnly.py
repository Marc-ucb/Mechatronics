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
    print("\n=== IR-Only Robot Control (DIAGNOSTIC VERSION) ===")
    print("Starting in 3 seconds...\n")
    time.sleep(3)
    
    error_count = 0
    max_errors = 5
    iteration = 0
    last_iteration_time = time.time()
    
    while True:
        try:
            iteration += 1
            current_time = time.time()
            time_since_last = current_time - last_iteration_time
            
            print(f"\n[ITER {iteration}] --- Starting iteration (time since last: {time_since_last:.3f}s) ---")
            
            if time_since_last > 2.0:
                print(f"[WARNING] Large gap between iterations! Possible hang/block detected.")
            
            step_start = time.time()
            
            # Step 1: Read sensors
            print(f"[ITER {iteration}] Reading IR sensors...")
            left_on_track, right_on_track = read_ir_sensors()
            step_time = time.time() - step_start
            print(f"[ITER {iteration}] Sensors read ({step_time:.3f}s): left={left_on_track}, right={right_on_track}")
            
            if step_time > 0.5:
                print(f"[WARNING] Sensor read took {step_time:.3f}s - unusually slow!")
            
            # Step 2: Determine action
            print(f"[ITER {iteration}] Determining action...")
            step_start = time.time()
            
            # Both sensors on track -> GO STRAIGHT
            if left_on_track and right_on_track:
                print(f"[ITER {iteration}] Decision: Both ON track -> STRAIGHT (200, 200)")
                print(f"[ITER {iteration}] Sending motor command...")
                send_motor_command(200, 200)
                step_time = time.time() - step_start
                print(f"[ITER {iteration}] Motor command sent successfully ({step_time:.3f}s)")
            
            # Left on track, Right off track -> TOO FAR RIGHT -> NUDGE LEFT
            elif left_on_track and not right_on_track:
                print(f"[ITER {iteration}] Decision: Right OFF track -> NUDGE LEFT (-80, 80)")
                print(f"[ITER {iteration}] Sending motor command...")
                send_motor_command(-80, 80)
                step_time = time.time() - step_start
                print(f"[ITER {iteration}] Motor command sent successfully ({step_time:.3f}s)")
            
            # Right on track, Left off track -> TOO FAR LEFT -> NUDGE RIGHT
            elif right_on_track and not left_on_track:
                print(f"[ITER {iteration}] Decision: Left OFF track -> NUDGE RIGHT (80, -80)")
                print(f"[ITER {iteration}] Sending motor command...")
                send_motor_command(80, -80)
                step_time = time.time() - step_start
                print(f"[ITER {iteration}] Motor command sent successfully ({step_time:.3f}s)")
            
            # Both sensors off track -> EMERGENCY BACKUP
            else:
                print(f"[ITER {iteration}] Decision: BOTH OFF track -> BACKUP (-150, -150)")
                print(f"[ITER {iteration}] Sending backup motor command...")
                send_motor_command(-150, -150)
                step_time = time.time() - step_start
                print(f"[ITER {iteration}] Backup command sent ({step_time:.3f}s), sleeping 0.5s...")
                time.sleep(0.5)
                print(f"[ITER {iteration}] Sending stop command...")
                send_motor_command(0, 0)
                print(f"[ITER {iteration}] Stop command sent, sleeping 0.1s...")
                time.sleep(0.1)
                print(f"[ITER {iteration}] Backup sequence complete")
            
            if step_time > 0.5:
                print(f"[WARNING] Motor command took {step_time:.3f}s - unusually slow!")
            
            # Reset error count on successful iteration
            error_count = 0
            print(f"[ITER {iteration}] Iteration complete, sleeping 0.05s...")
            time.sleep(0.05)
            print(f"[ITER {iteration}] --- End iteration ---")
            
            last_iteration_time = time.time()
            
        except KeyboardInterrupt:
            print("\n[INTERRUPT] Keyboard interrupt detected")
            send_motor_command(0, 0)
            print("\n\nStopped.")
            break
            
        except Exception as e:
            import traceback
            error_count += 1
            print(f"\n{'='*80}")
            print(f"[ERROR] Exception in iteration {iteration}")
            print(f"[ERROR] Exception type: {type(e).__name__}")
            print(f"[ERROR] Exception message: {e}")
            print(f"[ERROR] Error count: {error_count}/{max_errors}")
            print(f"\n[TRACEBACK] Full traceback:")
            traceback.print_exc()
            print(f"{'='*80}\n")
            
            # Stop motors on error
            print("[ERROR] Attempting to stop motors...")
            try:
                send_motor_command(0, 0)
                print("[ERROR] Motors stopped successfully")
            except Exception as stop_error:
                print(f"[ERROR] Failed to stop motors: {stop_error}")
            
            if error_count >= max_errors:
                print(f"\n[FATAL] Too many consecutive errors ({max_errors}), shutting down.")
                try:
                    send_motor_command(0, 0)
                except:
                    pass
                break
            
            # Wait a bit before retrying
            print(f"[RECOVERY] Waiting 1 second before continuing...")
            time.sleep(1)
            print(f"[RECOVERY] Resuming...\n")
            
            last_iteration_time = time.time()


if __name__ == "__main__":
    main()
