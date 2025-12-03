# -*- coding: utf-8 -*-
import time
import sys
import signal
import board
from digitalio import DigitalInOut, Direction

# Open log file
log_file = open('/tmp/robot_debug.log', 'w', buffering=1)

def log(msg):
    """Write to both stdout and log file"""
    print(msg)
    log_file.write(msg + '\n')
    sys.stdout.flush()
    log_file.flush()

# Flag to track if we're shutting down
shutting_down = False

def signal_handler(sig, frame):
    global shutting_down
    log(f"\n[SIGNAL] Received signal {sig}")
    shutting_down = True
    
signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)

# ==========================================================================================================
# Serial Setup for Arduino Motor Control ===================================================================
# ==========================================================================================================

ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 115200

try:
    import serial
    _ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.02)
    _serial_ok = True
    log(f"[INFO] Serial OK on {ARDUINO_PORT}")
except Exception as e:
    log(f"[warn] Serial not available ({e}). Will print commands instead.")
    _ser = None
    _serial_ok = False


def _send_line(line: str):
    """Send a line to Arduino serial."""
    msg = (line.rstrip() + "\n").encode("ascii", errors="ignore")
    if _serial_ok and _ser is not None:
        try:
            _ser.write(msg)
        except Exception as e:
            log(f"[serial err] {e}. Falling back to print.")
            log(line.rstrip())
    else:
        log(line.rstrip())


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

log("[OK] IR sensors initialized")


def read_ir_sensors():
    """
    Read IR sensors.
    True  = dark surface (ON TRACK)
    False = light surface (OFF TRACK / bumper)
    """
    return (ir_left.value, ir_right.value)


# ==========================================================================================================
# Main Control Loop with Watchdog ==========================================================================
# ==========================================================================================================

def main():
    global shutting_down
    
    log("\n=== IR-Only Robot Control with Watchdog ===")
    log("Log file: /tmp/robot_debug.log")
    log("Watchdog: Will reset if no motor command for 4 seconds")
    log("Starting in 3 seconds...\n")
    time.sleep(3)
    
    iteration = 0
    reset_count = 0
    start_time = time.time()
    
    log("[INFO] Entering main loop")
    
    while not shutting_down:
        try:
            # Watchdog: Track last motor command time
            last_motor_command_time = time.time()
            loop_start_time = time.time()
            
            log(f"\n[LOOP START] Reset #{reset_count}, Iteration {iteration}")
            
            while not shutting_down:
                try:
                    iteration += 1
                    current_time = time.time()
                    time_since_motor_cmd = current_time - last_motor_command_time
                    runtime = current_time - start_time
                    
                    # WATCHDOG CHECK: Reset if no motor command in 4 seconds
                    if time_since_motor_cmd > 4.0:
                        log(f"\n[WATCHDOG] No motor command for {time_since_motor_cmd:.1f}s - RESETTING LOOP")
                        log(f"[WATCHDOG] Last iteration: {iteration}, Runtime: {runtime:.1f}s")
                        send_motor_command(0, 0)  # Stop motors
                        reset_count += 1
                        time.sleep(0.5)
                        break  # Break inner loop to reset
                    
                    # Heartbeat every 50 iterations
                    if iteration % 50 == 0:
                        log(f"[HEARTBEAT] Iter {iteration}, Runtime: {runtime:.1f}s, Resets: {reset_count}")
                    
                    # Read sensors
                    left_on_track, right_on_track = read_ir_sensors()
                    
                    # Determine action and send command
                    if left_on_track and right_on_track:
                        # Both on track - go straight
                        send_motor_command(200, 200)
                        last_motor_command_time = time.time()
                        if iteration % 20 == 0:  # Print every 20th to reduce spam
                            log(f"[{iteration}] STRAIGHT (L={left_on_track} R={right_on_track})")
                    
                    elif left_on_track and not right_on_track:
                        # Right off track - nudge left
                        log(f"[{iteration}] RIGHT OFF -> NUDGE LEFT")
                        send_motor_command(-80, 80)
                        last_motor_command_time = time.time()
                    
                    elif right_on_track and not left_on_track:
                        # Left off track - nudge right
                        log(f"[{iteration}] LEFT OFF -> NUDGE RIGHT")
                        send_motor_command(80, -80)
                        last_motor_command_time = time.time()
                    
                    else:
                        # Both off track - backup
                        log(f"[{iteration}] BOTH OFF -> BACKUP")
                        send_motor_command(-150, -150)
                        last_motor_command_time = time.time()
                        time.sleep(0.5)
                        send_motor_command(0, 0)
                        last_motor_command_time = time.time()
                        time.sleep(0.1)
                    
                    time.sleep(0.05)  # Small delay between readings
                    
                except KeyboardInterrupt:
                    log("\n[INTERRUPT] Keyboard interrupt")
                    shutting_down = True
                    break
                    
                except Exception as e:
                    import traceback
                    log(f"\n[ERROR] Inner loop exception: {type(e).__name__}: {e}")
                    for line in traceback.format_exc().split('\n'):
                        log(line)
                    
                    # Stop motors and break to reset
                    try:
                        send_motor_command(0, 0)
                    except:
                        pass
                    
                    log("[ERROR] Breaking to reset loop")
                    reset_count += 1
                    time.sleep(1)
                    break
            
            # If we broke out of inner loop (not shutting down), continue outer loop to reset
            if not shutting_down:
                log(f"[RESET] Loop reset complete, restarting...")
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            log("\n[INTERRUPT] Keyboard interrupt in outer loop")
            shutting_down = True
            
        except Exception as e:
            import traceback
            log(f"\n[FATAL] Outer loop exception: {type(e).__name__}: {e}")
            for line in traceback.format_exc().split('\n'):
                log(line)
            
            try:
                send_motor_command(0, 0)
            except:
                pass
            
            reset_count += 1
            log(f"[FATAL] Attempting recovery, reset count: {reset_count}")
            
            if reset_count > 10:
                log("[FATAL] Too many resets, giving up")
                shutting_down = True
            else:
                time.sleep(2)
    
    # Clean shutdown
    log("\n[SHUTDOWN] Stopping motors...")
    send_motor_command(0, 0)
    log("[SHUTDOWN] Complete. Exited gracefully.")
    log_file.close()


if __name__ == "__main__":
    main()
