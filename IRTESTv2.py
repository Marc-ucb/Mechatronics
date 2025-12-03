# -*- coding: utf-8 -*-
import time
import board
from digitalio import DigitalInOut, Direction

ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 115200

# Try to open serial; fall back gracefully if not available
try:
    import serial
    _ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.02)
    _serial_ok = True
    print(f"[INFO] Serial OK on {ARDUINO_PORT} @ {ARDUINO_BAUD}")
except Exception as _e:
    print(f"[WARN] Serial not available ({_e}). Running without analog IR.")
    _ser = None
    _serial_ok = False

# ======================================================================
# IR setup (matches your main code pins)
# ======================================================================

# Digital IR pins (you said: DARK = track)
IR_LEFT_DIGITAL_PIN = board.D23
IR_RIGHT_DIGITAL_PIN = board.D24

ir_left_digital = DigitalInOut(IR_LEFT_DIGITAL_PIN)
ir_left_digital.direction = Direction.INPUT

ir_right_digital = DigitalInOut(IR_RIGHT_DIGITAL_PIN)
ir_right_digital.direction = Direction.INPUT

# Analog IR from Arduino via serial: "IR L=1.23 R=4.56"
_ir_analog_left = None
_ir_analog_right = None

ANALOG_DARK_THRESHOLD = 1.0   # for later logic if you want
ANALOG_LIGHT_THRESHOLD = 4.0  # not used here, just kept for completeness

# ======================================================================
# Helpers to talk to Arduino (for analog)
# ======================================================================

def _read_serial_line():
    """Read a line from Arduino serial if available."""
    if _serial_ok and _ser is not None:
        try:
            if _ser.in_waiting > 0:
                line = _ser.readline().decode("ascii", errors="ignore").strip()
                return line
        except Exception:
            pass
    return None


def _parse_ir_from_serial():
    """
    Parse IR sensor readings from Arduino serial.
    Expected format: 'IR L=1.23 R=4.56'
    Updates global _ir_analog_left/right
    """
    global _ir_analog_left, _ir_analog_right

    line = _read_serial_line()
    if not line:
        return

    if line.startswith("IR "):
        try:
            parts = line.split()
            for part in parts:
                if part.startswith("L="):
                    _ir_analog_left = float(part[2:])
                elif part.startswith("R="):
                    _ir_analog_right = float(part[2:])
        except Exception:
            # ignore bad parse
            return


def read_ir_analog():
    """
    Read analog voltage from both IR sensors (via Arduino serial).
    Returns: (left_voltage, right_voltage) in volts (0.0 to 5.0V) or (None, None)
    """
    _parse_ir_from_serial()
    return (_ir_analog_left, _ir_analog_right)


def read_ir_digital():
    """
    Read digital state from both IR sensors.

    With your current wiring / behavior:
    - True  = dark surface (ON TRACK)
    - False = light surface (OFF TRACK / bumper)
    """
    return (ir_left_digital.value, ir_right_digital.value)

# ======================================================================
# Diagnostic loop
# ======================================================================

def diagnostic_loop():
    print("[INFO] Starting IR diagnostic loop (FLIPPED LOGIC)...")
    print("      DIGITAL: True = dark/track, False = light/bumper")
    print("      Status: ON_TRACK / GOING_LEFT / GOING_RIGHT / BOTH_LIGHT")
    print("----------------------------------------------------------------")

    while True:
        # Digital side
        left_dig, right_dig = read_ir_digital()

        # True  = dark (track)
        # False = light (bumper / off)
        if left_dig and right_dig:
            status = "ON_TRACK"        # both on dark track
        elif left_dig and not right_dig:
            status = "GOING_LEFT"      # left on track, right on bumper -> robot too far right
        elif right_dig and not left_dig:
            status = "GOING_RIGHT"     # right on track, left on bumper -> robot too far left
        else:
            status = "BOTH_LIGHT"      # both see light -> probably off track

        # Analog side: just show volts (may be NaN at startup)
        left_v, right_v = read_ir_analog()
        lv = left_v if left_v is not None else float("nan")
        rv = right_v if right_v is not None else float("nan")

        print(
            f"DIGITAL  L={left_dig} R={right_dig}  -> {status:<12}  "
            f"ANALOG  L={lv:4.2f}V  R={rv:4.2f}V"
        )

        time.sleep(0.05)  # ~20 Hz updates; tweak if you want faster/slower


def main():
    try:
        diagnostic_loop()
    except KeyboardInterrupt:
        print("\n[INFO] Exiting IR diagnostic.")


if __name__ == "__main__":
    main()
