# -*- coding: utf-8 -*-
import time
import math

import board            # Blinka pin names for Raspberry Pi
import busio
from digitalio import DigitalInOut, Direction
from adafruit_vl53l0x import VL53L0X

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

def send_set_vel_pwm(left_pwm: int, right_pwm: int):
    """Clamp to [0,255] and send SET_VEL with integer PWM values."""
    L = max(0, min(255, int(round(left_pwm))))
    R = max(0, min(255, int(round(right_pwm))))
    _send_line(f"SET_VEL L={L} R={R}")

# ===== Speed & steering tuning (PWM mode) =====
BASE_PWM   = 64         # <- normal operating speed = quarter of 255 ≈ 64
SLOW_SCALE = 0.50       # front slow-down multiplier when approaching
K_STEER    = 0.70       # max inside-wheel reduction fraction for dynamic turns
SLOPE_NORM = 600.0      # |sDiff| (mm/s) that maps to full K_STEER

# ---- Tight-arc 90° turn (timed) – ONLY sending speeds ----
TURN_OUTER_PWM   = 200   # outer wheel PWM during turn
TURN_INNER_SCALE = 0.35  # inner = outer * scale (tightness)
TURN_MS_90       = 650   # duration (ms) ~90° (calibrate!)

# -------- Addresses (Right, Left, Front1, Front2) --------
LOX1_ADDRESS = 0x30   # Right
LOX2_ADDRESS = 0x31   # Left
LOX3_ADDRESS = 0x32   # Front1
LOX4_ADDRESS = 0x33   # Front2

# -------- XSHUT pins (choose your actual GPIOs) --------
SHT_LOX1_PIN = board.D5  # Right XSHUT
SHT_LOX2_PIN = board.D17   # Left  XSHUT
SHT_LOX3_PIN = board.D6  # Front1 XSHUT
SHT_LOX4_PIN = board.D27   # Front2 XSHUT

# -------- Measurement struct analog --------
class Measurement:
    __slots__ = ("RangeMilliMeter", "RangeStatus")
    def __init__(self, mm=0, status=4):
        self.RangeMilliMeter = mm
        self.RangeStatus = status  # 0=ok; 4=OOR (emulated)

# -------- Trend state (differential only) --------
prevPairR = 0
prevPairL = 0
havePrevPair = False

lastSampleMs = 0
SAMPLE_PERIOD_MS = 90  # ~11.1 Hz

# ===== EMA low-pass =====
diffFilt = float("nan")   # (Right - Left)
frontFilt = float("nan")  # averaged Front (Front1/Front2)
ALPHA = 0.20

# ===== Slope thresholds (mm/s) =====
SLOPE_THRESH_MMPS = 60.0
SLOPE_HARD_MMPS   = 1000.0

# ===== Position thresholds for (R-L) hysteresis (mm) =====
DIFF_ON_MM    = 40.0
DIFF_OFF_MM   = 25.0
DIFF_HARD_MM  = 150.0

def present(m: Measurement) -> bool:
    return m.RangeStatus != 4

def millis() -> int:
    return int(time.monotonic() * 1000.0)

def _apply_speed_scale_pwm(pwm: float, scale: float) -> int:
    """Scale a PWM value and clamp to 0..255."""
    return max(0, min(255, int(round(pwm * scale))))

def _steer_reduction_from_slope(slope_mmps: float) -> float:
    """Return fraction (0..K_STEER) to reduce inside motor based on |sDiff|."""
    mag = abs(slope_mmps)
    x = max(0.0, min(1.0, mag / SLOPE_NORM))
    return K_STEER * x

def sign_to_decision(v: float, soft_thresh: float, hard_thresh: float) -> int:
    if v <= -hard_thresh: return -2
    if v >=  hard_thresh: return +2
    if v <= -soft_thresh: return -1
    if v >=  soft_thresh: return +1
    return 0

# ---- hardware bring-up (VL53L0X) ----
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)

# XSHUT controls
xshut1 = DigitalInOut(SHT_LOX1_PIN); xshut1.direction = Direction.OUTPUT
xshut2 = DigitalInOut(SHT_LOX2_PIN); xshut2.direction = Direction.OUTPUT
xshut3 = DigitalInOut(SHT_LOX3_PIN); xshut3.direction = Direction.OUTPUT
xshut4 = DigitalInOut(SHT_LOX4_PIN); xshut4.direction = Direction.OUTPUT

# Sensor objects
lox1 = lox2 = lox3 = lox4 = None

def safe_read_sensor(sensor: VL53L0X) -> Measurement:
    try:
        mm = int(sensor.range)
        if 20 <= mm <= 4000:
            return Measurement(mm=mm, status=0)
        else:
            return Measurement(mm=mm, status=4)
    except Exception:
        return Measurement(mm=0, status=4)

def setID():
    global lox1, lox2, lox3, lox4
    # all reset (XSHUT LOW)
    for x in (xshut1, xshut2, xshut3, xshut4):
        x.value = False
    time.sleep(0.01)

    # Bring up Right -> 0x30
    xshut1.value = True; time.sleep(0.01)
    lox1 = VL53L0X(i2c); time.sleep(0.01); lox1.set_address(LOX1_ADDRESS); time.sleep(0.01)

    # Left -> 0x31
    xshut2.value = True; time.sleep(0.01)
    lox2 = VL53L0X(i2c); time.sleep(0.01); lox2.set_address(LOX2_ADDRESS); time.sleep(0.01)

    # Front1 -> 0x32
    xshut3.value = True; time.sleep(0.01)
    lox3 = VL53L0X(i2c); time.sleep(0.01); lox3.set_address(LOX3_ADDRESS); time.sleep(0.01)

    # Front2 -> 0x33
    xshut4.value = True; time.sleep(0.01)
    lox4 = VL53L0X(i2c); time.sleep(0.01); lox4.set_address(LOX4_ADDRESS); time.sleep(0.01)

    # Warm-up a couple readings
    for _ in range(2):
        _ = safe_read_sensor(lox1)
        _ = safe_read_sensor(lox2)
        _ = safe_read_sensor(lox3)
        _ = safe_read_sensor(lox4)
        time.sleep(0.05)

# ---- main sensing/decision loop (ONLY emits SET_VEL with PWM) ----
def read_dual_sensors():
    global lastSampleMs, havePrevPair, prevPairR, prevPairL, diffFilt, frontFilt

    # persistent turning state (timed arc)
    if not hasattr(read_dual_sensors, "turn_active"):
        read_dual_sensors.turn_active = False
        read_dual_sensors.turn_end_ms = 0
        read_dual_sensors.turn_dir = 0  # -1 = LEFT, +1 = RIGHT

    now = millis()
    if (now - lastSampleMs) < SAMPLE_PERIOD_MS:
        return
    dt_ms = now - lastSampleMs if lastSampleMs != 0 else SAMPLE_PERIOD_MS
    lastSampleMs = now

    # read sensors
    mR = safe_read_sensor(lox1)  # Right
    mL = safe_read_sensor(lox2)  # Left
    mF1 = safe_read_sensor(lox3) # Front1
    mF2 = safe_read_sensor(lox4) # Front2

    # -------- SIDE DECISION --------
    decision = 0  # -2..+2
    sDiff = 0.0

    if present(mR) and present(mL):
        currR = int(mR.RangeMilliMeter)
        currL = int(mL.RangeMilliMeter)
        currDiff = currR - currL  # mm

        if not hasattr(read_dual_sensors, "latchedDecision"):
            read_dual_sensors.latchedDecision = 0

        if not havePrevPair:
            havePrevPair = True
            prevPairR = currR; prevPairL = currL
            diffFilt = float(currDiff)
        else:
            prevFilt = diffFilt
            diffRaw  = float(currDiff)
            if math.isnan(diffFilt): diffFilt = diffRaw
            diffFilt = ALPHA * diffRaw + (1.0 - ALPHA) * diffFilt
            sDiff    = (diffFilt - prevFilt) * (1000.0 / float(dt_ms))

            absDiff = abs(diffFilt)
            d_slope   = sign_to_decision(sDiff, SLOPE_THRESH_MMPS, SLOPE_HARD_MMPS)
            d_pos_raw = sign_to_decision(diffFilt, DIFF_ON_MM, DIFF_HARD_MM)

            if absDiff >= DIFF_ON_MM:
                d_pos = d_pos_raw
            elif absDiff <= DIFF_OFF_MM:
                d_pos = 0
            else:
                d_pos = read_dual_sensors.latchedDecision

            slope_ok = d_slope if absDiff < DIFF_ON_MM else 0
            if slope_ok != 0 and (sDiff * diffFilt) < 0:
                slope_ok = 0

            decision = d_pos if d_pos != 0 else slope_ok

            if decision != 0:
                read_dual_sensors.latchedDecision = decision
            elif absDiff <= DIFF_OFF_MM:
                read_dual_sensors.latchedDecision = 0

        prevPairR = currR; prevPairL = currL

    # -------- FRONT (AVERAGED) SPEED + ESCALATION --------
    # Average front sensors if both present; else use the one available.
    front_present = False
    fvals = []
    if present(mF1): fvals.append(float(mF1.RangeMilliMeter))
    if present(mF2): fvals.append(float(mF2.RangeMilliMeter))
    if len(fvals) == 2:
        front_raw = 0.5 * (fvals[0] + fvals[1]); front_present = True
    elif len(fvals) == 1:
        front_raw = fvals[0]; front_present = True
    else:
        front_raw = float("nan")

    speed_scale = 1.0
    sFront = 0.0

    if front_present:
        prevFront = frontFilt
        if math.isnan(frontFilt):
            frontFilt = front_raw
            sFront = 0.0
        else:
            frontFilt = ALPHA * front_raw + (1.0 - ALPHA) * frontFilt
            sFront = (frontFilt - prevFront) * (1000.0 / float(dt_ms))

        # approaching -> slow (temporary)
        if sFront <= -SLOPE_THRESH_MMPS:
            speed_scale = SLOW_SCALE

        # escalation: rapidly decreasing & close -> force hard (-2/+2)
        if (sFront <= -SLOPE_HARD_MMPS) and (frontFilt < 430.0):
            if decision == -1:
                decision = -2
            elif decision == +1:
                decision = +2
            elif decision == 0 and not math.isnan(diffFilt):
                decision = +2 if diffFilt >= 0 else -2

    # =========================================================
    # ONLY SEND SPEEDS (PWM): straight / dynamic / timed turn
    # =========================================================

    # Handle timed tight-arc turns using only speeds
    if read_dual_sensors.turn_active:
        if now >= read_dual_sensors.turn_end_ms:
            read_dual_sensors.turn_active = False
        else:
            outer = _apply_speed_scale_pwm(TURN_OUTER_PWM, speed_scale)
            inner = max(0, min(255, int(round(outer * TURN_INNER_SCALE))))
            if read_dual_sensors.turn_dir > 0:  # RIGHT: left outer, right inner
                left = outer; right = inner
            else:                               # LEFT: left inner, right outer
                left = inner; right = outer
            send_set_vel_pwm(left, right)
            return

    # If hard decision arises and we're not already turning, start timed arc
    if decision in (-2, +2):
        read_dual_sensors.turn_active = True
        read_dual_sensors.turn_end_ms = now + TURN_MS_90
        read_dual_sensors.turn_dir = +1 if decision == +2 else -1

        outer = _apply_speed_scale_pwm(TURN_OUTER_PWM, speed_scale)
        inner = max(0, min(255, int(round(outer * TURN_INNER_SCALE))))
        if read_dual_sensors.turn_dir > 0:  # RIGHT
            left = outer; right = inner
        else:                                # LEFT
            left = inner; right = outer
        send_set_vel_pwm(left, right)
        return

    # Otherwise: straight / dynamic steering around BASE_PWM
    base = _apply_speed_scale_pwm(BASE_PWM, speed_scale)  # <- returns to 64 when clear

    if decision == 0:
        # STRAIGHT at base (¼ speed or slowed)
        send_set_vel_pwm(base, base)
    elif decision == -1:
        # DYNAMIC LEFT: right same, left decreases with |sDiff|
        red_frac = _steer_reduction_from_slope(sDiff)
        left  = int(round(base * (1.0 - red_frac)))
        right = base
        send_set_vel_pwm(left, right)
    elif decision == +1:
        # DYNAMIC RIGHT: left same, right decreases with |sDiff|
        red_frac = _steer_reduction_from_slope(sDiff)
        left  = base
        right = int(round(base * (1.0 - red_frac)))
        send_set_vel_pwm(left, right)
    else:
        # Fallback: gentle straight at half of base
        send_set_vel_pwm(max(0, base // 2), max(0, base // 2))

def main():
    print("Starting...")
    setID()

    global lastSampleMs
    lastSampleMs = millis()

    while True:
        read_dual_sensors()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting.")
