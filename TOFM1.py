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

# =======================
# Tuning (25% speed cap)
# =======================
BASE_PWM   = 64          # ~25% of 255
SLOW_SCALE = 0.60        # front slow-down multiplier when approaching

# Centering controller (PD on (R-L))
Kp_ERR         = 0.45
Kd_DERR        = 0.25
ERR_NORM_MM    = 120.0   # mm -> full-scale for proportional term
DERR_NORM_MMPS = 400.0   # mm/s -> full-scale for derivative term
MAX_REDUCTION  = 0.85    # max inside-wheel reduction fraction (0..1)

# Tight-arc 90° turn (timed) – keep at 25% overall
TURN_OUTER_PWM   = 64
TURN_INNER_SCALE = 0.35
TURN_MS_90       = 650

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

# -------- Measurement struct analog --------
class Measurement:
    __slots__ = ("RangeMilliMeter", "RangeStatus")
    def __init__(self, mm=0, status=4):
        self.RangeMilliMeter = mm
        self.RangeStatus = status  # 0=ok; 4=OOR (emulated)

def present(m: Measurement) -> bool:
    return m.RangeStatus != 4

def millis() -> int:
    return int(time.monotonic() * 1000.0)

def _apply_speed_scale_pwm(pwm: float, scale: float) -> int:
    return max(0, min(255, int(round(pwm * scale))))

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
        if 20 <= mm <= 2000:
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

# ===== Sampling / Filters / State =====
SAMPLE_PERIOD_MS = 90  # ~11 Hz
ALPHA = 0.20           # EMA

lastSampleMs = 0

# (R-L) EMA and derivative
diffFilt = float("nan")
prevDiffFilt = float("nan")

# Front average EMA and derivative
frontFilt = float("nan")
prevFrontFilt = float("nan")

# -------- Corner/Collision thresholds --------
FRONT_FAST_MMPS    = -900.0  # very fast approach
FRONT_CREEP_MM     = 500.0   # begin creep/probe when closer than this
FRONT_HARD_STOP_MM = 260.0   # never drive forward closer than this
FRONT_CREEP_PWM    = 28      # very low crawl while probing
SIDE_BIG_MM        = 1200.0  # a side suddenly “far/open”
CREEP_TIMEOUT_MS   = 900     # after arming, commit a turn if no open side

# ---- main sensing/decision loop (ONLY emits SET_VEL with PWM) ----
def read_dual_sensors():
    global lastSampleMs, diffFilt, prevDiffFilt, frontFilt, prevFrontFilt

    now = millis()
    if (now - lastSampleMs) < SAMPLE_PERIOD_MS:
        return
    dt_ms = now - lastSampleMs if lastSampleMs != 0 else SAMPLE_PERIOD_MS
    lastSampleMs = now
    dt = float(dt_ms) / 1000.0

    # read sensors
    mR = safe_read_sensor(lox1)  # Right
    mL = safe_read_sensor(lox2)  # Left
    mF1 = safe_read_sensor(lox3) # Front1
    mF2 = safe_read_sensor(lox4) # Front2

    # -------- Compute (R-L) EMA and derivative --------
    err_present = False
    derr = 0.0
    if present(mR) and present(mL):
        err_present = True
        currDiff = float(mR.RangeMilliMeter - mL.RangeMilliMeter)  # mm
        if math.isnan(diffFilt):
            diffFilt = currDiff
            prevDiffFilt = diffFilt
            derr = 0.0
        else:
            prevDiffFilt = diffFilt
            diffFilt = ALPHA * currDiff + (1.0 - ALPHA) * diffFilt
            derr = (diffFilt - prevDiffFilt) / dt  # mm/s

    # -------- Front average EMA and derivative --------
    front_present = False
    sFront = 0.0
    if present(mF1) or present(mF2):
        front_present = True
        if present(mF1) and present(mF2):
            front_raw = 0.5 * (mF1.RangeMilliMeter + mF2.RangeMilliMeter)
        else:
            front_raw = float(mF1.RangeMilliMeter if present(mF1) else mF2.RangeMilliMeter)

        if math.isnan(frontFilt):
            frontFilt = front_raw
            prevFrontFilt = frontFilt
            sFront = 0.0
        else:
            prevFrontFilt = frontFilt
            frontFilt = ALPHA * front_raw + (1.0 - ALPHA) * frontFilt
            sFront = (frontFilt - prevFrontFilt) / dt  # mm/s

    # ------------------------------
    # Turn state (timed arc at 25%)
    # ------------------------------
    if read_dual_sensors.turn_active:
        if now >= read_dual_sensors.turn_end_ms:
            # clear both turn and corner states
            read_dual_sensors.turn_active = False
            read_dual_sensors.turn_end_ms = 0
            read_dual_sensors.turn_dir = 0
            read_dual_sensors.corner_armed = False
            read_dual_sensors.corner_armed_ms = 0
            read_dual_sensors.last_turn_hint = 0
        else:
            outer = TURN_OUTER_PWM
            inner = max(0, min(255, int(round(outer * TURN_INNER_SCALE))))
            if read_dual_sensors.turn_dir > 0:  # RIGHT
                send_set_vel_pwm(outer, inner)
            else:                                # LEFT
                send_set_vel_pwm(inner, outer)
            return

    # ----------------------------------------------------
    # Corner arming: creep & probe, stop, then commit turn
    # ----------------------------------------------------
    # Arm corner behavior if approaching fast and within creep zone
    if front_present and (sFront <= FRONT_FAST_MMPS) and (frontFilt <= FRONT_CREEP_MM):
        if not read_dual_sensors.corner_armed:
            read_dual_sensors.corner_armed = True
            read_dual_sensors.corner_armed_ms = now
        # keep a turn hint: which side looks more open? (R-L >= 0 -> more space on right)
        if err_present:
            read_dual_sensors.last_turn_hint = (+1 if diffFilt >= 0 else -1)

    # If armed, either creep, stop, or commit to turn
    if read_dual_sensors.corner_armed:
        # If dangerously close, STOP and commit immediately
        if front_present and frontFilt <= FRONT_HARD_STOP_MM:
            right_open = (not present(mR)) or (present(mR) and mR.RangeMilliMeter >= SIDE_BIG_MM)
            left_open  = (not present(mL)) or (present(mL) and mL.RangeMilliMeter >= SIDE_BIG_MM)
            if right_open and not left_open:
                dir_sel = +1
            elif left_open and not right_open:
                dir_sel = -1
            elif right_open and left_open:
                dir_sel = (+1 if (err_present and diffFilt >= 0) else -1)
            else:
                dir_sel = (read_dual_sensors.last_turn_hint or (+1 if (err_present and diffFilt >= 0) else -1))

            send_set_vel_pwm(0, 0)  # hard stop
            read_dual_sensors.turn_active = True
            read_dual_sensors.turn_end_ms = now + TURN_MS_90
            read_dual_sensors.turn_dir = dir_sel
            return

        # If a side opens before hard-stop, commit to that turn
        right_open = (present(mR) and mR.RangeMilliMeter >= SIDE_BIG_MM) or (not present(mR))
        left_open  = (present(mL) and mL.RangeMilliMeter >= SIDE_BIG_MM) or (not present(mL))
        if right_open ^ left_open:  # exactly one side open
            dir_sel = (+1 if right_open else -1)
            read_dual_sensors.turn_active = True
            read_dual_sensors.turn_end_ms = now + TURN_MS_90
            read_dual_sensors.turn_dir = dir_sel
            outer = TURN_OUTER_PWM
            inner = max(0, min(255, int(round(outer * TURN_INNER_SCALE))))
            if dir_sel > 0:
                send_set_vel_pwm(outer, inner)
            else:
                send_set_vel_pwm(inner, outer)
            return

        # If we’ve been creeping for a while with no open side, stop & commit using hint
        if now - read_dual_sensors.corner_armed_ms >= CREEP_TIMEOUT_MS:
            dir_sel = (read_dual_sensors.last_turn_hint or (+1 if (err_present and diffFilt >= 0) else -1))
            send_set_vel_pwm(0, 0)  # brief stop before arc
            read_dual_sensors.turn_active = True
            read_dual_sensors.turn_end_ms = now + TURN_MS_90
            read_dual_sensors.turn_dir = dir_sel
            return

        # Otherwise: creep forward very slowly and keep centering
        creep_pwm = FRONT_CREEP_PWM
        if err_present:
            # PD centering at creep speed
            e_term = (diffFilt / ERR_NORM_MM) if ERR_NORM_MM > 0 else 0.0
            d_term = (derr     / DERR_NORM_MMPS) if DERR_NORM_MMPS > 0 else 0.0
            u = (Kp_ERR * e_term) + (Kd_DERR * d_term)
            red = max(0.0, min(MAX_REDUCTION, abs(u)))
            if u >= 0:
                left, right = creep_pwm, int(round(creep_pwm * (1.0 - red)))
            else:
                left, right = int(round(creep_pwm * (1.0 - red))), creep_pwm
            send_set_vel_pwm(left, right)
        else:
            send_set_vel_pwm(creep_pwm, creep_pwm)
        return

    # =========================================================
    # Normal: Centering PD (no reversing) at 25% speed
    # =========================================================
    speed_scale = 1.0
    if front_present and sFront < 0:
        speed_scale = SLOW_SCALE
    base = _apply_speed_scale_pwm(BASE_PWM, speed_scale)

    # Safety: never drive forward inside hard-stop
    if front_present and frontFilt <= FRONT_HARD_STOP_MM:
        send_set_vel_pwm(0, 0)
        # arm for next loop so we will commit a turn promptly
        if not read_dual_sensors.corner_armed:
            read_dual_sensors.corner_armed = True
            read_dual_sensors.corner_armed_ms = now
            if err_present:
                read_dual_sensors.last_turn_hint = (+1 if diffFilt >= 0 else -1)
        return

    if err_present:
        e_term = (diffFilt / ERR_NORM_MM) if ERR_NORM_MM > 0 else 0.0
        d_term = (derr     / DERR_NORM_MMPS) if DERR_NORM_MMPS > 0 else 0.0
        u = (Kp_ERR * e_term) + (Kd_DERR * d_term)
        red = max(0.0, min(MAX_REDUCTION, abs(u)))
        if u >= 0:
            left  = base
            right = int(round(base * (1.0 - red)))
        else:
            left  = int(round(base * (1.0 - red)))
            right = base
        send_set_vel_pwm(left, right)
    else:
        send_set_vel_pwm(base, base)

def main():
    print("Starting (25% speed, PD-centering, creep+stop+commit corners)…")
    setID()

    global lastSampleMs
    lastSampleMs = millis()

    while True:
        read_dual_sensors()

# ---- initialize per-callable state AFTER the function is defined ----
read_dual_sensors.turn_active = False
read_dual_sensors.turn_end_ms = 0
read_dual_sensors.turn_dir = 0      # -1 = LEFT, +1 = RIGHT

read_dual_sensors.corner_armed = False
read_dual_sensors.corner_armed_ms = 0
read_dual_sensors.last_turn_hint = 0  # -1 L, +1 R

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting.")
