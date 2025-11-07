# -*- coding: utf-8 -*-
import time
import math

import board            # Blinka pin names for Raspberry Pi
import busio
from digitalio import DigitalInOut, Direction
from adafruit_vl53l0x import VL53L0X

# -------- Addresses (dual sensors + front) --------
LOX1_ADDRESS = 0x30   # Right
LOX2_ADDRESS = 0x31   # Left
LOX3_ADDRESS = 0x32   # Front

# -------- XSHUT pins (choose your actual GPIOs) --------
# Example uses D7 (GPIO7), D6 (GPIO6), D5 (GPIO5); change if your wiring differs.
SHT_LOX1_PIN = board.D7   # Right XSHUT
SHT_LOX2_PIN = board.D6   # Left  XSHUT
SHT_LOX3_PIN = board.D5   # Front XSHUT

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
diffFilt = float("nan")   # for (Right - Left)
frontFilt = float("nan")  # for Front distance
ALPHA = 0.20  # lower = smoother

# ===== Slope thresholds (mm/s) =====
SLOPE_THRESH_MMPS = 60.0
SLOPE_HARD_MMPS   = 1000.0

# ===== Position thresholds for (R-L) hysteresis (mm) =====
DIFF_ON_MM    = 40.0
DIFF_OFF_MM   = 25.0
DIFF_HARD_MM  = 150.0

# ---- helpers ----
def present(m: Measurement) -> bool:
    # Treat only status 4 as true Out-Of-Range
    return m.RangeStatus != 4

def validStrict(m: Measurement) -> bool:
    # Usable statuses (telemetry only)
    return m.RangeStatus in (0, 1, 2, 7)

def updateDirectionDebounced(decision: int):
    """
    Debounce & announce a provided decision:
      -2 L hard, -1 L, 0 straight, +1 R, +2 R hard
    """
    if not hasattr(updateDirectionDebounced, "sameCount"):
        updateDirectionDebounced.sameCount = 0
        updateDirectionDebounced.lastDecision = 0

    if decision == updateDirectionDebounced.lastDecision:
        updateDirectionDebounced.sameCount += 1
    else:
        updateDirectionDebounced.lastDecision = decision
        updateDirectionDebounced.sameCount = 1

    if updateDirectionDebounced.sameCount >= 3:
        if decision == -2:
            print("robot is turning hard left")
        elif decision == -1:
            print("robot is dynamically adjusting left")
        elif decision == 0:
            print("robot stays straight")
        elif decision == +1:
            print("robot is dynamically adjusting right")
        else:
            print("robot is turning hard right")
        updateDirectionDebounced.sameCount = 0  # emit once per stable decision

# ---- hardware bring-up ----
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)

# XSHUT controls
xshut1 = DigitalInOut(SHT_LOX1_PIN); xshut1.direction = Direction.OUTPUT
xshut2 = DigitalInOut(SHT_LOX2_PIN); xshut2.direction = Direction.OUTPUT
xshut3 = DigitalInOut(SHT_LOX3_PIN); xshut3.direction = Direction.OUTPUT

# Sensor objects
lox1 = None  # Right
lox2 = None  # Left
lox3 = None  # Front

def setID():
    global lox1, lox2, lox3

    # all reset (XSHUT LOW)
    xshut1.value = False
    xshut2.value = False
    xshut3.value = False
    time.sleep(0.01)

    # Bring up Right only
    xshut1.value = True
    time.sleep(0.01)
    try:
        lox1 = VL53L0X(i2c)  # default address 0x29
    except Exception as e:
        print("Failed to boot first VL53L0X (Right):", e)
        raise SystemExit(1)
    time.sleep(0.01)
    lox1.set_address(LOX1_ADDRESS)
    time.sleep(0.01)

    # Bring up Left only
    xshut2.value = True
    time.sleep(0.01)
    try:
        lox2 = VL53L0X(i2c)  # at 0x29 (left was in reset)
    except Exception as e:
        print("Failed to boot second VL53L0X (Left):", e)
        raise SystemExit(1)
    time.sleep(0.01)
    lox2.set_address(LOX2_ADDRESS)
    time.sleep(0.01)

    # Bring up Front only
    xshut3.value = True
    time.sleep(0.01)
    try:
        lox3 = VL53L0X(i2c)  # at 0x29 (front was in reset)
    except Exception as e:
        print("Failed to boot third VL53L0X (Front):", e)
        raise SystemExit(1)
    time.sleep(0.01)
    lox3.set_address(LOX3_ADDRESS)
    time.sleep(0.01)

    # Warm-up a couple readings
    for _ in range(2):
        _ = safe_read_sensor(lox1)
        _ = safe_read_sensor(lox2)
        _ = safe_read_sensor(lox3)
        time.sleep(0.05)

def millis() -> int:
    return int(time.monotonic() * 1000.0)

def safe_read_sensor(sensor: VL53L0X) -> Measurement:
    """
    Emulate RangeStatus:
      - 0: OK (finite, plausible)
      - 4: OOR / failure
    """
    try:
        mm = int(sensor.range)
        if 20 <= mm <= 4000:
            return Measurement(mm=mm, status=0)
        else:
            return Measurement(mm=mm, status=4)
    except Exception:
        return Measurement(mm=0, status=4)

def sign_to_decision(v: float, soft_thresh: float, hard_thresh: float) -> int:
    """
    Map a signed value to {-2,-1,0,+1,+2} given soft/hard thresholds on |v|.
    """
    if v <= -hard_thresh:
        return -2
    if v >=  hard_thresh:
        return +2
    if v <= -soft_thresh:
        return -1
    if v >=  soft_thresh:
        return +1
    return 0

def read_dual_sensors():
    global lastSampleMs, havePrevPair, prevPairR, prevPairL, diffFilt, frontFilt

    # sample timing
    now = millis()
    if (now - lastSampleMs) < SAMPLE_PERIOD_MS:
        return
    dt_ms = now - lastSampleMs if lastSampleMs != 0 else SAMPLE_PERIOD_MS
    lastSampleMs = now

    # read sensors
    m1 = safe_read_sensor(lox1)  # Right
    m2 = safe_read_sensor(lox2)  # Left
    mF = safe_read_sensor(lox3)  # Front

    # ---- print raw readings ----
    print("1: ", end="")
    print(f"{m1.RangeMilliMeter}" if present(m1) else "Out of range", end="")
    print("  2: ", end="")
    print(f"{m2.RangeMilliMeter}" if present(m2) else "Out of range", end="")
    print("  F: ", end="")
    print(f"{mF.RangeMilliMeter}" if present(mF) else "Out of range", end="")

    # ---- side-based decision (existing functionality) ----
    decision = 0  # -2..+2
    absDiff = 0.0  # for later front escalation fallback

    if present(m1) and present(m2):
        currR = int(m1.RangeMilliMeter)
        currL = int(m2.RangeMilliMeter)
        currDiff = currR - currL  # raw diff (mm)

        # static latch for hysteresis across calls
        if not hasattr(read_dual_sensors, "latchedDecision"):
            read_dual_sensors.latchedDecision = 0

        if not havePrevPair:
            havePrevPair = True
            prevPairR = currR
            prevPairL = currL
            diffFilt = float(currDiff)  # init EMA
            print("  |  sDiff: N/A (priming)", end="")
        else:
            # (R-L) EMA
            prevFilt = diffFilt
            diffRaw = float(currDiff)
            if math.isnan(diffFilt):
                diffFilt = diffRaw
            diffFilt = ALPHA * diffRaw + (1.0 - ALPHA) * diffFilt

            # derivative of filtered diff → mm/s
            sDiff = (diffFilt - prevFilt) * (1000.0 / float(dt_ms))
            absDiff = abs(diffFilt)

            # telemetry (side)
            print(f"  |  diffFilt: {diffFilt:.1f}  sDiff: {sDiff:.1f} mm/s  |  ", end="")

            # slope-based quick onset
            d_slope = sign_to_decision(sDiff, SLOPE_THRESH_MMPS, SLOPE_HARD_MMPS)

            # position-based with hysteresis
            d_pos_raw = sign_to_decision(diffFilt, DIFF_ON_MM, DIFF_HARD_MM)
            if absDiff >= DIFF_ON_MM:
                d_pos = d_pos_raw
            elif absDiff <= DIFF_OFF_MM:
                d_pos = 0
            else:
                d_pos = read_dual_sensors.latchedDecision

            # safer combination (prefer position; slope only near center and not reversing)
            slope_ok = d_slope if absDiff < DIFF_ON_MM else 0
            if slope_ok != 0 and (sDiff * diffFilt) < 0:
                slope_ok = 0

            decision = d_pos if d_pos != 0 else slope_ok

            # update latch
            if decision != 0:
                read_dual_sensors.latchedDecision = decision
            else:
                if absDiff <= DIFF_OFF_MM:
                    read_dual_sensors.latchedDecision = 0

            label = ("HARD LEFT" if decision == -2 else
                     "LEFT"       if decision == -1 else
                     "STRAIGHT"   if decision ==  0 else
                     "RIGHT"      if decision == +1 else
                     "HARD RIGHT")
            print(f"state: {label}  |  ", end="")

            # Debounced announcement (existing)
            updateDirectionDebounced(decision)

            # Status warnings (after state)
            if not (validStrict(m1) and validStrict(m2)):
                print(f"[warn status R={m1.RangeStatus} L={m2.RangeStatus}]", end="")
        prevPairR = currR
        prevPairL = currL
    else:
        print("  |  state: UNKNOWN (one side sensor OOR)", end="")
        print(f"  |  status R={m1.RangeStatus} L={m2.RangeStatus}", end="")

    # ---- front-based behavior (added; does not remove side behavior) ----
    # Compute front EMA + slope if present
    if present(mF):
        prevFront = frontFilt
        frontRaw = float(mF.RangeMilliMeter)
        if math.isnan(frontFilt):
            frontFilt = frontRaw
            sFront = 0.0
        else:
            frontFilt = ALPHA * frontRaw + (1.0 - ALPHA) * frontFilt
            sFront = (frontFilt - prevFront) * (1000.0 / float(dt_ms))

        # approaching if negative slope
        approaching = (sFront <= -SLOPE_THRESH_MMPS)

        # speed advisory
        speed_label = "SLOW" if approaching else "NORMAL"
        print(f"  |  frontFilt: {frontFilt:.1f}  sFront: {sFront:.1f} mm/s  |  speed: {speed_label}", end="")

        # escalation rule: only hard turn if rapidly decreasing AND < 430 mm
        if (sFront <= -SLOPE_HARD_MMPS) and (frontFilt < 430.0):
            # If already turning, escalate that direction to HARD.
            # If straight, pick side with more clearance (turn toward the larger distance).
            if decision == -1:
                decision = -2
            elif decision == +1:
                decision = +2
            elif decision == 0:
                # Use diffFilt sign to pick direction (fallback if not available: keep straight)
                if not math.isnan(diffFilt):
                    decision = +2 if diffFilt >= 0 else -2  # turn toward farther side
            if decision != 0:
                print("  |  FRONT ESCALATION → HARD TURN", end="")
                # Also emit debounced message based on the escalated decision
                updateDirectionDebounced(decision)
    else:
        print("  |  front: OOR", end="")

    print()  # newline

def main():
    print("Starting...")
    setID()

    global lastSampleMs
    lastSampleMs = millis()

    while True:
        read_dual_sensors()
        # Sampling period enforced inside

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting.")
