# -*- coding: utf-8 -*-
import time
import math

import board            # Blinka pin names for Raspberry Pi
import busio
from digitalio import DigitalInOut, Direction
from adafruit_vl53l0x import VL53L0X

# -------- Addresses (dual sensors) --------
LOX1_ADDRESS = 0x30   # Right
LOX2_ADDRESS = 0x31   # Left

# -------- XSHUT pins (choose your actual GPIOs) --------
# Example uses board.D6 and board.D7; change if your wiring differs.
SHT_LOX1_PIN = board.D7   # Right XSHUT
SHT_LOX2_PIN = board.D6   # Left  XSHUT

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

# ===== EMA low-pass state for (Right - Left) =====
diffFilt = float("nan")
ALPHA = 0.20  # lower = smoother

# ===== Slope-based thresholds (mm/s) — same behavior as before =====
SLOPE_THRESH_MMPS = 60.0
SLOPE_HARD_MMPS   = 1000.0

# ===== New: Position-based thresholds (mm) with hysteresis =====
# Turn engages at DIFF_ON, releases only after falling inside DIFF_OFF.
DIFF_ON_MM    = 40.0     # engage gentle turn when |diff| >= this
DIFF_OFF_MM   = 25.0     # release back to STRAIGHT when |diff| <= this
DIFF_HARD_MM  = 150.0    # hard turn when |diff| >= this

# ---- helpers ----
def present(m: Measurement) -> bool:
    # Treat only status 4 as true Out-Of-Range (same behavior as your C++)
    return m.RangeStatus != 4

def validStrict(m: Measurement) -> bool:
    # Mirror your "usable but warn" semantics: 0,1,2,7 are considered ok for telemetry
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

# Placeholders to hold sensor objects
lox1 = None  # Right
lox2 = None  # Left

def setID():
    global lox1, lox2

    # all reset (XSHUT LOW)
    xshut1.value = False
    xshut2.value = False
    time.sleep(0.01)

    # Bring up Right only
    xshut1.value = True
    time.sleep(0.01)

    # init LOX1 at default 0x29 then set to 0x30
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

    # init LOX2 at default 0x29 then set to 0x31
    try:
        lox2 = VL53L0X(i2c)  # still at 0x29 because left was held in reset
    except Exception as e:
        print("Failed to boot second VL53L0X (Left):", e)
        raise SystemExit(1)
    time.sleep(0.01)
    lox2.set_address(LOX2_ADDRESS)
    time.sleep(0.01)

    # Warm-up a couple readings (some boards need this)
    for _ in range(2):
        _ = safe_read_sensor(lox1)
        _ = safe_read_sensor(lox2)
        time.sleep(0.05)

def millis() -> int:
    return int(time.monotonic() * 1000.0)

def safe_read_sensor(sensor: VL53L0X) -> Measurement:
    """
    Read a single measurement and emulate RangeStatus:
      - 0: OK when a finite, positive range is returned
      - 4: OOR if an exception occurs or an implausible value is returned
    CircuitPython driver does not expose the full status codes; this mirrors
    your logic paths without changing printed behavior or decisions.
    """
    try:
        mm = int(sensor.range)
        if 20 <= mm <= 4000:   # typical usable envelope; adjust if needed
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
    global lastSampleMs, havePrevPair, prevPairR, prevPairL, diffFilt

    # sample timing
    now = millis()
    if (now - lastSampleMs) < SAMPLE_PERIOD_MS:
        return
    dt_ms = now - lastSampleMs if lastSampleMs != 0 else SAMPLE_PERIOD_MS
    lastSampleMs = now

    # read both sensors
    m1 = safe_read_sensor(lox1)  # Right
    m2 = safe_read_sensor(lox2)  # Left

    # ---- print raw readings (present = not real OOR) ----
    print("1: ", end="")
    if present(m1):
        print(f"{m1.RangeMilliMeter}", end="")
    else:
        print("Out of range", end="")

    print("  2: ", end="")
    if present(m2):
        print(f"{m2.RangeMilliMeter}", end="")
    else:
        print("Out of range", end="")

    # ---- compute control whenever both are present (highest priority: state) ----
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
            # EMA low-pass on diff before differentiating
            prevFilt = diffFilt
            diffRaw = float(currDiff)
            if math.isnan(diffFilt):
                diffFilt = diffRaw
            diffFilt = ALPHA * diffRaw + (1.0 - ALPHA) * diffFilt

            # derivative of filtered diff → mm/s
            sDiff = (diffFilt - prevFilt) * (1000.0 / float(dt_ms))

            # --- telemetry ---
            print(f"  |  diffFilt: {diffFilt:.1f}  sDiff: {sDiff:.1f} mm/s  |  ", end="")

            # 1) Slope-based instantaneous decision (fast onset)
            d_slope = sign_to_decision(sDiff, SLOPE_THRESH_MMPS, SLOPE_HARD_MMPS)

            # 2) Position-based decision with hysteresis
            absDiff = abs(diffFilt)
            d_pos_raw = sign_to_decision(diffFilt, DIFF_ON_MM, DIFF_HARD_MM)

            if absDiff >= DIFF_ON_MM:
                d_pos = d_pos_raw  # engage/keep
            elif absDiff <= DIFF_OFF_MM:
                d_pos = 0          # release to straight
            else:
                # in the hysteresis band: keep last nonzero direction if any
                d_pos = read_dual_sensors.latchedDecision

            # -------------------------------
            # 3) SAFER COMBINATION (changed):
            #    Prefer position; only use slope near center and only if it
            #    agrees with the current diff trend (no opposite flips).
            # -------------------------------
            slope_ok = d_slope if absDiff < DIFF_ON_MM else 0
            if slope_ok != 0 and (sDiff * diffFilt) < 0:
                # slope tries to reverse the current offset direction; ignore it
                slope_ok = 0

            decision = d_pos if d_pos != 0 else slope_ok
            # -------------------------------

            # Update latch (only store nonzero decisions)
            if decision != 0:
                read_dual_sensors.latchedDecision = decision
            else:
                # If fully straight, clear latch
                if absDiff <= DIFF_OFF_MM:
                    read_dual_sensors.latchedDecision = 0

            label = ("HARD LEFT" if decision == -2 else
                     "LEFT"       if decision == -1 else
                     "STRAIGHT"   if decision ==  0 else
                     "RIGHT"      if decision == +1 else
                     "HARD RIGHT")
            print(f"state: {label}  |  ", end="")

            # Debounced announcement uses the combined decision now
            updateDirectionDebounced(decision)

            # If not strictly valid, show why—but AFTER state so state is never hidden
            if not (validStrict(m1) and validStrict(m2)):
                print(f"[warn status R={m1.RangeStatus} L={m2.RangeStatus}]", end="")

        prevPairR = currR
        prevPairL = currL
    else:
        # At least one is truly OOR; we can't compute direction this cycle
        print("  |  state: UNKNOWN (one sensor OOR)", end="")
        print(f"  |  status R={m1.RangeStatus} L={m2.RangeStatus}", end="")

    print()  # newline

def main():
    print("Starting...")
    setID()

    global lastSampleMs
    lastSampleMs = millis()

    while True:
        read_dual_sensors()
        # No explicit sleep; sampling period enforced inside

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting.")
