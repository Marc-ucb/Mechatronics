# -*- coding: utf-8 -*-
import time
import math

import board            # Blinka pin names for Raspberry Pi
import busio
from digitalio import DigitalInOut, Direction, Pull
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

# ---- helpers ----
def present(m: Measurement) -> bool:
    # Treat only status 4 as true Out-Of-Range (same behavior as your C++)
    return m.RangeStatus != 4

def validStrict(m: Measurement) -> bool:
    # Mirror your "usable but warn" semantics: 0,1,2,7 are considered ok for telemetry
    return m.RangeStatus in (0, 1, 2, 7)

def updateDirectionDebounced(sDiff: float):
    # static storage via function attributes
    if not hasattr(updateDirectionDebounced, "sameCount"):
        updateDirectionDebounced.sameCount = 0
        updateDirectionDebounced.lastDecision = 0  # -2 L hard, -1 L, 0 straight, +1 R, +2 R hard

    d = 0
    if sDiff < -1000:
        d = -2
    elif sDiff > 1000:
        d = +2
    elif sDiff <= -60:
        d = -1
    elif sDiff >= +60:
        d = +1
    else:
        d = 0

    if d == updateDirectionDebounced.lastDecision:
        updateDirectionDebounced.sameCount += 1
    else:
        updateDirectionDebounced.lastDecision = d
        updateDirectionDebounced.sameCount = 1

    if updateDirectionDebounced.sameCount >= 3:
        if d == -2:
            print("robot is turning hard left")
        elif d == -1:
            print("robot is dynamically adjusting left")
        elif d == 0:
            print("robot stays straight")
        elif d == +1:
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
        # .range returns millimeters (int). Typical OOR can be large or 0.
        mm = int(sensor.range)
        if 20 <= mm <= 4000:   # typical usable envelope; adjust if needed
            return Measurement(mm=mm, status=0)
        else:
            return Measurement(mm=mm, status=4)
    except Exception:
        return Measurement(mm=0, status=4)

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

            # --- instantaneous state (always printed) ---
            if   sDiff < -1000: d = -2
            elif sDiff >  1000: d = +2
            elif sDiff <=  -60: d = -1
            elif sDiff >=   60: d = +1
            else:               d = 0

            label = ("HARD LEFT" if d == -2 else
                     "LEFT"       if d == -1 else
                     "STRAIGHT"   if d ==  0 else
                     "RIGHT"      if d == +1 else
                     "HARD RIGHT")
            print(f"state: {label}  |  ", end="")

            # --- debounced announcement (prints only when stable) ---
            updateDirectionDebounced(sDiff)

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
    # Ensure I2C bus is ready before toggling XSHUTs
    # (bus created above)

    # Bring up and readdress sensors
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
