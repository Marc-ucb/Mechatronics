# servo_test.py
import time
import sys

PORT = "/dev/ttyACM0"   # change if needed (e.g., COM3 on Windows)
BAUD = 115200
STEP_DEG = 30           # how far to raise (CCW)
PAUSE_SEC = 2.0         # wait between commands

def main():
    try:
        import serial
    except ImportError:
        print("Install pyserial:  pip install pyserial")
        sys.exit(1)

    ser = serial.Serial(PORT, BAUD, timeout=0.5)
    time.sleep(2.0)  # let Arduino reset

    def send(cmd: str):
        line = (cmd.strip() + "\n").encode("ascii", errors="ignore")
        ser.write(line)
        # read one line back (non-fatal if nothing)
        try:
            resp = ser.readline().decode("ascii", "ignore").strip()
            if resp:
                print("<<", resp)
        except Exception:
            pass

    print(">> SERVO POS")
    send("SERVO POS")

    print(f">> SERVO A={STEP_DEG} (raise CCW)")
    send(f"SERVO A={STEP_DEG}")
    time.sleep(PAUSE_SEC)

    print(">> SERVO REV (return CW to home)")
    send("SERVO REV")
    time.sleep(PAUSE_SEC)

    print("Done.")
    ser.close()

if __name__ == "__main__":
    main()
