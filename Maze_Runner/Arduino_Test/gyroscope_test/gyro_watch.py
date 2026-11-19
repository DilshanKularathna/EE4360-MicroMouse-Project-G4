#!/usr/bin/env python3
"""
gyro_watch.py — EE4360 Micromouse Group 4

Serial monitor for gyroscope_calibration_test.cpp that also writes the
results into lib/Gyroscope/gyroscope_initial_data.h.

  * Shows everything the board prints (replaces the PlatformIO Serial Monitor,
    so close that first: only one program can open the COM port).
  * Anything you type + Enter is sent to the board (use it for G / S).
  * Lines of the form  @CAL,<KEY>,<VALUE>  are captured.
  * When the board prints  @CAL,DONE,1  the header file is updated and the
    script exits. Ctrl+C before that writes nothing (unless --save-partial).

Run with PlatformIO's Python (it already has pyserial):

    %USERPROFILE%\\.platformio\\penv\\Scripts\\python.exe gyro_watch.py
    ... gyro_watch.py --port COM3 --log run1.txt
    ... gyro_watch.py --dry-run          # show changes, don't write the file

Opening the port resets the Mega, so the test suite restarts from TEST 1.
"""

import argparse
import datetime
import re
import sys
import threading
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is not installed. Run this script with PlatformIO's Python:\n"
             r"  %USERPROFILE%\.platformio\penv\Scripts\python.exe gyro_watch.py")

DEFAULT_HEADER = (Path(__file__).resolve().parents[2]
                  / "lib" / "Gyroscope" / "gyroscope_initial_data.h")

# @CAL key -> int16 constant in SECTION 3
BASELINE_KEYS = {
    "BASE_GX": "GYRO_BASELINE_GYRO_X_RAW",
    "BASE_GY": "GYRO_BASELINE_GYRO_Y_RAW",
    "BASE_GZ": "GYRO_BASELINE_GYRO_Z_RAW",
    "BASE_AX": "GYRO_BASELINE_ACCEL_X_RAW",
    "BASE_AY": "GYRO_BASELINE_ACCEL_Y_RAW",
    "BASE_AZ": "GYRO_BASELINE_ACCEL_Z_RAW",
}

# @CAL key -> (STORED_GYRO_CALIBRATION field, decimals; None = integer)
STRUCT_KEYS = {
    "OFF_GX":  ("gyroOffsetX", 2),
    "OFF_GY":  ("gyroOffsetY", 2),
    "OFF_GZ":  ("gyroOffsetZ", 2),
    "OFF_AX":  ("accelOffsetX", 2),
    "OFF_AY":  ("accelOffsetY", 2),
    "OFF_AZ":  ("accelOffsetZ", 2),
    "SAMPLES": ("sampleCount", None),
    "SIGMA_Z": ("gyroNoiseSigmaZ", 3),
}

# @CAL key -> (float constant in SECTION 6, decimals)
FLOAT_KEYS = {
    "TEMP_C":      ("GYRO_CAL_TEMPERATURE_C", 2),
    "TURN90_MEAN": ("GYRO_TURN90_MEAN_DEG", 3),
    "TURN90_STD":  ("GYRO_TURN90_STDDEV_DEG", 3),
    "TURN180_MEAN": ("GYRO_TURN180_MEAN_DEG", 3),
    "TURN180_STD": ("GYRO_TURN180_STDDEV_DEG", 3),
    "SCALE":       ("GYRO_HEADING_SCALE_FACTOR", 4),   # computed below
}

NUM = r"-?\d+(?:\.\d+)?"


def find_port():
    ports = list(list_ports.comports())
    if not ports:
        sys.exit("No serial ports found. Is the board plugged in?")
    likely = [p for p in ports
              if p.vid in (0x2341, 0x2A03, 0x1A86, 0x0403, 0x10C4)  # Arduino, CH340, FTDI, CP210x
              or "arduino" in (p.description or "").lower()]
    choice = likely if likely else ports
    if len(choice) == 1:
        return choice[0].device
    names = ", ".join(f"{p.device} ({p.description})" for p in ports)
    sys.exit(f"Several serial ports found: {names}\nPick one with --port COMx")


def compute_scale(data):
    """sum(targets) / sum(measured) over all turn trials that were captured."""
    target_sum = measured_sum = 0.0
    for target in (90, 180):
        mean = data.get(f"TURN{target}_MEAN")
        n = data.get(f"TURN{target}_N")
        if mean is None or n is None:
            continue
        target_sum += target * n
        measured_sum += mean * n
    if measured_sum > 1e-3:
        data["SCALE"] = target_sum / measured_sum


def update_header(path, data, dry_run=False):
    with open(path, encoding="utf-8", newline="") as f:
        text = f.read()
    changes = []

    def replace(pattern, new_value, label):
        nonlocal text
        m = re.search(pattern, text)
        if not m:
            print(f"  [watch] WARNING: could not find {label} in {path.name}")
            return
        old = m.group(2)
        text = text[:m.start(2)] + new_value + text[m.end(2):]
        changes.append((label, old, new_value))

    for key, name in BASELINE_KEYS.items():
        if key in data:
            replace(rf"(static const int16_t {name}\s*=\s*)({NUM})",
                    str(int(round(data[key]))), name)

    if any(k in data for k in STRUCT_KEYS):
        for key, (field, dec) in STRUCT_KEYS.items():
            if key in data:
                val = str(int(data[key])) if dec is None else f"{data[key]:.{dec}f}f"
                replace(rf"(/\*\s*{field}\s*\*/\s*)({NUM}f?)", val, field)
        if all(k in data for k in STRUCT_KEYS):
            replace(r"(/\*\s*isValid\s*\*/\s*)(true|false)", "true", "isValid")

    for key, (name, dec) in FLOAT_KEYS.items():
        if key in data:
            replace(rf"(static const float {name}\s*=\s*)({NUM}f)",
                    f"{data[key]:.{dec}f}f", name)

    if not changes:
        print("  [watch] Nothing to write.")
        return

    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    text = re.sub(r"(// Status: ).*", rf"\g<1>auto-updated by gyro_watch.py on {stamp}", text)
    text = re.sub(r"[ \t]*// TODO: (?:measure|fill from calibration output)", "", text)

    print(f"\n  [watch] {'Would update' if dry_run else 'Updating'} {path}")
    for label, old, new in changes:
        mark = "" if old == new else "   (changed)"
        print(f"    {label:28s} {old:>12s} -> {new}{mark}")

    if dry_run:
        print("  [watch] --dry-run: file not written.")
        return
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(text)
    print("  [watch] Saved.")


def forward_keyboard(ser):
    """Send each line typed in the terminal to the board."""
    for line in sys.stdin:
        try:
            ser.write((line.strip() + "\n").encode())
        except serial.SerialException:
            return


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port, e.g. COM3 (auto-detected if omitted)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--header", type=Path, default=DEFAULT_HEADER,
                    help="path to gyroscope_initial_data.h")
    ap.add_argument("--log", type=Path, help="also save the full session to this file")
    ap.add_argument("--dry-run", action="store_true", help="show changes without writing")
    ap.add_argument("--save-partial", action="store_true",
                    help="on Ctrl+C, still write the values captured so far")
    args = ap.parse_args()

    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")   # Windows consoles vs. box-drawing chars

    if not args.header.is_file():
        sys.exit(f"Header not found: {args.header}")

    port = args.port or find_port()
    try:
        ser = serial.Serial(port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"Could not open {port}: {e}\n"
                 "Close any other Serial Monitor (PlatformIO / Arduino IDE) using it.")

    print(f"[watch] Connected to {port} @ {args.baud}. Type G or S + Enter to control the test.")
    print(f"[watch] Results will be written to {args.header}")
    print("[watch] Ctrl+C to quit.\n")

    log = open(args.log, "w", encoding="utf-8") if args.log else None
    threading.Thread(target=forward_keyboard, args=(ser,), daemon=True).start()

    data = {}
    buf = b""
    finished = False
    try:
        while not finished:
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", errors="replace").rstrip("\r")
                if log:
                    log.write(line + "\n")
                    log.flush()

                if line.startswith("@CAL,"):
                    parts = line.split(",")
                    if len(parts) != 3:
                        continue
                    key, value = parts[1], parts[2]
                    if key == "DONE":
                        finished = True
                        break
                    try:
                        data[key] = float(value)
                    except ValueError:
                        print(f"  [watch] ignored bad value: {line}")
                        continue
                    print(f"  [watch] captured {key} = {value}")
                else:
                    print(line)
    except KeyboardInterrupt:
        print("\n[watch] Stopped before the test finished.")
        if not args.save_partial:
            print("[watch] Nothing written (rerun, or use --save-partial).")
            return
    except serial.SerialException as e:
        print(f"\n[watch] Serial connection lost: {e}")
        if not args.save_partial:
            return
    finally:
        ser.close()
        if log:
            log.close()

    compute_scale(data)
    update_header(args.header, data, dry_run=args.dry_run)


if __name__ == "__main__":
    main()
