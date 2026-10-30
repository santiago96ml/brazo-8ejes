#!/usr/bin/env python3
"""Serial smoke test: verify the board answers without commanding any motion.

Opens the port (which resets the Mega), waits for the READY line, then sends
V, I and Q and prints whatever comes back. Safe to run with the arm powered.
"""

import argparse
import sys
import time

import serial


def read_line(ser):
    raw = ser.readline()
    if not raw:
        return None
    return raw.decode("utf-8", "replace").strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM4")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--boot-wait", type=float, default=3.5, help="seconds to wait for READY")
    parser.add_argument("--reply-wait", type=float, default=2.0, help="seconds to wait per command")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as exc:
        print("cannot open %s: %s" % (args.port, exc))
        return 2

    ok = False
    try:
        deadline = time.time() + args.boot_wait
        while time.time() < deadline:
            line = read_line(ser)
            if not line:
                continue
            print("<<", line)
            if '"READY"' in line:
                ok = True
                break

        if not ok:
            print("no READY line within %.1fs (board missing, wrong port, or old firmware)"
                  % args.boot_wait)
            return 3

        for command in ("V", "I", "Q"):
            ser.write((command + "\n").encode("ascii"))
            ser.flush()
            deadline = time.time() + args.reply_wait
            answered = False
            while time.time() < deadline:
                line = read_line(ser)
                if not line:
                    continue
                print(">>", command, "<<", line)
                answered = True
                break
            if not answered:
                print(">>", command, "<< (no reply)")
                return 4
    finally:
        ser.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
