#!/usr/bin/env python3
"""Minimal USB-CANFD host helper (CDC ACM line protocol)."""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.stderr.write("pip install pyserial\n")
    raise


def main() -> None:
    p = argparse.ArgumentParser(description="G474 USB-CANFD serial helper")
    p.add_argument("port", help="COMx or /dev/ttyACMx")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--send", action="append", default=[], help="TX line, e.g. 't 123 fb 8 11 22 33 44 55 66 77 88'")
    p.add_argument("--listen", type=float, default=0.0, help="seconds to print RX after send (0 = forever)")
    args = p.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    time.sleep(0.3)
    ser.reset_input_buffer()
    for line in args.send:
        payload = line.strip() + "\n"
        ser.write(payload.encode("ascii"))
        print("TX", payload.strip(), flush=True)

    deadline = None if args.listen <= 0 else time.time() + args.listen
    try:
        while deadline is None or time.time() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            print(raw.decode("ascii", errors="replace").rstrip(), flush=True)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
