"""Drive WK-10 from Python over the car's own Wi-Fi (stdlib only).

Join the car's "WK10-xxxx" network, then:
    python C:/Users/bman0/Code/rc-car/tools/drive.py figure8
    python C:/Users/bman0/Code/rc-car/tools/drive.py state

The car stops on its own if commands stop for failsafeMs (250 ms default), so a crash,
Ctrl+C or a dropped link all end at neutral. Commands are sent at 20 Hz.
"""
import json
import math
import sys
import time
import urllib.request

CAR = "http://192.168.4.1"
HZ = 20


def send(steer, throttle, deadman=True):
    s = max(-1000, min(1000, round(steer * 1000)))
    t = max(-1000, min(1000, round(throttle * 1000)))
    url = f"{CAR}/drive?s={s}&t={t}&d={1 if deadman else 0}"
    with urllib.request.urlopen(url, timeout=0.5) as r:
        return json.load(r)


def run(program, seconds):
    """program(t) -> (steer -1..1, throttle -1..1). Stops the car however it ends."""
    t0 = time.monotonic()
    try:
        while (t := time.monotonic() - t0) < seconds:
            st = send(*program(t))
            print(f"\r{t:5.1f}s  {st}", end="")
            time.sleep(1 / HZ)
    finally:
        for _ in range(3):
            try:
                send(0, 0, deadman=False)
            except OSError:
                pass
        print()


def figure8(t):
    return math.sin(2 * math.pi * t / 8), 0.25      # full lock each way every 8 s, quarter power


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "state"
    if cmd == "figure8":
        run(figure8, 16)
    elif cmd == "state":
        with urllib.request.urlopen(f"{CAR}/state", timeout=1) as r:
            print(json.load(r))
    else:
        sys.exit("usage: drive.py figure8|state")
