"""Drive WK-10 from Python over the car's own Wi-Fi (stdlib only).

Join the car's "WK10-xxxx" network, then:
    python tools/drive.py figure8
    python tools/drive.py state

The car stops on its own if commands stop for failsafeMs (250 ms default), so a crash,
Ctrl+C or a dropped link all end at neutral. Commands are sent at 20 Hz with a rising
sequence number n, so a request delivered late cannot override a newer one. The car only
accepts throttle after it has seen the dead-man released, so run() sends a few d=0 first.
"""
import json
import math
import sys
import time
import urllib.request

CAR = "http://192.168.4.1"
HZ = 20
_seq = int(time.time() * 1000) % 2_000_000_000


def send(steer, throttle, deadman=True):
    global _seq
    _seq += 1
    s = max(-1000, min(1000, round(steer * 1000)))
    t = max(-1000, min(1000, round(throttle * 1000)))
    url = f"{CAR}/drive?s={s}&t={t}&d={1 if deadman else 0}&n={_seq}"
    with urllib.request.urlopen(url, timeout=0.2) as r:       # under the car's 250 ms failsafe
        return json.load(r)


def run(program, seconds):
    """program(t) -> (steer -1..1, throttle -1..1). Stops the car however it ends."""
    for _ in range(3):                              # arm: the car must see the dead-man released
        st = send(0, 0, deadman=False)
        time.sleep(1 / HZ)
    if st.get("cal") or not st["armed"]:
        send(0, 0, deadman=False)
        raise RuntimeError(f"car not ready to drive (ESC calibration mode, or not armed): {st}")
    t0 = time.monotonic()
    try:
        while (t := time.monotonic() - t0) < seconds:
            st = send(*program(t))
            print(f"\r{t:5.1f}s  {st}", end="")
            if st["src"] != "web" or not st["armed"] or st.get("cal"):
                # a slow request let the failsafe fire (or the TX took over); the car disarmed
                # and will not drive again until it sees d=0, so stop loudly instead of sending
                # d=1 into a dead car for the rest of the program. ESC calibration mode removes
                # every cap, so a script never drives during it.
                raise RuntimeError(f"car left web control: {st}")
            time.sleep(1 / HZ)
    finally:
        for _ in range(3):
            try:
                send(0, 0, deadman=False)
            except Exception:                       # keep trying; the car's failsafe is the backstop
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
