"""Checks drive.py against a fake car on localhost UDP (stdlib only, no hardware):
    python tools/test_drive.py
1. run() arms (d=0) and waits for the mux before it sends any throttle, at about 50 Hz,
   with one token and rising sequence numbers,
2. it always ends with d=0 stop frames,
3. if the radio takes the mux back, or the car disarms, mid-run, run() raises instead of carrying on,
4. it never drives when the car is silent, the mux never comes, or another client holds the lock,
5. a small negative throttle goes out as 0 (the ESC could read it either way); a real brake goes out.
Does NOT test: real Wi-Fi latency or loss, the car firmware, or the failsafe timing itself.
"""
import json
import socket
import statistics
import threading
import time

import drive

drive.ENGAGE_S = 0.4                 # keep the refusal cases quick
log = []                             # (token, seq, throttle, d, arrival time)
car = {}


def reset(**kw):
    log.clear()
    car.clear()
    car.update(dict(mux_after=3, drop_after=None, disarm_after=None, silent=False, other_lock=False), **kw)


def serve(sock):
    while True:
        try:
            data, addr = sock.recvfrom(256)
        except OSError:
            return
        tok, n, s, t, d = (int(x) for x in data.decode().split())
        log.append((tok, n, t, d, time.monotonic()))
        if car["silent"]:
            continue
        k = len(log)
        lost = car["drop_after"] is not None and k > car["drop_after"]
        disarmed = car["disarm_after"] is not None and k > car["disarm_after"]
        st = {"src": "none" if disarmed else "web", "armed": not disarmed, "mux": k > car["mux_after"] and not lost,
              "lock": "deadbeef" if car["other_lock"] else f"{tok:08x}", "esc_us": 1500, "steer_us": 1500, "p99_ms": 20}
        sock.sendto(json.dumps(st).encode(), addr)


srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
srv.bind(("127.0.0.1", 0))
threading.Thread(target=serve, args=(srv,), daemon=True).start()
drive.CAR, drive.PORT = "127.0.0.1", srv.getsockname()[1]


def refused(**kw):
    reset(**kw)
    try:
        drive.run(lambda t: (0.0, 0.5), 5)
        raise AssertionError(f"should have raised: {kw}")
    except RuntimeError:
        pass
    time.sleep(0.05)
    assert log[-1][2:4] == (0, 0), f"must end with a stop: {kw}"
    return [d for _, _, _, d, _ in log]


reset()
drive.run(lambda t: (0.0, 0.5), 0.5)
time.sleep(0.05)
first = next(i for i, e in enumerate(log) if e[3] == 1)
assert first >= 3 and all(e[3] == 0 for e in log[:first]), "must arm with d=0 until the mux is handed over"
assert any(e[3] == 1 and e[2] > 0 for e in log), "never drove"
assert log[-1][2:4] == (0, 0), "must end with a stop"
assert all(b[1] > a[1] for a, b in zip(log, log[1:])), "sequence numbers must rise"
assert len({e[0] for e in log}) == 1, "one token per run"
gaps = [b[4] - a[4] for a, b in zip(log, log[1:]) if a[3] == 1 and b[3] == 1]
assert statistics.median(gaps) < 0.03, f"must stream at about 50 Hz, median gap {statistics.median(gaps):.3f} s"

reset()
drive.run(lambda t: (0.0, -0.05 if t < 0.1 else -0.5), 0.2)
time.sleep(0.05)
assert not any(-200 < e[2] < 0 for e in log), "a small negative throttle must be sent as 0"
assert any(e[2] == -500 for e in log), "a real brake must still be sent"

# two frames of slack: a frame is sent before the reply that reports the change is read
assert 1 not in refused(drop_after=10)[13:], "kept driving after the radio took the mux back"
assert 1 not in refused(disarm_after=10)[13:], "kept driving after the car disarmed"
assert 1 not in refused(silent=True), "drove a car that never answered"
assert 1 not in refused(mux_after=10**9), "drove without the mux"
assert 1 not in refused(other_lock=True), "drove while another client held the lock"
srv.close()
print("test_drive: pass")
