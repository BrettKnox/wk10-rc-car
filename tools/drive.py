"""Drive WK-10 from Python over the car's own Wi-Fi (stdlib only).

Join the car's "WK10-xxxx" network, close the phone page, flip the radio's CH3 switch to
autonomy, then:
    python tools/drive.py figure8
    python tools/drive.py state

Frames go to UDP port 4210 at 50 Hz, fire and forget: "<token> <seq> <steer> <throttle> <d>",
with this script's random token and a rising sequence number. The car answers every frame
with its state. It takes frames from one client at a time (the token holding the lock, freed
after 1 s of silence), so the phone page and a script cannot fight.

The ESP32 only drives while the radio has handed it the mux, so run() arms (d=0), waits for
"mux": true, then drives. It stops loudly if the radio takes control back, the car disarms,
another client holds the lock, or the car stops answering for QUIET_S. However run() ends it
sends stop frames; the car's own failsafe (brake, then hand back to the radio) is the backstop.
"""
import json
import math
import random
import socket
import sys
import time
import urllib.request

CAR = "192.168.4.1"
PORT = 4210
HZ = 50
QUIET_S = 0.5        # no state reply for this long = link lost
ENGAGE_S = 5.0       # wait this long for the radio to hand over the mux
MIN_BRAKE = 0.2      # negative throttle smaller than this is sent as 0: the ESC could read it as
                     # neutral or as brake/reverse (the car snaps it to neutral too)


class Link:
    def __init__(self, host=None, port=None):
        self.addr = (host or CAR, port or PORT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.tok = random.randint(1, 0xFFFFFFFF)
        self.seq = 0
        self.state = None
        self.heard = None              # monotonic time of the newest reply

    def send(self, steer, throttle, deadman=True):
        """Send one frame, then take the newest state the car has sent back (may be None)."""
        self.seq += 1
        s = max(-1000, min(1000, round(steer * 1000)))
        if -MIN_BRAKE < throttle < 0:
            throttle = 0
        t = max(-1000, min(1000, round(throttle * 1000)))
        self.sock.sendto(f"{self.tok} {self.seq} {s} {t} {1 if deadman else 0}".encode(), self.addr)
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
            except (BlockingIOError, ConnectionResetError):   # nothing waiting; Windows reports ICMP as a reset
                break
            try:
                self.state = json.loads(data)
                self.heard = time.monotonic()
            except ValueError:
                pass
        return self.state

    def mine(self):
        return bool(self.state) and self.state.get("lock") == f"{self.tok:08x}"


def run(program, seconds, link=None):
    """program(t) -> (steer -1..1, throttle -1..1; below 0 brakes, or reverses after a stop).
    Stops the car however it ends."""
    link = link or Link()
    try:
        t0 = time.monotonic()
        while True:                    # arm: the car must see the dead-man released, and the radio hand over
            st = link.send(0, 0, deadman=False)
            if link.mine() and st.get("armed") and st.get("mux"):
                break
            if time.monotonic() - t0 > ENGAGE_S:
                raise RuntimeError(f"car not ready: flip CH3 to autonomy and close the phone page. State: {st}")
            time.sleep(1 / HZ)
        t0 = due = time.monotonic()
        while (t := time.monotonic() - t0) < seconds:
            st = link.send(*program(t))
            if time.monotonic() - link.heard > QUIET_S:
                raise RuntimeError("car stopped answering")
            if not link.mine() or st["src"] != "web" or not st["armed"] or not st["mux"]:
                # the radio took the mux back, the failsafe fired, or another client has the lock;
                # the car will not drive again until it sees d=0, so stop loudly
                raise RuntimeError(f"car left web control: {st}")
            print(f"\r{t:5.1f}s  esc {st['esc_us']}us  steer {st['steer_us']}us  p99 {st['p99_ms']} ms", end="")
            due += 1 / HZ
            time.sleep(max(0.0, due - time.monotonic()))
    finally:
        for _ in range(5):
            try:
                link.send(0, 0, deadman=False)
            except OSError:            # keep trying; the car's failsafe is the backstop
                pass
            time.sleep(1 / HZ)
        print()


def figure8(t):
    return math.sin(2 * math.pi * t / 8), 0.25      # full lock each way every 8 s, quarter power


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "state"
    if cmd == "figure8":
        run(figure8, 16)
    elif cmd == "state":
        with urllib.request.urlopen(f"http://{CAR}/state", timeout=1) as r:
            print(json.load(r))
    else:
        sys.exit("usage: drive.py figure8|state")
