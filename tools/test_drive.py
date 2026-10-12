"""Checks drive.py against a fake car (stdlib only, no hardware):
    python C:/Users/bman0/Code/rc-car/tools/test_drive.py
1. run() arms (d=0) before it sends any throttle,
2. it always ends with d=0 stop commands,
3. if the car reports it left web control mid-run, run() raises instead of carrying on.
Does NOT test: real Wi-Fi latency, the car firmware, or the failsafe timing itself.
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs, urlparse

import drive

log = []
drop_after = None          # after this many requests the fake car reports failsafe


class Car(BaseHTTPRequestHandler):
    def do_GET(self):
        q = {k: v[0] for k, v in parse_qs(urlparse(self.path).query).items()}
        log.append((int(q["t"]), int(q["d"]), int(q["n"])))
        lost = drop_after is not None and len(log) > drop_after
        body = json.dumps({"src": "failsafe" if lost else "web", "armed": not lost}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass


srv = HTTPServer(("127.0.0.1", 0), Car)
threading.Thread(target=srv.serve_forever, daemon=True).start()
drive.CAR = f"http://127.0.0.1:{srv.server_port}"

drive.run(lambda t: (0.0, 0.5), 0.3)
assert all(d == 0 for _, d, _ in log[:3]), "must arm with d=0 first"
assert any(d == 1 and t > 0 for t, d, _ in log), "never drove"
assert log[-1][:2] == (0, 0), "must end with a stop"
assert all(b[2] > a[2] for a, b in zip(log, log[1:])), "sequence numbers must rise"

log.clear()
drop_after = 6
try:
    drive.run(lambda t: (0.0, 0.5), 5)
    raise AssertionError("should have raised when the car left web control")
except RuntimeError:
    pass
assert log[-1][:2] == (0, 0), "must stop after an abort"
srv.shutdown()
print("test_drive: pass")
