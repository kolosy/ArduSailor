#!/usr/bin/env python3
"""ArduSailor simulator: builds the firmware natively, runs it against a simulated boat and serves the UI.

    python3 sim/run.py              then open http://localhost:8765 (it opens by itself)

Needs only Python 3 and a C++ compiler (on a Mac: xcode-select --install). The firmware's EEPROM (PID tunings,
compass calibration) is kept in sim/.state/eeprom.bin between runs; --reset-eeprom starts it blank.
"""

import argparse
import collections
import json
import os
import queue
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import build

SIM = os.path.dirname(os.path.abspath(__file__))
WEB = os.path.join(SIM, "web")
STATE = os.path.join(SIM, ".state")
EEPROM = os.path.join(STATE, "eeprom.bin")

CONTENT_TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
                 ".css": "text/css; charset=utf-8", ".svg": "image/svg+xml"}


class Simulator:
    """The native simulator process: commands in on stdin, JSON lines out to every connected browser."""

    def __init__(self, binary, speed):
        self.binary = binary
        self.speed = speed
        self.proc = None
        self.lock = threading.Lock()
        self.clients = set()
        self.info = None
        self.last_frame = None
        # a frame every half simulated second, so a browser that connects later still gets the track and chart
        self.recent = collections.deque(maxlen=1200)
        # the latest change to each setting and to the waypoints since the info message, for the same reason
        self.changes = {}
        self.settings = {}    # "set"/"tune" commands, passed on the command line when restarting
        self.waypoints = {}   # moved waypoints, sent again after a restart

    def start(self):
        args = [self.binary, "--eeprom", EEPROM, "--speed", str(self.speed)]
        for (kind, key), value in sorted(self.settings.items()):
            args += ["--set" if kind == "set" else "--tune", f"{key}={value}"]
        with self.lock:
            self.info = None
            self.last_frame = None
            self.recent.clear()
            self.changes.clear()
            self.proc = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
            proc = self.proc
        threading.Thread(target=self._read, args=(proc,), daemon=True).start()
        for line in self.waypoints.values():
            self._write(line)

    def stop(self):
        with self.lock:
            proc, self.proc = self.proc, None
        if proc and proc.poll() is None:
            try:
                proc.stdin.write(b"quit\n")
                proc.stdin.flush()
                proc.wait(timeout=2)
            except Exception:
                proc.kill()

    def restart(self):
        self.stop()
        self.start()
        self._broadcast(json.dumps({"type": "event", "text": "firmware restarted"}))

    def erase_eeprom(self):
        self.stop()
        if os.path.exists(EEPROM):
            os.remove(EEPROM)
        self.start()
        self._broadcast(json.dumps({"type": "event", "text": "EEPROM erased, firmware restarted"}))

    def command(self, line):
        parts = line.split()
        if not parts:
            return
        if parts[0] in ("set", "tune") and len(parts) == 3:
            self.settings[(parts[0], parts[1])] = parts[2]
        elif parts[0] == "wp" and len(parts) == 4:
            self.waypoints[parts[1]] = line
        elif parts[0] == "speed" and len(parts) == 2:
            self.speed = parts[1]
        elif parts[0] == "defaults":
            # forget every change, then restart with the firmware's and simulator's defaults
            self.settings.clear()
            self.waypoints.clear()
            self.restart()
            return
        self._write(line)

    def _write(self, line):
        with self.lock:
            proc = self.proc
        if proc and proc.poll() is None:
            try:
                proc.stdin.write((line.strip() + "\n").encode())
                proc.stdin.flush()
            except (BrokenPipeError, OSError):
                pass

    def _read(self, proc):
        for raw in proc.stdout:
            line = raw.decode("utf-8", "replace").strip()
            if not line:
                continue
            kind = line[9:line.find('"', 9)] if line.startswith('{"type":"') else ""
            with self.lock:
                # keep what a browser that connects later needs, unless this simulator is being replaced
                if self.proc is proc and kind == "info":
                    self.info = line
                elif self.proc is proc and kind in ("param", "tunable", "waypoints"):
                    m = json.loads(line)
                    self.changes[(kind, m.get("key", m.get("name")))] = line
                elif self.proc is proc and kind == "frame":
                    self.last_frame = line
                    t = json.loads(line)["t"]
                    if not self.recent or t >= self.recent[-1][0] + 0.5:
                        self.recent.append((t, line))
            self._broadcast(line)
        code = proc.wait()
        if self.proc is proc:
            self._broadcast(json.dumps({"type": "event", "text": f"simulator exited ({code})"}))

    def subscribe(self):
        q = queue.Queue(maxsize=2000)
        with self.lock:
            self.clients.add(q)
            first = [self.info] + list(self.changes.values()) + [line for _, line in self.recent] + [self.last_frame]
            first = [line for line in first if line]
        for l in first:
            q.put(l)
        return q

    def unsubscribe(self, q):
        with self.lock:
            self.clients.discard(q)

    def _broadcast(self, line):
        with self.lock:
            clients = list(self.clients)
        for q in clients:
            try:
                q.put_nowait(line)
            except queue.Full:
                pass  # a stalled browser misses messages rather than holding up the simulator


def handler_for(sim):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):
            pass

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/api/events":
                return self._events()
            if path == "/":
                path = "/index.html"
            file = os.path.normpath(os.path.join(WEB, path.lstrip("/")))
            if not file.startswith(WEB + os.sep) or not os.path.isfile(file):
                return self._reply(404, b"not found", "text/plain")
            with open(file, "rb") as fh:
                body = fh.read()
            self._reply(200, body, CONTENT_TYPES.get(os.path.splitext(file)[1], "application/octet-stream"))

        def do_POST(self):
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length).decode("utf-8", "replace")
            if self.path == "/api/command":
                for line in body.splitlines():
                    sim.command(line)
            elif self.path == "/api/restart":
                sim.restart()
            elif self.path == "/api/erase":
                sim.erase_eeprom()
            else:
                return self._reply(404, b"not found", "text/plain")
            self._reply(204, b"", "text/plain")

        def _reply(self, code, body, content_type):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _events(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            q = sim.subscribe()
            try:
                while True:
                    try:
                        line = q.get(timeout=10)
                        chunk = f"data: {line}\n\n"
                    except queue.Empty:
                        chunk = ": keep-alive\n\n"
                    self.wfile.write(chunk.encode())
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                sim.unsubscribe(q)
                self.close_connection = True

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--speed", type=float, default=1, help="simulated seconds per second to start with (0 = flat out)")
    ap.add_argument("--no-browser", action="store_true", help="don't open a browser window")
    ap.add_argument("--reset-eeprom", action="store_true", help="start with a blank EEPROM (no stored PID or compass calibration)")
    ap.add_argument("--rebuild", action="store_true", help="rebuild the firmware even if nothing changed")
    args = ap.parse_args()

    binary = build.build(force=args.rebuild)
    os.makedirs(STATE, exist_ok=True)
    if args.reset_eeprom and os.path.exists(EEPROM):
        os.remove(EEPROM)

    sim = Simulator(binary, args.speed)
    sim.start()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler_for(sim))
    server.daemon_threads = True
    url = f"http://localhost:{args.port}/"
    print(f"ArduSailor simulator running at {url}  (Ctrl-C to stop)", flush=True)
    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        sim.stop()
        server.server_close()


if __name__ == "__main__":
    sys.exit(main())
