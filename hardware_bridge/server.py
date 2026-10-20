from __future__ import annotations
import argparse
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
import secrets
import threading
import time
from urllib.parse import urlsplit, unquote
import webbrowser
from .common import API_VERSION, BridgeError, MAX_BODY, require
from .controller import HardwareController
from .lerobot_bus import ports

ROOT = Path(__file__).resolve().parent.parent

class BridgeServer(ThreadingHTTPServer):
    daemon_threads = True
    def __init__(self, port=8765, demo=False, token=None, root=ROOT, controller=None):
        super().__init__(("127.0.0.1", port), BridgeHandler)
        self.controller = controller or HardwareController(demo=demo)
        self.token = token or secrets.token_urlsafe(32)
        self.root = Path(root).resolve()
        self.stopping = threading.Event()
        self.origins = {"https://robobuddy.dev", f"http://127.0.0.1:{self.server_port}", f"http://localhost:{self.server_port}"}
        self.allowed_hosts = {f"127.0.0.1:{self.server_port}", f"localhost:{self.server_port}"}
    def start_watchdog(self):
        def run():
            while not self.stopping.wait(self.controller.PERIOD):
                self.controller.tick()
        self.watchdog = threading.Thread(target=run, daemon=True, name="so101-motor-owner")
        self.watchdog.start()
    def shutdown_safely(self):
        self.stopping.set()
        with self.controller.lock:
            try:
                self.controller._disconnect()
            except Exception as exc:
                self.controller._fault("SHUTDOWN_STOP_UNCONFIRMED", str(exc))
        self.shutdown()

class BridgeHandler(BaseHTTPRequestHandler):
    server_version = "RoboBuddyHardware/1"
    def setup(self):
        super().setup()
        self.connection.settimeout(5)
    def log_message(self, fmt, *args):
        # Never log tokens, URLs, request bodies or calibration files.
        pass
    def _origin(self):
        return self.headers.get("Origin")
    def _security(self, auth=True, write=False):
        require(self.headers.get("Host") in self.server.allowed_hosts, "HOST_REJECTED", "Use the loopback helper address.", 403)
        origin = self._origin()
        require(origin is None or origin in self.server.origins, "ORIGIN_REJECTED", "This website is not allowed to control the helper.", 403)
        if write:
            require(origin in self.server.origins, "ORIGIN_REQUIRED", "Hardware mutations require an approved origin.", 403)
        if auth:
            supplied = self.headers.get("Authorization", "")
            require(secrets.compare_digest(supplied, "Bearer " + self.server.token), "AUTH_REQUIRED", "Pair this panel with the local helper.", 401)
    def _send(self, status, body=None, mime="application/json; charset=utf-8"):
        data = body if isinstance(body, bytes) else json.dumps(body or {}, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Cross-Origin-Resource-Policy", "same-site")
        if self._origin() in self.server.origins:
            self.send_header("Access-Control-Allow-Origin", self._origin())
            self.send_header("Vary", "Origin")
        self.end_headers()
        self.wfile.write(data)
    def _error(self, exc):
        if isinstance(exc, BridgeError):
            self._send(exc.status, exc.payload())
        else:
            self._send(500, {"error": {"code": "HELPER_ERROR", "message": "Local helper error; physical movement must be checked locally."}})
    def do_OPTIONS(self):
        try:
            self._security(auth=False)
            require(self._origin() in self.server.origins, "ORIGIN_REJECTED", "Origin is not approved.", 403)
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", self._origin())
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Authorization, Content-Type")
            self.send_header("Vary", "Origin")
            self.end_headers()
        except Exception as exc:
            self._error(exc)
    def do_GET(self):
        try:
            path = urlsplit(self.path).path
            if path == "/api/health":
                self._security(auth=False)
                return self._send(200, {"apiVersion": API_VERSION, "mode": "demo" if self.server.controller.demo else "hardware"})
            if path == "/api/bootstrap":
                self._security(auth=False)
                # Only the helper's same-origin page may obtain a pairing token.
                # Public origins must receive it from the user, never via CORS.
                local_origins = {f"http://127.0.0.1:{self.server.server_port}", f"http://localhost:{self.server.server_port}"}
                site = self.headers.get("Sec-Fetch-Site")
                require(self._origin() in (None, *local_origins) and site in (None, "same-origin", "none"), "BOOTSTRAP_REJECTED", "Open the helper's local hardware panel.", 403)
                return self._send(200, {"token": self.server.token})
            if path.startswith("/api/"):
                self._security()
                if path == "/api/state":
                    return self._send(200, self.server.controller.snapshot())
                if path == "/api/ports":
                    data = [{"device": "DEMO", "description": "Demo motor bus — no physical connection"}] if self.server.controller.demo else ports()
                    return self._send(200, {"ports": data})
                raise BridgeError("NOT_FOUND", "API operation not found.", 404)
            self._security(auth=False)
            relative = unquote(path).lstrip("/") or "hardware.html"
            target = (self.server.root / relative).resolve()
            require(target.is_relative_to(self.server.root) and not any(p.startswith(".") for p in Path(relative).parts), "NOT_FOUND", "File not found.", 404)
            allowed = relative in ("hardware.html", "hardware.css", "gesture-control.css", "design-tokens.css", "theme-system.css") or relative.startswith(("src/", "assets/"))
            require(allowed and target.suffix in (".html", ".css", ".js", ".mjs", ".json", ".svg", ".woff2", ".png") and target.is_file(), "NOT_FOUND", "File not found.", 404)
            mime = mimetypes.guess_type(target)[0] or "application/octet-stream"
            return self._send(200, target.read_bytes(), mime)
        except Exception as exc:
            self._error(exc)
    def do_POST(self):
        try:
            self._security(write=True)
            require(self.headers.get("Content-Type", "").split(";")[0].strip() == "application/json", "CONTENT_TYPE", "JSON is required.", 415)
            length = self.headers.get("Content-Length", "")
            require(length.isdigit() and 0 < int(length) <= MAX_BODY, "BODY_LIMIT", "Request body is missing or exceeds 32 KiB.", 413)
            try:
                payload = json.loads(self.rfile.read(int(length)), parse_constant=lambda _: (_ for _ in ()).throw(ValueError("Nonfinite JSON")))
            except (ValueError, UnicodeError) as exc:
                raise BridgeError("INVALID_JSON", "Invalid JSON request.", 400) from exc
            require(isinstance(payload, dict), "INVALID_ARGUMENT", "A JSON object is required.", 400)
            path = urlsplit(self.path).path
            ctl = self.server.controller
            if path == "/api/session":
                return self._send(200, ctl.claim())
            if path == "/api/heartbeat":
                return self._send(200, ctl.heartbeat(payload.get("session")))
            if path == "/api/command":
                result = ctl.mutate(payload.get("session"), payload.get("sequence"), payload.get("issuedAtMs"),
                                    payload.get("operation"), payload.get("args", {}))
                return self._send(200, result)
            if path == "/api/shutdown":
                with ctl.lock:
                    ctl._owner(payload.get("session"))
                    ctl._disconnect()
                self._send(200, {"shutdown": True, "torque": "not_monitored"})
                threading.Thread(target=self.server.shutdown_safely, daemon=True).start()
                return
            raise BridgeError("NOT_FOUND", "API operation not found.", 404)
        except Exception as exc:
            self._error(exc)

def main():
    parser = argparse.ArgumentParser(description="Local SO-101 manual-control bridge. Starting never opens USB.")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--demo", action="store_true", help="Explicit fake bus; cannot control physical hardware")
    parser.add_argument("--open", action="store_true", help="Open the local hardware panel")
    args = parser.parse_args()
    require(1024 <= args.port <= 65535, "INVALID_ARGUMENT", "Choose an unprivileged port.", 400)
    server = BridgeServer(args.port, demo=args.demo)
    server.start_watchdog()
    print(f"RoboBuddy {'DEMO — NO HARDWARE' if args.demo else 'hardware'} helper: http://127.0.0.1:{server.server_port}/hardware.html", flush=True)
    print("No serial port is opened until Connect. Close with Quit Helper in the panel.", flush=True)
    if args.open:
        webbrowser.open(f"http://127.0.0.1:{server.server_port}/hardware.html")
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        with server.controller.lock:
            try:
                server.controller._disconnect()
            except Exception:
                pass
    finally:
        server.stopping.set()
        server.server_close()

if __name__ == "__main__":
    main()

