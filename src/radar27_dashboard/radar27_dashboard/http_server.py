"""Standard-library HTTP/SSE server with bounded clients and no history queues."""
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time
from urllib.parse import urlsplit


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, state, web_root, radar_hz=5.0, performance_hz=1.0, map_path=None):
        self.state = state
        self.web_root = Path(web_root)
        self.map_path = map_path
        self.intervals = {'radar': 1 / radar_hz, 'performance': 1 / performance_hz}
        self.stop_event = threading.Event()
        self.client_slots = threading.BoundedSemaphore(16)
        self.cache_lock = threading.Lock()
        self.cache = {}
        super().__init__(address, DashboardHandler)

    def process_request(self, request, client_address):
        if not self.client_slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self.client_slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.client_slots.release()

    def payload(self, channel):
        # Shared lazy cache: adding tabs does not multiply ROS-to-JSON conversion.
        with self.cache_lock:
            now = time.monotonic()
            cached = self.cache.get(channel)
            if cached and now - cached[0] < self.intervals.get(channel, 0.2):
                return cached[1]
            body = json.dumps(self.state.snapshot(channel), ensure_ascii=False,
                              allow_nan=False, separators=(',', ':')).encode()
            self.cache[channel] = (now, body)
            return body

    def close(self):
        self.stop_event.set()
        self.shutdown()
        self.server_close()


class DashboardHandler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(3)

    def log_message(self, *_):
        pass

    def do_GET(self):
        path = urlsplit(self.path).path
        try:
            if path == '/api/state':
                self.respond(self.server.payload('state'), 'application/json; charset=utf-8')
            elif path == '/map.png' and self.server.map_path:
                self.respond(self.server.map_path.read_bytes(), 'image/png')
            elif path in ('/api/radar-stream', '/api/performance-stream'):
                channel = 'radar' if path == '/api/radar-stream' else 'performance'
                self.stream(channel)
            elif path in ('/', '/index.html', '/app.js', '/styles.css'):
                filename = 'index.html' if path == '/' else path[1:]
                mime = {'html': 'text/html', 'js': 'text/javascript', 'css': 'text/css'}
                self.respond((self.server.web_root / filename).read_bytes(),
                             mime[filename.rsplit('.', 1)[1]] + '; charset=utf-8')
            elif path == '/favicon.ico':
                self.respond(b'', 'image/x-icon', 204)
            else:
                self.send_error(404)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    def respond(self, body, content_type, status=200):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(body)

    def stream(self, channel):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('X-Accel-Buffering', 'no')
        self.end_headers()
        self.wfile.write(b'retry: 2000\n\n')
        while not self.server.stop_event.is_set():
            self.wfile.write(b'event: ' + channel.encode() + b'\ndata: '
                             + self.server.payload(channel) + b'\n\n')
            self.wfile.flush()
            if self.server.stop_event.wait(self.server.intervals[channel]):
                break
