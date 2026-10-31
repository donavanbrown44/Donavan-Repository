"""Tiny MJPEG server so you can watch what the Jetson sees from a laptop.

The drone has no screen. Open http://<jetson-ip>:<port>/ in a browser to see the
rectified dial, the needle mask and the current heading. Frames are only encoded
while someone is watching, capped at ~10 fps.
"""

import logging
import socketserver
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2

log = logging.getLogger(__name__)
BOUNDARY = b"frame"


class _Server(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class DebugStream(object):
    def __init__(self, port, max_fps=10.0):
        self.max_period = 1.0 / max_fps
        self._cond = threading.Condition()
        self._jpeg = None
        self._seq = 0
        self._last_encode = 0.0
        self.clients = 0
        stream = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                with stream._cond:
                    stream.clients += 1
                seq = 0
                try:
                    while True:
                        with stream._cond:
                            stream._cond.wait_for(lambda: stream._seq != seq, timeout=5.0)
                            jpeg, seq = stream._jpeg, stream._seq
                        if jpeg is None:
                            continue
                        self.wfile.write(b"--" + BOUNDARY + b"\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(("Content-Length: %d\r\n\r\n" % len(jpeg)).encode())
                        self.wfile.write(jpeg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    with stream._cond:
                        stream.clients -= 1

        self._server = _Server(("0.0.0.0", int(port)), Handler)
        threading.Thread(target=self._server.serve_forever, name="debug-http", daemon=True).start()
        log.info("debug video on http://<jetson-ip>:%d/", port)

    def wants_frame(self):
        return self.clients > 0 and (time.monotonic() - self._last_encode) >= self.max_period

    def update(self, bgr):
        self._last_encode = time.monotonic()
        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if not ok:
            return
        with self._cond:
            self._jpeg = buf.tobytes()
            self._seq += 1
            self._cond.notify_all()

    def close(self):
        self._server.shutdown()
