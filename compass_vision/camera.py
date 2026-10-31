"""Camera capture that always hands out the NEWEST frame.

OpenCV/GStreamer queue frames internally; if processing ever falls behind you end
up sending stale headings. A background thread reads continuously and keeps only
the latest frame plus the monotonic time it arrived.

Sources:
  csi       - Jetson CSI camera through nvarguscamerasrc (IMX219/IMX477...)
  usb       - V4L2 webcam
  gstreamer - your own pipeline string (must end in appsink)
  file      - an image or video file, for bench testing without a camera

Note: use the OpenCV that ships with JetPack (it has GStreamer). The pip
`opencv-python` wheel does NOT, and CSI cameras won't open with it.
"""

import logging
import threading
import time

import cv2

log = logging.getLogger(__name__)

IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp")


def csi_pipeline(ccfg):
    exposure = ""
    if ccfg.get("csi_exposure_range_ns"):
        exposure = ' exposuretimerange="%s"' % ccfg["csi_exposure_range_ns"]
    return ("nvarguscamerasrc sensor-id={sid}{exp} ! "
            "video/x-raw(memory:NVMM),width={w},height={h},framerate={fps}/1,format=NV12 ! "
            "nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false").format(
                sid=int(ccfg.get("csi_sensor_id", 0)), exp=exposure,
                w=int(ccfg["width"]), h=int(ccfg["height"]), fps=int(ccfg["fps"]))


class Camera(object):
    def __init__(self, ccfg):
        self.cfg = dict(ccfg)
        self.source = str(self.cfg.get("source", "csi")).lower()
        self._cap = None
        self._still = None
        self._cond = threading.Condition()
        self._frame = None
        self._t = 0.0
        self._seq = 0
        self._running = True
        self._open()
        self._thread = threading.Thread(target=self._run, name="camera", daemon=True)
        self._thread.start()

    def _open(self):
        src = self.source
        if src == "file":
            path = self.cfg.get("file_path") or ""
            if path.lower().endswith(IMAGE_EXTS):
                self._still = cv2.imread(path)
                if self._still is None:
                    raise IOError("could not read image %r" % path)
                return
            self._cap = cv2.VideoCapture(path)
        elif src == "csi":
            pipe = csi_pipeline(self.cfg)
            log.info("CSI pipeline: %s", pipe)
            self._cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
        elif src == "gstreamer":
            self._cap = cv2.VideoCapture(self.cfg["gstreamer_pipeline"], cv2.CAP_GSTREAMER)
        elif src == "usb":
            self._cap = cv2.VideoCapture(int(self.cfg.get("usb_index", 0)), cv2.CAP_V4L2)
            self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.cfg["width"]))
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.cfg["height"]))
            self._cap.set(cv2.CAP_PROP_FPS, int(self.cfg["fps"]))
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        else:
            raise ValueError("camera.source must be csi, usb, gstreamer or file (got %r)" % src)
        if not self._cap.isOpened():
            hint = ""
            if src in ("csi", "gstreamer"):
                hint = (" (is this the JetPack OpenCV with GStreamer? Check "
                        "`python3 -c \"import cv2; print(cv2.getBuildInformation())\" | grep GStreamer`)")
            raise IOError("could not open camera source %r%s" % (src, hint))

    def _publish(self, frame):
        with self._cond:
            self._frame = frame
            self._t = time.monotonic()
            self._seq += 1
            self._cond.notify_all()

    def _run(self):
        period = 1.0 / max(1.0, float(self.cfg.get("fps", 30)))
        failures = 0
        while self._running:
            if self._still is not None:
                self._publish(self._still)
                time.sleep(period)
                continue
            ok, frame = self._cap.read()
            if ok and frame is not None:
                failures = 0
                self._publish(frame)
                if self.source == "file":
                    time.sleep(period)  # play files at roughly real time
                continue
            if self.source == "file":  # loop the video
                self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            failures += 1
            if failures >= 30:
                log.error("camera stopped delivering frames, reopening")
                try:
                    self._cap.release()
                    time.sleep(1.0)
                    self._open()
                except Exception as e:  # keep trying; main loop reports no frames
                    log.error("camera reopen failed: %s", e)
                    time.sleep(2.0)
                failures = 0
            else:
                time.sleep(0.01)

    def read(self, last_seq=0, timeout=1.0):
        """Wait for a frame newer than last_seq. Returns (frame, t_capture, seq) or None."""
        deadline = time.monotonic() + timeout
        with self._cond:
            while self._seq <= last_seq:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._cond.wait(remaining)
            return self._frame, self._t, self._seq

    def close(self):
        self._running = False
        self._thread.join(timeout=2.0)
        if self._cap is not None:
            self._cap.release()
