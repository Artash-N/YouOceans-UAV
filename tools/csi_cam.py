#!/usr/bin/env python3
"""
csi_cam.py — ultra-lean CSI camera grabber for Jetson (no gi/pygst required)

- Spawns `gst-launch-1.0 nvarguscamerasrc ... multipartmux ! fdsink`
- Parses the MJPEG multipart stream and decodes to OpenCV BGR or GRAY
- Single-element buffer: `read()` always returns the most recent frame
- Designed for AprilTag / CV pipelines where low latency matters

Dependencies: OpenCV (cv2), numpy
"""

import subprocess, threading, time, re, os, signal
from typing import Optional, Tuple, Iterator
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import cv2


@dataclass
class CSIConfig:
    sensor_id: int = 0
    width: int = 1280
    height: int = 720
    fps: int = 30
    boundary: bytes = b"frame"   # must match multipartmux boundary
    color: str = "bgr"           # 'bgr' or 'gray'
    jpeg_quality: Optional[int] = None  # None=nvjpeg default; (not exposed in current pipeline)


def _gst_cmd(cfg: CSIConfig) -> list[str]:
    # I420 → jpegenc → multipartmux → fdsink
    return [
        "gst-launch-1.0", "-q",
        "nvarguscamerasrc", f"sensor-id={cfg.sensor_id}",
        "!", f"video/x-raw(memory:NVMM),width={cfg.width},height={cfg.height},framerate={cfg.fps}/1",
        "!", "nvvidconv",
        "!", "video/x-raw,format=I420",
        "!", "jpegenc",
        "!", "multipartmux", f"boundary={cfg.boundary.decode()}",
        "!", "fdsink", "fd=1"
    ]


def _iter_mjpeg_parts(stream, boundary: bytes) -> Iterator[bytes]:
    dash_boundary = b"--" + boundary
    buf = bytearray()
    while True:
        chunk = stream.read(65536)
        if not chunk:
            break
        buf += chunk
        # parse complete parts
        while True:
            bstart = buf.find(dash_boundary)
            if bstart == -1:
                break
            h_end = buf.find(b"\r\n\r\n", bstart)
            if h_end == -1:
                break
            header = buf[bstart:h_end]
            m = re.search(rb"Content-Length:\s*(\d+)", header, re.IGNORECASE)
            if not m:
                # cannot parse length; drop up to header end and continue
                del buf[:h_end+4]
                continue
            length = int(m.group(1))
            img_start = h_end + 4
            img_end = img_start + length
            if len(buf) < img_end:
                break
            jpeg = bytes(buf[img_start:img_end])
            yield jpeg
            del buf[:img_end]


class CSIStream:
    """
    Usage:
        from csi_cam import CSIStream, CSIConfig
        cam = CSIStream(CSIConfig(sensor_id=0, width=1280, height=720, fps=30, color="gray"))
        cam.start()
        try:
            while True:
                frame, ts = cam.read()
                if frame is not None:
                    # process frame (BGR or GRAY)
                    pass
        finally:
            cam.stop()
    """

    def __init__(self, cfg: CSIConfig = CSIConfig()):
        self.cfg = cfg
        self._proc: Optional[subprocess.Popen] = None
        self._thr: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._latest: Optional[np.ndarray] = None
        self._latest_ts: float = 0.0
        self._running = False
        self._err_text = ""

    def start(self) -> None:
        if self._running:
            return
        cmd = _gst_cmd(self.cfg)
        self._proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0
        )
        if not self._proc or not self._proc.stdout:
            raise RuntimeError("failed to launch gst pipeline")

        self._running = True
        self._thr = threading.Thread(target=self._loop, name="CSIReader", daemon=True)
        self._thr.start()

        # small wait to verify frames begin flowing
        start = time.time()
        while time.time() - start < 2.0:
            if self._latest is not None:
                return
            if self._proc.poll() is not None:
                self._capture_stderr()
                raise RuntimeError(f"gst exited early: {self._err_text.strip()}")
            time.sleep(0.02)
        # no frame yet, but still running — allow user code to continue

    def _capture_stderr(self) -> None:
        try:
            if self._proc and self._proc.stderr:
                self._err_text = self._proc.stderr.read().decode(errors="ignore")
        except Exception:
            pass

    def _loop(self) -> None:
        assert self._proc and self._proc.stdout
        boundary = self.cfg.boundary
        color_flag = cv2.IMREAD_GRAYSCALE if self.cfg.color.lower() == "gray" else cv2.IMREAD_COLOR

        for jpeg in _iter_mjpeg_parts(self._proc.stdout, boundary):
            if not self._running:
                break
            arr = np.frombuffer(jpeg, np.uint8)
            frame = cv2.imdecode(arr, color_flag)
            if frame is None:
                continue
            with self._lock:
                self._latest = frame
                self._latest_ts = time.time()

        # pipeline ended
        self._capture_stderr()
        self._running = False

    def read(self) -> Tuple[Optional[np.ndarray], float]:
        """Return the most recent frame (copy) and its timestamp (monotonic seconds)."""
        with self._lock:
            if self._latest is None:
                return None, 0.0
            return self._latest.copy(), self._latest_ts

    def stop(self) -> None:
        self._running = False
        if self._proc:
            try:
                # be nice first
                self._proc.terminate()
                try:
                    self._proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    os.kill(self._proc.pid, signal.SIGKILL)
            except Exception:
                pass
        if self._thr and self._thr.is_alive():
            self._thr.join(timeout=1.0)
        self._proc = None
        self._thr = None

    # Context manager convenience
    def __enter__(self):
        self.start(); return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()


# --- quick self-test ---
if __name__ == "__main__":
    cfg = CSIConfig(sensor_id=0, width=1280, height=720, fps=30, color="gray")
    with CSIStream(cfg) as cam:
        t0 = time.time()
        frames = 0
        while time.time() - t0 < 5.0:
            frm, ts = cam.read()
            if frm is not None:
                frames += 1
        print(f"Grabbed {frames} frames in 5s")

