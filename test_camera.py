#!/usr/bin/env python3
"""
show_stream.py — use csi_cam library to show Jetson CSI camera at ~20 FPS
"""

import cv2
import time
from tools.csi_cam import CSIStream, CSIConfig

def main():
    # config: grayscale=fast, bgr=color
    cfg = CSIConfig(sensor_id=0, width=1280, height=720, fps=30, color="bgr")

    with CSIStream(cfg) as cam:
        print("✅ Camera started. Press 'q' to quit.")
        delay = 1.0 / 20.0  # target 20 fps
        last = 0

        while True:
            frame, ts = cam.read()
            if frame is None:
                time.sleep(0.01)
                continue

            # Throttle display to ~20 fps
            now = time.time()
            if now - last < delay:
                continue
            last = now

            cv2.imshow("CSI Camera Stream", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

