#!/usr/bin/env python3
"""
detect_apriltag.py — AprilTag (36h11, id=3) pose from Jetson CSI camera

Requirements (install in your env):
  pip install pupil-apriltags opencv-python numpy

Usage:
  python detect_apriltag.py --calib camera_calib.npz --sensor 0 --w 1280 --h 720 --fps 30
"""

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
from pupil_apriltags import Detector

from tools.csi_cam import CSIStream, CSIConfig

# ------------------ CLI ------------------
def parse_args():
    ap = argparse.ArgumentParser(description="AprilTag 36h11 ID=3 pose with CSI camera")
    ap.add_argument("--calib", required=True, help="Path to calibration .npz (from your calibrator)")
    ap.add_argument("--sensor", type=int, default=0, help="CSI sensor-id (default 0)")
    ap.add_argument("--w", type=int, default=1280, help="Width")
    ap.add_argument("--h", type=int, default=720, help="Height")
    ap.add_argument("--fps", type=int, default=30, help="FPS request")
    ap.add_argument("--tag-id", type=int, default=3, help="AprilTag ID to lock onto")
    ap.add_argument("--tag-size-m", type=float, default=0.38, help="Tag size (outer black square) in meters")
    ap.add_argument("--show", action="store_true", help="Show annotated preview window")
    return ap.parse_args()

# ------------------ Math helpers ------------------
def rodrigues_to_euler_xyz(R):
    """
    Convert rotation matrix to intrinsic Tait-Bryan XYZ (roll, pitch, yaw) in radians.
    Camera frame convention (OpenCV): +X right, +Y down, +Z forward.
    """
    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    singular = sy < 1e-6
    if not singular:
        roll  = np.arctan2(R[2,1], R[2,2])
        pitch = np.arctan2(-R[2,0], sy)
        yaw   = np.arctan2(R[1,0], R[0,0])
    else:
        roll  = np.arctan2(-R[1,2], R[1,1])
        pitch = np.arctan2(-R[2,0], sy)
        yaw   = 0.0
    return roll, pitch, yaw

def draw_axes(img, K, dist, rvec, tvec, axis_len=0.1):
    """Draw XYZ axes (X=red, Y=green, Z=blue). Length in meters."""
    pts_obj = np.float32([
        [0,0,0],
        [axis_len,0,0],
        [0,axis_len,0],
        [0,0,axis_len]
    ])
    pts_img, _ = cv2.projectPoints(pts_obj, rvec, tvec, K, dist)
    p0 = tuple(pts_img[0].ravel().astype(int))
    px = tuple(pts_img[1].ravel().astype(int))
    py = tuple(pts_img[2].ravel().astype(int))
    pz = tuple(pts_img[3].ravel().astype(int))
    cv2.line(img, p0, px, (0,0,255), 2)
    cv2.line(img, p0, py, (0,255,0), 2)
    cv2.line(img, p0, pz, (255,0,0), 2)

# ------------------ Main ------------------
def main():
    args = parse_args()

    # Load calibration
    data = np.load(args.calib)
    K    = data["K"]
    dist = data["dist"]
    img_w = int(data["img_w"])
    img_h = int(data["img_h"])
    print(f"[calib] loaded {args.calib} | image size {img_w}x{img_h}")

    # AprilTag detector (CPU, fast + accurate defaults)
    detector = Detector(
        families="tag36h11",
        nthreads=2,
        quad_decimate=1.0,   # if you need more speed, try 1.5–2.0 (less accurate)
        quad_sigma=0.0,
        refine_edges=True,
        decode_sharpening=0.25
    )

    # Tag object corners in meters (OpenCV solvePnP expects the 3D order matching 2D points)
    # AprilTag corner order from detector: [top-left, top-right, bottom-right, bottom-left]
    s = args.tag_size_m / 2.0
    obj_corners = np.float32([
        [-s,  s, 0.0],   # top-left
        [ s,  s, 0.0],   # top-right
        [ s, -s, 0.0],   # bottom-right
        [-s, -s, 0.0]    # bottom-left
    ])

    # Camera config
    cfg = CSIConfig(sensor_id=args.sensor, width=args.w, height=args.h, fps=args.fps, color="bgr")

    # FPS tracking
    t_last = time.perf_counter()
    frames = 0
    fps = 0.0

    with CSIStream(cfg) as cam:
        print("✅ Camera started. Press 'q' to quit.")
        while True:
            frame, ts = cam.read()
            if frame is None:
                # small nap to avoid busy loop if camera hiccups
                time.sleep(0.002)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            # to grayscale for detector
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # detect
            detections = detector.detect(
                gray,
                estimate_tag_pose=False,  # we do our own solvePnP using your calib
                camera_params=None,
                tag_size=args.tag_size_m
            )

            target = None
            for det in detections:
                if det.tag_id == args.tag_id:
                    target = det
                    break

            if target is not None:
                # img points as float32 in the same order as obj_corners
                img_corners = np.array(target.corners, dtype=np.float32)  # (4,2)
                # pose from corners
                ok, rvec, tvec = cv2.solvePnP(obj_corners, img_corners, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    ok, rvec, tvec = cv2.solvePnP(obj_corners, img_corners, K, dist)

                if ok:
                    # rotation & euler
                    R, _ = cv2.Rodrigues(rvec)
                    roll, pitch, yaw = rodrigues_to_euler_xyz(R)
                    # translation (camera frame, meters) : X right, Y down, Z forward
                    x, y, z = tvec.reshape(3)
                    dist_m = float(np.linalg.norm(tvec))

                    # Console output (single line)
                    print(f"tag36h11 id={args.tag_id} | tvec[m]=[{x:+.3f},{y:+.3f},{z:+.3f}] | "
                          f"R/P/Y[deg]=[{np.degrees(roll):+.1f},{np.degrees(pitch):+.1f},{np.degrees(yaw):+.1f}] | "
                          f"range={dist_m:.3f} m")

                    if args.show:
                        # draw quad
                        pts = img_corners.astype(int)
                        cv2.polylines(frame, [pts], True, (0,255,255), 2, cv2.LINE_AA)
                        # center
                        cxy = tuple(np.round(target.center).astype(int))
                        cv2.circle(frame, cxy, 3, (0,255,255), -1, cv2.LINE_AA)
                        # axes
                        draw_axes(frame, K, dist, rvec, tvec, axis_len=min(0.1, args.tag_size_m*0.25))
                        # text
                        txt = f"id={args.tag_id} z={z:.2f}m yaw={np.degrees(yaw):+.1f}°"
                        cv2.putText(frame, txt, (cxy[0]+8, cxy[1]-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3, cv2.LINE_AA)
                        cv2.putText(frame, txt, (cxy[0]+8, cxy[1]-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)

            else:
                # Optional console note when not found
                # print("tag not found")
                pass

            # fps
            frames += 1
            now = time.perf_counter()
            if now - t_last >= 1.0:
                fps = frames / (now - t_last)
                t_last = now
                frames = 0

            if args.show:
                cv2.putText(frame, f"FPS: {fps:4.1f}", (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 3, cv2.LINE_AA)
                cv2.putText(frame, f"FPS: {fps:4.1f}", (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 1, cv2.LINE_AA)
                cv2.imshow("AprilTag Pose", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

