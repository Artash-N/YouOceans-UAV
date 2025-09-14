#!/usr/bin/env python3
import argparse, time
from pathlib import Path
import cv2, numpy as np
from pupil_apriltags import Detector
from tools.csi_cam import CSIStream, CSIConfig

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", required=True)
    ap.add_argument("--sensor", type=int, default=0)
    ap.add_argument("--w", type=int, default=960)   # lower default
    ap.add_argument("--h", type=int, default=540)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--tag-id", type=int, default=3)
    ap.add_argument("--tag-size-m", type=float, default=0.38)
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--roi", action="store_true", help="use ROI when tracking (faster)")
    return ap.parse_args()

def rodrigues_to_euler_xyz(R):
    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    if sy >= 1e-6:
        roll  = np.arctan2(R[2,1], R[2,2])
        pitch = np.arctan2(-R[2,0], sy)
        yaw   = np.arctan2(R[1,0], R[0,0])
    else:
        roll  = np.arctan2(-R[1,2], R[1,1]); pitch = np.arctan2(-R[2,0], sy); yaw = 0.0
    return roll, pitch, yaw

def draw_axes(img, K, dist, rvec, tvec, axis_len=0.1):
    pts = np.float32([[0,0,0],[axis_len,0,0],[0,axis_len,0],[0,0,axis_len]])
    uv, _ = cv2.projectPoints(pts, rvec, tvec, K, dist)
    p0, px, py, pz = [tuple(p.ravel().astype(int)) for p in uv]
    cv2.line(img, p0, px, (0,0,255), 2)
    cv2.line(img, p0, py, (0,255,0), 2)
    cv2.line(img, p0, pz, (255,0,0), 2)

def main():
    args = parse_args()
    calib = np.load(args.calib)
    K, dist = calib["K"], calib["dist"]

    # detector tuned for speed
    detector = Detector(
        families="tag36h11",
        nthreads=4,
        quad_decimate=1.5,      # 2.0–3.0 = faster, slightly less accurate
        quad_sigma=0.0,
        refine_edges=True,     # faster
        decode_sharpening=0.25
    )

    # Provide camera params so detector returns pose directly (fx, fy, cx, cy)
    fx, fy = K[0,0], K[1,1]
    cx, cy = K[0,2], K[1,2]
    cam_params = (fx, fy, cx, cy)

    # Optionally do ROI detection when tracking
    have_roi = False
    roi_rect = None
    roi_pad = 40  # pixels

    cfg = CSIConfig(sensor_id=args.sensor, width=args.w, height=args.h, fps=args.fps, color="gray")
    t_last, frames, fps = time.perf_counter(), 0, 0.0

    with CSIStream(cfg) as cam:
        print("✅ AprilTag fast detector running. Press 'q' to quit.")
        while True:
            frame, ts = cam.read()
            if frame is None:
                time.sleep(0.002)
                if cv2.waitKey(1) & 0xFF == ord('q'): break
                continue

            gray = frame  # already grayscale from CSI

            # Select ROI or full image
            if args.roi and have_roi:
                x,y,w,h = roi_rect
                gray_in = gray[y:y+h, x:x+w]
            else:
                gray_in = gray

            # Detect (ask detector to estimate pose using our camera & tag size)
            dets = detector.detect(
                gray_in,
                estimate_tag_pose=True,
                camera_params=cam_params,
                tag_size=args.tag_size_m
            )

            # Pick our target id
            target = next((d for d in dets if d.tag_id == args.tag_id), None)
            if target is not None:
                # r,t from detector are tag pose w.r.t camera (already estimated)
                rvec, _ = cv2.Rodrigues(target.pose_R)
                tvec = target.pose_t.reshape(3,1)

                # If ROI was used, offsets are irrelevant to pose (only affects drawing)
                if args.show:
                    # corners in input subimage → map back to full image
                    corners = target.corners.copy()
                    if args.roi and have_roi:
                        corners[:,0] += x
                        corners[:,1] += y
                    pts = corners.astype(int)
                    cv2.polylines(gray if len(gray.shape)==2 else frame, [pts], True, 255, 2)

                    cxy = tuple(np.round(target.center + ([x,y] if (args.roi and have_roi) else [0,0])).astype(int))
                    cv2.circle(gray, cxy, 3, 255, -1)

                    # Draw axes on a BGR preview
                    disp = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                    draw_axes(disp, K, dist, rvec, tvec, axis_len=min(0.1, args.tag_size_m*0.25))

                R = target.pose_R
                roll, pitch, yaw = rodrigues_to_euler_xyz(R)
                x_t, y_t, z_t = tvec.ravel()
                rng = float(np.linalg.norm(tvec))

                print(f"id={args.tag_id} | t=[{x_t:+.3f},{y_t:+.3f},{z_t:+.3f}] m | "
                      f"R/P/Y=[{np.degrees(roll):+.1f},{np.degrees(pitch):+.1f},{np.degrees(yaw):+.1f}] deg | "
                      f"range={rng:.3f} m")

                # Update ROI for next frame
                if args.roi:
                    x0, y0 = np.min(pts, axis=0)
                    x1, y1 = np.max(pts, axis=0)
                    x0 = max(int(x0 - roi_pad), 0); y0 = max(int(y0 - roi_pad), 0)
                    x1 = min(int(x1 + roi_pad), gray.shape[1]-1)
                    y1 = min(int(y1 + roi_pad), gray.shape[0]-1)
                    roi_rect = (x0, y0, x1-x0+1, y1-y0+1)
                    have_roi = True
            else:
                have_roi = False
                disp = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            # FPS calc
            frames += 1
            now = time.perf_counter()
            if now - t_last >= 1.0:
                fps = frames / (now - t_last)
                t_last = now
                frames = 0

            if args.show:
                cv2.putText(disp, f"FPS:{fps:4.1f}", (10,26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 3, cv2.LINE_AA)
                cv2.putText(disp, f"FPS:{fps:4.1f}", (10,26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 1, cv2.LINE_AA)
                cv2.imshow("AprilTag Fast", disp)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    return

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

