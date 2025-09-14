#!/usr/bin/env python3
"""
detect_apriltag_pose.py — robust AprilTag (36h11, id=3) pose from Jetson CSI camera
Outputs camera->tag translation (x,y,z) in meters that stays correct under tilt.

Reqs:
  pip install pupil-apriltags opencv-python numpy
"""

import argparse, time, cv2, numpy as np
from pupil_apriltags import Detector
from tools.csi_cam import CSIStream, CSIConfig

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", required=True, help="camera_calib.npz from your calibration")
    ap.add_argument("--sensor", type=int, default=0)
    ap.add_argument("--w", type=int, default=1280)
    ap.add_argument("--h", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--tag-id", type=int, default=3)
    ap.add_argument("--tag-size-m", type=float, default=0.38)  # outer black square
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--threads", type=int, default=2)
    ap.add_argument("--decimate", type=float, default=1.5, help="quad_decimate (↑=faster, ↓=more accurate)")
    return ap.parse_args()

def solve_pose_square(img_corners_px, tag_size_m, K, dist):
    """
    img_corners_px: (4,2) float32 in order [TL, TR, BR, BL]
    returns ok, rvec(3,1), tvec(3,1)
    """
    s = tag_size_m * 0.5
    obj = np.float32([
        [-s,  s, 0.0],  # TL
        [ s,  s, 0.0],  # TR
        [ s, -s, 0.0],  # BR
        [-s, -s, 0.0],  # BL
    ]).reshape(-1, 1, 3)

    # 1) Undistort image points -> pixel coords with no distortion
    #    Using P=K projects normalized points back to pixel metric
    undist = cv2.undistortPoints(
        img_corners_px.reshape(-1,1,2).astype(np.float32),
        cameraMatrix=K, distCoeffs=dist, R=None, P=K
    ).reshape(-1,2)

    # 2) SolvePnP with NO distortion (we already undistorted)
    #    Prefer IPPE_SQUARE for planar squares
    flag_ippe = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
    ok, rvec, tvec = cv2.solvePnP(
        objectPoints=obj,
        imagePoints=undist.reshape(-1,1,2),
        cameraMatrix=K,
        distCoeffs=None,
        flags=flag_ippe
    )
    if not ok and flag_ippe != cv2.SOLVEPNP_ITERATIVE:
        ok, rvec, tvec = cv2.solvePnP(obj, undist.reshape(-1,1,2), K, None, flags=cv2.SOLVEPNP_ITERATIVE)
    return ok, rvec, tvec

def draw_axes(img, K, rvec, tvec, axis_len=0.1):
    pts_obj = np.float32([[0,0,0],[axis_len,0,0],[0,axis_len,0],[0,0,axis_len]]).reshape(-1,3)
    pts_img, _ = cv2.projectPoints(pts_obj, rvec, tvec, K, None)
    p0, px, py, pz = [tuple(p.ravel().astype(int)) for p in pts_img]
    cv2.line(img, p0, px, (0,0,255), 2)
    cv2.line(img, p0, py, (0,255,0), 2)
    cv2.line(img, p0, pz, (255,0,0), 2)

def main():
    args = parse_args()

    # Load calibration
    data = np.load(args.calib)
    K, dist = data["K"], data["dist"]
    print(f"[calib] loaded {args.calib}  fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}")

    # AprilTag detector
    detector = Detector(
        families="tag36h11",
        nthreads=args.threads,
        quad_decimate=args.decimate,
        quad_sigma=0.0,
        refine_edges=True,
        decode_sharpening=0.25
    )

    # CSI camera
    cfg = CSIConfig(sensor_id=args.sensor, width=args.w, height=args.h, fps=args.fps, color="bgr")

    # FPS
    t0 = time.perf_counter(); n = 0; fps = 0.0

    with CSIStream(cfg) as cam:
        print("✅ Camera started. Press 'q' to quit.")
        while True:
            frame, ts = cam.read()
            if frame is None:
                time.sleep(0.002)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            dets = detector.detect(
                gray,
                estimate_tag_pose=False,  # we do our own undistort+PnP
                camera_params=None,
                tag_size=args.tag_size_m
            )

            # Find requested ID
            target = next((d for d in dets if d.tag_id == args.tag_id), None)

            if target is not None and target.corners is not None and len(target.corners) == 4:
                img_corners = np.asarray(target.corners, dtype=np.float32)  # TL,TR,BR,BL (pupil-apriltags order)

                ok, rvec, tvec = solve_pose_square(img_corners, args.tag_size_m, K, dist)
                if ok:
                    x, y, z = tvec.ravel().tolist()  # meters, camera frame (+x right, +y down, +z forward)

                    # Pretty print
                    print(f"id={args.tag_id}  tvec [m] = X:{x:+.3f}  Y:{y:+.3f}  Z:{z:+.3f}  | range={np.linalg.norm(tvec):.3f} m")

                    if args.show:
                        pts = img_corners.astype(int)
                        cv2.polylines(frame, [pts], True, (0,255,255), 2, cv2.LINE_AA)
                        cxy = tuple(np.round(target.center).astype(int))
                        cv2.circle(frame, cxy, 3, (0,255,255), -1, cv2.LINE_AA)
                        draw_axes(frame, K, rvec, tvec, axis_len=min(0.1, args.tag_size_m*0.25))
                        cv2.putText(frame, f"X:{x:+.2f} Y:{y:+.2f} Z:{z:+.2f} m",
                                    (cxy[0]+8, cxy[1]-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    (0,0,0), 3, cv2.LINE_AA)
                        cv2.putText(frame, f"X:{x:+.2f} Y:{y:+.2f} Z:{z:+.2f} m",
                                    (cxy[0]+8, cxy[1]-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    (255,255,255), 1, cv2.LINE_AA)

            # FPS overlay
            n += 1
            now = time.perf_counter()
            if now - t0 >= 1.0:
                fps = n / (now - t0); n = 0; t0 = now
            if args.show:
                cv2.putText(frame, f"FPS: {fps:4.1f}", (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 3, cv2.LINE_AA)
                cv2.putText(frame, f"FPS: {fps:4.1f}", (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 1, cv2.LINE_AA)
                cv2.imshow("AprilTag Pose (camera frame)", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

