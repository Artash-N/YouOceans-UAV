#!/usr/bin/env python3
"""
detect_apriltag_pose_body.py — AprilTag (36h11) pose with Jetson CSI camera

• Outputs tag translation in PX4 body frame (FRD) for your camera mounting:
    X_body = forward, Y_body = right, Z_body = down
    Mapping from camera frame: Xb=-Xc, Yb=-Yc, Zb=+Zc

Reqs: pip install pupil-apriltags opencv-python numpy
"""

import argparse, time, cv2, numpy as np
from pupil_apriltags import Detector
from tools.csi_cam import CSIStream, CSIConfig

# --- fixed camera->body rotation for your mount (left=front, top=right, optical axis down)
R_CB = np.array([[-1., 0., 0.],
                 [ 0., 1., 0.],
                 [ 0., 0., 1.]], dtype=float)

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", required=True, help="camera_calib.npz from your calibration")
    ap.add_argument("--sensor", type=int, default=0)
    ap.add_argument("--w", type=int, default=1280)
    ap.add_argument("--h", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--tag-id", type=int, default=3)
    ap.add_argument("--tag-size-m", type=float, default=0.38)  # outer black square
    ap.add_argument("--threads", type=int, default=2)
    ap.add_argument("--decimate", type=float, default=1.5, help="quad_decimate (↑=faster, ↓=more accurate)")
    ap.add_argument("--show", action="store_true", help="show annotated preview")
    ap.add_argument("--frame", choices=["body","cam"], default="body",
                    help="which frame to print/export (default body)")
    return ap.parse_args()

def solve_pose_square(img_corners_px, tag_size_m, K, dist):
    """PnP on undistorted corners. img_corners_px in order [TL, TR, BR, BL]."""
    s = tag_size_m * 0.5
    obj = np.float32([[-s, s, 0.0],
                      [ s, s, 0.0],
                      [ s,-s, 0.0],
                      [-s,-s, 0.0]]).reshape(-1,1,3)

    undist = cv2.undistortPoints(
        img_corners_px.reshape(-1,1,2).astype(np.float32),
        cameraMatrix=K, distCoeffs=dist, R=None, P=K
    ).reshape(-1,2)

    flag_ippe = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
    ok, rvec, tvec = cv2.solvePnP(obj, undist.reshape(-1,1,2), K, None, flags=flag_ippe)
    if not ok and flag_ippe != cv2.SOLVEPNP_ITERATIVE:
        ok, rvec, tvec = cv2.solvePnP(obj, undist.reshape(-1,1,2), K, None)
    return ok, rvec, tvec

def draw_axes(img, K, rvec, tvec, axis_len=0.1, color_distort=None):
    """Draw camera-frame axes (X red, Y green, Z blue)."""
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
    print(f"[calib] {args.calib}  fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}")

    detector = Detector(
        families="tag36h11",
        nthreads=args.threads,
        quad_decimate=args.decimate,
        quad_sigma=0.0,
        refine_edges=True,
        decode_sharpening=0.25
    )

    cfg = CSIConfig(sensor_id=args.sensor, width=args.w, height=args.h, fps=args.fps, color="bgr")

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
                gray, estimate_tag_pose=False, camera_params=None, tag_size=args.tag_size_m
            )

            target = next((d for d in dets if d.tag_id == args.tag_id), None)

            if target is not None and target.corners is not None and len(target.corners) == 4:
                img_corners = np.asarray(target.corners, dtype=np.float32)  # TL,TR,BR,BL

                ok, rvec_c, tvec_c = solve_pose_square(img_corners, args.tag_size_m, K, dist)
                if ok:
                    # camera-frame translation (meters)
                    x_c, y_c, z_c = tvec_c.ravel()

                    # map to PX4 body FRD using your mount
                    tvec_b = (R_CB @ tvec_c.reshape(3,1)).ravel()
                    x_b, y_b, z_b = tvec_b.tolist()

                    # which to print
                    if args.frame == "body":
                        print(f"id={args.tag_id}  BODY tvec [m]  Xf:{x_b:+.3f}  Yr:{y_b:+.3f}  Zd:{z_b:+.3f}  | range={np.linalg.norm(tvec_c):.3f}")
                        txt = f"Xf:{x_b:+.2f} Yr:{y_b:+.2f} Zd:{z_b:+.2f} m"
                    else:
                        print(f"id={args.tag_id}  CAM  tvec [m]  Xr:{x_c:+.3f}  Yd:{y_c:+.3f}  Zf:{z_c:+.3f}  | range={np.linalg.norm(tvec_c):.3f}")
                        txt = f"Xr:{x_c:+.2f} Yd:{y_c:+.2f} Zf:{z_c:+.2f} m"

                    if args.show:
                        pts = img_corners.astype(int)
                        cv2.polylines(frame, [pts], True, (0,255,255), 2, cv2.LINE_AA)
                        cxy = tuple(np.round(target.center).astype(int))
                        cv2.circle(frame, cxy, 3, (0,255,255), -1, cv2.LINE_AA)

                        # axes (camera frame)
                        draw_axes(frame, K, rvec_c, tvec_c, axis_len=min(0.1, args.tag_size_m*0.25))

                        # label
                        cv2.putText(frame, txt, (cxy[0]+8, cxy[1]-8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3, cv2.LINE_AA)
                        cv2.putText(frame, txt, (cxy[0]+8, cxy[1]-8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)

            # FPS overlay
            n += 1
            now = time.perf_counter()
            if now - t0 >= 1.0:
                fps = n / (now - t0); n = 0; t0 = now
            if args.show:
                cv2.putText(frame, f"FPS: {fps:4.1f}", (10, 26),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 3, cv2.LINE_AA)
                cv2.putText(frame, f"FPS: {fps:4.1f}", (10, 26),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 1, cv2.LINE_AA)
                cv2.imshow("AprilTag Pose (body frame output)", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
