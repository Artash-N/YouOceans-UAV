#!/usr/bin/env python3
"""
Headless camera calibration for Jetson over SSH (no GUI).

- Supports CSI (Argus/GStreamer) or V4L2/USB
- On-demand snapshot capture (CLI)
- Chessboard detection (inner corners)
- Live reprojection error after each accepted frame
- Final calibration saved to NPZ
- Optional annotated snapshots saved to a folder

Examples
--------
# CSI camera, 1080p@30, sensor-id 0, mode 2 (1080p@30 on IMX219)
python calibrate_headless.py --csi --sensor-id 0 --mode 2 --width 1920 --height 1080 --fps 30 \
    --cols 8 --rows 5 --square-mm 25 --frames 20 --save camera_calib.npz --out ./snaps

# USB/V4L2 camera on /dev/video0
python calibrate_headless.py --cam 0 --cols 8 --rows 5 --square-mm 25 --frames 20 --out ./snaps
"""
import argparse, time
from pathlib import Path
import cv2
import numpy as np


# --------------------------- CLI ---------------------------

def parse_args():
    ap = argparse.ArgumentParser(description="Headless camera calibration (chessboard) for Jetson/SSH")

    # Board
    ap.add_argument("--cols", type=int, default=8, help="Chessboard inner corners across (X)")
    ap.add_argument("--rows", type=int, default=5, help="Chessboard inner corners down (Y)")
    ap.add_argument("--square-mm", type=float, default=25.0, help="Square edge length (mm)")
    ap.add_argument("--frames", type=int, default=20, help="Target snapshots to collect")

    # Output
    ap.add_argument("--save", default="camera_calib.npz", help="Output calibration file (.npz)")
    ap.add_argument("--out", default="", help="Optional folder to save annotated snapshots (jpg)")
    ap.add_argument("--warmup", type=int, default=5, help="Frames to grab before a snapshot for auto-exposure")

    # Camera (choose ONE path)
    ap.add_argument("--csi", action="store_true", help="Use CSI camera via GStreamer (Argus)")
    # CSI options
    ap.add_argument("--sensor-id", type=int, default=0, help="CSI sensor-id (Argus)")
    ap.add_argument("--mode", type=int, default=2, help="CSI sensor-mode (e.g., 2 for 1920x1080@30 on IMX219)")
    ap.add_argument("--width", type=int, default=1920, help="Capture width")
    ap.add_argument("--height", type=int, default=1080, help="Capture height")
    ap.add_argument("--fps", type=int, default=30, help="Capture FPS")
    # V4L2/USB fallback
    ap.add_argument("--cam", type=int, default=0, help="OpenCV camera index (e.g., 0 => /dev/video0)")

    return ap.parse_args()


# --------------------------- Helpers ---------------------------

def reproj_error(K, dist, rvecs, tvecs, objpoints, imgpoints) -> float:
    total_err_sq, total_pts = 0.0, 0
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        proj = proj.reshape(-1, 2)
        actual = imgpoints[i].reshape(-1, 2)
        err = cv2.norm(actual, proj, cv2.NORM_L2)
        total_err_sq += err * err
        total_pts += len(proj)
    return float(np.sqrt(total_err_sq / max(total_pts, 1)))


def quick_live_error(img_size, objpoints, imgpoints, min_frames=10):
    if not img_size or len(objpoints) < min_frames:
        return None
    flags = 0
    rms, Kt, dt, rts, tts = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None, flags=flags)
    return reproj_error(Kt, dt, rts, tts, objpoints, imgpoints)


def build_board(cols, rows, square_m):
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_m
    return objp


def gstreamer_csi_pipeline(sensor_id, mode, w, h, fps):
    """
    Build a robust Argus → nvvidconv → BGR appsink pipeline for OpenCV.
    """
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} sensor-mode={mode} ! "
        f"video/x-raw(memory:NVMM), width={w}, height={h}, framerate={fps}/1 ! "
        f"nvvidconv ! video/x-raw, format=BGR ! "
        f"appsink drop=true max-buffers=1 sync=false"
    )


def open_camera(args):
    if args.csi:
        pipe = gstreamer_csi_pipeline(args.sensor_id, args.mode, args.width, args.height, args.fps)
        cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            raise RuntimeError(
                "Failed to open CSI camera via GStreamer. "
                "Check nvargus-daemon and your sensor-id/mode/width/height/fps."
            )
        return cap
    else:
        cap = cv2.VideoCapture(args.cam)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open camera index {args.cam}")
        # Try to set resolution if provided
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS, args.fps)
        return cap


def warmup_camera(cap, n=5):
    for _ in range(max(0, n)):
        cap.read()


def capture_frame(cap):
    ok, frame = cap.read()
    if not ok or frame is None:
        raise RuntimeError("Failed to grab frame from camera.")
    return frame


def detect_chessboard(gray, pattern_size):
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags=cv2.CALIB_CB_FAST_CHECK)
    if not found:
        found, corners = cv2.findChessboardCorners(gray, pattern_size)
    return found, corners


def refine_corners(gray, corners):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return corners


def save_annotated(outdir: Path, idx: int, frame, pattern_size, corners, found):
    if not outdir:
        return
    outdir.mkdir(parents=True, exist_ok=True)
    vis = frame.copy()
    if found:
        cv2.drawChessboardCorners(vis, pattern_size, corners, found)
    path = outdir / f"snap_{idx:02d}.jpg"
    cv2.imwrite(str(path), vis)


def print_menu(args, have_frames, live_err, calibrated_err):
    print("\n=== Camera Calibration (Headless) ===")
    print(f"Board: inner corners {args.cols} x {args.rows} | square {args.square_mm:.1f} mm")
    print(f"Collected frames: {have_frames}/{args.frames}")
    print(f"Live error (needs ≥10): {'—' if live_err is None else f'{live_err:.4f} px'}")
    if calibrated_err is not None:
        print(f"Last calibration error: {calibrated_err:.4f} px")
    print("\nCommands:")
    print("  snap       -> capture one frame & detect board (adds if found)")
    print("  undo       -> remove last accepted frame")
    print("  calibrate  -> run full calibrate & save NPZ")
    print("  reset      -> clear all collected frames")
    print("  quit       -> exit\n")


# --------------------------- Main ---------------------------

def main():
    args = parse_args()

    pattern_size = (args.cols, args.rows)
    square_m = args.square_mm / 1000.0
    objp_one = build_board(args.cols, args.rows, square_m)

    cap = open_camera(args)
    outdir = Path(args.out) if args.out else None
    save_path = Path(args.save).resolve()

    objpoints, imgpoints = [], []
    img_size = None
    calibrated_err = None

    print("Camera opened. Make sure the chessboard fills a good portion of the frame and is captured at various tilts/positions.")
    while True:
        live_err = quick_live_error(img_size, objpoints, imgpoints, min_frames=10)
        print_menu(args, len(objpoints), live_err, calibrated_err)

        cmd = input("> ").strip().lower()

        if cmd in ("q", "quit", "exit"):
            break

        elif cmd in ("s", "snap"):
            warmup_camera(cap, n=args.warmup)
            frame = capture_frame(cap)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            img_size = (gray.shape[1], gray.shape[0])

            found, corners = detect_chessboard(gray, pattern_size)
            if not found:
                print("  - Chessboard NOT found. Try again with better lighting/coverage.")
                continue

            corners = refine_corners(gray, corners)
            objpoints.append(objp_one.copy())
            imgpoints.append(corners.reshape(-1, 2))

            save_annotated(outdir, len(objpoints), frame, pattern_size, corners, found)

            live_err = quick_live_error(img_size, objpoints, imgpoints, min_frames=10)
            msg_err = "—" if live_err is None else f"{live_err:.4f} px"
            print(f"  ✓ Added frame {len(objpoints)}/{args.frames}. Live error: {msg_err}")

        elif cmd in ("u", "undo"):
            if objpoints:
                objpoints.pop()
                imgpoints.pop()
                print(f"  ↺ Removed last. Now {len(objpoints)} frames.")
            else:
                print("  - Nothing to undo.")

        elif cmd in ("r", "reset"):
            objpoints.clear()
            imgpoints.clear()
            img_size = None
            calibrated_err = None
            print("  ↺ Reset all collected frames and calibration.")

        elif cmd in ("c", "calibrate"):
            if not imgpoints or not objpoints:
                print("  - No frames collected yet.")
                continue
            need = max(10, min(20, args.frames // 2))
            if len(objpoints) < need:
                print(f"  - Collect more frames first (≥{need}). Currently {len(objpoints)}.")
                continue

            print("  … Calibrating …")
            flags = 0  # keep model simple; consider cv2.CALIB_RATIONAL_MODEL if needed
            rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
                objpoints, imgpoints, img_size, None, None, flags=flags
            )
            err = reproj_error(K, dist, rvecs, tvecs, objpoints, imgpoints)
            calibrated_err = err

            np.savez(
                save_path,
                K=K, dist=dist, img_w=img_size[0], img_h=img_size[1],
                square_size_m=square_m,
                inner_cols=args.cols, inner_rows=args.rows,
                mean_reproj_error=err,
                timestamp=time.time()
            )
            print(f"  ✓ Saved calibration to: {save_path}")
            print(f"  ✓ Mean reprojection error: {err:.4f} px")

            # Optional undistorted preview
            try:
                warmup_camera(cap, n=2)
                frm = capture_frame(cap)
                und = cv2.undistort(frm, K, dist)
                und_file = save_path.with_suffix("").parent / (save_path.stem + "_undistorted.jpg")
                cv2.imwrite(str(und_file), und)
                print(f"  ✓ Wrote undistorted preview: {und_file}")
            except Exception as e:
                print(f"  (undistorted preview skipped: {e})")

        else:
            print("  - Unknown command. Try: snap | undo | calibrate | reset | quit")

    cap.release()
    print("Done.")


if __name__ == "__main__":
    main()
