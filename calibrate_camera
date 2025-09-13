#!/usr/bin/env python3
"""
Headless camera calibration for Jetson over SSH (no GUI).
- On-demand snapshot capture
- Chessboard detection (inner corners)
- Live reprojection error after each accepted frame
- Final calibration saved to NPZ
- Optional annotated snapshots saved to a folder

Example:
  python calibrate_headless.py --cols 8 --rows 5 --square-mm 25 --frames 20 --cam 0 --save camera_calib.npz --out ./snaps
"""
import argparse, time
from pathlib import Path

import cv2
import numpy as np


def parse_args():
    ap = argparse.ArgumentParser(description="Headless camera calibration (chessboard) for Jetson/SSH")
    ap.add_argument("--cols", type=int, default=8, help="Chessboard inner corners across (X)")
    ap.add_argument("--rows", type=int, default=5, help="Chessboard inner corners down (Y)")
    ap.add_argument("--square-mm", type=float, default=25.0, help="Square edge length in millimeters")
    ap.add_argument("--frames", type=int, default=20, help="Target snapshots to collect")
    ap.add_argument("--cam", type=int, default=0, help="OpenCV camera index (e.g., 0 => /dev/video0)")
    ap.add_argument("--save", default="camera_calib.npz", help="Output calibration file (.npz)")
    ap.add_argument("--out", default="", help="Optional folder to save annotated snapshots (jpg)")
    ap.add_argument("--warmup", type=int, default=5, help="Frames to grab before each snapshot for auto-exposure")
    return ap.parse_args()


def reproj_error(K, dist, rvecs, tvecs, objpoints, imgpoints) -> float:
    """Mean pixel reprojection error (RMS)."""
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
    """Fast estimate of current calibration error (no initial guess)."""
    if len(objpoints) < min_frames:
        return None
    flags = 0
    rms, Kt, dt, rts, tts = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None, flags=flags)
    return reproj_error(Kt, dt, rts, tts, objpoints, imgpoints)


def build_board(cols, rows, square_m):
    """3D chessboard points for one view (z=0)."""
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_m
    return objp


def open_camera(index: int):
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {index}")
    return cap


def warmup_camera(cap, n=5):
    for _ in range(max(0, n)):
        cap.read()


def capture_frame(cap):
    ok, frame = cap.read()
    if not ok:
        raise RuntimeError("Failed to grab frame from camera.")
    return frame


def detect_chessboard(gray, pattern_size):
    # Fast check first
    flags = cv2.CALIB_CB_FAST_CHECK
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags=flags)
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


def main():
    args = parse_args()

    pattern_size = (args.cols, args.rows)
    square_m = args.square_mm / 1000.0
    objp_one = build_board(args.cols, args.rows, square_m)

    cap = open_camera(args.cam)
    outdir = Path(args.out) if args.out else None
    save_path = Path(args.save).resolve()

    objpoints, imgpoints = [], []
    img_size = None
    calibrated_err = None

    print("Camera opened. Use the commands below. Make sure the chessboard fills a good portion of the frame and is seen at various tilts/positions.")

    while True:
        # Print menu with current status
        live_err = quick_live_error(img_size, objpoints, imgpoints, min_frames=10) if img_size else None
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
            if len(objpoints) < max(10, min(20, args.frames // 2)):
                print(f"  - Collect more frames first (≥{max(10, min(20, args.frames // 2))}). Currently {len(objpoints)}.")
                continue

            print("  … Calibrating (this can take a few seconds)…")
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

            # Optional: write an undistorted example if we still have the camera up
            try:
                warmup_camera(cap, n=2)
                frm = capture_frame(cap)
                und = cv2.undistort(frm, K, dist)
                und_path = save_path.with_suffix("")  # strip .npz
                und_file = Path(str(und_path) + "_undistorted.jpg")
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
