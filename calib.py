#!/usr/bin/env python3
# calib_csi_fixed.py — Chessboard calibration using tools.csi_cam with (frame, ts) read()

import cv2
import numpy as np
from pathlib import Path
import time
from tools.csi_cam import CSIConfig, CSIStream

# ===================== USER SETTINGS =====================
INNER_COLS = 8
INNER_ROWS = 5
SQUARE_SIZE_MM = 25.0
FRAMES_NEEDED = 20

# CSI camera config
SENSOR_ID = 0
WIDTH = 1280
HEIGHT = 720
FPS = 30
COLOR = "bgr"  # important: we want BGR, not gray, so we can convert reliably

SAVE_PATH = "camera_calib.npz"

LIVE_ERROR_MIN_FRAMES = 10
LIVE_ERROR_DEFAULT = False
# =========================================================

pattern_size = (INNER_COLS, INNER_ROWS)
square_size_m = SQUARE_SIZE_MM / 1000.0

# 3D board points (z=0 plane)
objp = np.zeros((INNER_ROWS * INNER_COLS, 3), np.float32)
objp[:, :2] = np.mgrid[0:INNER_COLS, 0:INNER_ROWS].T.reshape(-1, 2)
objp *= square_size_m

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

def safe_reproj_error(K, dist, rvecs, tvecs, objpoints, imgpoints):
    total_err_sq, total_pts = 0.0, 0
    for i in range(len(objpoints)):
        imgpts2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        imgpts2 = imgpts2.reshape(-1, 2)
        img_actual = imgpoints[i].reshape(-1, 2)
        err = cv2.norm(img_actual, imgpts2, cv2.NORM_L2)
        total_err_sq += (err * err)
        total_pts += len(imgpts2)
    return float(np.sqrt(total_err_sq / max(total_pts, 1)))

def quick_live_error(img_size, objpoints, imgpoints):
    if len(objpoints) < LIVE_ERROR_MIN_FRAMES:
        return None
    rms, K_tmp, dist_tmp, rvecs_tmp, tvecs_tmp = cv2.calibrateCamera(
        objpoints, imgpoints, img_size, None, None, flags=0
    )
    return safe_reproj_error(K_tmp, dist_tmp, rvecs_tmp, tvecs_tmp, objpoints, imgpoints)

def draw_hud(frame, found, count, calibrated, mean_error, live_err, live_on):
    overlay = frame.copy()
    pad = 10
    lines = [
        f"Chessboard (inner corners): {INNER_COLS} x {INNER_ROWS} | Square: {SQUARE_SIZE_MM:.1f} mm",
        f"Collected frames: {count}/{FRAMES_NEEDED}   (SPACE=add, BACKSPACE=undo)",
        "C=Calibrate & Save   U=Undistort preview   V=Toggle Live Error   R=Reset   ESC/Q=Quit",
        f"Corners: {'FOUND (press SPACE to add)' if found else 'not found'}",
    ]
    if live_on:
        lines.append(f"Live error (needs ≥{LIVE_ERROR_MIN_FRAMES}): "
                     f"{'—' if live_err is None else f'{live_err:.4f} px'}")
    else:
        lines.append("Live error: OFF (press V to enable)")
    if calibrated and mean_error is not None:
        lines.append(f"Calibrated ✓  Mean reprojection error: {mean_error:.4f} px")

    y = pad + 22
    for txt in lines:
        cv2.putText(overlay, txt, (pad, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(overlay, txt, (pad, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
        y += 24
    return overlay

def main():
    print("Controls: SPACE=add frame, BACKSPACE=undo, C=calibrate & save, U=undistort preview, V=toggle live error, R=reset, ESC/Q=quit")

    # ---- move mutable state INSIDE main ----
    objpoints = []
    imgpoints = []
    img_size = None

    calibrated = False
    K = None
    dist = None
    mean_error = None
    live_mean_error = None
    live_enabled = LIVE_ERROR_DEFAULT
    # ----------------------------------------

    cfg = CSIConfig(sensor_id=SENSOR_ID, width=WIDTH, height=HEIGHT, fps=FPS, color=COLOR)

    try:
        with CSIStream(cfg) as cam:
            while True:
                frame, ts = cam.read()
                if frame is None:
                    if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                        break
                    continue

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                img_size = (gray.shape[1], gray.shape[0])

                # fast then full
                found, corners = cv2.findChessboardCorners(gray, pattern_size, flags=cv2.CALIB_CB_FAST_CHECK)
                if not found:
                    found, corners = cv2.findChessboardCorners(gray, pattern_size)

                vis = frame.copy()
                if found:
                    cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
                    cv2.drawChessboardCorners(vis, pattern_size, corners, found)

                # live error
                if live_enabled:
                    live_mean_error = quick_live_error(img_size, objpoints, imgpoints)

                vis = draw_hud(
                    vis, found, len(objpoints), calibrated,
                    mean_error if calibrated else None,
                    live_mean_error, live_enabled
                )
                cv2.imshow("Calibration Capture (CSI)", vis)

                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord('q')):
                    break

                if key == 32 and found:  # SPACE
                    objpoints.append(objp.copy())
                    imgpoints.append(corners.reshape(-1, 2))
                    if live_enabled:
                        live_mean_error = quick_live_error(img_size, objpoints, imgpoints)
                    print(f"Added frame {len(objpoints)}/{FRAMES_NEEDED}. "
                          f"Live error: {'—' if live_mean_error is None else f'{live_mean_error:.4f} px'}")

                if key == 8:  # BACKSPACE
                    if objpoints:
                        objpoints.pop()
                        imgpoints.pop()
                        if live_enabled:
                            live_mean_error = quick_live_error(img_size, objpoints, imgpoints)
                        print(f"Removed last. Now {len(objpoints)} frames. "
                              f"Live error: {'—' if live_mean_error is None else f'{live_mean_error:.4f} px'}")

                if key in (ord('v'), ord('V')):
                    live_enabled = not live_enabled
                    print(f"Live error {'ENABLED' if live_enabled else 'DISABLED'}.")

                if key in (ord('r'), ord('R')):
                    objpoints.clear()
                    imgpoints.clear()
                    calibrated = False
                    K = dist = None
                    mean_error = None
                    live_mean_error = None
                    print("Reset collections and calibration.")

                if key in (ord('c'), ord('C')):
                    if len(objpoints) < max(10, min(20, FRAMES_NEEDED // 2)):
                        print(f"Collect more frames first. Currently {len(objpoints)}; target {FRAMES_NEEDED}.")
                        continue

                    print("Calibrating...")
                    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
                        objpoints, imgpoints, img_size, None, None, flags=0
                    )
                    mean_error = safe_reproj_error(K, dist, rvecs, tvecs, objpoints, imgpoints)
                    calibrated = True

                    np.savez(
                        SAVE_PATH,
                        K=K, dist=dist, img_w=img_size[0], img_h=img_size[1],
                        square_size_m=square_size_m,
                        inner_cols=INNER_COLS, inner_rows=INNER_ROWS,
                        mean_reproj_error=mean_error,
                        timestamp=time.time()
                    )
                    print(f"Saved calibration to {Path(SAVE_PATH).resolve()}")
                    print(f"Mean reprojection error: {mean_error:.4f} px")

                if key in (ord('u'), ord('U')):
                    if not calibrated or K is None:
                        print("Calibrate first (press C).")
                        continue
                    print("Undistort preview: press any key to exit preview.")
                    while True:
                        f2, ts2 = cam.read()
                        if f2 is None:
                            break
                        und = cv2.undistort(f2, K, dist)
                        cv2.imshow("Undistorted Preview (press any key to exit)", und)
                        if cv2.waitKey(1) & 0xFF != 255:
                            break

    except Exception as e:
        print(f"❌ Camera/loop error: {e}")
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

