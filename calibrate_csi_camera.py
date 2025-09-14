#!/usr/bin/env python3
"""
calibrate_csi_fast.py — Jetson CSI camera chessboard calibration (FAST only).

Controls:
  SPACE      = add current detection
  BACKSPACE  = undo last
  C          = calibrate & save
  U          = undistort preview (after calibrate)
  V          = toggle live error (computed ONLY on add/remove)
  T          = toggle timings overlay
  F          = toggle fast-only (default ON)  [kept in case you ever want to compare]
  R          = reset
  ESC/Q      = quit
"""

import cv2
import numpy as np
import time
from pathlib import Path
from collections import deque, defaultdict

from tools.csi_cam import CSIStream, CSIConfig

# ===================== USER SETTINGS =====================
INNER_COLS = 9
INNER_ROWS = 6
SQUARE_SIZE_MM = 24
FRAMES_NEEDED = 20

CFG = CSIConfig(sensor_id=0, width=1280, height=720, fps=10, color="bgr")
SAVE_PATH = "camera_calib.npz"

LIVE_ERROR_ENABLED = True
# =========================================================

pattern_size = (INNER_COLS, INNER_ROWS)
square_size_m = SQUARE_SIZE_MM / 1000.0

objp = np.zeros((INNER_ROWS * INNER_COLS, 3), np.float32)
objp[:, :2] = np.mgrid[0:INNER_COLS, 0:INNER_ROWS].T.reshape(-1, 2)
objp *= square_size_m

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

objpoints, imgpoints = [], []
img_size = None

calibrated = False
K = None
dist = None
mean_error = None
live_mean_error = None
live_enabled = LIVE_ERROR_ENABLED

# perf
timings_ovl = False
t_sums = defaultdict(float)
t_counts = defaultdict(int)
fps_window = deque(maxlen=90)

def tmark(): return time.perf_counter()
def tack(name, t0):
    dt = time.perf_counter() - t0
    t_sums[name] += dt
    t_counts[name] += 1
    return dt

def print_stats():
    if not t_counts: return
    print("\n--- perf (avg) ---")
    for k in sorted(t_counts.keys()):
        avg_ms = (t_sums[k] / t_counts[k]) * 1000.0
        print(f"{k:>12s}: {avg_ms:6.2f} ms")
    t_sums.clear(); t_counts.clear()

def safe_reproj_error(K, dist, rvecs, tvecs, objpoints, imgpoints):
    total_err_sq, total_pts = 0.0, 0
    for i in range(len(objpoints)):
        imgpts2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        imgpts2 = imgpts2.reshape(-1, 2)
        img_actual = imgpoints[i].reshape(-1, 2)
        err = cv2.norm(img_actual, imgpts2, cv2.NORM_L2)
        total_err_sq += err * err
        total_pts += len(imgpts2)
    return float(np.sqrt(total_err_sq / max(total_pts, 1)))

def quick_live_error(img_size, objpoints, imgpoints):
    if len(objpoints) < max(6, FRAMES_NEEDED // 4):
        return None
    t0 = tmark()
    rms, Kt, dt, rvt, tvt = cv2.calibrateCamera(
        objpoints, imgpoints, img_size, None, None, flags=0
    )
    tack("calib_live", t0)
    return safe_reproj_error(Kt, dt, rvt, tvt, objpoints, imgpoints)

def overlay_timings():
    if not timings_ovl or not t_counts:
        return None
    keys = ["grab", "gray", "detect_fast", "subpix", "hud", "imshow"]
    out = {}
    for k in keys:
        if t_counts.get(k, 0):
            out[k] = (t_sums[k] / t_counts[k]) * 1000.0
    return out

def draw_hud(frame, found, count, calibrated, mean_error, live_err, live_on, fps, ovl_timings):
    pad, y = 10, 32
    lines = [
        f"Chessboard (inner): {INNER_COLS} x {INNER_ROWS} | Square: {SQUARE_SIZE_MM:.1f} mm | FPS {fps:.1f}",
        f"Collected: {count}/{FRAMES_NEEDED}   (SPACE=add, BACKSPACE=undo)",
        "C=Calibrate  U=Undistort  V=LiveErr  T=Timings  F=FastOnly  R=Reset  ESC/Q=Quit",
        f"Corners: {'FOUND (SPACE to add)' if found else 'not found'}",
        f"Live error: {'OFF' if not live_on else ('—' if live_err is None else f'{live_err:.4f} px')}",
    ]
    if calibrated and mean_error is not None:
        lines.append(f"Calibrated ✓  Mean reprojection error: {mean_error:.4f} px")
    if ovl_timings:
        lines.append("Timings(ms): " + " | ".join([f"{k}:{v:.1f}" for k, v in ovl_timings.items()]))

    for txt in lines:
        cv2.putText(frame, txt, (pad, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(frame, txt, (pad, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
        y += 24
    return frame

def main():
    global calibrated, K, dist, mean_error, live_mean_error, live_enabled, timings_ovl

    print("Controls: SPACE add | BACKSPACE undo | C calibrate | U undistort | V live error | T timings | F fast-only | R reset | ESC/Q quit")

    fast_only = True  # hard default as requested

    with CSIStream(CFG) as cam:
        # Warm start to get size
        while True:
            t0 = tmark()
            frame, _ = cam.read(); tack("grab", t0)
            if frame is not None:
                t0 = tmark()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY); tack("gray", t0)
                h, w = gray.shape[:2]
                img_size = (w, h)
                break
            if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                return

        last_print = time.perf_counter()

        while True:
            t0 = tmark()
            frame, _ = cam.read(); tack("grab", t0)
            if frame is None:
                if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                    break
                continue

            # FPS window
            now = time.perf_counter()
            fps_window.append(now)
            while fps_window and (now - fps_window[0]) > 1.0:
                fps_window.popleft()
            fps = (len(fps_window) / max(1e-6, (fps_window[-1] - fps_window[0]))) if len(fps_window) > 1 else 0.0

            # grayscale
            t0 = tmark()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY); tack("gray", t0)

            # FAST only detection
            t0 = tmark()
            found, corners = cv2.findChessboardCorners(
                gray, pattern_size,
                flags=cv2.CALIB_CB_FAST_CHECK
            )
            tack("detect_fast", t0)

            if found:
                t0 = tmark()
                cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
                cv2.drawChessboardCorners(frame, pattern_size, corners, True)
                tack("subpix", t0)

            # HUD
            t0 = tmark()
            frame = draw_hud(
                frame, found, len(objpoints), calibrated,
                (mean_error if calibrated else None),
                (live_mean_error if live_enabled else None),
                live_enabled, fps, overlay_timings()
            )
            tack("hud", t0)

            t0 = tmark()
            cv2.imshow("Calibration (FAST only)", frame); tack("imshow", t0)

            # periodic perf print
            if (now - last_print) > 2.0:
                print_stats()
                last_print = now

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break

            if key == 32 and found:  # SPACE add
                objpoints.append(objp.copy())
                imgpoints.append(corners.reshape(-1, 2))
                if live_enabled:
                    live_mean_error = quick_live_error(img_size, objpoints, imgpoints)
                print(f"Added frame {len(objpoints)}/{FRAMES_NEEDED}. "
                      f"Live error: {'—' if not live_enabled or live_mean_error is None else f'{live_mean_error:.4f} px'}")

            if key == 8:  # BACKSPACE undo
                if objpoints:
                    objpoints.pop(); imgpoints.pop()
                    live_mean_error = quick_live_error(img_size, objpoints, imgpoints) if live_enabled else None
                    print(f"Removed last. Now {len(objpoints)} frames. "
                          f"Live error: {'—' if not live_enabled or live_mean_error is None else f'{live_mean_error:.4f} px'}")

            if key in (ord('v'), ord('V')):
                live_enabled = not live_enabled
                print(f"Live error {'ENABLED' if live_enabled else 'DISABLED'}.")

            if key in (ord('t'), ord('T')):
                timings_ovl = not timings_ovl
                print(f"Timings overlay {'ON' if timings_ovl else 'OFF'}.")

            if key in (ord('f'), ord('F')):
                # Kept only for experimentation; fast_only is always True in this build
                fast_only = True
                print("Fast-only mode: ON")

            if key in (ord('r'), ord('R')):
                objpoints.clear(); imgpoints.clear()
                calibrated = False; K = dist = None
                mean_error = None; live_mean_error = None
                print("Reset.")

            if key in (ord('c'), ord('C')):
                if len(objpoints) < max(10, min(20, FRAMES_NEEDED//2)):
                    print(f"Collect more frames first. Currently {len(objpoints)}; target {FRAMES_NEEDED}.")
                    continue
                print("Calibrating (final)…")
                t0c = tmark()
                rms, Kf, df, rvecs, tvecs = cv2.calibrateCamera(
                    objpoints, imgpoints, img_size, None, None, flags=0
                )
                tack("calib_final", t0c)
                err = safe_reproj_error(Kf, df, rvecs, tvecs, objpoints, imgpoints)
                K, dist, mean_error, calibrated = Kf, df, err, True
                np.savez(
                    SAVE_PATH,
                    K=K, dist=dist, img_w=img_size[0], img_h=img_size[1],
                    square_size_m=square_size_m,
                    inner_cols=INNER_COLS, inner_rows=INNER_ROWS,
                    mean_reproj_error=mean_error,
                    timestamp=time.time()
                )
                print(f"Saved: {Path(SAVE_PATH).resolve()}")
                print(f"Mean reprojection error: {mean_error:.4f} px")

            if key in (ord('u'), ord('U')):
                if not calibrated or K is None:
                    print("Calibrate first (press C).")
                    continue
                print("Undistort preview (any key to exit)")
                while True:
                    frm, _ = cam.read()
                    if frm is None:
                        continue
                    und = cv2.undistort(frm, K, dist)
                    cv2.imshow("Undistorted Preview", und)
                    if cv2.waitKey(1) & 0xFF != 255:
                        break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

