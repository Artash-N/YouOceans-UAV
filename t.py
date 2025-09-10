#!/usr/bin/env python3

import asyncio
import csv
import os
import sys
import math
import argparse
from datetime import datetime, timezone

from mavsdk import System
from mavsdk.telemetry import FlightMode


DEFAULT_PORT = "/dev/ttyTHS1"
DEFAULT_BAUD = 115200
DEFAULT_HZ   = 5.0


CSV_FIELDS = [
    "ts_iso",
    "mode_name",
    "armed",
    "lat_deg","lon_deg","alt_m","agl_m",
    "vx_mps","vy_mps","vz_mps",
    "roll_deg","pitch_deg","yaw_deg",
    "vbatt_V","bat_rem_pct",
    "gps_fix","gps_sats",
    "rf_m"
]


def now_iso():
    return datetime.now(timezone.utc).astimezone().replace(tzinfo=None).isoformat(timespec="seconds")


def f(v, nd=1):
    return "-" if v is None else f"{v:.{nd}f}"


def flight_mode_str(mode: FlightMode) -> str:
    if mode is None:
        return "UNKNOWN"
    # Map to PX4-like names where it makes sense
    m = mode.name
    mapping = {
        "MANUAL": "MANUAL",
        "ALTCTL": "ALTCTL",
        "POSCTL": "POSCTL",
        "OFFBOARD": "OFFBOARD",
        "ACRO": "ACRO",
        "RATTITUDE": "RATTITUDE",
        "STABILIZED": "STABILIZED",
        "HOLD": "AUTO.LOITER",
        "RETURN_TO_LAUNCH": "AUTO.RTL",
        "TAKEOFF": "AUTO.TAKEOFF",
        "LAND": "AUTO.LAND",
        "MISSION": "AUTO.MISSION",
        "UNKNOWN": "UNKNOWN",
    }
    return mapping.get(m, m)


def gps_fix_label(v):
    # MAVSDK exposes an enum; we store numeric in state for CSV, and keep this for console labeling if wanted later
    table = {
        0: "NO_FIX",
        1: "FIX_1D",
        2: "FIX_2D",
        3: "FIX_3D",
        4: "DGPS",
        5: "RTK_FLOAT",
        6: "RTK_FIXED",
    }
    if v is None:
        return "-"
    try:
        return table.get(int(v), str(v))
    except Exception:
        return str(v)


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT, help="Serial device (e.g. /dev/ttyTHS1)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baudrate (e.g. 115200)")
    ap.add_argument("--hz",   type=float, default=DEFAULT_HZ, help="Console/CSV emit rate")
    ap.add_argument("--verbose", action="store_true", help="Print periodic status")
    args = ap.parse_args()

    serial_url = f"serial:///{args.port}:{args.baud}"

    # connect
    print("[monitor] waiting for connection…")
    drone = System()
    await drone.connect(system_address=serial_url)

    connected = False
    async for s in drone.core.connection_state():
        if s.is_connected:
            connected = True
            break
    if not connected:
        print("[monitor] no connection — check wiring/baud/permissions")
        sys.exit(1)

    print("✅ Connected")

    # state dictionary
    state = dict(
        mode_name=None, armed=False,
        lat_deg=None, lon_deg=None, alt_m=None, agl_m=None,
        vx_mps=None, vy_mps=None, vz_mps=None,
        roll_deg=None, pitch_deg=None, yaw_deg=None,
        vbatt_V=None, bat_rem_pct=None,
        gps_fix=None, gps_sats=0,
        rf_m=None
    )

    # logging file
    logdir = os.path.expanduser("~/logs/mav")
    os.makedirs(logdir, exist_ok=True)
    fname = os.path.join(logdir, f"mavsdk_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    print(f"[monitor] logging to {fname}")

    # subscriber tasks
    async def sub_flight_mode():
        async for m in drone.telemetry.flight_mode():
            state["mode_name"] = flight_mode_str(m)

    async def sub_armed():
        async for a in drone.telemetry.armed():
            state["armed"] = bool(a)

    async def sub_attitude():
        async for att in drone.telemetry.attitude_euler():
            state["roll_deg"]  = att.roll_deg
            state["pitch_deg"] = att.pitch_deg
            # Normalize yaw to [-180,180] like PX4 prints usually
            y = att.yaw_deg
            if y > 180.0:
                y -= 360.0
            state["yaw_deg"] = y

    async def sub_position():
        async for p in drone.telemetry.position():
            # p.latitude_deg may be NaN before GPS OK; guard with math.isfinite
            try:
                if p.latitude_deg is not None and math.isfinite(p.latitude_deg):
                    state["lat_deg"] = p.latitude_deg
                if p.longitude_deg is not None and math.isfinite(p.longitude_deg):
                    state["lon_deg"] = p.longitude_deg
            except Exception:
                pass
            # absolute and relative altitudes are meters AMSL and AGL (home), respectively
            state["alt_m"] = getattr(p, "absolute_altitude_m", None)
            state["agl_m"] = getattr(p, "relative_altitude_m", None)

    async def sub_velocity():
        async for v in drone.telemetry.velocity_ned():
            state["vx_mps"] = v.north_m_s
            state["vy_mps"] = v.east_m_s
            state["vz_mps"] = v.down_m_s  # note: down is positive down

    async def sub_battery():
        async for b in drone.telemetry.battery():
            # MAVSDK returns per-cell? No; voltage_v is total bus voltage
            state["vbatt_V"] = b.voltage_v
            # remaining_percent is 0..1
            rp = getattr(b, "remaining_percent", None)
            state["bat_rem_pct"] = None if rp is None else (rp * 100.0)

    async def sub_gps():
        async for g in drone.telemetry.gps_info():
            state["gps_sats"] = g.num_satellites
            # fix_type is an enum — use .value (int) safely
            try:
                state["gps_fix"] = g.fix_type.value
            except Exception:
                ft = getattr(g, "fix_type", None)
                state["gps_fix"] = getattr(ft, "value", None) if ft is not None else None

    async def sub_rangefinder():
        """
        Distance sensor stream (if available in your MAVSDK build/FW).
        If not available, this task exits silently.
        """
        try:
            async for d in drone.telemetry.distance_sensor():
                # current_distance_m is what we want; clamp to sane values
                dist = getattr(d, "current_distance_m", None)
                state["rf_m"] = dist if dist is None else float(dist)
        except Exception:
            # Feature may not be present; just ignore
            return

    # writer/console loop
    period = 1.0 / max(0.1, args.hz)

    async def emitter():
        with open(fname, "w", newline="") as fcsv:
            w = csv.DictWriter(fcsv, fieldnames=CSV_FIELDS)
            w.writeheader()
            while True:
                row = {
                    "ts_iso": now_iso(),
                    "mode_name": state["mode_name"] or "",
                    "armed": 1 if state["armed"] else 0,
                    "lat_deg": state["lat_deg"],
                    "lon_deg": state["lon_deg"],
                    "alt_m": state["alt_m"],
                    "agl_m": state["agl_m"],
                    "vx_mps": state["vx_mps"],
                    "vy_mps": state["vy_mps"],
                    "vz_mps": state["vz_mps"],
                    "roll_deg": state["roll_deg"],
                    "pitch_deg": state["pitch_deg"],
                    "yaw_deg": state["yaw_deg"],
                    "vbatt_V": state["vbatt_V"],
                    "bat_rem_pct": state["bat_rem_pct"],
                    "gps_fix": state["gps_fix"],
                    "gps_sats": state["gps_sats"],
                    "rf_m": state["rf_m"],
                }
                w.writerow(row)
                fcsv.flush()

                if args.verbose:
                    arm = "ARM" if state["armed"] else "DISARM"
                    lat = "-" if state["lat_deg"] is None else f"{state['lat_deg']:.7f}"
                    lon = "-" if state["lon_deg"] is None else f"{state['lon_deg']:.7f}"
                    alt = "-" if state["alt_m"] is None else f"{state['alt_m']:.1f}"
                    rf  = "-" if state["rf_m"] is None else f"{state['rf_m']:.2f}"
                    print(
                        f"{datetime.now().strftime('%H:%M:%S')} | {state['mode_name'] or 'UNKNOWN':<10} | {arm:6s}"
                        f" | VBat {f(state['vbatt_V'],2)}V | SoC {f(state['bat_rem_pct'],1)}%"
                        f" | GPS f{state['gps_fix'] if state['gps_fix'] is not None else 0} s{state['gps_sats'] or 0}"
                        f" | Lat {lat} | Lon {lon} | Alt {alt} | RF {rf}"
                        f" | R/P/Y {f(state['roll_deg'],1)}/{f(state['pitch_deg'],1)}/{f(state['yaw_deg'],1)}°"
                    )

                await asyncio.sleep(period)

    # kick off everything
    tasks = [
        asyncio.create_task(sub_flight_mode()),
        asyncio.create_task(sub_armed()),
        asyncio.create_task(sub_attitude()),
        asyncio.create_task(sub_position()),
        asyncio.create_task(sub_velocity()),
        asyncio.create_task(sub_battery()),
        asyncio.create_task(sub_gps()),
        asyncio.create_task(sub_rangefinder()),
        asyncio.create_task(emitter()),
    ]

    try:
        await asyncio.gather(*tasks)
    except asyncio.CancelledError:
        pass


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[monitor] stopping.")

