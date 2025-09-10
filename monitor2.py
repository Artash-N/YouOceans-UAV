#!/usr/bin/env python3
# monitor_mavsdk.py — lightweight MAVSDK state logger (with rangefinder)

import argparse, asyncio, csv, os, sys, math
from datetime import datetime

from mavsdk import System
from mavsdk.telemetry import FlightMode

CSV_FIELDS = [
    "ts_iso","mode","armed",
    "lat_deg","lon_deg","alt_m","agl_m",
    "vn","ve","vd",
    "roll_deg","pitch_deg","yaw_deg",
    "vbatt_V","bat_rem_pct","gps_fix","gps_sats",
    "range_m","range_min_m","range_max_m"
]

def f(v, nd=1):
    return "-" if v is None else f"{v:.{nd}f}"

async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial", default="serial:///dev/ttyTHS1:115200",
                    help="MAVSDK system_address (e.g. serial:///dev/ttyTHS1:115200)")
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    period = 1.0 / max(0.1, args.hz)

    drone = System()
    print(f"[monitor] connecting: {args.serial}  hz={args.hz}")
    await drone.connect(system_address=args.serial)

    print("[monitor] waiting for connection…")
    async for s in drone.core.connection_state():
        if s.is_connected:
            print("✅ Connected"); break

    # Shared state
    state = dict(
        mode=None, armed=None,
        lat=None, lon=None, alt=None, agl=None,
        vn=None, ve=None, vd=None,
        roll=None, pitch=None, yaw=None,
        vbatt=None, bat_pct=None,
        gps_fix=None, gps_sats=None,
        rng=None, rng_min=None, rng_max=None
    )

    # Subscriptions (each updates pieces of `state`)
    async def sub_flight_mode():
        async for fm in drone.telemetry.flight_mode():
            state["mode"] = fm.name if isinstance(fm, FlightMode) else str(fm)

    async def sub_armed():
        async for a in drone.telemetry.armed():
            state["armed"] = a

    async def sub_pos():
        async for p in drone.telemetry.position():
            state["lat"] = p.latitude_deg
            state["lon"] = p.longitude_deg
            state["alt"] = p.absolute_altitude_m
            state["agl"] = p.relative_altitude_m

    async def sub_vel():
        async for v in drone.telemetry.velocity_ned():
            state["vn"], state["ve"], state["vd"] = v.north_m_s, v.east_m_s, v.down_m_s

    async def sub_att():
        async for e in drone.telemetry.attitude_euler():
            state["roll"], state["pitch"], state["yaw"] = e.roll_deg, e.pitch_deg, e.yaw_deg

    async def sub_bat():
        async for b in drone.telemetry.battery():
            # Not all firmwares fill both; use what we get
            state["vbatt"] = getattr(b, "voltage_v", None)
            rem = getattr(b, "remaining_percent", None)
            state["bat_pct"] = None if rem is None else rem * 100.0

    async def sub_gps():
        async for g in drone.telemetry.gps_info():
            state["gps_sats"] = g.num_satellites
            # lock_type: 0=No Fix, 2=2D, 3=3D, etc. Map simply:
            state["gps_fix"] = int(getattr(g, "fix_type", 0))

    async def sub_range():
        try:
            async for d in drone.telemetry.distance_sensor():
                state["rng"]     = d.current_distance_m
                state["rng_min"] = d.minimum_distance_m
                state["rng_max"] = d.maximum_distance_m
        except Exception as e:
            # If plugin/msg not present, just keep RF as None
            pass

    # Writer/Printer
    logdir = os.path.expanduser("~/logs/mav"); os.makedirs(logdir, exist_ok=True)
    fname  = os.path.join(logdir, f"mavsdk_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    print(f"[monitor] logging to {fname}")

    async def emit_loop():
        with open(fname, "w", newline="") as fcsv:
            w = csv.DictWriter(fcsv, fieldnames=CSV_FIELDS)
            w.writeheader()
            while True:
                row = {
                    "ts_iso": datetime.now().isoformat(timespec="seconds"),
                    "mode": state["mode"] or "",
                    "armed": 1 if state["armed"] else 0 if state["armed"] is not None else "",
                    "lat_deg": state["lat"], "lon_deg": state["lon"],
                    "alt_m": state["alt"], "agl_m": state["agl"],
                    "vn": state["vn"], "ve": state["ve"], "vd": state["vd"],
                    "roll_deg": state["roll"], "pitch_deg": state["pitch"], "yaw_deg": state["yaw"],
                    "vbatt_V": state["vbatt"], "bat_rem_pct": state["bat_pct"],
                    "gps_fix": state["gps_fix"], "gps_sats": state["gps_sats"],
                    "range_m": state["rng"], "range_min_m": state["rng_min"], "range_max_m": state["rng_max"],
                }
                w.writerow(row)
                if args.verbose:
                    arm = "ARM" if state["armed"] else "DISARM"
                    print(
                        f"{datetime.now().strftime('%H:%M:%S')} | {row['mode'] or 'UNKNOWN':<8} | {arm:6s}"
                        f" | VBat {f(state['vbatt'],2)}V | SoC {f(state['bat_pct'],1)}%"
                        f" | GPS f{state['gps_fix'] or 0} s{state['gps_sats'] or 0}"
                        f" | Lat {f(state['lat'],7)} | Lon {f(state['lon'],7)} | Alt {f(state['alt'],1)} | AGL {f(state['agl'],1)}"
                        f" | RF {f(state['rng'],2)}"
                        f" | R/P/Y {f(state['roll'],1)}/{f(state['pitch'],1)}/{f(state['yaw'],1)}°"
                    )
                await asyncio.sleep(period)

    tasks = [
        asyncio.create_task(sub_flight_mode()),
        asyncio.create_task(sub_armed()),
        asyncio.create_task(sub_pos()),
        asyncio.create_task(sub_vel()),
        asyncio.create_task(sub_att()),
        asyncio.create_task(sub_bat()),
        asyncio.create_task(sub_gps()),
        asyncio.create_task(sub_range()),
        asyncio.create_task(emit_loop()),
    ]

    try:
        await asyncio.gather(*tasks)
    except asyncio.CancelledError:
        pass
    except KeyboardInterrupt:
        print("\n[monitor] stopping.")
    finally:
        for t in tasks:
            t.cancel()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[monitor] exiting.")

