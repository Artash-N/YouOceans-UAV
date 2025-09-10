#!/usr/bin/env python3
# monitor_mavsdk.py — lean 1 Hz monitor for PX4 via MAVSDK 3.10.x

import asyncio, argparse, os, csv, math
from datetime import datetime, timezone
from mavsdk import System
from grpc.aio import AioRpcError

DEFAULT_PORT = "/dev/ttyTHS1"
DEFAULT_BAUD = 230400
DEFAULT_HZ   = 1.0

CSV_FIELDS = [
    "ts_iso",
    "mode", "armed",
    "vbatt_V", "bat_rem_pct",
    "gps_fix", "gps_sats",
    "lat_deg", "lon_deg", "alt_m",
    "rf_m",  # rangefinder / distance sensor (down) if available
    "roll_deg", "pitch_deg", "yaw_deg",
]

def now_iso():
    return datetime.now(timezone.utc).astimezone().replace(tzinfo=None).isoformat(timespec="seconds")

def fmt(v, nd=1):
    return "-" if v is None else f"{v:.{nd}f}"

async def connect(system_address: str) -> System:
    drone = System()
    await drone.connect(system_address=system_address)
    # Wait for connection
    async for s in drone.core.connection_state():
        if s.is_connected:
            print("✅ Connected")
            break
    return drone

async def set_rates(drone: System, hz: float):
    """Clamp high-rate streams down to user rate to avoid queue backlogs."""
    try:
        await drone.telemetry.set_rate_attitude_euler(hz)
        await drone.telemetry.set_rate_position(hz)
        await drone.telemetry.set_rate_gps_info(hz)
        await drone.telemetry.set_rate_battery(hz)
        # Optional: enable if you actually read these
        # await drone.telemetry.set_rate_altitude(hz)
        # await drone.telemetry.set_rate_velocity_ned(hz)
        # await drone.telemetry.set_rate_distance_sensor(hz)
    except Exception as e:
        print(f"[monitor] rate-set warning: {e} (continuing)")

async def sub_mode_armed(drone: System, state: dict):
    try:
        async for fm in drone.telemetry.flight_mode():
            state["mode"] = str(fm.name) if hasattr(fm, "name") else str(fm)
    except AioRpcError:
        return
    except asyncio.CancelledError:
        return

async def sub_armed(drone: System, state: dict):
    try:
        async for a in drone.telemetry.armed():
            state["armed"] = bool(a)
    except AioRpcError:
        return
    except asyncio.CancelledError:
        return

async def sub_battery(drone: System, state: dict):
    try:
        async for b in drone.telemetry.battery():
            # remaining_percent documented as 0..100
            state["vbatt_V"] = float(b.voltage_v) if b.voltage_v == b.voltage_v else None
            rp = b.remaining_percent
            state["bat_rem_pct"] = float(rp) if rp == rp else None
    except AioRpcError:
        return
    except asyncio.CancelledError:
        return

async def sub_gps(drone: System, state: dict):
    try:
        async for g in drone.telemetry.gps_info():
            # FixType is Enum → use .value for numeric, .name for label
            state["gps_fix"] = int(getattr(g.fix_type, "value", 0))
            state["gps_sats"] = int(getattr(g, "num_satellites", 0))
    except AioRpcError:
        return
    except asyncio.CancelledError:
        return

async def sub_position(drone: System, state: dict):
    try:
        async for p in drone.telemetry.position():
            # If GPS not valid, fields may be NaN → guard with isnan
            lat = p.latitude_deg
            lon = p.longitude_deg
            alt = p.absolute_altitude_m
            state["lat"] = None if lat != lat else float(lat)
            state["lon"] = None if lon != lon else float(lon)
            state["alt"] = None if alt != alt else float(alt)
    except AioRpcError:
        return
    except asyncio.CancelledError:
        return

async def sub_attitude(drone: System, state: dict):
    try:
        async for a in drone.telemetry.attitude_euler():
            state["roll"]  = float(a.roll_deg)   if a.roll_deg == a.roll_deg   else None
            state["pitch"] = float(a.pitch_deg)  if a.pitch_deg == a.pitch_deg else None
            state["yaw"]   = float(a.yaw_deg)    if a.yaw_deg == a.yaw_deg     else None
    except AioRpcError:
        return
    except asyncio.CancelledError:
        return

async def sub_rangefinder(drone: System, state: dict):
    """Distance sensor (if available). PX4 typically reports downwards range as current_distance_m."""
    try:
        # Only set rate if you actually subscribe
        await drone.telemetry.set_rate_distance_sensor(1.0)
    except Exception:
        pass
    try:
        async for d in drone.telemetry.distance_sensor():
            dist = d.current_distance_m
            state["rf_m"] = None if dist != dist else float(dist)
    except AioRpcError:
        return
    except asyncio.CancelledError:
        return

async def printer(state: dict, hz: float, writer, verbose: bool):
    period = 1.0 / max(0.1, hz)
    while True:
        try:
            ts = now_iso()
            mode = state.get("mode", "UNKNOWN")
            armed = "ARM" if state.get("armed", False) else "DISARM"
            vb = state.get("vbatt_V")
            soc = state.get("bat_rem_pct")
            fix = state.get("gps_fix", 0)
            sats = state.get("gps_sats", 0)
            lat = state.get("lat")
            lon = state.get("lon")
            alt = state.get("alt")
            rf  = state.get("rf_m")
            roll  = state.get("roll")
            pitch = state.get("pitch")
            yaw   = state.get("yaw")

            # CSV row
            writer.writerow({
                "ts_iso": ts,
                "mode": mode,
                "armed": 1 if armed == "ARM" else 0,
                "vbatt_V": vb,
                "bat_rem_pct": soc,
                "gps_fix": fix,
                "gps_sats": sats,
                "lat_deg": lat, "lon_deg": lon, "alt_m": alt,
                "rf_m": rf,
                "roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw,
            })

            if verbose:
                print(
                    f"{datetime.now().strftime('%H:%M:%S')} | {mode:<7} | {armed:6s}"
                    f" | VBat {fmt(vb,2)}V | SoC {fmt(soc,0)}%"
                    f" | GPS f{fix} s{sats}"
                    f" | Lat {('-' if lat is None else f'{lat:.7f}')}"
                    f" | Lon {('-' if lon is None else f'{lon:.7f}')}"
                    f" | Alt {fmt(alt,1)}"
                    f" | RF {fmt(rf,2)}"
                    f" | R/P/Y {fmt(roll,1)}/{fmt(pitch,1)}/{fmt(yaw,1)}°"
                )
            await asyncio.sleep(period)
        except asyncio.CancelledError:
            return

async def main():
    ap = argparse.ArgumentParser(description="Lean MAVSDK monitor (1 Hz).")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--hz",   type=float, default=DEFAULT_HZ)
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    system_address = f"serial:///{args.port}:{args.baud}"
    print("[monitor] waiting for connection…")

    # Prepare log file
    logdir = os.path.expanduser("~/logs/mav")
    os.makedirs(logdir, exist_ok=True)
    fname  = os.path.join(logdir, f"mavsdk_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    print("✅ Connected" if False else "", end="")  # placeholder so order matches earlier runs

    while True:
        try:
            drone = await connect(system_address)
            print(f"[monitor] logging to {fname}")
            await set_rates(drone, args.hz)

            # Shared state
            state = {}

            # Open CSV and run tasks
            with open(fname, "w", newline="") as fcsv:
                writer = csv.DictWriter(fcsv, fieldnames=CSV_FIELDS)
                writer.writeheader()

                tasks = [
                    asyncio.create_task(sub_mode_armed(drone, state)),
                    asyncio.create_task(sub_armed(drone, state)),
                    asyncio.create_task(sub_battery(drone, state)),
                    asyncio.create_task(sub_gps(drone, state)),
                    asyncio.create_task(sub_position(drone, state)),
                    asyncio.create_task(sub_attitude(drone, state)),
                    asyncio.create_task(sub_rangefinder(drone, state)),  # safe if none present
                    asyncio.create_task(printer(state, args.hz, writer, args.verbose)),
                ]

                # Wait here until a task fails or Ctrl-C
                done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION)
                # If we reach here, either Ctrl-C (CancelledError) or a stream died
                for t in pending:
                    t.cancel()
                for t in done:
                    exc = t.exception()
                    if exc and not isinstance(exc, asyncio.CancelledError):
                        print(f"[monitor] stream stopped: {exc}")

        except KeyboardInterrupt:
            print("\n[monitor] stopping.")
            return
        except Exception as e:
            print(f"[monitor] reconnecting after error: {e}")
            await asyncio.sleep(2.0)  # brief backoff and try again

if __name__ == "__main__":
    asyncio.run(main())

