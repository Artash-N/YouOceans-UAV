#!/usr/bin/env python3
import asyncio, argparse, json, time
from mavsdk import System
from mavsdk.telemetry import TelemetryError

def j(tag, obj):
    def to_dict(x):
        if hasattr(x, "__dict__"):
            return {k: v for k, v in x.__dict__.items()}
        return x
    try:
        print(json.dumps({"t": time.strftime("%H:%M:%S"), "tag": tag, "data": to_dict(obj)}, default=str))
    except Exception as e:
        print(json.dumps({"t": time.strftime("%H:%M:%S"), "tag": tag, "error": str(e)}))

async def get_one(aiter, timeout=0.5):
    try:
        return await asyncio.wait_for(anext(aiter), timeout=timeout)
    except Exception:
        return None

async def set_rates(drone, hz=1.0):
    # Best-effort: some rates may not be supported; ignore those errors.
    for setter in (
        drone.telemetry.set_rate_battery,
        drone.telemetry.set_rate_gps_info,
        drone.telemetry.set_rate_position,
        drone.telemetry.set_rate_attitude_euler,
        drone.telemetry.set_rate_distance_sensor,
        drone.telemetry.set_rate_landed_state,
    ):
        try:
            await setter(hz)
        except TelemetryError:
            pass

async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyTHS1")
    ap.add_argument("--baud", type=int, default=230400)
    ap.add_argument("--hz", type=float, default=1.0)
    args = ap.parse_args()

    drone = System()
    await drone.connect(system_address=f"serial:///{args.port}:{args.baud}")

    # Wait until connected
    async for s in drone.core.connection_state():
        if s.is_connected:
            print("✅ connected")
            break

    # Low rates so the link/server isn’t stressed
    await set_rates(drone, hz=args.hz)

    period = 1.0 / max(0.1, args.hz)
    try:
        while True:
            # Pull ONE item per topic (short-lived iterators), each with a small timeout
            batt   = await get_one(drone.telemetry.battery(),           timeout=0.4)
            gps    = await get_one(drone.telemetry.gps_info(),          timeout=0.4)
            pos    = await get_one(drone.telemetry.position(),          timeout=0.4)
            att    = await get_one(drone.telemetry.attitude_euler(),    timeout=0.4)
            dist   = await get_one(drone.telemetry.distance_sensor(),   timeout=0.4)
            fmode  = await get_one(drone.telemetry.flight_mode(),       timeout=0.4)
            armed  = await get_one(drone.telemetry.armed(),             timeout=0.2)
            lstate = await get_one(drone.telemetry.landed_state(),      timeout=0.2)

            if fmode is not None: j("flight_mode", fmode)
            if armed is not None: j("armed", armed)
            if lstate is not None: j("landed_state", lstate)
            if batt is not None:   j("battery", batt)
            if gps is not None:    j("gps_info", gps)
            if pos is not None:    j("position", pos)
            if att is not None:    j("attitude_euler", att)
            if dist is not None:   j("distance_sensor", dist)

            await asyncio.sleep(period)
    except asyncio.CancelledError:
        pass

if __name__ == "__main__":
    asyncio.run(main())

