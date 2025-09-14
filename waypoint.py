#!/usr/bin/env python3
import asyncio
import contextlib
import logging
import math
from mavsdk import System, telemetry

logging.basicConfig(level=logging.INFO)

# ==== CONFIG ====
SERIAL = "serial:///dev/ttyTHS1:230400"
TAKEOFF_AGL_M = 7.0          # takeoff height above home (meters)
CRUISE_AGL_M = 10.0          # nav height above home (meters) - used for goto_location
REACH_HORIZ_M = 3.0          # consider "arrived" if within this horizontal radius (m)
REACH_VERT_M  = 2.0          # and within this vertical difference (m)
LOITER_SEC = 10              # loiter time at each point (seconds)

# Waypoints (decimal degrees). Given lat/longs:
#   43°53'23"N 79°31'38"W  ->  (43.8897222, -79.5272222)
#   43°53'22"N 79°31'40"W  ->  (43.8894444, -79.5277778)
WP1 = (43.889734945503655, -79.52743462479438)
WP2 = (43.88962283129214, -79.52786377823661)

async def wait_connected(drone: System):
    logging.info("Waiting for drone to connect…")
    async for st in drone.core.connection_state():
        if st.is_connected:
            logging.info("-- Connected to drone")
            break

async def wait_global_and_home_ok(drone: System):
    logging.info("Waiting for global position + home position…")
    async for h in drone.telemetry.health():
        if h.is_global_position_ok and h.is_home_position_ok:
            logging.info("-- Global & home position OK")
            break

async def get_home_abs_alt(drone: System) -> float:
    # Home gives absolute_altitude_m (AMSL)
    async for home in drone.telemetry.home():
        return home.absolute_altitude_m

async def status_text_printer(drone: System):
    try:
        async for st in drone.telemetry.status_text():
            print(f"STATUS[{st.type}]: {st.text}")
    except asyncio.CancelledError:
        pass

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    φ1, φ2 = math.radians(lat1), math.radians(lat2)
    dφ = φ2 - φ1
    dλ = math.radians(lon2 - lon1)
    a = math.sin(dφ/2)**2 + math.cos(φ1)*math.cos(φ2)*math.sin(dλ/2)**2
    return 2*R*math.asin(math.sqrt(a))

async def get_current_pos(drone: System):
    async for pos in drone.telemetry.position():
        return pos

async def wait_until_at(drone: System, tgt_lat, tgt_lon, tgt_abs_alt, horiz_m=REACH_HORIZ_M, vert_m=REACH_VERT_M, timeout_s=180):
    """Wait until within horiz/vert thresholds (with a timeout)."""
    deadline = asyncio.get_event_loop().time() + timeout_s
    while True:
        pos = await get_current_pos(drone)
        if pos.absolute_altitude_m is not None:
            horiz = haversine_m(pos.latitude_deg, pos.longitude_deg, tgt_lat, tgt_lon)
            vert = abs((pos.absolute_altitude_m or 0.0) - tgt_abs_alt)
            if horiz <= horiz_m and vert <= vert_m:
                return True
        if asyncio.get_event_loop().time() > deadline:
            return False
        await asyncio.sleep(0.5)

async def wait_for_offboard_trigger(drone: System):
    """Block until flight mode becomes OFFBOARD (trigger)."""
    logging.info("Waiting for OFFBOARD mode to start the sequence…")
    async for fm in drone.telemetry.flight_mode():
        if fm == telemetry.FlightMode.OFFBOARD:
            logging.info("-- OFFBOARD detected: starting sequence")
            return

async def do_sequence(drone: System):
    # Ensure healthy nav state
    await wait_global_and_home_ok(drone)
    home_abs = await get_home_abs_alt(drone)
    cruise_abs = home_abs + CRUISE_AGL_M

    # Set takeoff altitude then arm + take off
    logging.info(f"-- Setting takeoff altitude to {TAKEOFF_AGL_M} m AGL")
    await drone.action.set_takeoff_altitude(TAKEOFF_AGL_M)

    logging.info("-- Arming")
    await drone.action.arm()

    logging.info("-- Taking off")
    await drone.action.takeoff()

    # Wait until we’re reasonably near the takeoff AGL (don’t block too long)
    target_takeoff_abs = home_abs + TAKEOFF_AGL_M
    _ = await wait_until_at(drone, tgt_lat=None, tgt_lon=None, tgt_abs_alt=target_takeoff_abs,
                            horiz_m=999999, vert_m=1.5, timeout_s=45)  # only check altitude

    # Climb/descend to cruise altitude for navigation (if different from takeoff AGL)
    if abs(CRUISE_AGL_M - TAKEOFF_AGL_M) > 0.5:
        logging.info(f"-- Adjusting to cruise altitude {CRUISE_AGL_M} m AGL")
        # Use goto_location to current lat/lon but new absolute altitude
        pos = await get_current_pos(drone)
        await drone.action.goto_location(pos.latitude_deg, pos.longitude_deg, cruise_abs, float('nan'))
        await wait_until_at(drone, pos.latitude_deg, pos.longitude_deg, cruise_abs,
                            horiz_m=5.0, vert_m=1.0, timeout_s=60)

    # === Waypoint 1 ===
    logging.info(f"-- Going to WP1: {WP1[0]:.7f}, {WP1[1]:.7f} @ {CRUISE_AGL_M} m AGL")
    await drone.action.goto_location(WP1[0], WP1[1], cruise_abs, float('nan'))
    ok = await wait_until_at(drone, WP1[0], WP1[1], cruise_abs, REACH_HORIZ_M, REACH_VERT_M, timeout_s=180)
    if not ok:
        logging.warning("!! Timed out reaching WP1 (continuing anyway)")

    logging.info(f"-- Loitering {LOITER_SEC}s at WP1")
    await asyncio.sleep(LOITER_SEC)

    # === Waypoint 2 ===
    logging.info(f"-- Going to WP2: {WP2[0]:.7f}, {WP2[1]:.7f} @ {CRUISE_AGL_M} m AGL")
    await drone.action.goto_location(WP2[0], WP2[1], cruise_abs, float('nan'))
    ok = await wait_until_at(drone, WP2[0], WP2[1], cruise_abs, REACH_HORIZ_M, REACH_VERT_M, timeout_s=180)
    if not ok:
        logging.warning("!! Timed out reaching WP2 (continuing anyway)")

    logging.info(f"-- Loitering {LOITER_SEC}s at WP2")
    await asyncio.sleep(LOITER_SEC)

    # === Return to Launch ===
    logging.info("-- RTL")
    await drone.action.return_to_launch()

async def main():
    drone = System()
    await drone.connect(system_address=SERIAL)

    await wait_connected(drone)

    # Start status text printer
    status_task = asyncio.create_task(status_text_printer(drone))

    try:
        # Wait for OFFBOARD flip to trigger the whole sequence
        await wait_for_offboard_trigger(drone)
        await do_sequence(drone)

        # Keep the connection alive until disarmed or user stops
        logging.info("Sequence complete. Staying connected (Ctrl-C to exit).")
        while True:
            await asyncio.sleep(1)
    finally:
        status_task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await status_task

if __name__ == "__main__":
    asyncio.run(main())
