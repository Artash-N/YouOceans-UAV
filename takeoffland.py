#!/usr/bin/env python3
import asyncio
import logging
import contextlib
from mavsdk import System
from mavsdk.server_utility import StatusTextType

logging.basicConfig(level=logging.INFO)

SERIAL = "serial:///dev/ttyTHS1:230400"
TAKEOFF_ALT_M = 5.0
ALT_TOL_M = 0.7
REACH_TIMEOUT_S = 45
HOVER_SEC = 5

async def say(drone: System, text: str, level: StatusTextType = StatusTextType.INFO):
    await drone.server_utility.send_status_text(level, text)

async def wait_connected(drone: System):
    print("Waiting for drone to connect…")
    async for s in drone.core.connection_state():
        if s.is_connected:
            print("-- Connected")
            return

async def wait_global_and_home_ok(drone: System):
    print("Waiting for global & home position…")
    async for h in drone.telemetry.health():
        if h.is_global_position_ok and h.is_home_position_ok:
            print("-- Global & home OK")
            return

async def get_home_abs_alt(drone: System) -> float:
    async for home in drone.telemetry.home():
        return home.absolute_altitude_m

async def wait_until_alt_reached(drone: System, target_abs_alt: float,
                                 tol_m: float = ALT_TOL_M, timeout_s: int = REACH_TIMEOUT_S) -> bool:
    deadline = asyncio.get_event_loop().time() + timeout_s
    async for pos in drone.telemetry.position():
        if pos.absolute_altitude_m is not None:
            if abs(pos.absolute_altitude_m - target_abs_alt) <= tol_m:
                return True
        if asyncio.get_event_loop().time() > deadline:
            return False

async def status_text_printer(drone: System):
    try:
        async for st in drone.telemetry.status_text():
            print(f"PX4[{st.type}]: {st.text}")
    except asyncio.CancelledError:
        pass

async def main():
    drone = System()
    await drone.connect(system_address=SERIAL)

    await wait_connected(drone)
    await wait_global_and_home_ok(drone)

    printer = asyncio.create_task(status_text_printer(drone))
    try:
        # Precompute target absolute altitude (AGL -> AMSL using home.alt)
        home_abs = await get_home_abs_alt(drone)
        target_abs = home_abs + TAKEOFF_ALT_M

        await say(drone, "Companion: auto takeoff sequence")

        await say(drone, "Arming…")
        await drone.action.arm()
        await say(drone, "Armed")

        await drone.action.set_takeoff_altitude(TAKEOFF_ALT_M)
        await say(drone, f"Takeoff to {TAKEOFF_ALT_M:.1f} m")
        await drone.action.takeoff()

        reached = await wait_until_alt_reached(drone, target_abs, tol_m=ALT_TOL_M, timeout_s=REACH_TIMEOUT_S)
        if reached:
            await say(drone, f"Reached {TAKEOFF_ALT_M:.1f} m, hover {HOVER_SEC}s")
        else:
            await say(drone, "Warn: 5 m not reached in time", StatusTextType.WARNING)

        await asyncio.sleep(HOVER_SEC)

        await say(drone, "Landing…")
        await drone.action.land()

        async for armed in drone.telemetry.armed():
            if not armed:
                await say(drone, "Landed & disarmed")
                print("-- Landed & disarmed")
                break

    finally:
        printer.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await printer

if __name__ == "__main__":
    asyncio.run(main())
