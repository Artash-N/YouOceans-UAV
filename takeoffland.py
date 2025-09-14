#!/usr/bin/env python3
import asyncio
import logging
import contextlib
from mavsdk import System
from mavsdk.server_utility import StatusTextType

logging.basicConfig(level=logging.INFO)

SERIAL = "serial:///dev/ttyTHS1:230400"
TAKEOFF_ALT_M = 5.0

async def say(drone, text, level=StatusTextType.INFO):
    await drone.server_utility.send_status_text(level, text)

async def wait_connected(drone: System):
    async for s in drone.core.connection_state():
        if s.is_connected:
            return

async def wait_global_position(drone: System):
    async for h in drone.telemetry.health():
        if h.is_global_position_ok and h.is_home_position_ok:
            return

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
    await wait_global_position(drone)

    printer = asyncio.create_task(status_text_printer(drone))
    try:
        await say(drone, "Companion: starting auto takeoff")

        await say(drone, "Arming…")
        await drone.action.arm()
        await say(drone, "Armed")

        await drone.action.set_takeoff_altitude(TAKEOFF_ALT_M)
        await say(drone, f"Takeoff to {TAKEOFF_ALT_M} m")
        await drone.action.takeoff()

        await asyncio.sleep(5)  # brief hover
        await say(drone, "Landing…")
        await drone.action.land()

        # Wait for disarm confirmation from PX4
        async for armed in drone.telemetry.armed():
            if not armed:
                await say(drone, "Landed & disarmed")
                break
    finally:
        printer.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await printer

if __name__ == "__main__":
    asyncio.run(main())
