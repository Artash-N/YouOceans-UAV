#!/usr/bin/env python3
import asyncio
from mavsdk import System, telemetry

SERIAL = "serial:///dev/ttyTHS1:230400"

async def print_status_text(drone: System):
    try:
        async for st in drone.telemetry.status_text():
            print(f"STATUS[{st.type}]: {st.text}")
    except asyncio.CancelledError:
        pass

async def main():
    drone = System()
    await drone.connect(system_address=SERIAL)

    # Wait for connection
    print("Waiting for drone to connect...")
    async for s in drone.core.connection_state():
        if s.is_connected:
            print("-- Connected")
            break

    # Minimal health (only sensor calibrations so arming is allowed)
    print("-- Waiting for basic sensor health…")
    async for h in drone.telemetry.health():
        if (h.is_gyrometer_calibration_ok and
            h.is_accelerometer_calibration_ok and
            h.is_magnetometer_calibration_ok):
            print("-- Sensors OK")
            break

    # Start status text reader
    status_task = asyncio.create_task(print_status_text(drone))

    try:
        print("-- Arming (idle)")
        await drone.action.arm()

        print("-- Holding idle for 5s")
        await asyncio.sleep(5)

        print("-- Disarming")
        await drone.action.disarm()
        print("-- Done")
    finally:
        status_task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await status_task

if __name__ == "__main__":
    import contextlib
    asyncio.run(main())

