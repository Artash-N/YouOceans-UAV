#!/usr/bin/env python3
import asyncio
import logging
import sys
import contextlib
from mavsdk import System
from grpc.aio import AioRpcError

logging.basicConfig(level=logging.INFO)
SERIAL = "serial:///dev/ttyTHS1:230400"


async def print_status_text(drone: System):
    try:
        async for st in drone.telemetry.status_text():
            print(f"STATUS[{st.type}]: {st.text}")
    except (asyncio.CancelledError, AioRpcError):
        pass


async def main():
    drone = System()
    status_task = None
    try:
        await drone.connect(system_address=SERIAL)
        print("Waiting for drone to connect...")
        async for s in drone.core.connection_state():
            if s.is_connected:
                print("-- Connected")
                break
        else:
            raise TimeoutError("Drone failed to connect")

        print("-- Waiting for basic sensor health…")
        async for h in drone.telemetry.health():
            if (h.is_gyrometer_calibration_ok and
                h.is_accelerometer_calibration_ok and
                h.is_magnetometer_calibration_ok):
                print("-- Sensors OK")
                break

        status_task = asyncio.create_task(print_status_text(drone))

        print("-- Arming (idle)")
        await drone.action.arm()

        print("-- Holding idle for 5s")
        await asyncio.sleep(5)

        print("-- Disarming")
        await drone.action.disarm()
        print("-- Done")

    except AioRpcError as e:
        logging.error(f"MAVSDK RPC Error: {e.details() or e}")
        sys.exit(2)
    except Exception as e:
        logging.error(f"Unexpected error: {e}")
        sys.exit(1)
    finally:
        # Cancel and await background task
        if status_task:
            status_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await status_task

        # Try closing MAVSDK connection cleanly
        with contextlib.suppress(Exception):
            await drone.close()

        # Ensure all tasks stop and event loop exits
        await asyncio.sleep(0.1)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nAborted by user.")
        sys.exit(130)
    except AioRpcError as e:
        logging.error(f"RPC Fatal error: {e.details() or e}")
        sys.exit(2)
    except Exception as e:
        logging.error(f"Fatal error: {e}")
        sys.exit(1)
