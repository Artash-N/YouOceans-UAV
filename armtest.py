#!/usr/bin/env python3
import asyncio
import logging
import sys
import contextlib
import grpc
from grpc import aio
from mavsdk import System

logging.basicConfig(level=logging.INFO)
SERIAL = "serial:///dev/ttyTHS1:230400"

async def wait_for_connection(drone):
    async for s in drone.core.connection_state():
        if s.is_connected:
            logging.info("✅ Drone connected")
            return True
    return False

async def wait_for_health(drone):
    async for h in drone.telemetry.health():
        if h.is_gyrometer_calibration_ok and h.is_accelerometer_calibration_ok and h.is_magnetometer_calibration_ok:
            logging.info("✅ Sensors OK")
            return True
    return False

async def arm_once(drone):
    logging.info("🦾 Attempting to arm")
    await drone.action.arm()
    logging.info("✅ Armed")
    await asyncio.sleep(3)
    logging.info("🧊 Disarming")
    await drone.action.disarm()

async def reconnect_system(drone, retries=3):
    for i in range(retries):
        try:
            logging.info(f"🔌 Connecting (attempt {i+1})")
            await drone.connect(system_address=SERIAL)
            ok = await asyncio.wait_for(wait_for_connection(drone), timeout=10)
            if ok:
                return True
        except Exception as e:
            logging.warning(f"Reconnect failed: {e}")
        await asyncio.sleep(2)
    return False

async def main():
    drone = System()
    try:
        if not await reconnect_system(drone):
            logging.error("Could not connect to drone.")
            sys.exit(1)

        await wait_for_health(drone)

        try:
            await arm_once(drone)
        except aio.AioRpcError as e:
            if e.code() == grpc.StatusCode.UNAVAILABLE:  # ✅ Correct namespace
                logging.warning("⚠️ Link dropped mid-command (Socket closed). Reconnecting...")
                if await reconnect_system(drone):
                    await wait_for_health(drone)
                    await arm_once(drone)
                else:
                    logging.error("❌ Reconnect attempts failed.")
                    sys.exit(2)
            else:
                raise
        logging.info("✅ Done cleanly")

    except Exception as e:
        logging.exception(f"Fatal error: {e}")
        sys.exit(99)
    finally:
        with contextlib.suppress(asyncio.CancelledError):
            for t in asyncio.all_tasks():
                if t is not asyncio.current_task():
                    t.cancel()
        logging.info("🧹 Cleanup complete.")

if __name__ == "__main__":
    asyncio.run(main())
