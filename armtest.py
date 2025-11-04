#!/usr/bin/env python3
import asyncio
import subprocess
import sys
import time
import contextlib
from mavsdk import System, action

SERIAL = "serial:///dev/ttyTHS1:230400"
MAVSDK_PORT = 50051
MAVSDK_SERVER_CMD = ["mavsdk_server", "-p", str(MAVSDK_PORT), SERIAL]
CONNECT_TIMEOUT = 15  # seconds

async def print_status_text(drone: System):
    try:
        async for st in drone.telemetry.status_text():
            print(f"STATUS[{st.type}]: {st.text}")
    except asyncio.CancelledError:
        return

async def main():
    # start mavsdk_server (standalone)
    srv = subprocess.Popen(MAVSDK_SERVER_CMD, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        drone = System(mavsdk_server_address="localhost", port=MAVSDK_PORT)
        # wait for server/drone connect with timeout
        t0 = time.time()
        await drone.connect()  # this connects to the running mavsdk_server
        # wait until a system is discovered and connected
        connected = False
        async for cs in drone.core.connection_state():
            if cs.is_connected:
                print("-- Connected")
                connected = True
                break
            if time.time() - t0 > CONNECT_TIMEOUT:
                print("ERROR: timeout waiting for system connection", file=sys.stderr)
                return 1

        if not connected:
            print("ERROR: not connected", file=sys.stderr)
            return 1

        # wait for minimal sensor health
        print("-- Waiting for basic sensor health…")
        t0 = time.time()
        async for h in drone.telemetry.health():
            if (h.is_gyrometer_calibration_ok and
                h.is_accelerometer_calibration_ok and
                h.is_magnetometer_calibration_ok):
                print("-- Sensors OK")
                break
            if time.time() - t0 > CONNECT_TIMEOUT:
                print("ERROR: timeout waiting for sensors OK", file=sys.stderr)
                return 1

        # start status reader
        status_task = asyncio.create_task(print_status_text(drone))

        # Best-effort: try to set altitude-control style mode.
        # Try mavlink_passthrough (COMMAND_LONG -> MAV_CMD_DO_SET_MODE), fallback to action.hold()
        try:
            mp = drone.mavlink_passthrough
            # Build CommandLong: command=176 (MAV_CMD_DO_SET_MODE), param1=1 (custom), param2=2 (ALTCTL)
            # If the plugin fields/classes are exposed, use them; otherwise attribute access will fail.
            cmd = mp.CommandLong(
                target_sysid=1,
                target_compid=1,
                command=176,
                param1=1.0,
                param2=2.0,
                param3=0.0,
                param4=0.0,
                param5=0.0,
                param6=0.0,
                param7=0.0,
            )
            res = await mp.send_command_long(cmd)
            # send_command_long returns a result object in C++ bindings; in python it may raise/return - handle best-effort
            try:
                ok = (res == mp.Result.SUCCESS)  # if enum available
            except Exception:
                ok = True  # assume best-effort success if no enum
            if ok:
                print("-- Requested ALTCTL via passthrough")
        except Exception:
            # fallback to hold (PX4) — best-effort, may raise on ArduPilot but we ignore the failure
            try:
                await drone.action.hold()
                print("-- Requested hold() fallback")
            except Exception:
                print("-- Mode change attempt failed; continuing (will try arm)")

        # Arm
        try:
            print("-- Arming (idle)")
            await drone.action.arm()
        except Exception as ex:
            # COMMAND_DENIED and RPC socket errors are surfaced as exceptions
            print(f"ERROR: arm failed: {ex}", file=sys.stderr)
            # cleanup and return specific code for arm failure
            status_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await status_task
            return 2

        print("-- Holding idle for 5s")
        await asyncio.sleep(5)

        print("-- Disarming")
        await drone.action.disarm()
        print("-- Done")

        status_task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await status_task

        return 0

    except Exception as e:
        print(f"ERROR: unexpected: {e}", file=sys.stderr)
        return 3
    finally:
        # ensure mavsdk_server is killed
        with contextlib.suppress(Exception):
            srv.kill()
            srv.wait(timeout=2)

if __name__ == "__main__":
    try:
        code = asyncio.run(main())
        if isinstance(code, int):
            sys.exit(code)
        else:
            sys.exit(0)
    except KeyboardInterrupt:
        sys.exit(1)
