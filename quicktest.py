import asyncio
from mavsdk import System

SERIAL = "serial:///dev/ttyTHS1:115200"  # adjust if needed

async def main():
    drone = System()
    await drone.connect(system_address=SERIAL)

    print("⏳ Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Connected to system")
            break

    # Fetch UUID explicitly
    uuid = await drone.info.get_uuid()
    print("System UUID:", uuid.uuid)

    # Print one health snapshot
    async for h in drone.telemetry.health():
        print("Health:", h)
        break

    # Try to read one distance sensor sample (if available)
    try:
        async for ds in drone.telemetry.distance_sensor():
            print(
                f"DistanceSensor: {ds.current_distance_m:.2f} m "
                f"(min {ds.minimum_distance_m:.2f}, max {ds.maximum_distance_m:.2f})"
            )
            break
    except Exception as e:
        print("No distance_sensor stream (or plugin unsupported):", e)

if __name__ == "__main__":
    asyncio.run(main())

