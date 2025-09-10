#!/home/jetson/mavsdk-env/bin/python

import asyncio
from mavsdk import System
SERIAL = "serial:///dev/ttyTHS1:115200"

async def run():
    # Connect to the drone
    drone = System()
    await drone.connect(system_address=SERIAL)

    # Get the list of parameters
    all_params = await drone.param.get_all_params()

    # Iterate through all int parameters
    for param in all_params.int_params:
        print(f"{param.name}: {param.value}")

    for param in all_params.float_params:
        print(f"{param.name}: {param.value}")


# Run the asyncio loop
asyncio.run(run())
