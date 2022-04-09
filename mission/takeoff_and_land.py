#!/usr/bin/env python3

import asyncio
import mavsdk


async def run():
    drone = mavsdk.System()
    await drone.connect(system_address="udp://:14550")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Global position estimate ok")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(5)

    print("-- Do Action")
    await drone.action.goto_location(40.545148, -4.012101, 3, 0)
    await asyncio.sleep(5)
    #await drone.action.do_orbit(5,1,mavsdk.action.OrbitYawBehavior(0),40.545003, -4.013085,3)
    #await asyncio.sleep(5)

    print("-- Landing")
    #await drone.action.return_to_launch()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())

