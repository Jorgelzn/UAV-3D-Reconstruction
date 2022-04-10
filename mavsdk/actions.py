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

    print("Fetching amsl altitude at home location....")
    async for terrain_info in drone.telemetry.home():
        absolute_altitude = terrain_info.absolute_altitude_m
        break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await  drone.action.set_takeoff_altitude(3)
    await drone.action.set_return_to_launch_altitude(3)
    await drone.action.takeoff()

    await asyncio.sleep(20)

    flying_alt = absolute_altitude + 3

    print("-- Scan area")
    await drone.action.do_orbit(5,0.5,mavsdk.action.OrbitYawBehavior(0),40.545000, -4.013086,flying_alt)
    await asyncio.sleep(150)

    print("-- Landing")
    await drone.action.return_to_launch()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())

