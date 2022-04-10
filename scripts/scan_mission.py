#!/usr/bin/env python3

import asyncio
import mavsdk
import scan
import numpy as np

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

    print("-- Go to location")
    await drone.action.goto_location(40.545000, -4.013086,flying_alt,0)
    await asyncio.sleep(50)

    print("-- Scan area")
    area_radius = 5
    scan_speed = 0.5
    time_one_orbit = area_radius*2*np.pi/scan_speed
    n_orbits=1
    recognition_time = time_one_orbit*n_orbits+area_radius
    await drone.action.do_orbit(area_radius,scan_speed,mavsdk.action.OrbitYawBehavior(0),40.545000, -4.013086,flying_alt)
    await asyncio.sleep(area_radius*2)
    await lidar.execute(1.5,recognition_time)

    print("-- Landing")
    await drone.action.return_to_launch()

if __name__ == "__main__":
    lidar = scan.Lidar("Drone","Lidar")
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
