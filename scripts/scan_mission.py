#!/usr/bin/env python3

import asyncio
import mavsdk
import scan
import numpy as np

async def run(lat,lon):


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

    print("-- Go to location",lat,lon)
    await drone.action.goto_location(lat,lon,flying_alt,0)
    lat_range = 0.001
    lon_range = 0.000001
    async for state in drone.telemetry.position():
        if np.abs(lat)-lat_range<= np.abs(state.latitude_deg) <= np.abs(lat)+lat_range and np.abs(lon)-lon_range <= np.abs(state.longitude_deg) <= np.abs(lon)+lon_range:
            print(f"Location reached")
            break

    print("-- Scan area")
    area_radius = 5
    scan_speed = 0.5
    time_one_orbit = area_radius*2*np.pi/scan_speed
    n_orbits=1
    recognition_time = time_one_orbit*n_orbits
    await drone.action.do_orbit(area_radius,scan_speed,mavsdk.action.OrbitYawBehavior(0),lat, lon,flying_alt)
    await asyncio.sleep(area_radius*2)
    await lidar.make_scan(2,recognition_time)

    print("-- Landing")
    await drone.action.return_to_launch()

if __name__ == "__main__":
    lidar = scan.Lidar("Drone","Lidar")
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(40.545000,-4.013086))
