#!/usr/bin/env python3

import asyncio
from mavsdk import System
import mavsdk

async def run():

    drone = System()
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
    #await drone.action.goto_location(40.54490101775458, -4.0131660745683435, 5, 0)
    await drone.action.do_orbit(radius_m=5, velocity_ms=2,yaw_behavior=mavsdk.action.OrbitYawBehavior(0), latitude_deg=40.54490101775458, longitude_deg=-4.0131660745683435, absolute_altitude_m=2)

    #await asyncio.sleep(5)

    #print("-- Landing")
    #await drone.action.land()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
