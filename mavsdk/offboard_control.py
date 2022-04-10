#!/usr/bin/env python3

"""
Caveat when attempting to run the examples in non-gps environments:

`drone.offboard.stop()` will return a `COMMAND_DENIED` result because it
requires a mode switch to HOLD, something that is currently not supported in a
non-gps environment.
"""

from argparse import Action
import asyncio
import mavsdk
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

import pymap3d as pm

async def run():
    """ Does Offboard control using position NED coordinates. """

    drone = System()
    await drone.connect(system_address="udp://:14550")

    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # The local coordinate origin 
    lat0 = 40.544289 # deg
    lon0 = -4.012101  # deg
    h0 = 0     # meters

    # The point of interest
    lat = 40.545007  # deg
    lon = -4.013090  # deg
    h = 3      # meters

    pos = pm.geodetic2enu(lat0, lon0, h0, lat, lon, h)
    print("-- Go to ",pos,"in NED coordinates")
    await drone.offboard.set_position_ned(PositionNedYaw(pos[0],pos[1],pos[2], 0.0))
    await asyncio.sleep(50)
    print("avance")
    await drone.offboard.set_position_ned(PositionNedYaw(pos[0],50,pos[2], 0.0))
    #await drone.action.do_orbit(5,1,mavsdk.action.OrbitYawBehavior(0),40.545003, -4.013085,3)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
