import os
import scan
import mavsdk
import asyncio
import numpy as np
import pointcloud_processing

async def run(origin,target):

    #create mission folder
    airsim_path = os.path.join(os.path.expanduser('~'), 'Documents', 'AirSim')
    n_missions = os.listdir(os.path.join(airsim_path,"data"))
    if n_missions:
        mission_path = os.path.join(airsim_path,"data","mission_"+str(len(n_missions)))
        os.mkdir(mission_path)
    else:
        mission_path = os.path.join(airsim_path,"data","mission_0")
        os.mkdir(mission_path)

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
    await drone.action.set_takeoff_altitude(3)
    await drone.action.set_return_to_launch_altitude(3)
    await drone.action.takeoff()

    await asyncio.sleep(15)

    print("-- Go to location",target[0],target[1])
    flying_alt = absolute_altitude+10
    await drone.action.goto_location(target[0],target[1],flying_alt,0)
    cord_range = 0.000001
    async for state in drone.telemetry.position():
        if np.abs(target[0])-cord_range<= np.abs(state.latitude_deg) <= np.abs(target[0])+cord_range and np.abs(target[1])-cord_range <= np.abs(state.longitude_deg) <= np.abs(target[1])+cord_range:
            print(f"Location reached")
            break

    print("-- Scan area")
    area_radius = 30
    scan_speed = 5
    time_one_orbit = area_radius*2*np.pi/scan_speed
    n_orbits=1
    recognition_time = time_one_orbit*n_orbits
    await drone.action.do_orbit(area_radius,scan_speed,mavsdk.action.OrbitYawBehavior(0),target[0],target[1],flying_alt)
    await asyncio.sleep(area_radius)

    #create scan folder
    scan_path = os.path.join(mission_path,"recognition")
    os.mkdir(scan_path)

    await lidar.make_scan(0.5,recognition_time,scan_path)

    print("-- Detecting objets")
    await pointcloud_processing.recognition(mission_path,origin)

    objects_dirs = os.listdir(os.path.join(mission_path))
    objects_dirs.pop()

    print("-- objects detected:",len(objects_dirs))
    print("-- Starting individual scannings")

    for i in range(len(objects_dirs)):
        object_dir = os.path.join(mission_path,objects_dirs[i])
        file = open(os.path.join(object_dir,"object_data.txt"), "r")
        lines = file.readlines()
        object_lat = float(lines[1])
        object_lon = float(lines[2])
        altura = float(lines[3])

        area_radius = 5
        scan_speed = 1
        time_one_orbit = area_radius*2*np.pi/scan_speed
        n_orbits=1
        recognition_time = time_one_orbit*n_orbits
        flying_alt = absolute_altitude + altura
        
        print("-- Go to location of object",i,object_lat,object_lon)
        await drone.action.goto_location(object_lat,object_lon,flying_alt,0)
        cord_range = 0.0001
        async for state in drone.telemetry.position():
            if np.abs(object_lat)-cord_range<= np.abs(state.latitude_deg) <= np.abs(object_lat)+cord_range and np.abs(object_lon)-cord_range <= np.abs(state.longitude_deg) <= np.abs(object_lon)+cord_range:
                print(f"Location reached")
                break
        
        print("-- Scanning object",i)
        await drone.action.do_orbit(area_radius,scan_speed,mavsdk.action.OrbitYawBehavior(0),object_lat,object_lon,flying_alt)
        await asyncio.sleep(area_radius)
        await lidar.make_scan(0.5,recognition_time,object_dir)

    print("-- Landing")
    await drone.action.return_to_launch()

    print("-- Starting objects processing")
    for i in range(len(objects_dirs)):
        print("-- Processing object",i)
        pointcloud_processing.processing(os.path.join(mission_path,objects_dirs[i]))

if __name__ == "__main__":
    lidar = scan.Lidar("Drone","Lidar")
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run((40.544289,-4.012101),(40.544729,-4.012503)))
