import os
import scan
import mavsdk
import asyncio
import numpy as np
import pointcloud_processing
import image_classifier

async def go(drone,lat,lon,alt,pos_range,alt_range):
    print("-- Go to location",lat,lon)
    await drone.action.goto_location(lat,lon,alt,0)
    async for state in drone.telemetry.position():
        #print("drone target:",lat,lon,alt)
        #print("drone pos:",state.latitude_deg,state.longitude_deg,state.absolute_altitude_m)
        if ( np.abs(lat)-pos_range<= np.abs(state.latitude_deg) <= np.abs(lat)+pos_range and
             np.abs(lon)-pos_range <= np.abs(state.longitude_deg) <= np.abs(lon)+pos_range and
             np.abs(alt)-alt_range <= np.abs(state.absolute_altitude_m) <= np.abs(alt)+alt_range):
            print(f"Location reached")
            break

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
    flying_alt = absolute_altitude + 10
    await go(drone,target[0],target[1],flying_alt,0.00001,0.01)

    print("-- Scan area")
    area_radius = 30
    scan_speed = 5
    time_one_orbit = area_radius*2*np.pi/scan_speed
    n_orbits=1
    recognition_time = time_one_orbit*n_orbits
    await drone.action.do_orbit(area_radius,scan_speed,mavsdk.action.OrbitYawBehavior(0),target[0],target[1],flying_alt)
    await asyncio.sleep(20)

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

        with open(os.path.join(object_dir,"object_data.txt"), "r") as file:
            lines = file.readlines()

        object_lat = float(lines[1])
        object_lon = float(lines[2])
        object_alt = float(lines[3])
        object_width = float(lines[4])

        area_radius = object_width + 1   #+1 to make sure object is in camera angle

        object_alt = absolute_altitude + object_alt

        r_earth = 6371000 
        object_radius_lat  = object_lat  + (area_radius / r_earth) * (180 / np.pi);
        
        await go(drone,object_radius_lat,object_lon,flying_alt,0.00001,0.01)         #go in survey altitude to a safe distance to object ( to evade collisions)    
        
        #define orbits and height jumps
        jump = 5    #0.5
        ground_limit = absolute_altitude + 1
        n_orbits = 1
        scan_speed = 1
        time_one_orbit = area_radius*2*np.pi/scan_speed
        recognition_time = time_one_orbit*n_orbits

        while object_alt>=ground_limit:             #loop for orbits in multiple altitudes until reaching limit
                                 
            async for state in drone.telemetry.position():
                lat = state.latitude_deg
                lon = state.longitude_deg
                break 
            await go(drone,lat,lon,object_alt,0.00001,0.01)          #go to scanning altitud of object

            print("-- Scanning object",i)
            await drone.action.do_orbit(area_radius,scan_speed,mavsdk.action.OrbitYawBehavior(0),object_lat,object_lon,object_alt)
            await asyncio.sleep(10)
            await lidar.make_scan(0.5,recognition_time,object_dir)      #scan object
            
            object_alt-=jump

        async for state in drone.telemetry.position():
            lat = state.latitude_deg
            lon = state.longitude_deg
            break
        await go(drone,lat,lon,flying_alt,0.00001,0.01)      #after each scan, go up for survey altitude to evade colisions

    print("-- Landing")
    await drone.action.return_to_launch()

    print("-- Starting objects processing")
    for i in range(len(objects_dirs)):
        object_dir = os.path.join(mission_path,objects_dirs[i])
        print("-- Processing object",i)
        pointcloud_processing.processing(object_dir)

        print("-- Classifying object",i)

        images_dir = os.path.join(object_dir,"images")

        object_type,probability=image_classifier.classify(images_dir)

        with open(os.path.join(object_dir,"object_data.txt"), "a") as file:
            file.write(object_type+" with a mean probability of "+str(probability))

if __name__ == "__main__":
    origin_pos = (40.544289,-4.012101)
    target_pos = (40.544729,-4.012503)
    lidar = scan.Lidar("Drone","Lidar")
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(origin_pos,target_pos))
