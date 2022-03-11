import airsim
import os
import getpass
import numpy as np
import json
import open3d as o3d
import numpy as np
import keyboard
from tqdm import tqdm

class Lidar:

    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.VehicleClient()
        self.client.confirmConnection()
        print('Connected!\n')

    def execute(self,vehicle_name,lidar_name):
        print('Scanning Has Started\n')
        print("press s to stop")
        # Get the default directory for AirSim
        airsim_path = os.path.join(os.path.expanduser('~'), 'Documents', 'AirSim')

        # Load the settings file
        with open(os.path.join(airsim_path, 'settings.json'), 'r') as fp:
            data = json.load(fp)

        # Get the camera intrinsics
        capture_settings = data['CameraDefaults']['CaptureSettings'][0]
        img_width = capture_settings['Width']
        img_height = capture_settings['Height']
        img_fov = capture_settings['FOV_Degrees']

        # Compute the focal length
        fov_rad = img_fov * np.pi/180
        fdy = img_width /(np.tan(fov_rad/2.0)*2)
        fdx = img_height /(np.tan(fov_rad/2.0)*2)

        # Create the camera intrinsic object
        camera_intrinsics = o3d.camera.PinholeCameraIntrinsic()
        camera_intrinsics.set_intrinsics(img_width, img_height,fdx,fdy, img_width/2, img_height/2)

        pcd = o3d.geometry.PointCloud()
        scans = []
        lidar_positions = []
        lidar_rotations = []

        images = []
        camera_positions = []
        camera_rotations = []

        while True:
                
            #GET LIDAR DATA

            lidar_data = self.client.getLidarData(lidar_name=lidar_name,vehicle_name=vehicle_name)
            scans.append(lidar_data.point_cloud)

            #LIDAR EXTRINSICS
            qw, qx, qy, qz = lidar_data.pose.orientation.w_val, lidar_data.pose.orientation.x_val, lidar_data.pose.orientation.y_val, lidar_data.pose.orientation.z_val
            lidar_rotations.append(o3d.geometry.get_rotation_matrix_from_quaternion((qw, qx, qy, qz)))
            lidar_positions.append([lidar_data.pose.position.x_val,lidar_data.pose.position.y_val,lidar_data.pose.position.z_val])

            #GET IMAGE
            responses = self.client.simGetImages([airsim.ImageRequest("front", airsim.ImageType.Scene, False, False)])

            #IMAGE
            photo = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
                        
            img = photo.reshape(responses[0].height, responses[0].width, 3)
            images.append(img[...,::-1])   #brg to rgb

            #CAMERA EXTRINSICS
            qw, qx, qy, qz = responses[0].camera_orientation.w_val, responses[0].camera_orientation.x_val, responses[0].camera_orientation.y_val, responses[0].camera_orientation.z_val
            camera_rotations.append(o3d.geometry.get_rotation_matrix_from_quaternion((qw, qy, qz, qx)))
            camera_positions.append([responses[0].camera_position.y_val,responses[0].camera_position.z_val,responses[0].camera_position.x_val])

            if keyboard.is_pressed('s'):
                print("Scanning stopped")
                break


        print("Processing data")

        for a in tqdm(range(len(scans))):
            for i in range(0, len(scans[a]), 3):
                xyz = scans[a][i:i+3]
                
                xyz = lidar_rotations[a].dot(xyz)+lidar_positions[a]         # transformation from lidar pose to world

                xyz_cam = np.array([xyz[1],xyz[2],xyz[0]])          # cambio de sistema de xyz a yzx

                projection = np.linalg.inv(camera_rotations[a]).dot(xyz_cam-camera_positions[a])
                coordinates = [-fdx*projection[0]/projection[2],-fdy*projection[1]/projection[2]]
                pixel = [int(coordinates[0]+camera_intrinsics.width/2),int(camera_intrinsics.height/2-coordinates[1])]

                if  len(img[0])> pixel[0] > 0 and len(img)> pixel[1] > 0:
                    point = o3d.geometry.PointCloud()
                    point.points = o3d.utility.Vector3dVector(np.reshape(np.array(xyz),(1,3)))
                    point.colors =  o3d.utility.Vector3dVector(np.reshape(np.array(images[a][pixel[1]][pixel[0]]/255),(1,3)))

                    pcd+=point

        print("Done!\n")
        output = "C:/Users/"+getpass.getuser()+"/Documents/Airsim/lidar_rgb_scan.pcd"
        o3d.io.write_point_cloud(output, pcd)
        o3d.visualization.draw_geometries([pcd])

# main
if __name__ == "__main__":
    lidar = Lidar()
    vehicle = "Drone1"
    lidar_name = "LidarSensor1"
    lidar.execute(vehicle,lidar_name)