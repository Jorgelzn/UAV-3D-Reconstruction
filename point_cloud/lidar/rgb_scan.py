
import airsim
import os
import getpass
import numpy as np
import json
import open3d as o3d
import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

class Lidar:

    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.VehicleClient()
        self.client.confirmConnection()
        print('Connected!\n')

    def execute(self,vehicle_name,lidar_name):
        print('Scanning Has Started\n')

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
        fd = (img_width/2.0) / np.tan(fov_rad/2.0)
        #fd = 2 * np.arctan(img_width/2*fov_rad)

        # Create the camera intrinsic object
        camera_intrinsics = o3d.camera.PinholeCameraIntrinsic()
        camera_intrinsics.set_intrinsics(img_width, img_height, fd, fd, img_width/2, img_height/2)
        
        #pointcloud
        pcd = o3d.geometry.PointCloud()
        
        try:
            while True:
                
                #GET LIDAR DATA

                lidar_data = self.client.getLidarData(lidar_name=lidar_name,vehicle_name=vehicle_name)

                #GET IMAGE

                responses = self.client.simGetImages([airsim.ImageRequest("front", airsim.ImageType.Scene, False, False)])

                #IMAGE

                photo = np.fromstring(responses[0].image_data_uint8, dtype=np.uint8)
                    
                img = photo.reshape(responses[0].height, responses[0].width, 3)
                img = img[...,::-1]   #brg to rgb
                #img = cv.rotate(img, cv.ROTATE_180)

                #CAMERA EXTRINSICS

                qw, qy, qz, qx = responses[0].camera_orientation.w_val, responses[0].camera_orientation.x_val, responses[0].camera_orientation.y_val, responses[0].camera_orientation.z_val
                camera_rotation = o3d.geometry.get_rotation_matrix_from_quaternion((qw, qy, qz, qx))
                camera_translation =  [responses[0].camera_position.x_val,responses[0].camera_position.y_val,responses[0].camera_position.z_val]

                #LIDAR EXTRINSICS
                qw, qy, qz, qx = lidar_data.pose.orientation.w_val, lidar_data.pose.orientation.x_val, lidar_data.pose.orientation.y_val, lidar_data.pose.orientation.z_val
                lidar_rotation = o3d.geometry.get_rotation_matrix_from_quaternion((qw, qy, qz, qx))
                lidar_translation =  [lidar_data.pose.position.x_val,lidar_data.pose.position.y_val,lidar_data.pose.position.z_val]

                    
                for i in range(0, len(lidar_data.point_cloud), 3):

                    xyz = lidar_data.point_cloud[i:i+3]
                        
                    # transformation from lidar pose to world
                        
                    xyz = lidar_rotation.dot(xyz)+lidar_translation

                    #PROYECCION
                        
                    projection = cv.projectPoints(np.array(xyz),rvec=cv.Rodrigues(np.array(camera_rotation))[0],tvec=np.array(camera_translation),cameraMatrix=camera_intrinsics.intrinsic_matrix,distCoeffs=np.array([]))
                    pixel = [int(projection[0][0][0][0]),int(projection[0][0][0][1])]
                    
                    if len(img) > pixel[1] > 0 and len(img[0]) > pixel[0] > 0:

                        img[pixel[1]][pixel[0]][:]=[0,255,0]
                        print(np.array(img[pixel[1]][pixel[0]]))
                        point = o3d.geometry.PointCloud()
                        point.points = o3d.utility.Vector3dVector(np.reshape(np.array(xyz),(1,3)))
                        point.colors =  o3d.utility.Vector3dVector(np.reshape(np.array(img[pixel[1]][pixel[0]]),(1,3)))
                        pcd+=point

        except KeyboardInterrupt:
            airsim.wait_key('Press any key to stop running this script')
            print("Done!\n")
            print("camara:",responses[0].camera_position)
            plt.imshow(img, interpolation='nearest')
            plt.show()
            output = "C:/Users/"+getpass.getuser()+"/Documents/Airsim/lidar_rgb_scan.pcd"
            o3d.io.write_point_cloud(output, pcd)
            o3d.visualization.draw_geometries([pcd])

# main
if __name__ == "__main__":
    lidar = Lidar()
    vehicle = "Drone1"
    lidar_name = "LidarSensor1"

    lidar.execute(vehicle,lidar_name)


