from distutils.util import execute
import airsim
import os
import numpy as np
import json
import open3d as o3d
import numpy as np
import cv2
import time
import asyncio

from sklearn import tree

class Lidar:

    def __init__(self,vehicle_name,lidar_name):

        # connect to the AirSim simulator
        self.client = airsim.VehicleClient()
        self.client.confirmConnection()
        self.vehicle_name=vehicle_name
        self.lidar_name=lidar_name
        print('-- Connected to Airsim\n')

    async def make_scan(self,fps,limit_time,scan_path):
        init_time = time.time()

        # Get the default directory for AirSim
        airsim_path = os.path.join(os.path.expanduser('~'), 'Documents', 'AirSim')

        #create file for data
        camara_data_path = os.path.join(scan_path,"camera_data.txt")
        if os.path.exists(camara_data_path):
            camera_data_file = open(camara_data_path, 'r+')
            img_number=len(camera_data_file.readlines())
        else:
            camera_data_file = open(camara_data_path, 'w')
            img_number=0

        #create images folder
        images_path = os.path.join(scan_path,"images")
        if not os.path.isdir(images_path):
            os.mkdir(images_path)

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

        #path for pointcloud
        pointcloud_path = os.path.join(scan_path,"lidar_scan.ply")
        
        if os.path.exists(pointcloud_path):
            pcd = o3d.io.read_point_cloud(pointcloud_path)
        else:
            pcd = o3d.geometry.PointCloud()

        actual_frame= 0
        clock = 0

        while clock<limit_time:
            
            #GET LIDAR DATA
            lidar_data = self.client.getLidarData(lidar_name=self.lidar_name,vehicle_name=self.vehicle_name)

            #LIDAR EXTRINSICS
            qw, qx, qy, qz = lidar_data.pose.orientation.w_val, lidar_data.pose.orientation.x_val, lidar_data.pose.orientation.y_val, lidar_data.pose.orientation.z_val
            lidar_rotation = o3d.geometry.get_rotation_matrix_from_quaternion((qw, qx, qy, qz))
            lidar_translation =  [lidar_data.pose.position.x_val,lidar_data.pose.position.y_val,lidar_data.pose.position.z_val]


            #GET IMAGE
            responses = self.client.simGetImages([airsim.ImageRequest("front", airsim.ImageType.Scene, False, False)])

            #IMAGE
            photo = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
            img = photo.reshape(responses[0].height, responses[0].width, 3)

            #CAMERA EXTRINSICS
            qw, qx, qy, qz = responses[0].camera_orientation.w_val, responses[0].camera_orientation.x_val, responses[0].camera_orientation.y_val, responses[0].camera_orientation.z_val
            camera_rotation = o3d.geometry.get_rotation_matrix_from_quaternion((qw, qy, qz, qx))
            camera_translation =  [responses[0].camera_position.y_val,responses[0].camera_position.z_val,responses[0].camera_position.x_val]

            #control ratio of photos for photogrametry and nerf
            if clock-actual_frame>(1/fps):
                actual_frame=clock
                img_name = "image_"+str(img_number)+".png"
                cv2.imwrite(os.path.join(images_path,img_name), img)
                img_number+=1

                #registrar posición de la camara
                camera_data_file.write("%s %f %f %f %f %f %f %f\n" %
                    (img_name,camera_translation[0],camera_translation[1],camera_translation[2],qw,qy,qz,qx))


            img = img[...,::-1]   #brg to rgb

            #build point cloud
            for i in range(0, len(lidar_data.point_cloud), 3):
                xyz = lidar_data.point_cloud[i:i+3]

                xyz = lidar_rotation.dot(xyz)+lidar_translation         # transformation from lidar pose to world
                    
                xyz_cam = np.array([xyz[1],xyz[2],xyz[0]])          # cambio de sistema de xyz a yzx

                projection = np.linalg.inv(camera_rotation).dot(xyz_cam-camera_translation)
                coordinates = [-fdx*projection[0]/projection[2],-fdy*projection[1]/projection[2]]
                pixel = [int(coordinates[0]+camera_intrinsics.width/2),int(camera_intrinsics.height/2-coordinates[1])]

                if  len(img[0])> pixel[0] > 0 and len(img)> pixel[1] > 0:
                    point = o3d.geometry.PointCloud()
                    point.points = o3d.utility.Vector3dVector(np.reshape(np.array(xyz),(1,3)))
                    point.colors =  o3d.utility.Vector3dVector(np.reshape(np.array(img[pixel[1]][pixel[0]]/255),(1,3)))

                    pcd+=point

            clock=time.time()-init_time
  
        camera_data_file.close()
        o3d.io.write_point_cloud(pointcloud_path, pcd)

