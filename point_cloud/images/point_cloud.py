import numpy as np
import open3d as o3d
import os
import getpass

record_number = 3
img_type = "stereo"

record_path = os.path.dirname(__file__)+"/records/record_"+str(record_number)
color_images = os.listdir(record_path+"/mono_color_images")
depth_images = os.listdir(record_path+"/"+img_type+"_depth")
file = open(record_path+"/positions.txt", "r")
positions = file.read().splitlines()
pcd = o3d.geometry.PointCloud()

# Get the camera intrinsics
img_width = 672
img_height = 376
img_fov = 90

# Compute the focal length
fov_rad = img_fov * np.pi/180
fd = (img_width/2.0) / np.tan(fov_rad/2.0)

# Create the camera intrinsic object
intrinsic = o3d.camera.PinholeCameraIntrinsic()
intrinsic.set_intrinsics(img_width, img_height, fd, fd, img_width/2 - 0.5, img_height/2 - 0.5)

R = np.eye(4)
T = np.eye(4)
C = np.array([
        [ 1,  0,  0,  0],
        [ 0,  0, -1,  0],
        [ 0,  1,  0,  0],
        [ 0,  0,  0,  1]
        ])

for i in range(len(color_images)):

    #Crear matriz de transformación para posición de la camara
    actual_pos = positions[i].split()
    x, y, z =   [float(x) for x in actual_pos[:3]]
    T[:3,3] = [-y, -z, -x]
        
    qw, qx, qy, qz = [float(x) for x in actual_pos[3:]]
    R[:3,:3] = o3d.geometry.get_rotation_matrix_from_quaternion((qw, qy, qz, qx))

    F = R.T @ T @ C

    #create point cloud
    color_raw = o3d.io.read_image(record_path+"/mono_color_images/"+color_images[i])
    depth_raw = o3d.io.read_image(record_path+"/"+img_type+"_depth/"+depth_images[i])
    
    rgbd_img = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw,depth_raw,depth_scale=0.5, depth_trunc=1000, convert_rgb_to_intensity=False)
    pointcloud = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_img,intrinsic, extrinsic=F)
    pcd += pointcloud


o3d.io.write_point_cloud("C:/Users/"+getpass.getuser()+"/Documents/Airsim/images_pointscloud.pcd", pcd)
o3d.visualization.draw_geometries([pcd])