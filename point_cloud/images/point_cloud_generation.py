import numpy as np
import open3d as o3d
import os

record_number = 1
record_path = "records/record_"+str(record_number)
color_images = os.listdir(record_path+"/color_images")
depth_images = os.listdir(record_path+"/depth_images")
file = open(record_path+"/positions.txt", "r")
positions = file.read().splitlines()
pcd = o3d.geometry.PointCloud()

for i in range(len(color_images)):

    #Crear matriz de transformación para posición de la camara
    actual_pos = positions[i].split()
    x, y, z =   [float(x) for x in actual_pos[:3]]
    T = np.eye(4)
    T[:3,3] = [-y, -z, -x]
        
    qw, qx, qy, qz = [float(x) for x in actual_pos[3:]]
    R = np.eye(4)
    R[:3,:3] = o3d.geometry.get_rotation_matrix_from_quaternion((qw, qy, qz, qx))
        
    C = np.array([
        [ 1,  0,  0,  0],
        [ 0,  0, -1,  0],
        [ 0,  1,  0,  0],
        [ 0,  0,  0,  1]
    ])

    F = R.T @ T @ C

    #create point cloud
    color_raw = o3d.io.read_image(record_path+"/color_images/"+color_images[i])
    depth_raw = o3d.io.read_image(record_path+"/depth_images/"+depth_images[i])

    rgbd_img = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw,depth_raw)

    camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
        o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)

    pcd += o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_img,camera_intrinsic, extrinsic=F)


o3d.io.write_point_cloud("pointscloud.pcd", pcd)
o3d.visualization.draw_geometries([pcd])