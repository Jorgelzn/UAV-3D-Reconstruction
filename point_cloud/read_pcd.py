import open3d as o3d
from sys import argv
import numpy as np
import getpass

def read(file):
    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud(file)
    print(pcd)
    print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    sensor = 1
    file = "C:/Users/"+getpass.getuser()+"/Documents/Airsim/"+"Drone1_LidarSensor"+str(sensor)+"_pointcloud.pcd"
    file2 = "C:/Users/"+getpass.getuser()+"/Documents/Airsim/"+"Drone1_LidarSensor"+str(sensor)+"_pointcloud_rgb.pcd"
    read(file2)