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

    return pcd


if __name__ == "__main__":
    sensor = 1
    file = "C:/Users/"+getpass.getuser()+"/Documents/Airsim/lidar_rgb_scan.pcd"
    pcd = read(file)
    #plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    #inlier_cloud = pcd.select_by_index(inliers)
    #outlier_cloud = pcd.select_by_index(inliers, invert=True)
    #inlier_cloud.paint_uniform_color([1, 0, 0])
    #outlier_cloud.paint_uniform_color([0.6, 0.6, 0.6])
    #o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

