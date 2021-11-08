import open3d as o3d
from sys import argv
import numpy as np

def read(file):
    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud(file)
    print(pcd)
    print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    read(argv[1])