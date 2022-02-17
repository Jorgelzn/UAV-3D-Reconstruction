import open3d as o3d
from sys import argv
import numpy as np
import getpass

def read(file):
    print("Loading point cloud")
    pcd = o3d.io.read_point_cloud(file)
    print(pcd)
    #print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd])

    return pcd

def ball_pivoting(pcd):
    print("Calculating mesh with Ball Pivoting")
    radii = [0.3, 0.2, 0.2, 0.4]
    rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))
    o3d.visualization.draw_geometries([rec_mesh])
    
    return rec_mesh

def poisson_surface(pcd):
    print('Calculating mesh Poisson Surface Reconstruction')
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=9)
    o3d.visualization.draw_geometries([mesh])

    return mesh,densities

if __name__ == "__main__":
    file = "C:/Users/"+getpass.getuser()+"/Documents/Airsim/lidar_rgb_scan.pcd"
    pcd = read(file)
    
    #pointcloud segmentation
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.3, ransac_n=3, num_iterations=2000)
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    o3d.visualization.draw_geometries([pcd.select_by_index(inliers).paint_uniform_color([1, 0, 0]), pcd.select_by_index(inliers, invert=True).paint_uniform_color([0.6, 0.6, 0.6])])

    #normals estimation
    outlier_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    o3d.visualization.draw_geometries([outlier_cloud],point_show_normal=True)

    #mesh conversion
    mesh = poisson_surface(outlier_cloud)
