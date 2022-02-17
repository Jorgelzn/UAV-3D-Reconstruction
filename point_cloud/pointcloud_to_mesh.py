import open3d as o3d
from sys import argv
import numpy as np
import getpass
import matplotlib.pyplot as plt

def read(file):
    print("Loading point cloud")
    pcd = o3d.io.read_point_cloud(file)
    print(pcd)
    o3d.visualization.draw_geometries([pcd],width=1400,height=900)

    return pcd


def alpha_shape(pcd,alpha):
    print("Calculating mesh with Alpha Shape")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd,alpha)

    return mesh

def ball_pivoting(pcd,radii):
    print("Calculating mesh with Ball Pivoting")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))

    return mesh

def poisson_surface(pcd,depth):
    print('Calculating mesh Poisson Surface Reconstruction')
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd,depth)

    density_mesh = o3d.geometry.TriangleMesh()
    density_mesh.vertices = mesh.vertices
    density_mesh.triangles = mesh.triangles
    density_mesh.triangle_normals = mesh.triangle_normals
    density_mesh.vertex_colors = mesh.vertex_colors

    return density_mesh

if __name__ == "__main__":
    pcd_file = "C:/Users/"+getpass.getuser()+"/Documents/Airsim/lidar_rgb_scan.pcd"
    object_file = "C:/Users/"+getpass.getuser()+"/Documents/Airsim/object.ply"

    pcd = read(pcd_file)
    

    #pointcloud segmentation
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.3, ransac_n=3, num_iterations=2000)
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)

    o3d.visualization.draw_geometries([pcd.select_by_index(inliers).paint_uniform_color([1, 0, 0]), pcd.select_by_index(inliers, invert=True).paint_uniform_color([0.6, 0.6, 0.6])],width=1400,height=900)
    

    #normals estimation
    outlier_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    o3d.visualization.draw_geometries([outlier_cloud],point_show_normal=True,width=1400,height=900)


    #mesh conversion
    #mesh = alpha_shape(outlier_cloud,0.2)
    #mesh = ball_pivoting(outlier_cloud,[0.1, 0.2, 0.2, 0.4])
    mesh = poisson_surface(outlier_cloud,9)
    mesh.vertex_colors = o3d.utility.Vector3dVector(np.clip(np.asarray(mesh.vertex_colors), 0, 1))

    o3d.visualization.draw_geometries([mesh],width=1400,height=800)

    #export
    o3d.io.write_triangle_mesh(object_file, mesh, write_ascii=False, compressed=False, write_vertex_normals=True, write_vertex_colors=True, write_triangle_uvs=True, print_progress=True)
