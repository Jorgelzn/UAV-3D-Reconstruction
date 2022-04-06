import open3d as o3d
from sys import argv
import numpy as np
import getpass
import random
import os

def read(file):
    print("Loading point cloud")
    pcd = o3d.io.read_point_cloud(file)
    print(pcd)
    pcd_center = pcd.get_center()
    pcd = pcd.crop(o3d.geometry.AxisAlignedBoundingBox(np.array(pcd_center)-5,np.array(pcd_center)+5))
    o3d.io.write_point_cloud(scan_cloud_file, pcd)
    print("central point:",pcd_center)
    o3d.visualization.draw_geometries([pcd],width=1400,height=900)
    return pcd


def alpha_shape(pcd,alpha):
    print("Calculating mesh with Alpha Shape alpha:",alpha)
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    mesh.compute_vertex_normals()

    return mesh

def ball_pivoting(pcd,radii):
    print("Calculating mesh with Ball Pivoting radius:",radii)
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))

    return mesh

def poisson_surface(pcd,depth):
    print("Calculating mesh Poisson Surface Reconstruction depth:",depth)
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd,depth)

    density_mesh = o3d.geometry.TriangleMesh()
    density_mesh.vertices = mesh.vertices
    density_mesh.triangles = mesh.triangles
    density_mesh.triangle_normals = mesh.triangle_normals
    density_mesh.vertex_colors = mesh.vertex_colors

    return density_mesh

if __name__ == "__main__":

    # Get the default directory for scan
    scan_path = os.path.join(os.path.expanduser('~'), 'Documents', 'AirSim',"scan")

    scan_file = os.path.join(scan_path,"lidar_scan.ply")
    scan_cloud_file = os.path.join(scan_path,"scan_cloud.ply")
    object_cloud_file = os.path.join(scan_path,"object_cloud.ply")
    object_file = os.path.join(scan_path,"object.ply")

    pcd = read(scan_file)


    #pointcloud segmentation
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.05, ransac_n=3, num_iterations=5000)
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)

    o3d.visualization.draw_geometries([pcd.select_by_index(inliers).paint_uniform_color([1, 0, 0]), pcd.select_by_index(inliers, invert=True).paint_uniform_color([0.6, 0.6, 0.6])],width=1400,height=900)

    pcd = outlier_cloud
    o3d.io.write_point_cloud(object_cloud_file, pcd)
    print("object points:",pcd)
    #normals estimation
    #pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.5, max_nn=1000))
    #pcd.orient_normals_consistent_tangent_plane(15)
    #pcd.orient_normals_to_align_with_direction(orientation_reference=np.array([0., 0., -1.]))
    #pcd.orient_normals_towards_camera_location(camera_location=np.array([0., 5., 0.]))
    #o3d.visualization.draw_geometries([pcd],point_show_normal=True,width=1400,height=900)


    #mesh conversion
    mesh = alpha_shape(pcd,0.7)
    #mesh = ball_pivoting(pcd,np.arange(0.05, 0.2, 0.05))
    #mesh = poisson_surface(pcd,15)

    o3d.visualization.draw_geometries([mesh],width=1400,height=800)

    #export
    o3d.io.write_triangle_mesh(object_file, mesh, write_ascii=False, compressed=False, write_vertex_normals=True, write_vertex_colors=True, write_triangle_uvs=True, print_progress=True)
    