import open3d as o3d
import numpy as np
import os
import pymap3d as pm

def read(file):
    print(" Loading point cloud...")
    pcd = o3d.io.read_point_cloud(file)
    print(" ",pcd)

    return pcd

def crop(pcd,output,range=5):
    print(" Cropping point cloud...")
    pcd_center = pcd.get_center()
    pcd = pcd.crop(o3d.geometry.AxisAlignedBoundingBox(np.array(pcd_center)-range,np.array(pcd_center)+range))
    o3d.io.write_point_cloud(output, pcd)

    return pcd

def plane_segmentation(pcd,output,distance=0.05,ransac=3,iter=5000):
    print(" Making plane segmentation...")
    plane_model, inliers = pcd.segment_plane(distance_threshold=distance, ransac_n=ransac, num_iterations=iter)
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    o3d.io.write_point_cloud(output, outlier_cloud)

    return outlier_cloud
    
def alpha_shape(pcd,output,alpha=0.7):
    print(" Calculating mesh with Alpha Shape...")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    mesh.compute_vertex_normals()
    o3d.io.write_triangle_mesh(output, mesh, write_ascii=False, compressed=False, write_vertex_normals=True, write_vertex_colors=True, write_triangle_uvs=True, print_progress=True)
    return mesh

def ball_pivoting(pcd,output,radii=np.arange(0.05, 0.2, 0.05),radi=1.5,nn=1000,plane=15):
    print(" Calculating mesh with Ball Pivoting...")
    print(" Estimating normals...")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radi, max_nn=nn))
    print(" Orienting normals...")
    pcd.orient_normals_consistent_tangent_plane(plane)
    print(" Creating mesh...")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))
    o3d.io.write_triangle_mesh(output, mesh, write_ascii=False, compressed=False, write_vertex_normals=True, write_vertex_colors=True, write_triangle_uvs=True, print_progress=True)
    return mesh

def poisson_surface(pcd,output,depth=10,radi=1.5,nn=1000):
    print(" Calculating mesh Poisson Surface Reconstruction...")
    print(" Estimating normals...")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radi, max_nn=nn))
    print(" Orienting normals...")
    pcd.orient_normals_consistent_tangent_plane(15)
    print(" Creating mesh...")
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd,depth)

    density_mesh = o3d.geometry.TriangleMesh()
    density_mesh.vertices = mesh.vertices
    density_mesh.triangles = mesh.triangles
    density_mesh.triangle_normals = mesh.triangle_normals
    density_mesh.vertex_colors = mesh.vertex_colors

    o3d.io.write_triangle_mesh(output, density_mesh, write_ascii=False, compressed=False, write_vertex_normals=True, write_vertex_colors=True, write_triangle_uvs=True, print_progress=True)
    
    return density_mesh

async def recognition(mission_path,origin):
    recognition_path = os.path.join(mission_path,"recognition")
    scan_path = os.path.join(recognition_path,"lidar_scan.ply")
    crop_scan_path = os.path.join(recognition_path,"crop_scan.ply")
    segmented_scan_path = os.path.join(recognition_path,"segmented_scan.ply")

    pcd = read(scan_path)

    #pointcloud segmentation
    pcd = crop(pcd,crop_scan_path,range=30)
    pcd = plane_segmentation(pcd,segmented_scan_path,0.5,50,5000)

    #clustering
    print(" Making clusters...")
    labels = np.array(pcd.cluster_dbscan(eps=2, min_points=10))
    labels_id = np.unique(labels)
    objects = [o3d.geometry.PointCloud() for i in range(len(labels_id))]

    for i in range(len(labels)):
        point = o3d.geometry.PointCloud()
        point.points = o3d.utility.Vector3dVector(np.reshape(np.array(pcd.points[i]),(1,3)))
        point.colors = o3d.utility.Vector3dVector(np.reshape(np.array(pcd.colors[i]),(1,3)))
        objects[labels[i]] += point

    for i in range(len(objects)):
        if len(objects[i].points)>20:
            object_path = os.path.join(mission_path,"object_"+str(i))
            os.mkdir(object_path)
            o3d.io.write_point_cloud(os.path.join(object_path,"recognised_object.ply"), objects[i])
            width = max(objects[i].get_oriented_bounding_box().extent)
            #create file for data
            file = open(os.path.join(object_path,"object_data.txt"), 'w')

            center = objects[i].get_center()
            
            object_data = pm.ned2geodetic(center[0], center[1], center[2],origin[0],origin[1],80.33497619628906, ell=None, deg=True)

            #registrar posición global del objeto
            file.write("latitud,longitud,altura y anchura:\n%f\n%f\n%f\n%f\n" % (object_data[0],object_data[1],object_data[2],width))

            file.close()

def processing(object_path):
    lidar_scan_path = os.path.join(object_path,"lidar_scan.ply")
    recogised_object_path = os.path.join(object_path,"recognised_object.ply")
    crop_scan_path = os.path.join(object_path,"crop_scan.ply")
    segmented_scan_path = os.path.join(object_path,"segmented_scan.ply")
    object_mesh_path = os.path.join(object_path,"object.ply")

    lidar_scan_cloud = read(lidar_scan_path)
    recognised_object_cloud = read(recogised_object_path)

    #pointcloud segmentation
    lidar_scan_cloud = crop(lidar_scan_cloud,crop_scan_path,range=10)
    lidar_scan_cloud = plane_segmentation(lidar_scan_cloud,segmented_scan_path,0.1,20,5000)

    #merge pointclouds
    for i in range(len(recognised_object_cloud.points)):
        point = o3d.geometry.PointCloud()
        point.points = o3d.utility.Vector3dVector(np.reshape(np.array(recognised_object_cloud.points[i]),(1,3)))
        point.colors = o3d.utility.Vector3dVector(np.reshape(np.array(recognised_object_cloud.colors[i]),(1,3)))
        lidar_scan_cloud += point

    #make 3d object
    #alpha_shape(lidar_scan_cloud,object_mesh_path)
    ball_pivoting(lidar_scan_cloud,object_mesh_path)
    #poisson_surface(lidar_scan_cloud,object_mesh_path)


#if __name__ == "__main__":
#    processing("C:/Users/jorge/Documents/AirSim/data/mission_1/object_1")

    