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

def plane_segmentation(pcd,output,distance=0.01,ransac=50,iter=5000):
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
    mesh.transform([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]) #invert 
    mesh.scale(100, center=mesh.get_center())                                               #scale
    height_offset = mesh.get_oriented_bounding_box().extent[2]/2
    mesh.translate((0, 0, height_offset))                                                   #translate to origin with height
    o3d.io.write_triangle_mesh(output, mesh, compressed=False, write_vertex_normals=True, write_vertex_colors=True, write_triangle_uvs=False, print_progress=True)
    return mesh

def ball_pivoting(pcd,output,radii=np.arange(0.1, 0.2, 0.05),radi=1.5,nn=1000):
    print(" Calculating mesh with Ball Pivoting...")
    print(" Estimating normals...")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radi, max_nn=nn))
    print(" Orienting normals...")
    pcd.orient_normals_towards_camera_location(pcd.get_center())
    normals = np.asarray(pcd.normals)
    # flip normals
    pcd.normals = o3d.utility.Vector3dVector(-normals)
    print(" Creating mesh...")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))
    mesh.transform([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])                      #invert 
    mesh.scale(100, center=mesh.get_center())                                                       #scale
    height_offset = mesh.get_oriented_bounding_box().extent[2]/2
    mesh.translate((0, 0, height_offset))                                                   #translate to origin with height
    o3d.io.write_triangle_mesh(output, mesh, compressed=False, write_vertex_normals=True, write_vertex_colors=True, write_triangle_uvs=False, print_progress=True)
    return mesh

def poisson_surface(pcd,output,depth=15,radi=1.5,nn=1000):
    print(" Calculating mesh Poisson Surface Reconstruction...")
    box = pcd.get_axis_aligned_bounding_box()
    for i in range(len(pcd.points)//100):
        area = [np.random.uniform(box.get_min_bound()[0],box.get_max_bound()[0]),np.random.uniform(box.get_min_bound()[1],box.get_max_bound()[1]),box.get_max_bound()[2]]
        point = o3d.geometry.PointCloud()
        point.points = o3d.utility.Vector3dVector(np.reshape(np.array(area),(1,3)))
        point.colors = o3d.utility.Vector3dVector(np.reshape(np.array(pcd.colors[np.random.randint(0,len(pcd.points))]),(1,3)))
        pcd+=point
    print(" Estimating normals...")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radi, max_nn=nn))
    print(" Orienting normals...")
    pcd.orient_normals_towards_camera_location(pcd.get_center())
    normals = np.asarray(pcd.normals)
    # flip normals
    pcd.normals = o3d.utility.Vector3dVector(-normals)
    print(" Creating mesh...")
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd,depth)

    density_mesh = o3d.geometry.TriangleMesh()
    density_mesh.vertices = mesh.vertices
    density_mesh.triangles = mesh.triangles
    density_mesh.triangle_normals = mesh.triangle_normals
    density_mesh.vertex_colors = mesh.vertex_colors
    density_mesh.transform([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])  #invert 
    density_mesh.scale(100, center=density_mesh.get_center())                           #scale
    height_offset = mesh.get_oriented_bounding_box().extent[2]/2
    density_mesh.translate((0, 0, height_offset))                                                   #translate to origin with height
    o3d.io.write_triangle_mesh(output, density_mesh, compressed=False, write_vertex_normals=True, write_vertex_colors=True, write_triangle_uvs=False, print_progress=True)
    
    return density_mesh

async def recognition(mission_path,origin,scan_range):
    recognition_path = os.path.join(mission_path,"recognition")
    scan_path = os.path.join(recognition_path,"lidar_scan.ply")
    crop_scan_path = os.path.join(recognition_path,"crop_scan.ply")
    segmented_scan_path = os.path.join(recognition_path,"segmented_scan.ply")

    pcd = read(scan_path)

    #pointcloud segmentation
    pcd = crop(pcd,crop_scan_path,range=scan_range)
    pcd = plane_segmentation(pcd,segmented_scan_path,distance=0.7)

    #clustering
    print(" Making clusters...")
    labels = np.array(pcd.cluster_dbscan(eps=10, min_points=200))
    labels_id = np.unique(labels)
    objects = [o3d.geometry.PointCloud() for i in range(len(labels_id))]

    for i in range(len(labels)):
        point = o3d.geometry.PointCloud()
        point.points = o3d.utility.Vector3dVector(np.reshape(np.array(pcd.points[i]),(1,3)))
        point.colors = o3d.utility.Vector3dVector(np.reshape(np.array(pcd.colors[i]),(1,3)))
        objects[labels[i]] += point

    for i in range(len(objects)):
        object_path = os.path.join(mission_path,"object_"+str(i))
        os.mkdir(object_path)
        o3d.io.write_point_cloud(os.path.join(object_path,"recognised_object.ply"), objects[i])
        box = objects[i].get_axis_aligned_bounding_box()
        anchura = max(abs(box.get_min_bound()-box.get_max_bound())[:2])
        altura = abs(box.get_min_bound()[2])
        #create file for data
        file = open(os.path.join(object_path,"object_data.txt"), 'w')

        center = objects[i].get_center()
            
        object_data = pm.ned2geodetic(center[0], center[1], altura,origin[0],origin[1],80.33497619628906, ell=None, deg=True)

        #registrar posición global del objeto
        file.write("latitud | longitud | altura | orbita | clase\n%f\n%f\n%f\n%f\n" % (object_data[0],object_data[1],object_data[2],anchura))
        file.close()

def processing(object_path,origin,mode=3):
    lidar_scan_path = os.path.join(object_path,"lidar_scan.ply")
    recogised_object_path = os.path.join(object_path,"recognised_object.ply")
    crop_scan_path = os.path.join(object_path,"crop_scan.ply")
    segmented_scan_path = os.path.join(object_path,"segmented_scan.ply")
    object_mesh_path = os.path.join(object_path,"object.obj")

    lidar_scan_cloud = read(lidar_scan_path)
    recognised_object_cloud = read(recogised_object_path)

    #merge pointclouds
    for i in range(len(recognised_object_cloud.points)):
        point = o3d.geometry.PointCloud()
        point.points = o3d.utility.Vector3dVector(np.reshape(np.array(recognised_object_cloud.points[i]),(1,3)))
        point.colors = o3d.utility.Vector3dVector(np.reshape(np.array(recognised_object_cloud.colors[i]),(1,3)))
        lidar_scan_cloud += point

    #pointcloud segmentation
    box = recognised_object_cloud.get_axis_aligned_bounding_box()
    anchura = max(abs(box.get_min_bound()-box.get_max_bound())[:2])
    offset_radius = 2
    lidar_scan_cloud = crop(lidar_scan_cloud,crop_scan_path,range=anchura+offset_radius)
    lidar_scan_cloud = plane_segmentation(lidar_scan_cloud,segmented_scan_path,distance=0.01)

    #save geodata
    file = open(os.path.join(object_path,"object_geodata.txt"), 'w')
    geobox = lidar_scan_cloud.get_axis_aligned_bounding_box()
    c1=pm.ned2geodetic(geobox.get_min_bound()[0], geobox.get_min_bound()[1], 80.33497619628906,origin[0],origin[1],80.33497619628906, ell=None, deg=True)[:2]
    c2=pm.ned2geodetic(geobox.get_min_bound()[0], geobox.get_max_bound()[1], 80.33497619628906,origin[0],origin[1],80.33497619628906, ell=None, deg=True)[:2]
    c3=pm.ned2geodetic(geobox.get_max_bound()[0], geobox.get_min_bound()[1], 80.33497619628906,origin[0],origin[1],80.33497619628906, ell=None, deg=True)[:2]
    c4=pm.ned2geodetic(geobox.get_max_bound()[0], geobox.get_max_bound()[1], 80.33497619628906,origin[0],origin[1],80.33497619628906, ell=None, deg=True)[:2]
    file.write("Object coordinates\n"+str(c1)+"\n"+str(c2)+"\n"+str(c3)+"\n"+str(c4)+"\n")
    file.close()

    #make 3d object
    if mode==1:
        alpha_shape(lidar_scan_cloud,object_mesh_path)
    elif mode==2:
        ball_pivoting(lidar_scan_cloud,object_mesh_path)
    elif mode==3:
        poisson_surface(lidar_scan_cloud,object_mesh_path)



