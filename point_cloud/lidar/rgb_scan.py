
import airsim
import os
import getpass
import numpy as np
import json
import open3d as o3d
import numpy as np
import cv2 as cv

def worldpoint_to_pixel(world_points, camera_intrinsics, camera_rotation,camera_translation):

    pixel = []
    projection = cv.projectPoints(np.array(world_points),rvec=cv.Rodrigues(np.array(camera_rotation))[0],tvec=np.array(camera_translation),cameraMatrix=camera_intrinsics.intrinsic_matrix,distCoeffs=np.array([]))

    pixel.append(int(projection[0][0][0][1]))
    pixel.append(int(projection[0][0][0][0]))
    print(pixel)
    return pixel


def convert(filename):
    print ("the input file name is:%r." %filename)

    print ("open the file...")
    file = open(filename,"r+")
    count = 0
    # Statistical Source File Points
    for line in file:
        count=count+1
    print ("size is %d" %count)
    file.close()

    #output = open("out.pcd","w+")
    prefix = filename.split('.')[0]
    output_filename = f"{prefix}.pcd"
    output = open("C:/Users/"+getpass.getuser()+"/Documents/Airsim/"+output_filename,"w+")

    list = ['# .PCD v.7 - Point Cloud Data file format\n','VERSION .7\n','FIELDS x y z rgb\n','SIZE 4 4 4 4\n','TYPE F F F U\n','COUNT 1 1 1 1\n']

    output.writelines(list)
    output.write ('WIDTH ') # Note that there are spaces behind it
    output.write(str(count))
    output.write('\nHEIGHT ')
    output.write (str(1))# mandatory type conversion, file input can only be STR format
    output.write("\nVIEWPOINT 0 0 0 1 0 0 0")
    output.write('\nPOINTS ')
    output.write(str(count))
    output.write('\nDATA ascii\n')
    file = open(filename,"r")
    all = file.read()
    output.write(all)
    output.close()
    file.close()


class Lidar:

    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.VehicleClient()
        self.client.confirmConnection()
        print('Connected!\n')

    def execute(self,vehicle_name,lidar_names):
        print('Scanning Has Started\n')
        existing_data_cleared = False   #change to true to superimpose new scans onto existing .asc files


        # Get the default directory for AirSim
        airsim_path = os.path.join(os.path.expanduser('~'), 'Documents', 'AirSim')

        # Load the settings file
        with open(os.path.join(airsim_path, 'settings.json'), 'r') as fp:
            data = json.load(fp)

        # Get the camera intrinsics
        capture_settings = data['CameraDefaults']['CaptureSettings'][0]
        img_width = capture_settings['Width']
        img_height = capture_settings['Height']
        img_fov = capture_settings['FOV_Degrees']

        # Compute the focal length
        fov_rad = img_fov * np.pi/180
        fd = (img_width/2.0) / np.tan(fov_rad/2.0)
        #fd = 2 * np.arctan(img_width/2*fov_rad)

        # Create the camera intrinsic object
        intrinsic = o3d.camera.PinholeCameraIntrinsic()
        intrinsic.set_intrinsics(img_width, img_height, fd, fd, img_width/2-0.5, img_height/2-0.5)
        
        try:
            while True:

                #GET IMAGE

                responses = self.client.simGetImages([airsim.ImageRequest("front", airsim.ImageType.Scene, False, False)])

                #CAMERA EXTRINSICS

                qw, qy, qz, qx = responses[0].camera_orientation.w_val, responses[0].camera_orientation.x_val, responses[0].camera_orientation.y_val, responses[0].camera_orientation.z_val
                camera_rotation = o3d.geometry.get_rotation_matrix_from_quaternion((qw, qy, qz, qx))
                camera_translation =  [responses[0].camera_position.x_val,responses[0].camera_position.y_val,responses[0].camera_position.z_val]

                #IMAGE

                photo = np.fromstring(responses[0].image_data_uint8, dtype=np.uint8)
                
                img = photo.reshape(responses[0].height, responses[0].width, 3)
                img = img[...,::-1]   #brg to rgb
                #img = cv.rotate(img, cv.ROTATE_180)
                #LIDAR

                for lidar_name in lidar_names:
                    filename = f"{vehicle_name}_{lidar_name}_pointcloud_rgb.asc"
                    if not existing_data_cleared:
                        f = open(filename,'w')
                    else:
                        f = open(filename,'a')

                    lidar_data = self.client.getLidarData(lidar_name=lidar_name,vehicle_name=vehicle_name,)

                    #LIDAR EXTRINSICS
                    qw, qy, qz, qx = lidar_data.pose.orientation.w_val, lidar_data.pose.orientation.x_val, lidar_data.pose.orientation.y_val, lidar_data.pose.orientation.z_val
                    lidar_rotation = o3d.geometry.get_rotation_matrix_from_quaternion((qw, qy, qz, qx))
                    lidar_translation =  [lidar_data.pose.position.x_val,lidar_data.pose.position.y_val,lidar_data.pose.position.z_val]

                    
                    for i in reversed(range(0, len(lidar_data.point_cloud), 3)):

                        xyz = lidar_data.point_cloud[i:i+3]
                        
                        # transformation from lidar pose to world
                        
                        xyz = lidar_rotation.dot(xyz)+lidar_translation
                    
                        #PROYECCION

                        pixel = worldpoint_to_pixel(xyz,intrinsic,camera_rotation,camera_translation)
                        if len(img) > pixel[1] > 0 and len(img[0]) > pixel[0] > 0:
                            r,g,b = [color for color in img[pixel[1]][pixel[0]]]
                            #print("point:",xyz,"\nColor:",r,g,b)
                            #img[pixel[1]][pixel[0]][:]=[0,255,0]
                            rgb = int('%02x%02x%02x' % (r,g,b), 16)
                            f.write("%f %f %f %f\n" % (xyz[0],xyz[1],xyz[2],rgb))
                    print("camara:",responses[0].camera_position)
                    f.close()
                    return img
                existing_data_cleared = True
        except KeyboardInterrupt:
            airsim.wait_key('Press any key to stop running this script')
            print("Done!\n")
            return img

# main
if __name__ == "__main__":
    lidar = Lidar()
    vehicle = 'Drone1'
    lidar_sensors = ['LidarSensor1']

    data = lidar.execute(vehicle,lidar_sensors)
    from matplotlib import pyplot as plt
    plt.imshow(data, interpolation='nearest')
    plt.show()
    for lidar_name in lidar_sensors:
        convert(f"{vehicle}_{lidar_name}_pointcloud_rgb.asc")
        os.remove(f"{vehicle}_{lidar_name}_pointcloud_rgb.asc")
    
