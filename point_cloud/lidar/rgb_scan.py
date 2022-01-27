import airsim
import os
import getpass
import ctypes
from PIL import Image
import numpy as np
from matplotlib import pyplot as plt

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

        #CAMERA CONSTANTS

        img_width = 672
        center_x = img_width/2
        img_height = 376
        center_y = img_height/2
        img_fov = 90
        ratio = img_width/img_height

        epsilon = .001

        s_x = ( img_width - epsilon ) / 2
        d_x = ( img_width - epsilon ) / 2

        s_y = ( img_height - epsilon ) / (-2*ratio)
        d_y = ( img_height - epsilon ) / 2



        # Compute the focal length
        fov_rad = img_fov * np.pi/180
        focal = np.tan(fov_rad/2.0)
        
        try:
            while True:
                responses = self.client.simGetImages([airsim.ImageRequest("front", airsim.ImageType.Scene, False, False)])

                #CAMERA VALUES
                camera_x = responses[0].camera_position.x_val
                camera_y = responses[0].camera_position.y_val
                camera_z = responses[0].camera_position.z_val
                camera_orientation_w = responses[0].camera_orientation.w_val
                camera_orientation_x = responses[0].camera_orientation.x_val
                camera_orientation_y = responses[0].camera_orientation.y_val
                camera_orientation_z = responses[0].camera_orientation.z_val

                #IMAGE

                photo = np.fromstring(responses[0].image_data_uint8, dtype=np.uint8)
                img = photo.reshape(responses[0].height, responses[0].width, 3)
                img = img[...,::-1]   #brg to rgb


                for lidar_name in lidar_names:
                    filename = f"{vehicle_name}_{lidar_name}_pointcloud_rgb.asc"
                    if not existing_data_cleared:
                        f = open(filename,'w')
                    else:
                        f = open(filename,'a')

                    lidar_data = self.client.getLidarData(lidar_name=lidar_name,vehicle_name=vehicle_name)
                    points = len(lidar_data.point_cloud)/3
                    cloud = len(lidar_data.point_cloud)
                    
                    for i in range(0, cloud, 3):
                        xyz = lidar_data.point_cloud[i:i+3]

                        #PROYECCION
                        X_NDC = xyz[0]/(ratio*xyz[2]*focal)
                        Y_NDC = xyz[1]/(xyz[2]*focal)
                        pixel_x = int(s_x * X_NDC + d_x)
                        pixel_y = int(s_y * Y_NDC + d_y)
                        r,g,b = [color for color in img[pixel_y,pixel_x]]

                        rgb = int('%02x%02x%02x' % (r,g,b), 16)
                        f.write("%f %f %f %f\n" % (xyz[0],xyz[1],xyz[2],rgb))

                    f.close()
                existing_data_cleared = True
        except KeyboardInterrupt:
            airsim.wait_key('Press any key to stop running this script')
            print("Done!\n")

# main
if __name__ == "__main__":
    lidar = Lidar()
    vehicle = 'Drone1'
    lidar_sensors = ['LidarSensor1']

    lidar.execute(vehicle,lidar_sensors)

    for lidar_name in lidar_sensors:
        convert(f"{vehicle}_{lidar_name}_pointcloud_rgb.asc")
        os.remove(f"{vehicle}_{lidar_name}_pointcloud_rgb.asc")
    
