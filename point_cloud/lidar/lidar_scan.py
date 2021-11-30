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
        try:
            while True:
                responses = self.client.simGetImages([airsim.ImageRequest("front", airsim.ImageType.Scene, False, False)])
                photo = np.fromstring(responses[0].image_data_uint8, dtype=np.uint8)
                img = photo.reshape(responses[0].height, responses[0].width, 3)
                img = img[...,::-1]   #brg to rgb
                #print(img.shape)
                for lidar_name in lidar_names:
                    filename = f"{vehicle_name}_{lidar_name}_pointcloud.asc"
                    if not existing_data_cleared:
                        f = open(filename,'w')
                    else:
                        f = open(filename,'a')
                    lidar_data = self.client.getLidarData(lidar_name=lidar_name,vehicle_name=vehicle_name)
                    points = len(lidar_data.point_cloud)/3
                    print("points obtained: ",points)
                    for i in range(0, len(lidar_data.point_cloud), 3):
                        pixel = int(i/3)
                        if pixel < 400:
                            xyz = lidar_data.point_cloud[i:i+3]
                            print("pixel:",pixel)
                            r = img[399-pixel][350][0]
                            g = img[399-pixel][350][1]
                            b = img[399-pixel][350][2]
                            #print(r,g,b)
                            rgb = int('%02x%02x%02x' % (r, g, b), 16)
                            f.write("%f %f %f %f\n" % (xyz[0],xyz[1],xyz[2],rgb))
                        else:
                            break
                            
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
        convert(f"{vehicle}_{lidar_name}_pointcloud.asc")
        os.remove(f"{vehicle}_{lidar_name}_pointcloud.asc")
    
