import airsim
import os
import getpass

from airsim.client import VehicleClient

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

    list = ['# .PCD v.5 - Point Cloud Data file format\n','VERSION .5\n','FIELDS x y z\n','SIZE 4 4 4\n','TYPE F F F\n','COUNT 1 1 1\n']

    output.writelines(list)
    output. write ('WIDTH') # Note that there are spaces behind it
    output.write(str(count))
    output.write('\nHEIGHT')
    output. write (str(1))# mandatory type conversion, file input can only be STR format
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
                for lidar_name in lidar_names:
                    filename = f"{vehicle_name}_{lidar_name}_pointcloud.asc"
                    if not existing_data_cleared:
                        f = open(filename,'w')
                    else:
                        f = open(filename,'a')
                    lidar_data = self.client.getLidarData(lidar_name=lidar_name,vehicle_name=vehicle_name)

                    for i in range(0, len(lidar_data.point_cloud), 3):
                        xyz = lidar_data.point_cloud[i:i+3]
                        
                        f.write("%f %f %f %d %d %d \n" % (xyz[0],xyz[1],xyz[2],255,255,0))
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
    
