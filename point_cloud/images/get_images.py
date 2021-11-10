import os
import airsim
import numpy as np
import time
import keyboard  
from pynput.keyboard import Listener


#create directories for record
def create_folders():
    exists=True
    number=1

    while exists:
        try:
            # Create target Directory
            directory ="records/record_"+str(number)
            os.mkdir(directory)
            exists=False
        except FileExistsError:
            number+=1

    os.mkdir(directory+"/color_images")
    os.mkdir(directory+"/depth_images")
    print("Record",number,"folders created!") 
    
    return directory


def take_photo(client,record_path,img_number):

    responses = client.simGetImages([airsim.ImageRequest("front", airsim.ImageType.Scene, False, False)])

    # get numpy array
    photo = np.fromstring(responses[0].image_data_uint8, dtype=np.uint8) 

    # reshape array to 4 channel image array H X W X 4
    img_rgb = photo.reshape(responses[0].height, responses[0].width, 3)

    airsim.write_png(os.path.normpath(record_path+"/rgb_img_" +str(img_number)+".png"), img_rgb) 


def record(client,fps):
    directory = create_folders()
    file = open(directory+"/positions.txt", "w")
    rec = True
    frame = time.time()
    img_number=0
    while rec:

        clock = time.time()
        if clock-frame>fps:
            take_photo(client,directory+"/color_images",img_number)
            img_number+=1
            frame =clock

        if keyboard.is_pressed('s'):
            print("Stoping record")
            rec = False

    file.close()


def start_record(key):
    try:
        if key.char =='r':
            fps = 2
            print("Starting record at",1/fps,"FPS")
            record(client,fps)
        if key.char =='e':
            return False
    except:
        pass


if __name__=="__main__":

    client = airsim.VehicleClient()
    print("press R to start recording")
    print("Press S to stop recording")
    print("press e to exit")
    with Listener(on_release = start_record) as listener:
        listener.join()




