import os
import airsim
import numpy as np
import time
import keyboard  


#create directories for record
def create_folders():
    exists=True
    number=1

    while exists:
        try:
            # Create target Directory
            directory =os.path.dirname(__file__)+"/records/record_"+str(number)
            os.mkdir(directory)
            exists=False
        except FileExistsError:
            number+=1

    os.mkdir(directory+"/color_images")
    os.mkdir(directory+"/depth_images")
    print("Record",number,"folders created!") 
    
    return directory


def take_photo(client,record_path,img_number,file):

    responses = client.simGetImages([airsim.ImageRequest("front", airsim.ImageType.Scene, False, False)])

    #registrar posiciones de la camara
    file.write("%f %f %f %f %f %f %f\n" %
                  (responses[0].camera_position.x_val, responses[0].camera_position.y_val, responses[0].camera_position.z_val,
                   responses[0].camera_orientation.w_val, responses[0].camera_orientation.x_val,
                   responses[0].camera_orientation.y_val, responses[0].camera_orientation.z_val))

    # get numpy array
    photo = np.fromstring(responses[0].image_data_uint8, dtype=np.uint8) 

    # reshape array to 4 channel image array H X W X 4
    img_rgb = photo.reshape(responses[0].height, responses[0].width, 3)

    airsim.write_png(os.path.normpath(record_path+"/rgb_img_" +str(img_number)+".png"), img_rgb) 


def record(client,fps):
    directory = create_folders()
    file = open(directory+"/positions.txt", "w")
    frame = time.time()
    img_number=0
    print("Started Recording")
    while True:
        clock = time.time()
        if clock-frame>fps:
            take_photo(client,directory+"/color_images",img_number,file)
            img_number+=1
            frame =clock

        if keyboard.is_pressed('s'):
            print("Stoped recording")
            break

    file.close()


if __name__=="__main__":

    client = airsim.VehicleClient()
    client.confirmConnection()
    record(client,1)







