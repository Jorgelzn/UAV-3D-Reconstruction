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

    os.mkdir(directory+"/mono_color_images")
    os.mkdir(directory+"/stereo_color_images")
    os.mkdir(directory+"/stereo_color_images/left")
    os.mkdir(directory+"/stereo_color_images/right")
    os.mkdir(directory+"/mono_depth")
    os.mkdir(directory+"/stereo_depth")
    print("Record",number,"folders created!") 
    
    return directory


def take_photo(client,path,img_number,file):

    responses = client.simGetImages([airsim.ImageRequest("front", airsim.ImageType.Scene, False, False),
                                    airsim.ImageRequest("front-left", airsim.ImageType.Scene,False,False),
                                    airsim.ImageRequest("front-right", airsim.ImageType.Scene,False,False)])

    #registrar posiciones de la camara
    file.write("%f %f %f %f %f %f %f\n" %
                  (responses[0].camera_position.x_val, responses[0].camera_position.y_val, responses[0].camera_position.z_val,
                   responses[0].camera_orientation.w_val, responses[0].camera_orientation.x_val,
                   responses[0].camera_orientation.y_val, responses[0].camera_orientation.z_val))

    # get numpy array
    photo_mono = np.fromstring(responses[0].image_data_uint8, dtype=np.uint8) 
    photo_stereo_L = np.fromstring(responses[1].image_data_uint8, dtype=np.uint8) 
    photo_stereo_R = np.fromstring(responses[2].image_data_uint8, dtype=np.uint8) 

    # reshape array to 4 channel image array H X W X 4
    img_mono = photo_mono.reshape(responses[0].height, responses[0].width, 3)
    img_stereo_L = photo_stereo_L.reshape(responses[1].height, responses[1].width, 3)
    img_stereo_R = photo_stereo_R.reshape(responses[2].height, responses[2].width, 3)

    airsim.write_png(os.path.normpath(path+"/mono_color_images/img_mono_" +str(img_number)+".png"), img_mono)
    airsim.write_png(os.path.normpath(path+"/stereo_color_images/left/img_stereo_L_" +str(img_number)+".png"), img_stereo_L)  
    airsim.write_png(os.path.normpath(path+"/stereo_color_images/right/img_stereo_R_" +str(img_number)+".png"), img_stereo_R) 


def record(client,fps):
    directory = create_folders()
    file = open(directory+"/positions.txt", "w")
    frame = time.time()
    img_number=0
    print("Started Recording")
    while True:
        clock = time.time()
        if clock-frame>fps:
            take_photo(client,directory,img_number,file)
            img_number+=1
            frame =clock

        if keyboard.is_pressed('s'):
            print("Stoped recording")
            break

    file.close()


if __name__=="__main__":

    client = airsim.VehicleClient()
    client.confirmConnection()
    record(client,5)







