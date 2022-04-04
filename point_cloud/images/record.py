import os
import airsim
import numpy as np
import time
import keyboard  
import json

def generate_transform_matrix(pos,rot,center):
    def Rx(theta):
      return np.matrix([[ 1, 0            , 0            ],
                        [ 0, np.cos(theta),-np.sin(theta)],
                        [ 0, np.sin(theta), np.cos(theta)]])
    def Ry(theta):
      return np.matrix([[ np.cos(theta), 0, np.sin(theta)],
                        [ 0            , 1, 0            ],
                        [-np.sin(theta), 0, np.cos(theta)]])
    def Rz(theta):
      return np.matrix([[ np.cos(theta), -np.sin(theta), 0 ],
                        [ np.sin(theta), np.cos(theta) , 0 ],
                        [ 0            , 0             , 1 ]])

    R = Rz(rot[2]) * Ry(rot[1]) * Rx(rot[0])
    xf_rot = np.eye(4)
    xf_rot[:3,:3] = R

    xf_pos = np.eye(4)
    xf_pos[:3,3] = pos - center
    # barbershop_mirros_hd_dense:
    # - camera plane is y+z plane, meaning: constant x-values
    # - cameras look to +x

    # Don't ask me...
    extra_xf = np.matrix([
        [-1, 0, 0, 0],
        [ 0, 0, 1, 0],
        [ 0, 1, 0, 0],
        [ 0, 0, 0, 1]])
    # NerF will cycle forward, so lets cycle backward.
    shift_coords = np.matrix([
        [0, 0, 1, 0],
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]])
    xf = shift_coords @ extra_xf @ xf_pos
    assert np.abs(np.linalg.det(xf) - 1.0) < 1e-4
    xf = xf @ xf_rot
    return xf

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
    data ={

    }
    while True:
        clock = time.time()
        if clock-frame>fps:
            take_photo(client,directory,img_number,file)
            img_number+=1
            frame =clock

        if keyboard.is_pressed('s'):
            print("Stoped recording")
            with open(directory+"/transforms.json", 'w') as outfile:
                json.dump(data, outfile)
            break

    file.close()


if __name__=="__main__":

    client = airsim.VehicleClient()
    client.confirmConnection()
    record(client,0.2)







