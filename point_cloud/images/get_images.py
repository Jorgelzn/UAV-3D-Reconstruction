import os
import airsim
import numpy as np



client = airsim.VehicleClient()

responses = client.simGetImages([
    airsim.ImageRequest("front-left", airsim.ImageType.Scene, False, False),
    airsim.ImageRequest("front-right", airsim.ImageType.Scene, False, False)])


# get numpy array
left = np.fromstring(responses[0].image_data_uint8, dtype=np.uint8) 
right = np.fromstring(responses[1].image_data_uint8, dtype=np.uint8) 

# reshape array to 4 channel image array H X W X 4
img_rgb_left = left.reshape(responses[0].height, responses[0].width, 3)
img_rgb_right = left.reshape(responses[1].height, responses[1].width, 3)


# write to png 
airsim.write_png(os.path.normpath("leftimage" + '.png'), img_rgb_left) 
airsim.write_png(os.path.normpath("rightimage" + '.png'), img_rgb_right) 