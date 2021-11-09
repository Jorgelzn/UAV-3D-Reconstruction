import os
import airsim
import numpy as np
from matplotlib import pyplot as plt
import cv2 as cv

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

left_image = cv.imread("leftimage.png", cv.IMREAD_GRAYSCALE)
right_image = cv.imread("rightimage.png", cv.IMREAD_GRAYSCALE)

stereo = cv.StereoBM_create(numDisparities=16, blockSize=15)
depth = stereo.compute(left_image, right_image)

plt.imshow(depth)
plt.axis("off")
plt.show()