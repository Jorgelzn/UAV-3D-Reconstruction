import os
import numpy as np
from matplotlib import pyplot as plt
import cv2 as cv


record_number = 3
record_path = os.path.dirname(__file__)+"/records/record_"+str(record_number)
color_path =  record_path+"/stereo_color_images"
depth_path = record_path+"/stereo_depth"
color_left_images = os.listdir(color_path+"/left")
color_right_images = os.listdir(color_path+"/right")

left_imgs = [cv.imread(color_path+"/left/"+img_path,cv.IMREAD_GRAYSCALE) for img_path in color_left_images]
right_imgs = [cv.imread(color_path+"/right/"+img_path,cv.IMREAD_GRAYSCALE) for img_path in color_right_images]

stereo = cv.StereoBM_create(numDisparities=16, blockSize=15)

for img in range(len(left_imgs)):
    disparity = stereo.compute(left_imgs[img],right_imgs[img])
    cv.imwrite(depth_path+"/depth_img_"+str(img)+".png", disparity)

