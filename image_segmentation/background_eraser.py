from turtle import back
import numpy as np
import cv2
import os
from tqdm import tqdm
background_color = [198,215,20]
background_range_r = range(background_color[0]-5,background_color[0]+5)
background_range_g = range(background_color[1]-5,background_color[1]+5)
background_range_b = range(background_color[2]-5,background_color[2]+5)

images = os.listdir("input_images")

#clean output folder
img_dir = "output"
for f in os.listdir(img_dir):
    os.remove(os.path.join(img_dir, f))

for image in tqdm(images):
    img = cv2.imread("input_images/"+image)
    mask = cv2.imread("segmentation/"+image)
    mask = np.dstack((mask, np.zeros((len(mask),len(mask[0])))))
    for row in range(len(mask)):
        for  column in range(len(mask[0])):
            
            if int(mask[row][column][0]) not in background_range_r and int(mask[row][column][1]) not in background_range_g and int(mask[row][column][2]) not in background_range_b:
                mask[row][column][3]=255
                
    img = np.dstack((img,mask[:,:,3]))

    cv2.imwrite("output/"+image[:-4]+".png",img)