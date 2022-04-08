from keras_segmentation.pretrained import pspnet_50_ADE_20K , pspnet_101_cityscapes, pspnet_101_voc12
import os
from tqdm import tqdm
import cv2
import numpy as np

#load machine learning models

#model = pspnet_50_ADE_20K() # load the pretrained model trained on ADE20k dataset
#model = pspnet_101_cityscapes() # load the pretrained model trained on Cityscapes dataset
model = pspnet_101_voc12() # load the pretrained model trained on Pascal VOC 2012 dataset

# Get the default directory for images
scan_path = os.path.join(os.path.expanduser('~'), 'Documents', 'AirSim',"scan")
images_dir = os.path.join(scan_path,"images")
images = os.listdir(images_dir)

#Make the segmentation prediction and create masks


masks_dir = os.path.join(scan_path,"masks")

#clean masks folder
for f in os.listdir(masks_dir ):
    os.remove(os.path.join(masks_dir, f))

print("Detecting background...")

#make predictions
for img in tqdm(images): 
    out = model.predict_segmentation(
        inp=os.path.join(images_dir,img),
        out_fname=os.path.join(masks_dir,img)
    )

#Delete background using the masks

#Define default background values
background_color = [198,215,20]
background_range_r = range(background_color[0]-5,background_color[0]+5)
background_range_g = range(background_color[1]-5,background_color[1]+5)
background_range_b = range(background_color[2]-5,background_color[2]+5)


segmentation_dir = os.path.join(scan_path,"segmentation")

#clean segmentation folder
for f in os.listdir(segmentation_dir):
    os.remove(os.path.join(segmentation_dir, f))

print("Deleting background...")

#Segmentationf of the images
for img in tqdm(images):
    image = cv2.imread(os.path.join(images_dir,img))
    mask = cv2.imread(os.path.join(masks_dir,img))
    mask = np.dstack((mask, np.zeros((len(mask),len(mask[0])))))
    
    #create transparency mask
    for row in range(len(mask)):
        for  column in range(len(mask[0])):
            if int(mask[row][column][0]) not in background_range_r and int(mask[row][column][1]) not in background_range_g and int(mask[row][column][2]) not in background_range_b:
                mask[row][column][3]=255

    #Apply mask to image           
    image = np.dstack((image,mask[:,:,3]))

    cv2.imwrite(os.path.join(segmentation_dir,img),image)
