from keras_segmentation.pretrained import pspnet_50_ADE_20K , pspnet_101_cityscapes, pspnet_101_voc12
import os
from tqdm import tqdm
import cv2
import numpy as np

#model = pspnet_50_ADE_20K() # load the pretrained model trained on ADE20k dataset

#model = pspnet_101_cityscapes() # load the pretrained model trained on Cityscapes dataset

model = pspnet_101_voc12() # load the pretrained model trained on Pascal VOC 2012 dataset

# load any of the 3 pretrained models
images = os.listdir("input_images")

#clean segmentation folder
img_dir = "segmentation"
for f in os.listdir(img_dir):
    os.remove(os.path.join(img_dir, f))

for img in tqdm(images): 
    out = model.predict_segmentation(
        inp="input_images/"+img,
        out_fname="segmentation/"+img
    )

