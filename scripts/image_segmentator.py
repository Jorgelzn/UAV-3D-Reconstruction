from keras_segmentation.pretrained import pspnet_50_ADE_20K , pspnet_101_cityscapes, pspnet_101_voc12
import os
from tqdm import tqdm
import cv2
import numpy as np


# Get the default directory for images
def segmentate(object_dir):
    images_dir = os.path.join(object_dir,"images")
    images = os.listdir(images_dir)

    #load machine learning models

    #model = pspnet_50_ADE_20K() # load the pretrained model trained on ADE20k dataset
    #model = pspnet_101_cityscapes() # load the pretrained model trained on Cityscapes dataset
    model = pspnet_101_voc12() # load the pretrained model trained on Pascal VOC 2012 dataset

    #Make the segmentation prediction

    masks_dir = os.path.join(object_dir,"masks")
    os.mkdir(masks_dir)

    print("Detecting background...")

    #make predictions
    for img in tqdm(images): 
        out = model.predict_segmentation(
            inp=os.path.join(images_dir,img),
            out_fname=os.path.join(masks_dir,img)
        )

    #Delete background using the masks

    #Define default background values
    background_color = [197,215,20]

    segmentation_dir = os.path.join(object_dir,"segmentation")
    os.mkdir(segmentation_dir)

    print("Deleting background...")

    #Segmentationf of the images
    for img in tqdm(images):
        image = cv2.imread(os.path.join(images_dir,img),cv2.IMREAD_UNCHANGED)
        mask = cv2.imread(os.path.join(masks_dir,img),cv2.IMREAD_UNCHANGED)

        indices = np.where(np.all(mask != background_color, axis=-1))
        indices =(indices[0],indices[1],3)
        image = np.dstack((image, np.zeros(image.shape[:-1])))
        image[indices]=255
        cv2.imwrite(os.path.join(segmentation_dir,img),image)
