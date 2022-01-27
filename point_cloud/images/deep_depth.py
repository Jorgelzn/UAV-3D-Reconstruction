import numpy as np
import cv2
import getpass
import open3d as o3d
import os

model_name = "C:/Users/"+getpass.getuser()+"/Desktop/Workspace/models/model-f6b98070.onnx" #MiDaS v2.1 Large

model = cv2.dnn.readNet(model_name)

if (model.empty()):
    print("could not load the neural network")

model.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
model.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

record_number = 3
record_path = os.path.dirname(__file__)+"/records/record_"+str(record_number)
color_path =  record_path+"/mono_color_images"
depth_path = record_path+"/mono_depth"
color_images = os.listdir(color_path)

imgs = [cv2.imread(color_path+"/"+img_path) for img_path in color_images]
number=0

for img in imgs:

    imgHeight, imgWidth, channels = img.shape

    #create blob from image, input para el modelo, imagenes de 384x384
    blob = cv2.dnn.blobFromImage(img, 1/255., (384,384), (123.675, 116.28, 103.53),True,False)

    #input for the model
    model.setInput(blob)

    #model prediction
    output = model.forward()

    output = output[0,:,:]
    output = cv2.resize(output, (imgWidth, imgHeight))

    # normalize output
    #output = cv2.normalize(output, None, 0,255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    cv2.imwrite(depth_path+"/depth_img_"+str(number)+".png", output)
    number+=1




