import numpy as np
import cv2
import getpass
import open3d as o3d

model_name = "C:/Users/"+getpass.getuser()+"/Desktop/Workspace/models/model-f6b98070.onnx" #MiDaS v2.1 Large

model = cv2.dnn.readNet(model_name)

if (model.empty()):
    print("could not load the neural net!")

model.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
model.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

img = cv2.imread("leftimage.png")
print(img.shape)
imgHeight, imgWidth, channels = img.shape

#create blob from image
blob = cv2.dnn.blobFromImage(img, 1/255., (384,384), (123.675, 116.28, 103.53),True,False)

#input for the model
model.setInput(blob)

#model prediction
output = model.forward()

output = output[0,:,:]
output = cv2.resize(output, (imgWidth, imgHeight))

# normalize output
output = cv2.normalize(output, None, 0,255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
cv2.imwrite("leftdepth.png", output)


#create point cloud

color_raw = o3d.io.read_image("leftimage.png")
depth_raw = o3d.io.read_image("leftdepth.png")

rgbd_img = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw,depth_raw)

camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
    o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_img,camera_intrinsic)

pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
o3d.io.write_point_cloud("pointscloud.pcd", pcd)
o3d.visualization.draw_geometries([pcd])



