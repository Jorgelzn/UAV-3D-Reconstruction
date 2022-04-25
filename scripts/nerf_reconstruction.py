import os
from tqdm import tqdm
import cv2
import json
import numpy as np

def closest_point_2_lines(oa, da, ob, db): # returns point closest to both rays of form o+t*d, and a weight factor that goes to 0 if the lines are parallel
	da = da / np.linalg.norm(da)
	db = db / np.linalg.norm(db)
	c = np.cross(da, db)
	denom = np.linalg.norm(c)**2
	t = ob - oa
	ta = np.linalg.det([t, db, c]) / (denom + 1e-10)
	tb = np.linalg.det([t, da, c]) / (denom + 1e-10)
	if ta > 0:
		ta = 0
	if tb > 0:
		tb = 0
	return (oa+ta*da+ob+tb*db) * 0.5, denom

def rotmat(a, b):
	a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
	v = np.cross(a, b)
	c = np.dot(a, b)
	s = np.linalg.norm(v)
	kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
	return np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2 + 1e-10))

def variance_of_laplacian(image):
	return cv2.Laplacian(image, cv2.CV_64F).var()

def sharpness(imagePath):
	image = cv2.imread(imagePath)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	fm = variance_of_laplacian(gray)
	return fm

def qvec2rotmat(qvec):
	return np.array([
		[
			1 - 2 * qvec[2]**2 - 2 * qvec[3]**2,
			2 * qvec[1] * qvec[2] - 2 * qvec[0] * qvec[3],
			2 * qvec[3] * qvec[1] + 2 * qvec[0] * qvec[2]
		], [
			2 * qvec[1] * qvec[2] + 2 * qvec[0] * qvec[3],
			1 - 2 * qvec[1]**2 - 2 * qvec[3]**2,
			2 * qvec[2] * qvec[3] - 2 * qvec[0] * qvec[1]
		], [
			2 * qvec[3] * qvec[1] - 2 * qvec[0] * qvec[2],
			2 * qvec[2] * qvec[3] + 2 * qvec[0] * qvec[1],
			1 - 2 * qvec[1]**2 - 2 * qvec[2]**2
		]
	])

def generate_transforms(object_dir,scale):
    AABB_SCALE = scale

    # Get the default directory for AirSim
    airsim_path = os.path.join(os.path.expanduser('~'), 'Documents', 'AirSim')

    # Load the settings file
    with open(os.path.join(airsim_path, 'settings.json'), 'r') as fp:
        data = json.load(fp)

    # Get the camera intrinsics
    capture_settings = data['CameraDefaults']['CaptureSettings'][0]
    img_width = capture_settings['Width']
    img_height = capture_settings['Height']
    img_fov = capture_settings['FOV_Degrees']

    # Compute the focal length
    fov_rad = img_fov * np.pi/180
    fdy = img_width /(np.tan(fov_rad/2.0)*2)
    fdx = img_height /(np.tan(fov_rad/2.0)*2)


    with open(os.path.join(object_dir,"camera_data.txt"), "r") as f:
        bottom = np.array([0.0, 0.0, 0.0, 1.0]).reshape([1, 4])
        out = {
            "camera_angle_x": fov_rad,
            "camera_angle_y": fov_rad,
            "fl_x": fdy,
            "fl_y": fdx,
            "k1": 0,
            "k2": 0,
            "p1": 0,
            "p2": 0,
            "cx": img_width/2,
            "cy": img_height/2,
            "w": img_width,
            "h": img_height,
            "aabb_scale": AABB_SCALE,
            "frames": [],
        }
        up = np.zeros(3)

        for line in f:
            elems=line.split(" ") 
            name = os.path.join(object_dir,"images",elems[0])
            b=sharpness(name)
            print(elems[0], "sharpness=",b)
            tvec = np.array(tuple(map(float, elems[1:4])))
            qvec = np.array(tuple(map(float, elems[4:8])))
            R = qvec2rotmat(-qvec)
            t = tvec.reshape([3,1])
            m = np.concatenate([np.concatenate([R, t], 1), bottom], 0)
            c2w = np.linalg.inv(m)
            c2w[0:3,2] *= -1 # flip the y and z axis
            c2w[0:3,1] *= -1
            c2w = c2w[[1,0,2,3],:] # swap y and z
            c2w[2,:] *= -1 # flip whole world upside down

            up += c2w[0:3,1]
            path_name = "./images/"+str(elems[0])
            frame={"file_path":path_name,"sharpness":b,"transform_matrix": c2w}
            out["frames"].append(frame)

    nframes = len(out["frames"])
    up = up / np.linalg.norm(up)
    print("up vector was", up)
    R = rotmat(up,[0,0,1]) # rotate up vector to [0,0,1]
    R = np.pad(R,[0,1])
    R[-1, -1] = 1

    for f in out["frames"]:
        f["transform_matrix"] = np.matmul(R, f["transform_matrix"]) # rotate up to be the z axis

    # find a central point they are all looking at
    print("computing center of attention...")
    totw = 0.0
    totp = np.array([0.0, 0.0, 0.0])
    for f in out["frames"]:
        mf = f["transform_matrix"][0:3,:]
        for g in out["frames"]:
            mg = g["transform_matrix"][0:3,:]
            p, w = closest_point_2_lines(mf[:,3], mf[:,2], mg[:,3], mg[:,2])
            if w > 0.01:
                totp += p*w
                totw += w
    totp /= totw
    print(totp) # the cameras are looking at totp
    for f in out["frames"]:
        f["transform_matrix"][0:3,3] -= totp

    avglen = 0.
    for f in out["frames"]:
        avglen += np.linalg.norm(f["transform_matrix"][0:3,3])
    avglen /= nframes
    print("avg camera distance from origin", avglen)
    for f in out["frames"]:
        f["transform_matrix"][0:3,3] *= 4.0 / avglen # scale to "nerf sized"

    for f in out["frames"]:
        f["transform_matrix"] = f["transform_matrix"].tolist()
    print(nframes,"frames")
    print("writing file")
    with open(os.path.join(object_dir,"transforms.json"), "w") as outfile:
        json.dump(out, outfile, indent=2)


def segmentate(object_dir,mode=0):
    from keras_segmentation.pretrained import pspnet_50_ADE_20K , pspnet_101_cityscapes, pspnet_101_voc12,resnet_pspnet_VOC12_v0_1

    images_dir = os.path.join(object_dir,"images")
    images = os.listdir(images_dir)

    segmentation_dir = os.path.join(object_dir,"segmentation")
    if not os.path.isdir(segmentation_dir):
        os.mkdir(segmentation_dir)

    #load machine learning models
    if mode==0:
        model = pspnet_101_voc12() # load the pretrained model trained on Pascal VOC 2012 dataset
    elif mode==1:
        model = pspnet_50_ADE_20K() # load the pretrained model trained on ADE20k dataset
    elif mode==2:
        model = pspnet_101_cityscapes() # load the pretrained model trained on Cityscapes dataset
    elif mode==3:
        model = resnet_pspnet_VOC12_v0_1()

    #Make the segmentation prediction

    masks_dir = os.path.join(segmentation_dir,"masks")
    if not os.path.isdir(masks_dir):
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

    segmented_images_dir = os.path.join(segmentation_dir,"images")
    if not os.path.isdir(segmented_images_dir):
        os.mkdir(segmented_images_dir)

    print("Deleting background...")

    #Segmentationf of the images
    for img in tqdm(images):
        image = cv2.imread(os.path.join(images_dir,img),cv2.IMREAD_UNCHANGED)
        mask = cv2.imread(os.path.join(masks_dir,img),cv2.IMREAD_UNCHANGED)

        indices = np.where(np.all(mask != background_color, axis=-1))
        indices =(indices[0],indices[1],3)
        image = np.dstack((image, np.zeros(image.shape[:-1])))
        image[indices]=255
        cv2.imwrite(os.path.join(segmented_images_dir,img),image)

if __name__ == "__main__":

    segmentation = True
    transformation = True
    transform_mode = "colmap"
    target = "segmentation"
    n_mission = 0
    n_object = 0
    aabb_scale = 2
    cubes_res = 150
    train_steps=4000

    object_dir = os.path.join(os.path.expanduser('~'), 'Documents', 'AirSim',"data","mission_"+str(n_mission),"object_"+str(n_object))

        
    #segmentate object images
    if segmentation:
        segmentate(object_dir)

    #target for nerf and transforms
    if target=="segmentation":
        target_dir = os.path.join(object_dir,target)
    else:
        target_dir = object_dir
    
    #generate images transforms
    if transformation:

        if transform_mode=="colmap":
            #directory for the colmap script
            os.chdir(target_dir)
            #create transforms.json with colmap
            colmap_script = "python C:/Users/jorge/Desktop/Workspace/instant-ngp/scripts/colmap2nerf.py --colmap_matcher exhaustive --run_colmap --aabb_scale "+str(aabb_scale)
            os.system(colmap_script)
        else:
            generate_transforms(target_dir,aabb_scale)


    #train nerf for reconstruction
    nerf_script = 'python C:/Users/jorge/Desktop/Workspace/instant-ngp/scripts/run.py --scene="'+target_dir+'" --mode="nerf" --save_mesh="'+os.path.join(target_dir,"nerf_object.obj")+'" --marching_cubes_res='+str(cubes_res)+' --n_steps='+str(train_steps)
    os.system(nerf_script)
