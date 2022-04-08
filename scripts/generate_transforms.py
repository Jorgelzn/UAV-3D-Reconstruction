import os
import numpy as np
import json
import cv2

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



if __name__=="__main__":

    AABB_SCALE = 16
    FOLDER = "segmentation"

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


    with open(os.path.join(airsim_path,"scan","camera_data.txt"), "r") as f:
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
            name = os.path.join(airsim_path,"scan",FOLDER,elems[0])
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
    with open(os.path.join(airsim_path,"scan",FOLDER+"_transforms.json"), "w") as outfile:
        json.dump(out, outfile, indent=2)