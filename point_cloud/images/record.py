import os
import airsim
import numpy as np
import time
import keyboard  
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

#create directories for record
def create_folders():
    exists=True
    number=1

    while exists:
        try:
            # Create target Directory
            directory =os.path.dirname(__file__)+"/records/record_"+str(number)
            os.mkdir(directory)
            exists=False
        except FileExistsError:
            number+=1

    os.mkdir(directory+"/images")
    os.mkdir(directory+"/stereo_color_images")
    os.mkdir(directory+"/stereo_color_images/left")
    os.mkdir(directory+"/stereo_color_images/right")
    os.mkdir(directory+"/mono_depth")
    os.mkdir(directory+"/stereo_depth")
    print("Record",number,"folders created!") 
    
    return directory


def take_photo(client,path,img_number,file):

    responses = client.simGetImages([airsim.ImageRequest("front", airsim.ImageType.Scene, False, False),
                                    airsim.ImageRequest("front-left", airsim.ImageType.Scene,False,False),
                                    airsim.ImageRequest("front-right", airsim.ImageType.Scene,False,False)])

    #registrar posiciones de la camara
    file.write("%f %f %f %f %f %f %f\n" %
                  (responses[0].camera_position.x_val, responses[0].camera_position.y_val, responses[0].camera_position.z_val,
                   responses[0].camera_orientation.w_val, responses[0].camera_orientation.x_val,
                   responses[0].camera_orientation.y_val, responses[0].camera_orientation.z_val))
    qvec = np.array([responses[0].camera_orientation.w_val, responses[0].camera_orientation.x_val,
            responses[0].camera_orientation.y_val, responses[0].camera_orientation.z_val])

    tvec = np.array([responses[0].camera_position.x_val, responses[0].camera_position.y_val, responses[0].camera_position.z_val])
    # get numpy array
    photo_mono = np.fromstring(responses[0].image_data_uint8, dtype=np.uint8) 
    photo_stereo_L = np.fromstring(responses[1].image_data_uint8, dtype=np.uint8) 
    photo_stereo_R = np.fromstring(responses[2].image_data_uint8, dtype=np.uint8) 

    # reshape array to 4 channel image array H X W X 4
    img_mono = photo_mono.reshape(responses[0].height, responses[0].width, 3)
    img_stereo_L = photo_stereo_L.reshape(responses[1].height, responses[1].width, 3)
    img_stereo_R = photo_stereo_R.reshape(responses[2].height, responses[2].width, 3)

    airsim.write_png(os.path.normpath(path+"/images/img_mono_" +str(img_number)+".png"), img_mono)
    airsim.write_png(os.path.normpath(path+"/stereo_color_images/left/img_stereo_L_" +str(img_number)+".png"), img_stereo_L)  
    airsim.write_png(os.path.normpath(path+"/stereo_color_images/right/img_stereo_R_" +str(img_number)+".png"), img_stereo_R) 
    name = os.path.normpath("img_mono_" +str(img_number)+".png")
    print(tvec)
    return name,qvec,tvec

def record(client,fps):
    directory = create_folders()
    file = open(directory+"/positions.txt", "w")
    actual_frame = time.time()
    img_number=0
    AABB_SCALE = 16
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
    print("Started Recording")
    while True:
        clock = time.time()
        if clock-actual_frame>fps:
            name,qvec,tvec = take_photo(client,directory,img_number,file)
            img_number+=1

            b=sharpness(directory+"/images/"+name)
            print(name, "sharpness=",b)
            R = qvec2rotmat(-qvec)
            t = tvec.reshape([3,1])
            m = np.concatenate([np.concatenate([R, t], 1), bottom], 0)
            c2w = np.linalg.inv(m)
            c2w[0:3,2] *= -1 # flip the y and z axis
            c2w[0:3,1] *= -1
            c2w = c2w[[1,0,2,3],:] # swap y and z
            c2w[2,:] *= -1 # flip whole world upside down

            up += c2w[0:3,1]
            #print(c2w)
            name = "./images/"+name
            frame={"file_path":name,"sharpness":b,"transform_matrix": c2w}
            out["frames"].append(frame)

            actual_frame =clock

        if keyboard.is_pressed('s'):
            print("Stoped recording")
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
            print("writing",directory+"/transforms.json")
            with open(directory+"/transforms.json", "w") as outfile:
                json.dump(out, outfile, indent=2)
            break

    file.close()


if __name__=="__main__":

    client = airsim.VehicleClient()
    client.confirmConnection()
    record(client,1)







