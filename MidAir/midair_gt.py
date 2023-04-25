
import h5py
import cv2
import numpy as np
import pyquaternion as pyq
from PIL import Image as Pimage

        

base_path = "PLE_training/fall/"

def open_float16(image_path):
    pic = Pimage.open(image_path)
    img = np.asarray(pic, np.uint16)
    img.dtype = np.float16
    return img

with h5py.File(base_path + "sensor_records.hdf5", "r") as file:
    l = list(file.keys())
    print(l)
    f = file["trajectory_4000"].keys()
    print(f)
    camera_data = file["trajectory_4000"]["camera_data"]
    
    gt_rot = file["trajectory_4000"]["groundtruth"]["attitude"]
    

    img_number = 0

    for i, rot in enumerate(gt_rot):
        t = 1000 + i/100.0
        

        if i % 4*10 == 0:
            
            
            normals_filename = camera_data["normals"][img_number]
            
            normals = np.array(cv2.imread(base_path + normals_filename), dtype=np.float64)
            
            normals_z_dir = normals*2 - 255

        
            img_number += 10

            quaternion = pyq.Quaternion(rot)
            
            R = quaternion.rotation_matrix
            new_normals = np.zeros((normals_z_dir.shape[0], normals_z_dir.shape[1], 3, 1))
            
            
            new_normals[:,:,0,0] = normals_z_dir[:,:,2]
            new_normals[:,:,1,0] = normals_z_dir[:,:,0]
            new_normals[:,:,2,0] = normals_z_dir[:,:,1]
            
            test_tr = quaternion.rotate(new_normals[500,500])
            new_tr = np.matmul(R, new_normals)
            print(img_number-10, R.T)
            print(new_tr[500,500], test_tr)
           
            out_z_dir = np.array((-new_tr + 255)/2)
            out = out_z_dir.copy()
            out[:,:,0] = out_z_dir[:,:,1,:]
            out[:,:,1] = out_z_dir[:,:,2,:]
            out[:,:,2] = out_z_dir[:,:,0,:]

            print(normals[500,500], out[500,500], (quaternion.rotate(new_normals[500,500]) + 255)/2)

            ok = cv2.imwrite(normals_filename, out[:,:,:,0])
            
   






            



        



    