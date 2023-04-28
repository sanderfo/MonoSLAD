
import h5py
import cv2
import numpy as np
import pyquaternion as pyq
from PIL import Image as Pimage
import matplotlib.pyplot as plt

        

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
    step = 1
    est_filepath = "../eval/left.json"
            
    estimated_file = cv2.FileStorage(est_filepath, 0)

    for i, rot in enumerate(gt_rot):
        t = 1000 + i/100.0
        t_nsec = int(t*1e9)
        
        if i % (4*step) == 0:

            #print(t_nsec)
            
            
            
            estimated_node = estimated_file.getNode("final" + str(t_nsec))
                
            if not estimated_node.isNone():
                estimated = estimated_node.mat()
                
                
                depth_filename = camera_data["depth"][img_number]
                #print(estimated.shape)
                depths_16 = open_float16(base_path + depth_filename)
                depths = np.float32(depths_16)
                d_resized = cv2.resize(depths, (estimated.shape[0], estimated.shape[1]), interpolation=cv2.INTER_CUBIC)
                mask = np.ones((256,256), dtype=bool)
                d_resized[d_resized < -60000] = 65535.0

                print(d_resized[100,200], depths[100*4, 200*4], depths_16[100*4, 200*4])
                mask[estimated < 1.0] = False
                mask[d_resized < 0.0] = False
                mask[estimated > 20.0] = False

                est_masked = estimated[mask]
                gt_masked = d_resized[mask]

                prop1 = est_masked/gt_masked
                
                prop2 = gt_masked/est_masked
                
                con = np.concatenate((prop1[:,np.newaxis], prop2[:,np.newaxis]), axis=1)
                error = np.max(con, axis=1)
                print((100.0*error[error < 1.25].size)/(error.size))


                #index = np.unravel_index(np.argmin(d_resized), d_resized.shape)
                print(depths_16[200*4,200*4], estimated[200,200])
               
                #print(mask)
                
                #print(np.average(np.abs(estimated[estimated > 0.5] - d_resized[estimated > 0.5])))
        
            img_number += step

            
        
            
   






            



        



    