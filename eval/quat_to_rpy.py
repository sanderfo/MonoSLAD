import numpy as np
import pyquaternion as pyq

def rpy(wxyz_string):
    w, x, y, z, i, j, k = wxyz_string.split()
    w = float(w)
    x = float(x)
    y = float(y)
    z = float(z)

    q_optical_camleft = pyq.Quaternion(w=0.5, x=-0.5, y=0.5, z=-0.5)
    t_camleft_sensor = np.array([0.015, 0.055, 0.0065])
    q_sensor_base = pyq.Quaternion(w=0.707388, x=0, y=0.706825, z=0)
    t_sensor_base = np.array([0.1, 0, -0.03])

    T_optical_camleft = q_optical_camleft.transformation_matrix
    T_camleft_sensor = np.identity(4)
    T_camleft_sensor[:3, 3] = t_camleft_sensor
    

    T_sensor_base = q_sensor_base.transformation_matrix
    T_sensor_base[:3, 3] = t_sensor_base

    T_optical_base =  T_sensor_base @ T_camleft_sensor @ T_optical_camleft

    
    T_optical_w = pyq.Quaternion(w,x,y,z).transformation_matrix
    T_optical_w[:3, 3] = np.array([i, j, k])

    T_base_w =  T_optical_w @ T_optical_base
    #T_base_w = np.linalg.inv(T_base_w)
    q = pyq.Quaternion(matrix=T_base_w)
    
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    
    sinp = np.sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
    cosp = np.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
    pitch = 2 * np.arctan2(sinp, cosp) - np.pi / 2

    
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    print(roll)
    print(pitch)
    print(yaw)
    print(T_base_w[0,3], T_base_w[1,3], T_base_w[2,3])
    

rpy("-0.107186 0.742191 -0.660192 -0.042555 -49.927319 91.536131 26.860098")