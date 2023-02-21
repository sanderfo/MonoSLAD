import rospy
import rosbag
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Vector3, TransformStamped
from tf2_msgs.msg import TFMessage
import h5py
from cv_bridge import CvBridge
import cv2

base_path = "PLE_training/fall/"

with h5py.File(base_path + "sensor_records.hdf5", "r") as file:
    l = list(file.keys())
    print(l)
    f = file["trajectory_4000"].keys()
    print(f)
    camera_data = file["trajectory_4000"]["camera_data"]
    
    c_down = camera_data["color_down"]
    acc = file["trajectory_4000"]["imu"]["accelerometer"]
    gyr = file["trajectory_4000"]["imu"]["gyroscope"]
    gt_rot = file["trajectory_4000"]["groundtruth"]["attitude"]
    gt_pos = file["trajectory_4000"]["groundtruth"]["position"]

    bag = rosbag.Bag("midair.bag", "w")

    bridge = CvBridge()

    img_number = 0

    for i, (a, g, rot, pos) in enumerate(zip(acc, gyr, gt_rot, gt_pos)):
        t = 1000 + i/100.0
        
        imu_msg = Imu()
        imu_msg.header.frame_id = "imu"
        
        ros_time = rospy.Time.from_sec(t)
        imu_msg.header.stamp = ros_time
        imu_msg.angular_velocity = Vector3(g[1], g[0], -g[2])
        
        imu_msg.linear_acceleration = Vector3(a[1], a[0], -a[2])

        bag.write("imu", imu_msg, ros_time)

        


        if i % 4 == 0:
            
            down_filename = camera_data["color_down"][img_number]
            left_filename = camera_data["color_left"][img_number]
            
            image_down = cv2.imread(base_path + down_filename, cv2.IMREAD_GRAYSCALE)
            image_left = cv2.imread(base_path + left_filename, cv2.IMREAD_GRAYSCALE)
            img_number += 1

            down_msg = bridge.cv2_to_imgmsg(image_down, encoding="mono8")
            left_msg = bridge.cv2_to_imgmsg(image_left, encoding="mono8")

            down_msg.header.stamp = ros_time
            left_msg.header.stamp = ros_time

            bag.write("cam_down", down_msg, ros_time)
            bag.write("cam_left", left_msg, ros_time)

        if i % 8 == 0:
            
            posestamped = TransformStamped()

            posestamped.transform.rotation.w = rot[0]
            posestamped.transform.rotation.x = rot[2]
            posestamped.transform.rotation.y = rot[1]
            posestamped.transform.rotation.z = -rot[3]

        
            posestamped.transform.translation.x = pos[1]
            posestamped.transform.translation.y = pos[0]
            posestamped.transform.translation.z = -pos[2]

            posestamped.header.stamp = ros_time
            posestamped.header.seq = 0
            posestamped.header.frame_id = "world"
            posestamped.child_frame_id = "body"

            bag.write("/midair/transform", posestamped, ros_time)

    bag.close()






            



        



    