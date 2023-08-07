
import os
import shutil
import cv2
import rospy
import time
import numpy as np
import sys
from datetime import datetime
import rosbag
from tqdm import tqdm
import open3d as o3d
from threading import Thread


point_cloud_topic = "/ouster/points"
rgb_img_topic = "/ximea_cam2/image_raw"
thermal_img_topic = "/flir_boson/image_raw"

rgb_info_topic = "/ximea_cam2/camera_info"
thermal_info_topic = "/flir_boson/camera_info"

tf_topic = "/tf"
tf_static_topic = "/tf_static"

joint_topic = "/joint_states"

rosout_topic = "/rosout"
rosoutagg_topic = "/rosout_agg"


BAG_IN = "/home/shanks/catkin_ws/2023-05-02-15-10-12.bag"
BAG_OUT = "/home/shanks/catkin_ws/2023-05-02-15-10-12_fixed2.bag"
if __name__ == "__main__":
    
    
    print("[Convert Timestamp] running...")
    
    keep_topic = [rgb_img_topic,
                  thermal_img_topic,
                  rgb_info_topic,
                  thermal_info_topic,
                  tf_topic,
                  tf_static_topic,
                  joint_topic,
                  rosout_topic,
                  rosoutagg_topic]
    
    rospy.init_node("convert_timestamp_ouster_node", anonymous=True)
    
    first_timestamp = None 
    first_pc = True
    last_stamp = None
    last_pc_stamp = None
    
    with rosbag.Bag(BAG_OUT, 'w') as bag:
        for topic, msg, timestamp in rosbag.Bag(BAG_IN, 'r').read_messages():
            if topic in keep_topic:
                bag.write(topic, msg, timestamp)
                
                # if topic == rgb_img_topic:
                #     print("img:", msg.header.stamp.secs)
                    
                if last_stamp is None:
                    last_stamp = msg.header.stamp
                    
            elif topic == point_cloud_topic:
                if last_pc_stamp is None:
                    last_pc_stamp = msg.header.stamp                      

                delay = (msg.header.stamp - last_pc_stamp)
                half_delay =delay/2
                 
                stamp = last_stamp + half_delay
                msg.header.stamp = stamp
                bag.write(topic, msg, stamp)
                
                stamp = stamp + half_delay
                msg.header.stamp = stamp
                bag.write(topic, msg, stamp)
                
                print(delay, stamp)
                
                # print("pc:", msg.header.stamp.secs, stamp.secs)
                
                
    
