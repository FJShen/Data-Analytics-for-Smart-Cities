#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import time
import cv2
import numpy as np
from cv_bridge import CvBridge

def gps_callback(data):
        print("1: ", data.pose.covariance)
        file1.write(str(data.pose.covariance) + "\n")

def rgb_callback(data):        
        br = CvBridge()
        im = br.imgmsg_to_cv2(data, desired_encoding="rgb16")
        cv2.imwrite("/media/nvidia/ExtremeSSD/rgb_images/" + str(time.time()) + ".jpg", im)

def depth_callback(data):
        print("ROS sending delay: " + str(time.time() * (10 ** 3) - (data.header.stamp.secs * (10 ** 3) + data.header.stamp.nsecs * (10 ** -6))) + "ms")
        print("seq: " + str(data.header.seq))
        br = CvBridge()
        im = br.imgmsg_to_cv2(data, desired_encoding="mono16")
        # im.tofile("/media/nvidia/ExtremeSSD/depth_images/xxx.txt", sep="\t", format="%s")
        timestamp1 = time.time()
        filename = "/media/nvidia/ExtremeSSD/depth_images/" + str(timestamp1) + ".npz"
        np.savez_compressed(filename, im=im)
        timestamp2 = time.time()
        print("npz save time: " + str((timestamp2 - timestamp1) * (10 ** 3)) + "ms")
        print(filename)

def master():
	rospy.init_node('master', anonymous=True)
        # rospy.Subscriber('gps_meas', Odometry, gps_callback)
        # rospy.Subscriber('RGB', Image, rgb_callback)
        rospy.Subscriber('depth', Image, depth_callback)
        rospy.spin()
	

if __name__ == "__main__":
	master()