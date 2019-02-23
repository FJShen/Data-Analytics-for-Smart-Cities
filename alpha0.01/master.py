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
        br = CvBridge()
        im = br.imgmsg_to_cv2(data, desired_encoding="mono16")
        print(im.shape,type(im))
        im.tofile("/media/nvidia/ExtremeSSD/depth_images/xxx.txt", sep="\t", format="%s")
        #cv2.imwrite("/media/nvidia/ExtremeSSD/depth_images/" + str(time.time()) + ".jpg", im)

def master():
	rospy.init_node('master', anonymous=True)
        # rospy.Subscriber('gps_meas', Odometry, gps_callback)
        # rospy.Subscriber('RGB', Image, rgb_callback)
        rospy.Subscriber('depth', Image, depth_callback)
        rospy.spin()
	

if __name__ == "__main__":
	master()
