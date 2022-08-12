#!/usr/bin/env python
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        lower_wlane = np.array([0,0,205])
        upper_wlane = np.array([30,60,255])

        lower_ylane = np.array([0,70,120])
        upper_ylane = np.array([40,195,230])

        img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)
        img_ylane = cv2.inRange(img_hsv, lower_ylane, upper_ylane)

        img_wlane = cv2.cvtColor(img_wlane, cv2.COLOR_GRAY2BGR)
        img_ylane = cv2.cvtColor(img_ylane, cv2.COLOR_GRAY2BGR)

        img_lane = cv2.bitwise_or(img_wlane, img_ylane)
        
        img_concat = np.concatenate([img_wlane, img_ylane,img_lane], axis=1)

        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1) 


if __name__ == '__main__':

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 