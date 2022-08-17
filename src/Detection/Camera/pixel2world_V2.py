#!/usr/bin/env python2

import numpy as np
import math 

import rospy
import time

from darknet_ros_msgs.msg import BoundingBoxes
from vision_msg.msg import bbox_3d, bboxes_3d

################ Parameter ##################
VEHICLE_LENGTH = 5.1 ##  K5 
REAR2CAMERA = 3.1 ##  ( X: 2  +1 = 3)
HEIGHT = 1

fov_deg = 40 # deg
image_width = 640
image_height = 480

rot_roll  = 0
rot_pitch = 0
roll_yaw  = 0

Class_height = 1.8

class pixel2world:
    
    def __init__(self):

        rospy.init_node('px_2_world')

        box_sub_topic     = '/darknet_ros/bounding_boxes'

        bbox_sub          = rospy.Subscriber(box_sub_topic, BoundingBoxes, self.callbackBBOX)

        self.bboxes_3d_pub       = rospy.Publisher('/vision_bbox',bboxes_3d,queue_size=1)


        self.darknetFlag = False
        

    
    def bbox_conversion(self, bbox):

        xmin = bbox.xmin
        xmax = bbox.xmax
        ymin = bbox.ymin
        ymax = bbox.ymax
        bbox_class = bbox.Class

        C_x = image_width/2
        C_y = image_height/2

        fov_rad = np.deg2rad(fov_deg)

        f_x = C_x/(math.tan(fov_rad/2))
        f_y = C_y/(math.tan(fov_rad/2))

        if f_x>=f_y:
            f_max = f_x
        else:
            f_max = f_y

        skew_coeff = 0

        intrinsic_matrix = np.array([[f_max,skew_coeff,C_x],[0, f_max, 0],[0, 0, 1]])
        intrinsic_inv = np.linalg.pinv(intrinsic_matrix)

        bbox_TL = np.array([[xmin, ymin, 1]])
        bbox_TL = bbox_TL.reshape(3,1)

        bbox_BR = np.array([[xmax, ymax, 1]])
        bbox_BR = bbox_BR.reshape(3,1)

        bbox_TL_3d_no_gain = intrinsic_inv.dot(bbox_TL)
        bbox_BR_3d_no_gain = intrinsic_inv.dot(bbox_BR)

        height_norm = abs(bbox_TL_3d_no_gain[1,0] - bbox_BR_3d_no_gain[1,0])

        arbitrary_gain = Class_height/(height_norm)

        bbox_TL_3d = bbox_TL_3d_no_gain*arbitrary_gain 
        bbox_BR_3d = bbox_BR_3d_no_gain*arbitrary_gain     

        center_x = (bbox_BR_3d[2,0]+ bbox_TL_3d[2,0])/2 + VEHICLE_LENGTH - REAR2CAMERA
        center_y = -(bbox_BR_3d[0,0]+ bbox_TL_3d[0,0])/2

        
        return center_x, center_y

    def callbackBBOX(self,msg):

        self.darknetFlag = True

        self.bbox_3d_array = bboxes_3d()
        self.bbox_3d_array.bboxes_3d = []
        self.bbox_3d_array.header.frame_id = 'world'
        #self.bbox_3d_array.header.stamp = self.imageTime
        


        for bbox in msg.bounding_boxes:
            
            if bbox.Class =='person':
                bbox_3d_element = bbox_3d()
                bbox_3d_element.header.frame_id = 'world'
                bbox_3d_element.id = bbox.id
                
                center_x,center_y = self.bbox_conversion(bbox)

                bbox_3d_element.center_x = center_x
                bbox_3d_element.center_y = center_y

                bbox_3d_element.probability = bbox.probability
                bbox_3d_element.Class = bbox.Class

                self.bbox_3d_array.bboxes_3d.append(bbox_3d_element)


        print self.bbox_3d_array

        self.bboxes_3d_pub.publish(self.bbox_3d_array)
        
    def main(self):

        print '\n','hello px2world!','\n'

        while not self.darknetFlag:

            print 'Waiting for darknet....'

            time.sleep(0.5)
            
        
if __name__ == '__main__':

    try:

        p2w_node = pixel2world()
        p2w_node.main()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
