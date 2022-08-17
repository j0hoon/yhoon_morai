#!/usr/bin/env python2

'''
Ajou Univ ACL Seo Bomin
bominseo@ajou.ac.kr

Convert image (2D) to world coordinate (3D)
'''

import numpy as np
import math
import os
import cv2
# ROS packages
import rospy
import cv_bridge
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox, ObjectCount
from sensor_msgs.msg import Image
from vision_msg.msg import bbox_3d, bboxes_3d,traffic_light_state,traffic_sign_state
import time


class pixel2world:

    def __init__(self):

        self.probability_thres = 0.80

        rospy.init_node('px_2_world')

        box_sub_topic       = '/darknet_ros/bounding_boxes'
        # objectCount_topic = '/darknet_ros/found_object'
        image_topic         = '/vds_node_localhost_2211/image_raw'
        darknet_image_topic = '/darknet_ros/detection_image'

        bbox_sub           = rospy.Subscriber(box_sub_topic,BoundingBoxes,self.callbackBBOX)
        # objectCount_sub    = rospy.Subscriber(objectCount_topic,  ObjectCount,  self.callbackObjectCount)
        image_sub          = rospy.Subscriber(image_topic,Image,self.callbackImage)
        darknet_image_sub  = rospy.Subscriber(darknet_image_topic, Image, self.callbackDarkent)


        self.bboxes_3d_pub           = rospy.Publisher('/vision_bbox',bboxes_3d,queue_size=1)

        self.traffic_light_state_pub = rospy.Publisher('/traffic_light_state',traffic_light_state,queue_size=1)
        self.traffic_sign_state_pub = rospy.Publisher('/traffic_sign_state',traffic_light_state,queue_size=1)

        self.darknetFlag = False
        

############################## preprocessing ############################

    def getNearestBbox(self, bboxes):

        # bboxes = iter(bboxes)

        bboxes_array = []
        bboxes_distance = []
        
        for bbox in bboxes:
            
            bboxes_array.append(bbox)
            distance = bbox.center_x**2 +bbox.center_y**2

            bboxes_distance.append(distance)
            
        if len(bboxes_distance) > 1:

            nearest_bbox_idx = np.where(min(bboxes_distance))
            nearest_bbox = bboxes_array[nearest_bbox_idx[0][0]]

        else:
            nearest_bbox = bboxes

        return nearest_bbox


    def classificateTrafficLightState(self, bbox_image):


        traffic_light_bbox = self.getTrafficLightState(bbox_image)


        y = (traffic_light_bbox.ymax + traffic_light_bbox.ymin)/2
        x = (traffic_light_bbox.xmax + traffic_light_bbox.xmin)/2

        h = traffic_light_bbox.ymax - traffic_light_bbox.ymin
        w = traffic_light_bbox.xmax - traffic_light_bbox.xmin


        img_roi = traffic_light_bbox[y:y+h, x:x+w]

        start_time_imageLoad1 = time.time()
        img_hsv=cv2.cvtColor(img_roi, cv2.COLOR_BGR2HSV)
        end_time_imageLoad1 = time.time()
        print end_time_imageLoad1-start_time_imageLoad1


        state = 0

        ############## red ####################

        lower_red = np.array([0,50,50])
        upper_red = np.array([10,255,255])
        mask_red0 = cv2.inRange(img_hsv, lower_red, upper_red)

        lower_red = np.array([170,50,50])
        upper_red = np.array([180,255,255])
        mask_red1 = cv2.inRange(img_hsv, lower_red, upper_red)

        mask_red = mask_red0+mask_red1

        ################## green ##########################
        lower_green = np.array([36,25,25])
        upper_green = np.array([86,255,255])
        mask_green = cv2.inRange(img_hsv, lower_green, upper_green)

        ################### yellow ###########################

        lower_yellow = np.array([15,0,0])
        upper_yellow = np.array([36,255,255])
        mask_yellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

        red_shape = np.size(np.where(mask_red==0))
        yellow_shape =  np.size(np.where(mask_yellow==0))
        green_shape =  np.size(np.where(mask_green==0))
        ##################################

        print red_shape
        print yellow_shape
        print green_shape

        if red_shape == min([red_shape, yellow_shape, green_shape]):

            state = 3
            print 'red'

        elif yellow_shape == min([red_shape, yellow_shape, green_shape]):

            state = 2
            print 'yellow'

        elif green_shape == min([red_shape, yellow_shape, green_shape]):

            state = 1
            print 'green'

        return state 


    def getTrafficLightState(self,bboxes_traffic_light):

        traffic_light = traffic_light_state()
        traffic_light.header.frame_id = 'world'
        traffic_light.header.stamp = self.imageTime

        state = 0

        traffic_light_bbox = self.getNearestBbox(bboxes_traffic_light.bboxes_3d)

        longDistance = traffic_light_bbox[0].center_x

        if traffic_light_bbox[0].Class == 'redlight':

            state = 2

        elif traffic_light_bbox[0].Class == 'yellowlight':

            state = 1

        traffic_light.longDistance = longDistance
        traffic_light.state = state
        
        return traffic_light


    def getTrafficSignState(self,bboxes_traffic_sign):

        traffic_sign = traffic_sign_state()
        traffic_sign.header.frame_id = 'world'
        traffic_sign.header.stamp = self.imageTime

        traffic_sign_bbox = self.getNearestBbox(bboxes_traffic_sign.bboxes_3d)

        longDistance = traffic_sign_bbox.center_x
        
        velocity = int(traffic_sign_bbox.Class[-2:-1])
        traffic_sign.longDistance = longDistance
        traffic_sign.velocity = velocity
    
        return traffic_sign


    def selectBboxSize(self,bbox_class):

        if bbox_class=='car':

            Class_height = 1.35

        if bbox_class=='bus':

            Class_height = 3.7

        # elif bbox_class=='bicycle':

        #     Class_height = 0.25  

        elif bbox_class=='person':

            Class_height = 1.82

        elif bbox_class.endswith('light'):

            Class_height = 0.2

        elif bbox_class.startswith('trafficsign'):

            Class_height = 0.81

        elif bbox_class=='traffic sign':

            Class_height = 0.81

        elif bbox_class == 'drum':

            Class_height = 1.07

        else:

            Class_height = 999

        return Class_height
    

    def convertClass2int(self,bbox_class):
        
        if bbox_class == 'car':
            class_int = 1

        elif bbox_class == 'person'  :
            class_int = 2

        elif bbox_class == 'bus':
            class_int = 3

        elif bbox_class.endswith('light'):
            class_int = 4

        elif bbox_class == 'traffic sign':
            class_int = 5

        elif bbox_class.startswith('trafficsign'):
            class_int = 6

        elif bbox_class == 'warningtriangle':
            class_int = 7

        elif bbox_class == 'drum':
            class_int = 8

        else:
            class_int = 0

        return class_int


    def bbox_conversion(self,bbox):

        xmin = bbox.xmin
        xmax = bbox.xmax
        ymin = bbox.ymin
        ymax = bbox.ymax
        bbox_class = bbox.Class
        

        ########################### Need to change ################################
        VEHICLE_LENGTH = 5.1
        REAR2CAMERA = 3.1
        HEIGHT = 1
        fov_deg = 40 # deg
        image_width = 640
        image_height = 480
        rot_roll  = 0
        rot_pitch = 0
        roll_yaw  = 0

        image_width = 640
        image_height = 480


        ########################### Need to change ################################
        

        camera_translation = np.array([0, 1.35 , HEIGHT])
        camera_translation = camera_translation.reshape(1,3)
        camera_rotation  = self.Rot3d(np.deg2rad(rot_roll), 
                            np.deg2rad(rot_pitch), np.deg2rad(roll_yaw))

        c_x = image_width/2
        c_y = image_height/2

        fov_rad = np.deg2rad(fov_deg)
        f_x = c_x/(math.tan(fov_rad/2))
        f_y = c_y/(math.tan(fov_rad/2))
        
        if f_x>=f_y:
            f_max = f_x
        else:
            f_max = f_y

       

        skew_coeff = 0

        intrinsic_matrix = np.array([[f_max,0,0],[skew_coeff, f_max, 0],[c_x, c_y, 1]])

        bbox_TR = np.array([xmax,ymin,1])
        bbox_BL = np.array([xmin,ymax,1])
        bbox_TR = bbox_TR.reshape(1,3)
        bbox_BL = bbox_BL.reshape(1,3)


        intrinsic_inv = np.linalg.pinv(intrinsic_matrix)

        bbox_TR_3d_no_gain = bbox_TR.dot(intrinsic_inv)
        bbox_BL_3d_no_gain = bbox_BL.dot(intrinsic_inv)

        height_norm = abs(bbox_TR_3d_no_gain[0,1] - bbox_BL_3d_no_gain[0,1])
        
        Class_height = self.selectBboxSize(bbox_class)

        arbitrary_gain = Class_height/(height_norm)

        bbox_TR_3d = bbox_TR_3d_no_gain*arbitrary_gain * 1
        bbox_BL_3d = bbox_BL_3d_no_gain*arbitrary_gain * 1


        width_3d  = bbox_TR_3d[0,0] - bbox_BL_3d[0,0]
        length_3d = 0
        height_3d = bbox_BL_3d[0,1]- bbox_TR_3d[0,1]

        center_y = -(bbox_BL_3d[0,0]+ bbox_TR_3d[0,0])/2
        center_z = HEIGHT -(bbox_BL_3d[0,1]+ bbox_TR_3d[0,1])/2
        center_x = (bbox_BL_3d[0,2]+ bbox_TR_3d[0,2])/2 + VEHICLE_LENGTH - REAR2CAMERA
        # center_x = (bbox_BL_3d[0,2]+ bbox_TR_3d[0,2])/2 - VEHICLE_LENGTH + REAR2CAMERA


        return center_x,center_y,center_z, width_3d, length_3d, height_3d,Class_height


    def Rot3d(self, roll, pitch, yaw):

        '''
        unit :  deg
        ref : http://www.kwon3d.com/theory/euler/euler_angles.html
        '''

        Rotx = np.array([[1, 0, 0], [0, math.cos(roll), math.sin(roll)],\
            [0, -math.sin(roll), math.cos(roll)]])
        Roty = np.array([[math.cos(pitch), 0, -math.sin(pitch)], [0, 1, 0], \
            [math.sin(pitch), 0, math.cos(pitch)]])
        Rotz = np.array([[math.cos(yaw), math.sin(yaw), 0], \
            [-math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])

        rotation_matrix = Rotz*Roty*Rotx
        
        return rotation_matrix


    def callbackImage(self,msg):

        self.imageTime = msg.header.stamp 

        
    def callbackBBOX(self,msg):

        self.darknetFlag = True

        self.bbox_3d_array = bboxes_3d()
        self.bbox_3d_array.bboxes_3d = []
        self.bbox_3d_array.header.frame_id = 'world'
        #self.bbox_3d_array.header.stamp = self.imageTime
        
        
        self.traffic_light_array = bboxes_3d()
        self.traffic_light_array.bboxes_3d = []
        self.traffic_light_array.header.frame_id = 'world'
        #self.traffic_light_array.header.stamp = self.imageTime

        self.traffic_sign_array = bboxes_3d()
        self.traffic_sign_array.bboxes_3d = []
        self.traffic_sign_array.header.frame_id = 'world'
        #self.traffic_sign_array.header.stamp = self.imageTime        


        for bbox in msg.bounding_boxes:
        
            print np.shape(msg.bounding_boxes)

            bbox_3d_element = bbox_3d()
            bbox_3d_element.header.frame_id = 'world'
            #bbox_3d_element.header.stamp = self.imageTime
            bbox_3d_element.id = bbox.id
            bbox_3d_element.Class = self.convertClass2int(bbox.Class)

            center_x,center_y,center_z, width_3d, length_3d, height_3d, Class_height = \
                self.bbox_conversion(bbox)

            bbox_3d_element.length = length_3d
            bbox_3d_element.width = width_3d
            bbox_3d_element.height = height_3d

            bbox_3d_element.center_x = center_x
            bbox_3d_element.center_y = center_y
            bbox_3d_element.center_z = center_z

            bbox_3d_element.probability = bbox.probability
            # print bbox_3d_element

            if (bbox_3d_element.probability > self.probability_thres) and (Class_height != 999) :

                self.bbox_3d_array.bboxes_3d.append(bbox_3d_element)
                

                if bbox_3d_element.Class ==4:

                    self.traffic_light_array.bboxes_3d.append(bbox_3d_element)

                elif bbox_3d_element.Class == 5:
                    self.traffic_sign_array.bboxes_3d.append(bbox_3d_element)


        print self.bbox_3d_array

        self.bboxes_3d_pub.publish(self.bbox_3d_array)

        if len(self.traffic_light_array.bboxes_3d) > 0:

            traffic_light_state = self.getTrafficLightState(self.traffic_light_array)
            self.traffic_light_state_pub.publish(traffic_light_state)

        if len(self.traffic_sign_array.bboxes_3d) > 0:
            traffic_sign_state = self.getTrafficSignState(self.traffic_sign_array)
            self.traffic_sign_state_pub.publish(traffic_sign_state)

        
    def callbackDarkent(self,msg):

        detection_image = msg.data

        # self.getNearestBbox(detection_image,self.bbox_3d_array)


    def main(self):

        print '\n','hello px2world!','\n'
        print '\n','Probability thres : ',self.probability_thres,'\n'

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
