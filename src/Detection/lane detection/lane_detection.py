#! /usr/bin/env python

from code import interact
from email.mime import image

import cv2
from numpy import append
import rospy
import numpy as np
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge,CvBridgeError

class camera_sim():
    
    def __init__(self):
        rospy.init_node("image_to_receiver", anonymous=True)  
        self.pubcam = rospy.Publisher("/pub_img",Image,queue_size=10)
        self.subcam = rospy.Subscriber("/image_jpeg/compressed", CompressedImage , self.callback)
        self.bridge = CvBridge()
        rospy.on_shutdown(self.cam_shutdown)
        rospy.spin()

    def callback(self, data):
        # simulation cam -> cv2
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("converting error") 
            print(e) 
        # print cvimage, shape
        
        # cv2_bgr -> convert color 
        img_result = self.make_cvt_img(cv_image)
        
        #ROI (region of interest)
        # Converted ROI mask
        masked_img  = self.ROI(cv_image, img_result)
        blur_img = cv2.GaussianBlur(masked_img, (5,5),0)
        canny_img = cv2.Canny(blur_img,50,150)

        # display lines
        combo_img = self.Drow_line(canny_img,cv_image,masked_img)
 
        try : 
            self.pubcam.publish(self.bridge.cv2_to_imgmsg(combo_img,"rgb8"))
        except CvBridgeError as e:
            print("publish error") 
            print(e)

    def make_cvt_img(self, img):
        cvt_img = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        lower = (0,0,100)
        upper = (255,255,255)
        filtered_img = cv2.inRange(cvt_img, lower, upper)
        img_result = cv2.bitwise_and(img, img, mask = filtered_img)
        return img_result

    def ROI(self, original_img,  img_result):
        height , width , channel = original_img.shape
        region = np.array([[(0,height),(0,height*70/100),(width/2, height*45/100),(width, height*70/100),(width, height)]] , dtype = np.int32) 
        zero_img = np.zeros_like(original_img) 
        region_mask = cv2.fillPoly(zero_img, region , (255,255,255))
        masked_img = cv2.bitwise_and(img_result,region_mask)
        return masked_img

    def Drow_line(self, canny_img , original_img, masked_img) :
        lines = cv2.HoughLinesP(canny_img, rho = 2, theta = np.pi/180, threshold = 100, lines = np.array([]), minLineLength = 20 , maxLineGap = 5)   
        # average_slope_intercept
        height = original_img.shape[0] 
        left_fit =[]
        right_fit = []
        for line in lines :
            x3, y3, x4, y4 = line.reshape(4)
            parameters = np.polyfit((x3,x4),(y3,y4),1)
            slope = parameters[0]
            intercept = parameters[1]
            if slope < 0:
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))
        
        left_fit_average = np.average(left_fit ,axis=0)
        right_fit_average = np.average(right_fit, axis=0)
   

        y5 = height
        y6 = int(0.6*height)
        x5 = int((y5 - left_fit_average[1])/left_fit_average[0])
        x6 = int((y6 - left_fit_average[1])/left_fit_average[0])
        x7 = int((y5 - right_fit_average[1])/right_fit_average[0])
        x8 = int((y6 - right_fit_average[1])/right_fit_average[0])

        left_line = np.array([x5, y5 , x6, y6])
        right_line = np.array([x7, y5, x8, y6])

        average_lines = np.array([left_line , right_line])
        line_img = np.copy(masked_img)
        if average_lines is not None:
            for line in average_lines:
                x1, y1, x2, y2 = line.reshape(4)
                cv2.line(line_img, (x1, y1), (x2, y2), (255, 0 ,0), 10) 
        combo_img = cv2.addWeighted(original_img, 0.8, line_img, 1, 1 ) 
        print('left_dst = {} right_dst = {} \n left_angle = {} right_angle = {}'.format((x5+x6)/2 , (x7+x8)/2 , left_fit_average[0] ,right_fit_average[0]))
        return combo_img



    def cam_shutdown(self):
        print("I'm dead!!")

if __name__=="__main__":

    cs = camera_sim()

