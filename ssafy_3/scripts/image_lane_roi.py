#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

# image_lane_roi 는 카메라 센서를 통하여 받아온 이미지에 관심있는 부분만(차선) 만 남기고
# 나머지 부분은 마스킹 하는 이미리 처리입니다. 관심 영역을 지정하고, 마스크를 생성, 마스크를 이미지에 합치는 과정을
# 합니다. 

class IMGParser:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        
        # image_size
        x = 640
        y = 480

        #TODO: (1) 관심있는 영역만 지정.

        self.crop_pts = np.array(
                                    [[
                                        [int(x * 0.02), int(y * 0.75)],
                                        [int(x * 0.45), int(y * 0.52)],
                                        [int(x * 0.55), int(y * 0.52)],
                                        [int(x * 0.98), int(y * 0.75)]
                                    ]]
                                )
                                # (
                                #     [[
                                #         [0,0],
                                #         [0,0],
                                #         [0,0],
                                #         [0,0]
                                #     ]]
                                # )

    def callback(self, msg):
        # uint8 : unsined integer 0~255 로 만들기 위함입니다.
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        self.mask = self.mask_roi(img_bgr)
        # 이미지를 가로로 붙여 보기 위한 내용입니다.   
        if len(self.mask.shape)==3:
            img_concat = np.concatenate([img_bgr, self.mask], axis=1)
        else:
            img_concat = np.concatenate([img_bgr, cv2.cvtColor(self.mask, cv2.COLOR_GRAY2BGR)], axis=1)

        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1)

    def mask_roi(self, img):

        h = img.shape[0]
        w = img.shape[1]
        
        # img.shape == [H,W,C]의 3차원이고 C RGB를 갖는 3채널입니다. RGB 이미지 입니다.
        if len(img.shape)==3:

            c = img.shape[2]
            mask = np.zeros((h, w, c), dtype=np.uint8)

            mask_value = (255, 255, 255)

        # grayscale image일 경우. (시뮬레이터에서 주는 이미지는 항상 3차원이기 때문에 예외를 위해서 만들어 놓은 부분 입니다.)
        else:

            mask = np.zeros((h, w), dtype=np.uint8)

            mask_value = (255)
        
        # TODO (1) 에서 마스킹 영역을 만들었고, 관심 있는 부분만을 이미지 원본으로 하고 나머지는 255(검은색)로 반환 해 주는
        #내용이 들어가야 합니다.
        #마스킹 영역을 만들기 위해서 다양한 방법을 사용할 수 있습니다만, 코드에서 이미 까만 이미지를 생성했습니다.
        #이를 이용하는 방법을 찾아야 합니다.
        
        #TODO: (2) 
        cv2.fillPoly(mask, self.crop_pts, mask_value)

        #TODO : (3)
        mask = cv2.bitwise_and(mask, img)

        return mask


if __name__ == '__main__':

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 
