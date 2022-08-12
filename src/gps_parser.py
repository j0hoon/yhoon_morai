#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
import os
from pyproj import Proj
from std_msgs.msg import Float32MultiArray
from morai_msgs.msg import GPSMessage, EgoVehicleStatus

# gps_parser 는 GPS의 위경도 데이터를 UTM 좌표로 변환하는 예제입니다.
# Pyproj 라이브러리를 사용 

# 노드 실행 순서 
# 1. 변환 하고자 하는 좌표계를 선언  
# 2. 시뮬레이터에서 GPS 데이터를 받아오는 Callback 함수 생성 
# 3. 위도 경도 데이터를 UTM 좌표로 변환   
# 4. 위도 경도 데이터와 변환한 UTM 좌표를 터미널 창에 출력 하여 확인  

class LL2UTMConverter:
    def __init__(self, zone=52) :
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        # 초기화
        self.x, self.y = None, None

        #TODO: (1) 변환 하고자 하는 좌표계를 선언
        self.proj_UTM = Proj(proj='utm', zone=52, ellps = 'WGS84', preserve_units=False)

    #TODO: (2) 시뮬레이터에서 GPS 데이터를 받아오는 Callback 함수 생성
    def navsat_callback(self, gps_msg):
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude

        self.convertLL2UTM()

        utm_msg = Float32MultiArray()

        #TODO: (4) 위도 경도 데이터와 변환한 UTM 좌표를 터미널 창에 출력 하여 확인
        utm_msg.data = [self.x, self.y]
        os.system('clear')
        print(' lat : ', self.lat)
        print(' lon : ', self.lon)
        print(' utm X : ', utm_msg.data[0])
        print(' utm Y : ', utm_msg.data[1])


    #TODO: (3) 위도 경도 데이터를 UTM 좌표로 변환
    def convertLL2UTM(self):
        xy_zone = self.proj_UTM(self.lon, self.lat)

        self.x = xy_zone[0]
        self.y = xy_zone[1]


if __name__ == '__main__':

    rospy.init_node('gps_parser', anonymous=True)

    gps_parser = LL2UTMConverter()

    rospy.spin()
        
