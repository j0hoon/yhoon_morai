#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import GPSMessage

# gps_data_listener 는 시뮬레이터에서 송신하는 gps 센서 정보를 Subscriber 하는 예제 입니다.
# gps 센서 정보인 /gps 라는 메세지를 Subscribe 합니다.

# 노드 실행 순서 
# 1. Callback 함수 생성 및 데이터 출력

#TODO: (1) Callback 함수 생성 및 데이터 출력
def gps_callback(data):
    print("\n ------------------------------------ \n")
    rospy.loginfo("latitude {}".format(data.latitude))
    rospy.loginfo("longitude {}".format(data.longitude))
    rospy.loginfo("eastOffset {}".format(data.eastOffset))
    rospy.loginfo("northOffset {}".format(data.northOffset))

def listener():
    rospy.init_node('gps_data_listener', anonymous=True)

    rospy.Subscriber('/gps', GPSMessage, gps_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
