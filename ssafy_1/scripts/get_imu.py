#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
from sensor_msgs.msg import Imu

# imu_data_listener 는 시뮬레이터에서 송신하는 IMU 센서 정보를 Subscriber 하는 예제 입니다.
# IMU 센서 정보인 /imu 라는 메세지를 Subscribe 합니다.

# 노드 실행 순서 
# 1. Callback 함수 생성 및 데이터 출력

#TODO: (1) Callback 함수 생성 및 데이터 출력
def imu_callback(data):
    os.system('clear')
    rospy.loginfo('------------------ IMU Sensor Status ------------------')
    rospy.loginfo("orientation:")
    rospy.loginfo("x : {} y : {} z : {} w : {}".format(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
    rospy.loginfo("angular_velocity:")
    rospy.loginfo("x : {} y : {} z : {}".format(data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z))
    rospy.loginfo("linear_acceleration:")
    rospy.loginfo("x : {} y : {} z : {}".format(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z))

def listener():
    rospy.init_node('imu_data_listener', anonymous=True)

    rospy.Subscriber('/imu', Imu, imu_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
