#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import EgoVehicleStatus
# Ego_status_listener 는 시뮬레이터에서 송신하는 Ego 차량 정보를 Subscriber 하는 예제 입니다.
# 시뮬레이터 내 Ego 차량의 정보인 /Ego_topic 라는 메세지를 Subscribe 합니다.

# 노드 실행 순서 
# 1. ROS 노드 이름 선언
# 2. Subscriber 생성
# 3. Callback 함수 생성 및 데이터 출력

#TODO: (3) Callback 함수 생성 및 데이터 출력
def EgoStatus_callback(data):
    rospy.loginfo('------------------Ego Vehicle Status------------------')
    rospy.loginfo('position     : x = {0} , y = {1}, z = {2}'.format(data.position.x,data.position.y,data.position.z))
    rospy.loginfo('velocity     : x = {0} , y = {1}, z = {2} m/s^2'.format(data.velocity.x,data.velocity.y,data.velocity.z))
    rospy.loginfo('acceleration : x = {0} , y = {1}, z = {2} m/s'.format(data.acceleration.x,data.acceleration.y,data.acceleration.z))
    rospy.loginfo('heading      : {} deg'.format(data.heading))

def listener():
    #TODO: (1) ROS 노드 이름 선언
    rospy.init_node('Ego_status_listener', anonymous=True)

    #TODO: (2) Subscriber 생성
    rospy.Subscriber('/Ego_topic', EgoVehicleStatus, EgoStatus_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
