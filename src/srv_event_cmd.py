#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from morai_msgs.msg import EventInfo,Lamps
from morai_msgs.srv import MoraiEventCmdSrv

# MoraiEventCmd_client 는 Ego 차량의 상태를 제어하는 Client Node 작성 예제입니다.
# Ego 차량의 상태(전조등, 방향 지시등, 차량 Gear, 차량 제어 Mode)제어 메세지 송신 후 결과 값을 반환하는 Client Node 를 생성 합니다.

# 노드 실행 순서 
# 1. Service 가 생성 대기 함수 선언
# 2. 송신 될 메세지 변수 생성
# 3. Service 호출
# 4. Service 호출 결과 값 확인

def srv_client():
    rospy.init_node('MoraiEventCmd_client', anonymous=True)
    #TODO: (1) Service 가 생성 대기 함수 선언
    rospy.wait_for_service('/Service_MoraiEventCmd')

    #TODO: (2) 송신 될 메세지 변수 생성
    lamp_cmd = Lamps()
    lamp_cmd.turnSignal = 1
    lamp_cmd.emergencySignal = 0
        
    set_Event_control = EventInfo()
    set_Event_control.option = 7
    set_Event_control.ctrl_mode = 3
    set_Event_control.gear = 4
    set_Event_control.lamps = lamp_cmd

    rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        try:
            #TODO: (3) Service 호출
            ros_srv = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)
            result = ros_srv(set_Event_control)

            #TODO: (4) Service 호출 결과 값 확인
            rospy.loginfo(result)
        except rospy.ServiceException as e:
            rospy.logwarn('no respone')

        rate.sleep()

if __name__ == '__main__':
    try:
        srv_client()
    except rospy.ROSInterruptException:
        pass
