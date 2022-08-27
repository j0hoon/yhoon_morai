#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import CtrlCmd

# Ego_Control_Command 는 Simulator 에서 Ego 차량을 움직임을 제어하는 메세지 송신의 예제입니다.
# /ctrl_cmd 라는 메세지를 Publish 하여 Ego 차량을 제어 합니다.

# 노드 실행 순서 
# 1. publisher 생성
# 2. 송신 될 메세지 변수 생성
# 3. /ctrl_cmd 메세지 Publish

def talker():
    #TODO: (1) publisher 생성
    publisher = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=10)
    rospy.init_node('Ego_Control_Command', anonymous=True)

    #TODO: (2) 송신 될 메세지 변수 생성
    ctrl_cmd = CtrlCmd()
    ctrl_cmd.longlCmdType = 1
    ctrl_cmd.accel = 0.5
    ctrl_cmd.brake = 0.0
    ctrl_cmd.steering = 0.0

    rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        rospy.loginfo(ctrl_cmd)
        #TODO: (3) /ctrl_cmd 메세지 Publish
        publisher.publish(ctrl_cmd)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
