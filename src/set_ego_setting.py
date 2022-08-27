#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import MultiEgoSetting

# Ego_setting_Command 는 Simulator 에서 Ego 차량의 위치를 제어하는 메세지 송신의 예제입니다.
# /ego_setting 라는 메세지를 Publish 하여 Ego 차량을 제어 합니다.

# 노드 실행 순서 
# 1. publisher 생성
# 2. 송신 될 메세지 변수 생성
# 3. /ego_setting 메세지 Publish

def talker():
    #TODO: (1) publisher 생성
    publisher = rospy.Publisher('/ego_setting', MultiEgoSetting, queue_size=10)
    rospy.init_node('Ego_setting_Command', anonymous=True)

    #TODO: (2) 송신 될 메세지 변수 생성
    ego_setting_msg = MultiEgoSetting()
    ego_setting_msg.number_of_ego_vehicle=1
    ego_setting_msg.camera_index = 0
    ego_setting_msg.ego_index = [0]
    ego_setting_msg.global_position_x = [13.4]
    ego_setting_msg.global_position_y = [1099,68]
    ego_setting_msg.global_position_z = [3]
    ego_setting_msg.global_roll = [0]
    ego_setting_msg.global_pitch = [0]
    ego_setting_msg.global_yaw = [60.0]
    ego_setting_msg.velocity=[0]
    ego_setting_msg.gear=[4]
    ego_setting_msg.ctrl_mode=[16] 

    rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        rospy.loginfo(ego_setting_msg)
        #TODO: (3) /ego_setting 메세지 Publish
        publisher.publish(ego_setting_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
