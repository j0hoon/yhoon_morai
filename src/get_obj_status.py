#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
from morai_msgs.msg import ObjectStatusList

# Obj_status_listener 는 시뮬레이터에서 송신하는 Object 정보를 Subscriber 하는 예제 입니다.
# 시뮬레이터 내 Object 정보인 /Object_topic 라는 메세지를 Subscribe 합니다.

# 노드 실행 순서 
# 1. ROS 노드 이름 선언
# 2. Subscriber 생성
# 3. Callback 함수 생성 및 데이터 출력

#TODO: (3) Callback 함수 생성 및 데이터 출력
def Object_callback(data):
    os.system('clear')
    rospy.loginfo('-------------------- NPC Vehicle -------------------------')
    rospy.loginfo('NPC num :{}'.format(data.num_of_npcs))
    for i in range(data.num_of_npcs) :
        rospy.loginfo('--------------------Num {}-------------------------'.format(i))
        rospy.loginfo('name : {}'.format(data.npc_list[i].name))
        rospy.loginfo('position     : x = {0} , y = {1}, z = {2}'.format(data.npc_list[i].position.x,data.npc_list[i].position.y,data.npc_list[i].position.z))
        rospy.loginfo('velocity     : x = {0} , y = {1}, z = {2} m/s^2'.format(data.npc_list[i].velocity.x,data.npc_list[i].velocity.y,data.npc_list[i].velocity.z))
        rospy.loginfo('acceleration : x = {0} , y = {1}, z = {2} m/s'.format(data.npc_list[i].acceleration.x,data.npc_list[i].acceleration.y,data.npc_list[i].acceleration.z))
        rospy.loginfo('heading      : {} deg'.format(data.npc_list[i].heading))
        rospy.loginfo('size         : x = {0} , y = {1}, z = {2} m'.format(data.npc_list[i].size.x,data.npc_list[i].size.y,data.npc_list[i].size.z))
    rospy.loginfo('-------------------- Pedestrian -------------------------')
    rospy.loginfo('Pedestrian num :{}'.format(data.num_of_pedestrian))
    for i in range(data.num_of_pedestrian) :
        rospy.loginfo('--------------------Num {}-------------------------'.format(i))
        rospy.loginfo('name : {}'.format(data.pedestrian_list[i].name))
        rospy.loginfo('position     : x = {0} , y = {1}, z = {2}'.format(data.pedestrian_list[i].position.x,data.pedestrian_list[i].position.y,data.pedestrian_list[i].position.z))
        rospy.loginfo('velocity     : x = {0} , y = {1}, z = {2} m/s^2'.format(data.pedestrian_list[i].velocity.x,data.pedestrian_list[i].velocity.y,data.pedestrian_list[i].velocity.z))
        rospy.loginfo('acceleration : x = {0} , y = {1}, z = {2} m/s'.format(data.pedestrian_list[i].acceleration.x,data.pedestrian_list[i].acceleration.y,data.pedestrian_list[i].acceleration.z))
        rospy.loginfo('heading      : {} deg'.format(data.pedestrian_list[i].heading))
        rospy.loginfo('size         : x = {0} , y = {1}, z = {2} m'.format(data.pedestrian_list[i].size.x,data.pedestrian_list[i].size.y,data.pedestrian_list[i].size.z))
    rospy.loginfo('-------------------- Obstacle -------------------------')
    rospy.loginfo('Obstacle num :{}'.format(data.num_of_obstacle))
    for i in range(data.num_of_obstacle) :
        rospy.loginfo('--------------------Num {}-------------------------'.format(i))
        rospy.loginfo('name : {}'.format(data.obstacle_list[i].name))
        rospy.loginfo('position     : x = {0} , y = {1}, z = {2}'.format(data.obstacle_list[i].position.x,data.obstacle_list[i].position.y,data.obstacle_list[i].position.z))
        rospy.loginfo('velocity     : x = {0} , y = {1}, z = {2} m/s^2'.format(data.obstacle_list[i].velocity.x,data.obstacle_list[i].velocity.y,data.obstacle_list[i].velocity.z))
        rospy.loginfo('acceleration : x = {0} , y = {1}, z = {2} m/s'.format(data.obstacle_list[i].acceleration.x,data.obstacle_list[i].acceleration.y,data.obstacle_list[i].acceleration.z))
        rospy.loginfo('heading      : {} deg'.format(data.obstacle_list[i].heading))
        rospy.loginfo('size         : x = {0} , y = {1}, z = {2} m'.format(data.obstacle_list[i].size.x,data.obstacle_list[i].size.y,data.obstacle_list[i].size.z))

def listener():
    #TODO: (1) ROS 노드 이름 선언
    rospy.init_node('Obj_status_listener', anonymous=True)

    #TODO: (2) Subscriber 생성
    rospy.Subscriber('/Object_topic', ObjectStatusList, Object_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
