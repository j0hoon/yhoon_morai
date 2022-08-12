#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
import rospy
import rospkg
import tf
from math import cos,sin,pi,sqrt,pow,atan2
import numpy as np
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import ObjectStatus, ObjectStatusList, EgoVehicleStatus

# lane_change 는 차량의 차선변경 예제입니다.
# 차량 경로상의 장애물을 탐색하여 경로 상에 장애물이 있다면 차선 변경으로 회피 기동을 합니다.
# 차선 변경 시 단순히 목표 차선을 바꾸는 것이 아닌 3차 곡선을 그려 자연스러운 차선변경이 가능 하도록 경로를 만듭니다.

# 노드 실행 순서 
# 1. subscriber, publisher 선언
# 2. 두개의 차선 경로 의 텍스트파일을 읽기 모드로 열기
# 3. 읽어 온 경로 데이터를 Global Path 변수에 넣기
# 4. 주행 경로상의 장애물 유무 확인
# 5. 장애물이 있다면 주행 경로를 변경 하도록 로직 작성.
# 6. 좌표 변환 행렬 생성
# 7. 차선변경 시작점과 끝점을 이어주는 주행 경로 생성
# 8. 경로 데이터 Publish

class lc_path_pub :
    def __init__(self):
        rospy.init_node('lc_path_pub', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        object_topic_name = arg[1]

        #TODO: (1) subscriber, publisher 선언
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber(object_topic_name, ObjectStatusList, self.object_info_callback)
        self.global_path_pub = rospy.Publisher('/global_path',Path, queue_size=1)
        self.local_path_pub = rospy.Publisher('/lane_change_path',Path, queue_size=1)

        self.lc_1=Path()
        self.lc_1.header.frame_id='/map'
        self.lc_2=Path()
        self.lc_2.header.frame_id='/map'

        #TODO: (2) 두개의 차선 경로 의 텍스트파일을 읽기 모드로 열기
        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path('ssafy_3')
        lc_1 = pkg_path+'/path'+'/lc_1.txt'
        self.f=open(lc_1,'r')
        lines=self.f.readlines()
        for line in lines :
            tmp=line.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            read_pose.pose.orientation.w=1
            self.lc_1.poses.append(read_pose)        
        self.f.close()

        lc_2 = pkg_path+'/path'+'/lc_2.txt'
        self.f=open(lc_2,'r')
        lines=self.f.readlines()
        for line in lines :
            tmp=line.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            read_pose.pose.orientation.w=1
            self.lc_2.poses.append(read_pose)        
        self.f.close()

        self.is_object_info = False
        self.is_odom = False

        self.local_path_size = 50
        
        self.lane_change = False        
        self.current_lane = 1

        #TODO: (3) 읽어 온 경로 데이터를 Global Path 로 지정
        global_path = self.lc_1

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if self.is_object_info == True and self.is_odom == True:

                global_obj,local_obj = self.calc_vaild_obj([self.x,self.y,(self.vehicle_yaw)])
                self.local_path_make(global_path)

                currnet_waypoint = self.get_current_waypoint(self.x,self.y,global_path)

                global_path = self.lc_planning(global_obj,local_obj,currnet_waypoint,global_path)

                #TODO: (8) 경로 데이터 Publish
                self.local_path_pub.publish(self.local_path_msg)
                self.global_path_pub.publish(global_path)

            rate.sleep()

    def odom_callback(self,msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

        _,_,self.vehicle_yaw=tf.transformations.euler_from_quaternion(odom_quaternion)

        self.is_odom = True

    def object_info_callback(self,data): ## Object information Subscriber
        self.is_object_info = True
        self.object_num=data.num_of_npcs
        object_type=[]
        object_pose_x=[]
        object_pose_y=[]
        object_velocity=[]
        for num in range(data.num_of_npcs) :
            object_type.append(data.npc_list[num].type)
            object_pose_x.append(data.npc_list[num].position.x)
            object_pose_y.append(data.npc_list[num].position.y)
            object_velocity.append(data.npc_list[num].velocity.x)

        self.object_info=[object_type,object_pose_x,object_pose_y,object_velocity]   
    
    def get_current_waypoint(self,x,y,global_path):
        min_dist = float('inf')        
        currnet_waypoint = -1
        for i,pose in enumerate(global_path.poses):
            dx = x - pose.pose.position.x
            dy = y - pose.pose.position.y

            dist = sqrt(pow(dx,2)+pow(dy,2))
            if min_dist > dist :
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint
    
    def local_path_make(self,global_path):
                
        self.local_path_msg=Path()
        self.local_path_msg.header.frame_id='/map'
        
        x=self.x
        y=self.y
        min_dis=float('inf')
        current_waypoint=-1
        for i,waypoint in enumerate(global_path.poses) :
            distance=sqrt(pow(x-waypoint.pose.position.x,2)+pow(y-waypoint.pose.position.y,2))
            if distance < min_dis :
                min_dis=distance
                current_waypoint=i
        
        if current_waypoint != -1 :
            if current_waypoint + self.local_path_size < len(global_path.poses):
                for num in range(current_waypoint,current_waypoint + self.local_path_size ) :
                    tmp_pose=PoseStamped()
                    tmp_pose.pose.position.x=global_path.poses[num].pose.position.x
                    tmp_pose.pose.position.y=global_path.poses[num].pose.position.y
                    tmp_pose.pose.orientation.w=1
                    self.local_path_msg.poses.append(tmp_pose)                    
            else :
                for num in range(current_waypoint,len(global_path.poses) ) :
                    tmp_pose=PoseStamped()
                    tmp_pose.pose.position.x=global_path.poses[num].pose.position.x
                    tmp_pose.pose.position.y=global_path.poses[num].pose.position.y
                    tmp_pose.pose.orientation.w=1
                    self.local_path_msg.poses.append(tmp_pose)


    def calc_vaild_obj(self,ego_pose):  # x, y, heading
        global_object_info=[]
        loal_object_info=[]

        tmp_theta=ego_pose[2]
        tmp_translation=[ego_pose[0],ego_pose[1]]
        tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta),tmp_translation[0]],
                        [sin(tmp_theta),cos(tmp_theta),tmp_translation[1]],
                        [0,0,1]])
        tmp_det_t=np.array([[tmp_t[0][0],tmp_t[1][0],-(tmp_t[0][0]*tmp_translation[0]+tmp_t[1][0]*tmp_translation[1])   ],
                            [tmp_t[0][1],tmp_t[1][1],-(tmp_t[0][1]*tmp_translation[0]+tmp_t[1][1]*tmp_translation[1])   ],
                            [0,0,1]])

        for num in range(self.object_num):
            global_result=np.array([[self.object_info[1][num]],[self.object_info[2][num]],[1]])
            local_result=tmp_det_t.dot(global_result)
            if local_result[0][0]> 0 :
                global_object_info.append([self.object_info[0][num],self.object_info[1][num],self.object_info[2][num],self.object_info[3][num]])
                loal_object_info.append([self.object_info[0][num],local_result[0][0],local_result[1][0],self.object_info[3][num]])        

        return global_object_info,loal_object_info

    def lc_planning(self,global_obj,local_obj,currnet_waypoint,global_path):
        #TODO: (5) 장애물이 있다면 주행 경로를 변경 하도록 로직 작성
        if self.lane_change == True:
            if currnet_waypoint + self.local_path_size > len(global_path.poses):
                self.lane_change = False
                if self.current_lane == 1:
                    global_path = self.lc_1
                elif self.current_lane == 2:
                    global_path = self.lc_2
        else:
            self.check_object(self.local_path_msg,global_obj,local_obj)
        
        if self.object[0] == True:
            if self.current_lane != 1:
                self.current_lane = 1
                start_point         = self.lc_2.poses[currnet_waypoint]  
                end_waypoint_idx = self.get_current_waypoint(self.lc_1.poses[currnet_waypoint+60].pose.position.x,self.lc_1.poses[currnet_waypoint+60].pose.position.y,self.lc_1)
                end_point           = self.lc_1.poses[end_waypoint_idx]
                start_next_point    = self.lc_2.poses[currnet_waypoint+5]  
                global_path = self.getLaneChangePath(self.lc_2,self.lc_1,start_point,end_point,start_next_point, end_waypoint_idx)
                self.lane_change = True
                self.object=[False,0]
            else:
                self.current_lane = 2
                start_point         = self.lc_1.poses[currnet_waypoint]  
                end_waypoint_idx = self.get_current_waypoint(self.lc_2.poses[currnet_waypoint+60].pose.position.x,self.lc_2.poses[currnet_waypoint+60].pose.position.y,self.lc_2)
                end_point           = self.lc_2.poses[end_waypoint_idx]
                start_next_point    = self.lc_1.poses[currnet_waypoint+5]  
                global_path = self.getLaneChangePath(self.lc_1,self.lc_2,start_point,end_point,start_next_point, end_waypoint_idx)
                self.lane_change = True
                self.object=[False,0]

        return global_path

    def check_object(self,ref_path,global_vaild_object,local_vaild_object):
        #TODO: (4) 주행 경로상의 장애물 유무 확인
        self.object=[False,0]
        if len(global_vaild_object) >0  :
            min_rel_distance=float('inf')
            for i in range(len(global_vaild_object)):
                for path in ref_path.poses :   
                    if global_vaild_object[i][0]==1 or global_vaild_object[i][0]==2 :  
                        dis=sqrt(pow(path.pose.position.x-global_vaild_object[i][1],2)+pow(path.pose.position.y-global_vaild_object[i][2],2))
                        if dis<2.5:
                            rel_distance= sqrt(pow(local_vaild_object[i][1],2)+pow(local_vaild_object[i][2],2))                            
                            if rel_distance < min_rel_distance:
                                min_rel_distance=rel_distance
                                self.object=[True,i]

    def getLaneChangePath(self,ego_path,lc_path,start_point,end_point,start_next_point, end_waypoint_idx): ## 
        out_path=Path()  
        out_path.header.frame_id='/map'

        #TODO: (5) 차선변경 시작점과 끝점을 이어주는 주행 경로 생성

        point_to_point_distance = 0.5
        start_path_distance = sqrt(pow(end_point.pose.position.x-start_point.pose.position.x,2)+pow(end_point.pose.position.y-start_point.pose.position.y,2))
        start_path_repeat = int(start_path_distance/point_to_point_distance)

        theta=atan2(end_point.pose.position.y-start_point.pose.position.y,end_point.pose.position.x-start_point.pose.position.x)

        ratation_matric_1 = np.array([  [   cos(theta), -sin(theta)  ],
                                        [   sin(theta),  cos(theta)  ]    ])

        for k in range(0,start_path_repeat+1):
            ratation_matric_2 = np.array([  [ k*point_to_point_distance ],  
                                            [ 0                         ]   ])
            roation_matric_calc = np.matmul(ratation_matric_1,ratation_matric_2)
            read_pose=PoseStamped()
            read_pose.pose.position.x = start_point.pose.position.x + roation_matric_calc[0][0]
            read_pose.pose.position.y = start_point.pose.position.y + roation_matric_calc[1][0]
            read_pose.pose.position.z = 0
            read_pose.pose.orientation.x = 0
            read_pose.pose.orientation.y = 0
            read_pose.pose.orientation.z = 0
            read_pose.pose.orientation.w = 1
            out_path.poses.append(read_pose)
        
        # 직선 거리 추가

        for k in range(end_waypoint_idx,end_waypoint_idx+40):
            read_pose=PoseStamped()
            read_pose.pose.position.x = lc_path.poses[k].pose.position.x
            read_pose.pose.position.y = lc_path.poses[k].pose.position.y
            read_pose.pose.position.z=0
            read_pose.pose.orientation.x=0
            read_pose.pose.orientation.y=0
            read_pose.pose.orientation.z=0
            read_pose.pose.orientation.w=1
            out_path.poses.append(read_pose)

        return out_path

if __name__ == '__main__':
    try:
        test_track=lc_path_pub()
    except rospy.ROSInterruptException:
        pass
