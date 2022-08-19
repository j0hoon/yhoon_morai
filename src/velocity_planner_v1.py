#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import pid_utils



# path 의 기울기를 받아서 결과로 desired velocity 를 출력함

class velocityPlanning:
    def __init__(self , car_max_speed ,  road_friction ):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friction

    def curvedBasedVelocity(self, global_path, local_path, global_point_num, local_point_num,default_velocity ):
        # local path의 곡률이 일정구간 매우 급격한 커브 일때는 local 감속을 하고
        # 그외에는 global을 따른다 
        # 거시적으로 보았을때는 완만하지만 짧은 구간내에서는 차선이 급격하게 꺾이는 구간을 대비하기 위함
        # 두가지 방법이 생각남 

        v_min = 10
        out_vel_plan = []
        
        # init velocity max carspeed
        for i in range(0,global_point_num):
            out_vel_plan.append(self.car_max_speed)


        if local_path.deg > 60:
            for i in range(local_point_num, global_point_num) :
                local_x_list = []
                local_y_list = []
            for box in range(-local_point_num, local_point_num):
                x = local_path.poses[i+box].pose.position.x
                y = local_path.poses[i+box].pose.position.y
                local_x_list.append([-2*x, -2*y ,1])
                local_y_list.append((-x*x) - (y*y))

            x_matrix = np.array(local_x_list)
            y_matrix = np.array(local_y_list)
            x_trans = x_matrix.T

            a_matrix = np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a = a_matrix[0]
            b = a_matrix[1]
            c = a_matrix[2]
            r = sqrt(a*a+b*b-c)

            v_max = sqrt(r*9.8*self.road_friction)

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)
            

        
        # set velocity by global path curvature
        for i in range(global_point_num, len(global_path.poses) - global_point_num):
            x_list = []
            y_list = []
            for box in range(-global_point_num, global_point_num):
                x = global_path.poses[i+box].pose.position.x
                y = global_path.poses[i+box].pose.position.y
                x_list.append([-2*x, -2*y ,1])
                y_list.append((-x*x) - (y*y))

            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T

            a_matrix = np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a = a_matrix[0]
            b = a_matrix[1]
            c = a_matrix[2]
            r = sqrt(a*a+b*b-c)

            v_max = sqrt(r*9.8*self.road_friction)

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)
        
        
        # set velocity between global point num ~ end of global point 
        # default velocity was 30 as default but you can change it 
        for i in range(len(global_path.poses) - global_point_num, len(global_path.poses)-10):
            out_vel_plan.append(default_velocity) 

        for i in range(len(global_path.poses) - 10, len(global_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan       

        

