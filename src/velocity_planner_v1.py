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

# path 의 기울기를 받아서 결과로 desired velocity 를 출력함

class velocityPlanning:
    def __init__(self , car_max_speed ,  road_friction ):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friction

    def curvedBasedVelocity(self, global_path, local_path, point_num):
        # local path의 곡률이 일정구간 매우 급격한 커브 일때는 local 감속을 하고
        # 그외에는 global을 따른다 
        # 거시적으로 보았을때는 완만하지만 짧은 구간내에서는 차선이 급격하게 꺾이는 구간을 대비하기 위함
        # 두가지 방법이 생각남 
        # 1. 지금 처럼하기
        # 2. 

        v_min = 10
        desiredVelocity = []

        if local_path.deg > 60: # 임의로 정한 각도임 TODO 이부분 수정해야함
            desiredVelocity.append(v_min)
        

