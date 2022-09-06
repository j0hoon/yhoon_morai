#!/usr/bin/env python
# -*- coding: utf-8 -*-

from re import I
import rospy
import numpy as np
import rospkg
from math import sqrt
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus


class pathMaker_drivingMode :
    def __init__(self, pkg_name = 'ssafy_2', path_name = 'drivingmode_path'):
        rospy.init_node('path_maker', anonymous=True)     

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/global_path", Path, self.global_path_callback)          
        rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.status_callback)

        self.prev_x = 0
        self.prev_y = 0
        self.is_odom=False
        self.local_curvature = self.cal_curvature(self.global_path,pointnum = 50)

        rospack = rospkg.RosPack()
        pkg_path=rospack.get_path(pkg_name)
        full_path=pkg_path + '/' + 'path' + '/' + path_name+'.txt'
        self.f=open(full_path, 'w')

        while not rospy.is_shutdown():
            if self.is_odom == True :
                # Ego 위치 기록
                self.path_make()
        self.f.close()


    def global_path_callback(self,msg):
        self.global_path = msg
        self.is_global_path = True  

    def status_callback(self,msg): ## Vehicle Status Subscriber 
        self.is_status=True
        self.status_msg=msg      

    def odom_callback(self,msg):
        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def cal_curvature(self,global_path , point_num):
        for i in range(point_num, len(global_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
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
            curvature = sqrt(a*a+b*b-c)   
            

            return curvature

    def path_make(self):
        x = self.x
        y = self.y
        z = 0.0
        distance=sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))

        
        if distance >0.5:
            data='{0}\t{1}\t{2}\n'.format(x,y,z)
            self.f.write(data)
            self.prev_x=x
            self.prev_y=y
            self.prev_z=z
            
            print(' write [ x : {} , y : {} ]'.format(x,y))
            print(' curvature {}'.format(self.local_curvature))

if __name__ == '__main__' :
    try:
        p_m=pathMaker_drivingMode()
    except rospy.ROSInternalException:
        pass
