#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import csv
import matplotlib.pyplot as plt
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from std_msgs.msg import Float64
import time
from math import pi, sin, cos, atan2, sqrt
#from read_csv import read_csv, get_WP_index, read_txt
from utils.utils import Purepursuit, pidController, ACC
from utils.genWP import WP_X, WP_Y
from utils.SWG import SWG_make
from utils.Local_Motion_Planning import local_motion_planning
import numpy as np

################ Parameter ###############
WHEEL_BASE = 10
K = 4.5
##########################################



class Control:
    
    def __init__(self):

        ego_sub_topic     = '/Ego_topic'
        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64, queue_size=1)
        rospy.Subscriber(ego_sub_topic, EgoVehicleStatus, self.callbackego)
        self.pid = pidController()
        self.swg = SWG_make() 
        self.motor_msg = Float64()
        self.servo_msg = Float64()
        self.target_velocity = 2
        self.pure_pursuit = Purepursuit( WHEEL_BASE, K)
        self.is_status = False
        self.Ego_status = None
        self.FrameRate = 20
        self.rpm_gain = 4616
        self.steering_angle_to_servo_offset = 0.5304

        self.min_index = 0
        self.init_signal = 1
        self.old_WP_index = 0

        self.map = "path_name_1.txt"
        while True:
            if self.Ego_status:
                break
            else:
                print('Waiting for msg')
                time.sleep(0.5)
        self.main()

    def sub_ego(self, msg):

        ego_status_vel_x = msg.velocity.x/3.6
        ego_status_x     = msg.position.x
        ego_status_y     = msg.position.y
        ego_heading      = np.deg2rad(msg.heading)
        
        return ego_status_vel_x, ego_status_x, ego_status_y, ego_heading

    def callbackego(self,msg):

        self.Ego_status = msg


    def main(self):
        
        rate = rospy.Rate(self.FrameRate)
        
        while not rospy.is_shutdown():
            #WP_x , WP_y = read_txt(self.map)
            # index = get_WP_index(ego_status_x, ego_status_y, WP_x, WP_y)
            
            # target_WP_x = WP_x[index]
            # target_WP_y = WP_y[index]
            
            ego_status_vel_x, ego_status_x, ego_status_y, ego_heading = self.sub_ego(self.Ego_status) # Ego_status = GT?
            WP_index = self.swg.WAYPOINT_INITIALIZING(WP_X,WP_Y,ego_status_x,ego_status_y)
            lane_change_dir=0
            alpha,beta=local_motion_planning(WP_X,WP_Y,lane_change_dir)
            if (self.old_WP_index!=WP_index or min_index>50 or self.init_signal==1):
                WP_x,WP_y=self.swg.trajectory_make(alpha,beta,WP_index,ego_status_x,ego_status_y,ego_heading)
                self.init_signal=0
                TG_signal=1
            else:
                TG_signal=0

            #ego_status_vel_x, ego_status_x, ego_status_y, ego_heading = self.sub_ego(self.Ego_status) # Ego_status = GT?
            self.steering, min_index = self.pure_pursuit.pure_pursuit(WP_x, WP_y, ego_status_vel_x, ego_status_x,ego_status_y,ego_heading)
            self.control_input=self.pid.pid(self.target_velocity, ego_status_vel_x) 

            self.servo_msg = -self.steering   * 0.021    +  self.steering_angle_to_servo_offset 
            self.motor_msg = self.control_input * self.rpm_gain /3.6

            self.motor_pub.publish(self.motor_msg)    
            self.servo_pub.publish(self.servo_msg)

            self.old_WP_index = WP_index

            print(WP_index,min_index,TG_signal,lane_change_dir)

        
            rate.sleep()

if __name__ == '__main__':

    try:
        rospy.init_node('Control')
        Control()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
