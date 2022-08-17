#!/usr/bin/env python3

import rospy
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from std_msgs.msg import Float64
from maneuver_planner_msg.msg import maneuver_planner_msg
import time
from math import pi, sin, cos, atan2, sqrt
#from read_csv import read_csv, get_WP_index, read_txt
from utils.utils import Purepursuit, pidController, ACC
from utils.genWP import WP_X, WP_Y
from utils.SWG import SWG_make
from utils.Local_Motion_Planning import local_motion_planning
import numpy as np



################ Parameter ###############
WHEEL_BASE = 0.78
K = 0.4
##########################################

def cyclodial_curve(target_V, max_acc ,t):

    A = (target_V) ** 2 / (4 * max_acc)
    B = 2 * max_acc/(target_V)
    
    time_interval = target_V * np.pi / (2 * max_acc)

    if t < time_interval:

        return A * B *( 1 - np.cos(B * t))
        
    else:

        return target_V

class Control:
    
    def __init__(self):

        ego_sub_topic     = '/Ego_topic'
        driving_mode_topic      = '/mode'
        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64, queue_size=1)
        rospy.Subscriber(ego_sub_topic, EgoVehicleStatus, self.callbackego)
        rospy.Subscriber(driving_mode_topic, maneuver_planner_msg, self.callbackmode)

        self.pid = pidController()
        self.swg = SWG_make() 
        
        self.motor_msg = Float64()
        self.servo_msg = Float64()
        self.target_velocity = 6
        self.pure_pursuit = Purepursuit(WHEEL_BASE, K)
        self.is_status = False
        self.Ego_status = None
        self.FrameRate = 20
        self.rpm_gain = 4616 #4616
        self.steering_angle_to_servo_offset = 0.5304
        self.init_time = time.time()
        self.min_index = 0
        self.init_signal = 1
        self.old_WP_index = 0
        self.map = "path_name_1.txt"
        self.lane_change_dir = 0 # -1 lane# : 1, 0 lane# : 2
        self.delay=0
        self.max_acc = 5
        while True:
            if self.Ego_status:
                break
            else:
                print ('Waiting for msg')
                time.sleep(0.5)
        self.main()

    def get_rpm_gain(self, target_vel):

        rpm_gain = 0.28 * (target_vel - 4) + 1.24 
        return rpm_gain

    def sub_ego(self, msg):

        ego_status_acc_x = msg.acceleration.x
        ego_status_vel_x = msg.velocity.x
        
        ego_status_x     = msg.position.x 
        ego_status_y     = msg.position.y
        ego_heading      = np.deg2rad(msg.heading)
        
        return ego_status_acc_x, ego_status_vel_x, ego_status_x, ego_status_y, ego_heading

    def sub_mode(self, msg):

        driving_mode = msg.mode
        
        return driving_mode

    def callbackego(self,msg):

        self.Ego_status = msg

    def callbackmode(self,msg):

        self.mode_status = msg


    def main(self):
        
        rate = rospy.Rate(self.FrameRate)
        
        while not rospy.is_shutdown():
            #WP_x , WP_y = read_txt(self.map)
            # index = get_WP_index(ego_status_x, ego_status_y, WP_x, WP_y)
            
            # target_WP_x = WP_x[index]
            # target_WP_y = WP_y[index]
            t = time.time() - self.init_time
            self.target_vel = cyclodial_curve(self.target_velocity, self.max_acc, t)
            driving_mode = self.sub_mode(self.mode_status)
            
            print(driving_mode)
            if self.lane_change_dir == 0 and driving_mode == 3:
                self.lane_change_dir = -1
                

            print(self.lane_change_dir)
            ego_status_acc_x, ego_status_vel_x, ego_status_x, ego_status_y, ego_heading = self.sub_ego(self.Ego_status) # Ego_status = GT?
            WP_index = self.swg.WAYPOINT_INITIALIZING(WP_X,WP_Y,ego_status_x,ego_status_y)
            
            alpha,beta=local_motion_planning(WP_X,WP_Y,self.lane_change_dir)
            if (self.old_WP_index != WP_index or min_index > 50 or self.init_signal == 1):
                WP_x,WP_y = self.swg.trajectory_make(alpha,beta,WP_index,ego_status_x,ego_status_y,ego_heading)
                self.init_signal = 0

                TG_signal = 1
            else:
                TG_signal = 0

            
            #ego_status_vel_x, ego_status_x, ego_status_y, ego_heading = self.sub_ego(self.Ego_status) # Ego_status = GT?
            self.steering, min_index = self.pure_pursuit.pure_pursuit(WP_x, WP_y, ego_status_vel_x, ego_status_x,ego_status_y,ego_heading)
            self.control_input = self.pid.pid(self.target_vel, ego_status_vel_x * 3.6) 

            rpm_gain = self.get_rpm_gain(self.target_velocity)
            self.servo_msg = -self.steering * 0.51 +  self.steering_angle_to_servo_offset 
            self.motor_msg = 1000 * (self.control_input + rpm_gain)  #2350   target 4 = 1.240, target 6 = 1.80 target 8 = 2.350
            print("control_input:", self.control_input)
            
            self.motor_pub.publish(self.motor_msg)    
            self.servo_pub.publish(self.servo_msg)

            self.old_WP_index=WP_index

            # print(self.old_WP_index,min_index,TG_signal)
            # print(self.control_input , self.target_velocity )
            print(ego_status_vel_x *(3.6), self.target_vel, time.time()- self.init_time)
            rate.sleep()

if __name__ == '__main__':

    try:
        rospy.init_node('Control')
        
        Control()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass