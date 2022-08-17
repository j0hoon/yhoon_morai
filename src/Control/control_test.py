#!/usr/bin/env python2

import rospy
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
import time
from std_msgs.msg import Float64,Int16,Float32MultiArray
from math import pi, sin, cos, atan2, sqrt
from read_csv import read_csv, get_WP_index, read_txt
import numpy as np

WHEEL_BASE = 10
K = 5
LW = WHEEL_BASE


def pure_pursuit(ref_x, ref_y, V_x, ego_x, ego_y, yaw):
    # yaw: (rad/s)
    #
    Lp = K * V_x
    Lp_x = ego_x + Lp * np.cos(yaw)
    Lp_y = ego_y + Lp * np.sin(yaw)

    dis_P2 = np.sqrt((np.array(ref_x) - Lp_x)**2+(np.array(ref_y) - Lp_y)**2)

    min_index = np.argmin(dis_P2)

    Way_x = ref_x[min_index]
    Way_y = ref_y[min_index]
    print Way_x, Way_y
    x_2 = (Way_x - ego_x) * np.cos(yaw) + (Way_y - ego_y) * np.sin(yaw)
    y_2 = - (Way_x - ego_x) * np.sin(yaw) + (Way_y - ego_y) * np.cos(yaw)

    L_bar = np.sqrt(x_2**2 +y_2**2)

    sin_alpha = y_2/L_bar

    k = 2 * sin_alpha/L_bar

    steer_angle = LW * 2 * y_2 / (L_bar)**2

    return steer_angle

class pidController : 
    def __init__(self):
        self.p_gain= 0.5
        self.i_gain=0.0
        self.d_gain=0.1
        self.controlTime=0.033
        self.prev_error=0
        self.i_control=0


    def pid(self,target_vel,current_vel):
        error= target_vel-current_vel
        
        p_control=self.p_gain*error
        self.i_control+=self.i_gain*error*self.controlTime
        d_control=self.d_gain*(error-self.prev_error)/self.controlTime

        output=p_control+self.i_control+d_control
        self.prev_error=error
        return output

class Control:
    
    def __init__(self):

        ego_sub_topic     = '/Ego_topic'
        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64, queue_size=1)
        rospy.Subscriber(ego_sub_topic, EgoVehicleStatus, self.callbackego)
        self.pid =pidController()
        self.target_velocity = 2
        self.motor_msg=Float64()
        self.servo_msg=Float64()
        self.is_status = False
        self.Ego_status = None
        self.FrameRate = 20
        self.rpm_gain = 4616
        self.steering_angle_to_servo_offset = 0.5304
        self.map = "path_name_1.txt"
        while True:
            if self.Ego_status:
                break
            else:
                print 'Waiting for msg'
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
        
        while True:
            WP_x , WP_y = read_txt(self.map)
            # index = get_WP_index(ego_status_x, ego_status_y, WP_x, WP_y)
            
            # target_WP_x = WP_x[index]
            # target_WP_y = WP_y[index]

            
            ego_status_vel_x, ego_status_x, ego_status_y, ego_heading = self.sub_ego(self.Ego_status)
            self.steering =   pure_pursuit(WP_x, WP_y, ego_status_vel_x, ego_status_x,ego_status_y,ego_heading)
            self.control_input=self.pid.pid(self.target_velocity, ego_status_vel_x) 

            self.servo_msg = -self.steering   * 0.021    +  self.steering_angle_to_servo_offset 
            self.motor_msg = self.control_input * self.rpm_gain /3.6
            self.motor_pub.publish(self.motor_msg)    
            self.servo_pub.publish(self.servo_msg)

            
            # print self.servo_msg-self.steering_angle_to_servo_offset

            rate.sleep()

if __name__ == '__main__':

    try:
        rospy.init_node('Control')
        Control()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass