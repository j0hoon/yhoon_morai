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

# pid_contorl 은 차량의 차량의 종 횡 방향 제어 예제입니다.
# 횡방향 제어 입력은 주행할 Local Path (지역경로) 와 차량의 상태 정보 Odometry 를 받아 차량을 제어 합니다.
# 종방향 제어 입력은 목표 속도를 지정 한뒤 목표 속도에 도달하기 위한 Throttle control 을 합니다.
# 종방향 제어 입력은 longlCmdType 1(Throttle control) 이용합니다.

# 노드 실행 순서 
# 1. subscriber, publisher 선언
# 2. 좌표 변환 행렬 생성
# 3. Steering 각도 계산
# 4. PID 제어 생성
# 5. 제어입력 메세지 Publish


class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        #TODO: (1) subscriber, publisher 선언
        rospy.Subscriber("local_path", Path, self.path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.status_callback)  
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd',CtrlCmd, queue_size=1)


        self.ctrl_cmd_msg=CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType=1

        self.is_path=False
        self.is_odom=False          
        self.is_status=False

        self.is_look_forward_point=False

        self.forward_point=Point()
        self.current_postion=Point()

        self.vehicle_length = 1
        self.lfd = 5
        self.target_vel = 60

        self.pid = pidControl()

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():

            if self.is_path == True and self.is_odom == True and self.is_status == True:

                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point :
                    self.ctrl_cmd_msg.steering = steering
                else : 
                    print("no found forward point")
                    self.ctrl_cmd_msg.steering=0.0

                output = self.pid.pid(self.target_vel,self.status_msg.velocity.x*3.6)

                if output > 0.0:
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                #TODO: (5) 제어입력 메세지 Publish
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            rate.sleep()

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status=True
        self.status_msg=msg

    def calc_pure_pursuit(self,): 
        vehicle_position=self.current_postion
        self.is_look_forward_point= False

        translation = [vehicle_position.x, vehicle_position.y]

        #TODO: (2) 좌표 변환 행렬 생성
        trans_matrix = np.array([
                [cos(self.vehicle_yaw), -sin(self.vehicle_yaw),translation[0]],
                [sin(self.vehicle_yaw),cos(self.vehicle_yaw),translation[1]],
                [0                    ,0                    ,1            ]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for num,i in enumerate(self.path.poses) :
            path_point=i.pose.position

            global_path_point = [path_point.x,path_point.y,1]
            local_path_point = det_trans_matrix.dot(global_path_point)    

            if local_path_point[0]>0 :
                dis = sqrt(pow(local_path_point[0],2)+pow(local_path_point[1],2))
                if dis >= self.lfd :
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break
        
        #TODO: (3) Steering 각도 계산
        theta = atan2(local_path_point[1],local_path_point[0])
        steering = atan2((2*self.vehicle_length*sin(theta)),self.lfd)

        return steering

class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.00
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel

        #TODO: (4) PID 제어 생성
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output

if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
