#!/usr/bin/env python3

from decimal import DivisionByZero
import math
import rospy
import numpy as np
import time
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float64
from morai_msgs.msg import EgoVehicleStatus
from maneuver_planner_msg.msg import Lidar_track1, Lidar_track2
import copy
class lidar2D:
    def __init__(self):
        
        self.sub_lidar2D = rospy.Subscriber("/lidar2D",LaserScan, self.callback)
        self.sub_ego = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.callback_ego)
        self.pub_lidar_track1 = rospy.Publisher('/lidar_track1',Lidar_track1, queue_size=3)
        self.pub_lidar_track2 = rospy.Publisher('/lidar_track2',Lidar_track2, queue_size=3)

        self.FrameRate = 20
        self.lidar2D = None
        while True:
            if self.lidar2D:
                break
            else:
                print('Waiting for lidar msg')
                time.sleep(0.5)

        self.main()
    
    ############ sub topic #########3
    def sub_egostatus(self, msg):
        
        #ego_status_velocity = math.sqrt(msg.velocity.x ** 2 + msg.velocity.y ** 2 + msg.velocity.z ** 2)
        ego_status_x        = msg.position.x
        ego_status_y        = msg.position.y
        ego_status_heading_angle = msg.heading
        ego_status_heading_angle = np.deg2rad(ego_status_heading_angle)

        return ego_status_x, ego_status_y , ego_status_heading_angle
    
    def sub_lidar(self, msg):
        ranges          = msg.ranges
        angle_min       = msg.angle_min
        angle_max       = msg.angle_max
        angle_increment = msg.angle_increment
        ranges = list(ranges)
        
        max_range = 3 # parameter default = 3
        inrange_index = []

        for i in range(len(ranges)):
            if ranges[i] < max_range:
                inrange_index.append(i) # [1 2 3 8 9]

        arr = np.zeros((len(inrange_index),4))
        
        for idx in range(len(inrange_index)):
            number = inrange_index[idx]
            idx_range = ranges[number]
            x_value = idx_range*math.cos((angle_min+angle_increment * (number-1)))
            y_value = idx_range*math.sin((angle_min+angle_increment * (number-1)))
            arr[idx, 0] = idx_range
            arr[idx, 1] = x_value
            arr[idx, 2] = y_value
            arr[idx, 3] = number
        
       
        return arr




                     
    ############ callback ###########
    def callback(self, msg):
        self.lidar2D = msg
    
    def callback_ego(self, msg):
        
        self.Ego_status = msg


    ########### module ###############
    def findObstacle(self, arr, ego_status_x, ego_status_y, ego_status_heading_angle):
        index = []
        arr_new = copy.deepcopy(arr)
        
        for i in range(arr.shape[0]):
            arr_new[i, 1] =  np.cos(ego_status_heading_angle) * arr[i,1] + -np.sin(ego_status_heading_angle) * arr[i,2] + ego_status_x # 0: distance 1:x , 2:, 3:point index
            
            arr_new[i, 2] =  np.sin(ego_status_heading_angle) * arr[i,1] + np.cos(ego_status_heading_angle) * arr[i,2]+ ego_status_y
            
            if arr_new[i, 2] < -6.7 or arr_new[i, 2] > 6.7 or  arr_new[i, 1] < -20 or arr_new[i, 1] > 20: # (arr_new[i, 2] >1 and arr_new[i, 2] < 4.5))
            
                index.append(i)
                

        obstacle = np.delete(arr_new,index,0)
        D_max = 0.5 # parameter #0.3
        clustering_1 = []
        clustering_2 = []       
        standard_point = 0
        cnt = 0
        for idx in range(obstacle.shape[0]):
            
            if idx == 0:
                continue
            else:
                
                k_1_point = obstacle[idx-1,1] # k-1 distancew
                
                k_point = obstacle[idx,1] # k distance
                k_1_point_d = obstacle[idx-1,0]
                k_point_d = obstacle[idx,0]
                #k_point_x = obstacle[idx,1]
                #k_point_y = obstacle[idx,2]
                if abs(k_point - k_1_point) < D_max : # close distance
                    a = list(obstacle[idx,:])
                    
                    clustering_1.append(a)
                    standard_point = k_point_d
                    
                else: # far distance                      
                    if standard_point < k_point_d:
                        a = list(obstacle[idx,:])
                        clustering_2.append(a)
                        #continue
                    else:
                        clustering_2 = []
                        clustering_2 = clustering_1
                        clustering_1 = []                      
                        a = list(obstacle[idx,:])
                        clustering_1.append(a)
        
        sum_x_1 = 0
        sum_y_1 = 0
        sum_d_1 = 0
        avg_x_1 = 0
        avg_y_1 = 0
        avg_d_1 = 0
        rel_x_1 = 0
        rel_y_1 = 0

        sum_x_2 = 0
        sum_y_2 = 0
        sum_d_2 = 0
        avg_x_2 = 0
        avg_y_2 = 0
        avg_d_2 = 0
        rel_x_2 = 0
        rel_y_2 = 0

        for i in range(len(clustering_1)):
            d = clustering_1[i][0]
            x = clustering_1[i][1]
            y = clustering_1[i][2]
            sum_d_1 = sum_d_1 + d
            sum_x_1 = sum_x_1 + x
            sum_y_1 = sum_y_1 + y
            
        for i in range(len(clustering_2)):
            #print('empty')
            d2 = clustering_2[i][0]
            x2 = clustering_2[i][1]
            y2 = clustering_2[i][2]
            sum_d_2 = sum_d_2 + d2
            sum_x_2 = sum_x_2 + x2
            sum_y_2 = sum_y_2 + y2
        
        # 빈 행렬들어올때랑 아닌 행렬 들어올때
        try:
            avg_x_1  = sum_x_1 /len(clustering_1)
            avg_y_1  = sum_y_1 /len(clustering_1)
            rel_x_1 = np.cos(ego_status_heading_angle) * (avg_x_1 - ego_status_x) + np.sin(ego_status_heading_angle) * (avg_y_1 -ego_status_y)
            rel_y_1 = - np.sin(ego_status_heading_angle) * (avg_x_1 - ego_status_x) + np.cos(ego_status_heading_angle) * (avg_y_1 -ego_status_y)
            avg_d_1  = sum_d_1 /len(clustering_1)

            avg_x_2  = sum_x_2 /len(clustering_2)
            avg_y_2  = sum_y_2 /len(clustering_2)        
            rel_x_2 = np.cos(ego_status_heading_angle) * (avg_x_2 - ego_status_x) + np.sin(ego_status_heading_angle) * (avg_y_2 -ego_status_y)
            rel_y_2 = - np.sin(ego_status_heading_angle) * (avg_x_2 - ego_status_x) + np.cos(ego_status_heading_angle) * (avg_y_2 -ego_status_y)
            avg_d_2  = sum_d_2 /len(clustering_2)
        except ZeroDivisionError:
            pass
            #print('zero')




                        
        Lidar_Track_list = []

        return avg_x_1,avg_y_1,avg_d_1, rel_x_1, rel_y_1,avg_x_2,avg_y_2,avg_d_2, rel_x_2, rel_y_2, clustering_1, clustering_2

    def main(self):
        while not rospy.is_shutdown():

            rate = rospy.Rate(self.FrameRate)
            arr = self.sub_lidar(self.lidar2D)
            ego_status_x, ego_status_y, ego_status_heading_angle = self.sub_egostatus(self.Ego_status)

            #avg_x, avg_y,avg_d, rel_x , rel_y = self.findObstacle(arr,ego_status_x,ego_status_y,ego_status_heading_angle)
            avg_x_1,avg_y_1,avg_d_1, rel_x_1, rel_y_1,avg_x_2,avg_y_2,avg_d_2, rel_x_2, rel_y_2,clustering_1, clustering_2 = self.findObstacle(arr,ego_status_x,ego_status_y,ego_status_heading_angle)

            avg_x_1 = round(avg_x_1,4)
            avg_y_1 = round(avg_y_1,4)
            avg_d_1 = round(avg_d_1,4)
            rel_x_1 = round(rel_x_1,4)
            rel_y_1 = round(rel_y_1,4)

            avg_x_2 = round(avg_x_2,4)
            avg_y_2 = round(avg_y_2,4)
            avg_d_2 = round(avg_d_2,4)
            rel_x_2 = round(rel_x_2,4)
            rel_y_2 = round(rel_y_2,4)
            ego_status_x = round(ego_status_x,4)
            ego_status_y = round(ego_status_y,4)
            


            #print('Lidar Track')
            #print(f'avg_x_1 = {avg_x_1}, avg_y_1 = {avg_y_1}, avg_d_1 = {avg_d_1}')
            #print(f'rel_x_1 = {rel_x_1}, rel_y_1 = {rel_y_1}')
             #print(f'ego_status_x = {ego_status_x}, ego_status_y = {ego_status_y}')
            #print(f'avg_x_2 = {avg_x_2}, avg_y_2 = {avg_y_2}, avg_d_2 = {avg_d_2}')
            #print(f'rel_x_2 = {rel_x_2}, rel_y_2 = {rel_y_2}')


            Lidar_Track_msg1 = Lidar_track1()

            Lidar_Track_msg1.avg_x = avg_x_1
            Lidar_Track_msg1.avg_y = avg_y_1
            Lidar_Track_msg1.avg_d = avg_d_1
            Lidar_Track_msg1.rel_x = rel_x_1
            Lidar_Track_msg1.rel_y = rel_y_1
            self.pub_lidar_track1.publish(Lidar_Track_msg1)

            Lidar_Track_msg2 = Lidar_track1()

            Lidar_Track_msg2.avg_x = avg_x_2
            Lidar_Track_msg2.avg_y = avg_y_2
            Lidar_Track_msg2.avg_d = avg_d_2
            Lidar_Track_msg2.rel_x = rel_x_2
            Lidar_Track_msg2.rel_y = rel_y_2
            self.pub_lidar_track2.publish(Lidar_Track_msg2)

            rate.sleep()

        #print(angle_min)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('lidar2D')
        lidar2D()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass





