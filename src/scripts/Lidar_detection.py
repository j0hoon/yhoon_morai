from decimal import DivisionByZero
import math
import rospy
import numpy as np
import time
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float64
from morai_msgs.msg import EgoVehicleStatus
from maneuver_planner_msg.msg import Lidar_track
import copy
class lidar2D:
    def __init__(self):
        
        self.sub_lidar2D = rospy.Subscriber("/lidar2D",LaserScan, self.callback)
        self.sub_ego = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.callback_ego)
        self.pub_lidar_track = rospy.Publisher('/lidar_track',Lidar_track, queue_size=3)
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
        
        max_range = 2.8 # parameter default = 3
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
        # if not arr.all():
        for i in range(arr.shape[0]):
            arr_new[i, 1] =  np.cos(ego_status_heading_angle) * arr[i,1] + -np.sin(ego_status_heading_angle) * arr[i,2] + ego_status_x # 0: distance 1:x , 2:, 3:point index
            
            arr_new[i, 2] =  np.sin(ego_status_heading_angle) * arr[i,1] + np.cos(ego_status_heading_angle) * arr[i,2]+ ego_status_y
            #if arr[i, 2] <-6  or (arr[i, 2] > 5.5 or arr[i, 2] < 4.5) or arr[i, 1] < -20 or arr[i, 1] > 20:
            if arr_new[i, 2] < -6.7 or arr_new[i, 2] > 6.7 or  arr_new[i, 1] < -20 or arr_new[i, 1] > 20: # (arr_new[i, 2] >1 and arr_new[i, 2] < 4.5))
            
                index.append(i)
                #print(index)

        obstacle = np.delete(arr_new,index,0)
        D_max = 0.3 # parameter
        clustering = []
        cnt = 0
        standard_point = 0
        for idx in range(obstacle.shape[0]):
            if idx == 0:
                continue
            else:
                k_1_point = obstacle[idx-1,0] # k-1 distancew
                #k_1_point_x = obstacle[idx-1,1]
                k_point = obstacle[idx,0] # k distance
                #k_point_x = obstacle[idx,1]
                if abs(k_point - k_1_point) < D_max : # close distance
                    a = list(obstacle[idx,:])
                    
                    clustering.append(a)
                    standard_point = k_point
                    #cnt = cnt+1
                else: # far distance
                    if standard_point < k_point:
                        continue
                    else:
                        clustering = []
                        #cnt = 0
                        a = list(obstacle[idx,:])
                        clustering.append(a)
        #clustering = list(clustering)
        sum_x = 0
        sum_y = 0
        sum_d = 0
        avg_x = 0
        avg_y = 0
        avg_d = 0
        rel_x = 0
        rel_y = 0
        for i in range(len(clustering)):
            d = clustering[i][0]
            x = clustering[i][1]
            y = clustering[i][2]
            sum_d = sum_d + d
            sum_x = sum_x + x
            sum_y = sum_y + y
        try:
            avg_x  = sum_x /len(clustering)

            avg_y  = sum_y /len(clustering)

            rel_x = np.cos(ego_status_heading_angle) * (avg_x - ego_status_x) + np.sin(ego_status_heading_angle) * (avg_y -ego_status_y)
            rel_y = - np.sin(ego_status_heading_angle) * (avg_x - ego_status_x) + np.cos(ego_status_heading_angle) * (avg_y -ego_status_y)
            avg_d  = sum_d /len(clustering)
        except ZeroDivisionError:
            print('zero')

        # avg_x = clustering[:][:,1] / len(clustering)
        # avg_y = clustering[:][:,2] / len(clustering)
        # else:
        #     avg_x  = 0
        #     avg_y  = 0
        #     avg_d  = 0
                        
        Lidar_Track_list = []

        return avg_x,avg_y,avg_d, rel_x, rel_y

    def main(self):
        while not rospy.is_shutdown():

            rate = rospy.Rate(self.FrameRate)
            arr = self.sub_lidar(self.lidar2D)
            ego_status_x, ego_status_y, ego_status_heading_angle = self.sub_egostatus(self.Ego_status)

            avg_x, avg_y,avg_d, rel_x , rel_y = self.findObstacle(arr,ego_status_x,ego_status_y,ego_status_heading_angle)
            avg_x = round(avg_x,4)
            avg_y = round(avg_y,4)
            avg_d = round(avg_d,4)
            rel_x = round(rel_x,4)
            rel_y = round(rel_y,4)
            ego_status_x = round(ego_status_x,4)
            ego_status_y = round(ego_status_y,4)
            
            print('Lidar Track')
            print(f'avg_x = {avg_x}, avg_y = {avg_y}, avg_d = {avg_d}')
            print(f'ego_status_x = {ego_status_x}, ego_status_y = {ego_status_y}')
            # print(f'avg_y = {avg_y}')
            # print(f'avg_d = {avg_d}')
            print(f'rel_x = {rel_x}, rel_y = {rel_y}')
            # print(f'rel_y = {rel_y}')
            #print(clustering[:][:,1])
            #print(len(clustering))

            Lidar_Track_msg = Lidar_track()

            Lidar_Track_msg.avg_x = avg_x
            Lidar_Track_msg.avg_y = avg_y
            Lidar_Track_msg.avg_d = avg_d
            Lidar_Track_msg.rel_x = rel_x
            Lidar_Track_msg.rel_y = rel_y
            self.pub_lidar_track.publish(Lidar_Track_msg)
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





