#!/usr/bin/env python3
from decimal import DivisionByZero
import numpy as np
import time
import math
import rospy
from std_msgs.msg import Int32
from maneuver_planner_msg.msg import maneuver_planner_msg, LC_complete_msg
from morai_msgs.msg import GetTrafficLightStatus, EgoVehicleStatus, ObjectStatusList # 신호등 신호
from vision_msg.msg import bboxes_3d
from maneuver_planner_msg.msg import Lidar_track1, Lidar_track2, Mission


########### this is parameter before getting lane value
lane_curvature = 0.00001
lane_curvature_for_curve = 0.26
lane_offset = 0.15 # init 0.18
lane_offset_for_mission_5 = 0.4


class ManeuverPlanner():
    def __init__(self):
        
        # Publish
        self.mode_pub = rospy.Publisher('/mode', maneuver_planner_msg , queue_size=3)
        # subscriber
        self.sub_ego = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.callback_ego)
        self.sub_trafficlight = rospy.Subscriber('/GetTrafficLightStatus', GetTrafficLightStatus, self.callback_traffic)
        self.sub_Object_1 = rospy.Subscriber('/lidar_track1', Lidar_track1, self.callback_object)  # 라이다 토픽 받을 때
        self.sub_Object_2 = rospy.Subscriber('/lidar_track2', Lidar_track2, self.callback_object_2)
        self.sub_mission_num = rospy.Subscriber('/mission', Mission, self.callback_mission)
        self.FrameRate = 20
        
        self.isAlive = 0
        self.Ego_status = None
        self.isAliveCount_TrafficLight = None
        self.mission_number = None 
        self.Object_status = None
        self.maneuver_mode = 1


        while not rospy.is_shutdown():
            if self.Ego_status:
                break
            else:
                print('Waiting for Ego status msg')
                time.sleep(0.5)
            if self.Object_status:
                break
            else:
                print('Waiting for object msg')
                time.sleep(0.5)
            if self.mission_number:
                break
            else:
                print('Waiting for mission_number')
                time.sleep(0.5)           
        self.main()
    
    ############# sub topic ############

    def sub_egostatus(self, msg):
        
        ego_status_velocity = math.sqrt(msg.velocity.x ** 2 + msg.velocity.y ** 2 + msg.velocity.z ** 2)
        ego_status_x        = msg.position.x
        ego_status_y        = msg.position.y
        ego_heading         = msg.heading
        return ego_status_velocity, ego_status_x, ego_status_y, ego_heading

    def sub_Objectstatus(self, msg):

        object_class = None
        object_status_x = msg.avg_x
        object_status_y = msg.avg_y
        object_status_d = msg.avg_d
        object_status_rel_x = msg.rel_x
        object_status_rel_y = msg.rel_y
        

        return object_class, object_status_x, object_status_y, object_status_d, object_status_rel_x, object_status_rel_y

    def sub_trafficsign(self, msg):


        trafficlight_signal = msg

        return trafficlight_signal

    def sub_mission(self, msg):
        
        
        mission_number = msg.mission


        return mission_number

    ############ callback ############

    def callback_ego(self, msg):
        
        self.Ego_status = msg

    def callback_traffic(self, msg):
        
        self.Traffic_signal = msg.trafficLightStatus
        self.isAliveCount_TrafficLight = msg.header.seq

    def callback_object(self, msg):

        self.Object_status = msg

    def callback_object_2(self, msg):
        self.Object_status_2 = msg

    def callback_mission(self, msg):

        self.mission_number = msg

    ############ module ##############              

    def threatAssessment(self, D_nearest, V_ego):

        IVT = D_nearest / (V_ego +0.001) # InterVehicularTime 

        MSD = D_nearest # MinimalSafeDistance
        
        # IVT risk 판별
        IVT_min_time= 2
        if IVT < IVT_min_time:
            Risk_IVT = 1
        else:
            Risk_IVT = 0
        
        MSD_min_distance = 2
        if MSD < MSD_min_distance:
            Risk_MSD = 1
        else:
            Risk_MSD = 0
           
        RISK_F = Risk_IVT + Risk_MSD
        if RISK_F >= 1:
            risk = 1
        else:
            risk = 0
        
        return risk, IVT, MSD, RISK_F
        
        
    def RecognitionObject(self, lane_offset, lane_curvature, rel_x,rel_y):

        epsilon     = lane_offset # lane_offset = 0.2 , lane_curvature = 0.00001
        #rel_x       = object_status_x - ego_status_x
        #rel_y       = object_status_y - ego_status_y
        lane_radius = 1 / lane_curvature

        p_f = lane_radius - math.sqrt(rel_x ** 2 + (rel_y - lane_radius) ** 2)

        if p_f < - epsilon :
            object_state = -1 # FVL

        elif p_f > epsilon :
            object_state = 1 # FVR

        else :
            object_state = 0 # FVI
        
        return object_state

    def ManeuverMode(self, risk, object_state, traffic_signal): # + WP_index --> mode change

        # 곡률에 따른 모드 넣으면 좋을듯
        # Traffic Light == 1 - red
        # Traffic Light == 4 - yellow
        # Traffic Light == 16 - green
        # Traffic Light == 33 - red and green(left)
        # Traffic Light == 5 - red and yellow

        if risk == 1 and object_state != 0 and traffic_signal != 1: # 위험물체 잡혔지만 내차선에는 없고 빨간불 x 
            maneuver_mode = 8 # driving mode

        elif risk == 1 and object_state == 0: # 위험객체 있고 내 차선에 
            maneuver_mode = 3 # lane change mode
        
        elif risk == 0 and object_state != 0:
            maneuver_mode = 8 # driving mode

        else:
            maneuver_mode = 8

        return maneuver_mode

    def main(self):
        rate = rospy.Rate(self.FrameRate) # init self.FrameRate = 20
        isAliveCount_TrafficLight_old = None
        while not rospy.is_shutdown():

            ego_status_velocity, ego_status_x, ego_status_y, ego_heading = self.sub_egostatus(self.Ego_status)
            object_class_1, object_status_x_1, object_status_y_1, object_status_d_1,object_status_rel_x_1, object_status_rel_y_1  = self.sub_Objectstatus(self.Object_status)
            object_class_2, object_status_x_2, object_status_y_2, object_status_d_2,object_status_rel_x_2, object_status_rel_y_2  = self.sub_Objectstatus(self.Object_status_2)

            # Calculate min distance

            if object_status_rel_x_1 > 0:
                object_status_x = object_status_x_1
                object_status_y = object_status_y_1
                object_status_d = object_status_d_1
                object_status_rel_x = object_status_rel_x_1
                object_status_rel_y = object_status_rel_y_1
                          
            else:
                if object_status_rel_x_2 < 0:
                    object_status_x = 0
                    object_status_y = 0
                    object_status_d = 0
                    object_status_rel_x = 0
                    object_status_rel_y = 0  
                else:
                    object_status_x = object_status_x_2
                    object_status_y = object_status_y_2
                    object_status_d = object_status_d_2
                    object_status_rel_x = object_status_rel_x_2
                    object_status_rel_y = object_status_rel_y_2   
                        
            
            try:
                mission_number                                  = self.sub_mission(self.mission_number)
            except AttributeError:
                mission_number = 0

            print("mission :",mission_number )
            
            if (self.isAliveCount_TrafficLight is not None) and (self.isAliveCount_TrafficLight != isAliveCount_TrafficLight_old):

                trafficlight_signal = self.Traffic_signal
                
            else:

                trafficlight_signal = 0

            if mission_number == 0 :
                self.maneuver_mode = 1
                self.mode_pub.publish(self.maneuver_mode)
                k_1_object_status_d = 0

                rate.sleep()


            elif mission_number == 1:
                self.maneuver_mode = 5
                print("maneuver_mode : ", self.maneuver_mode)
                
                    
                self.mode_pub.publish(self.maneuver_mode)


            elif mission_number == 2:
                rotary_mode = 0
                # front and back 1
                if (object_status_rel_x_1 > 0 and object_status_rel_x_2 < 0):
                    object_status_rel_x_back = object_status_rel_x_2
                    object_status_d_back = object_status_d_2
                    
                    object_status_rel_x_front = object_status_rel_x_1
                    object_status_d_front = object_status_d_1
                    rotary_mode = 1

                
                # front and back 2
                elif (object_status_rel_x_1 < 0 and object_status_rel_x_2 > 0):
                    object_status_rel_x_back = object_status_rel_x_1
                    object_status_d_back = object_status_d_1
                    
                    object_status_rel_x_front = object_status_rel_x_2
                    object_status_d_front = object_status_d_2
                    rotary_mode = 2

                # one object detect and it is behind me             
                elif object_status_rel_x_1 < 0 and (object_status_rel_x_2 == 0 or object_status_rel_x_2 < 0):
                    object_status_rel_x_back = object_status_rel_x_1
                    object_status_d_back = object_status_d_1
                    
                    object_status_rel_x_front = 5
                    object_status_d_front = 5
                    rotary_mode = 3
                
                # one object detect and it is in front of me 
                elif object_status_rel_x_1 > 0 and (object_status_rel_x_2 == 0 or object_status_rel_x_2>0):
                    object_status_rel_x_back = -5
                    object_status_d_back = -5
                    
                    object_status_rel_x_front = object_status_rel_x_1
                    object_status_d_front = object_status_d_1
                    rotary_mode = 4
                
                else:
                    object_status_rel_x_front = object_status_rel_x_1
                    object_status_d_front = object_status_d_1

                    object_status_rel_x_back = object_status_rel_x_2
                    object_status_d_back = object_status_d_2
                    rotary_mode = 5

                if rotary_mode == 1 or rotary_mode == 4:
                    if object_status_d_front > 1.5:
                        self.maneuver_mode = 6 # acc
                        self.mode_pub.publish(self.maneuver_mode)  
                        print("maneuver_mode + acceration : ", self.maneuver_mode) 
                        rate.sleep()
                    elif object_status_d_front <= 1.5 and object_status_d_front > 1:
                        self.maneuver_mode = 7 # target_spd = 5
                        self.mode_pub.publish(self.maneuver_mode)  
                        print("maneuver_mode : ", self.maneuver_mode) 
                        rate.sleep()
                    
                    
                    else:
                        self.maneuver_mode = 2 
                        self.mode_pub.publish(self.maneuver_mode)
                        print("maneuver_mode + deceleration : ", self.maneuver_mode)
                        rate.sleep()


                elif rotary_mode == 2 or rotary_mode == 3:
                    
                    self.maneuver_mode = 6 # acc
                    self.mode_pub.publish(self.maneuver_mode)  
                    print("maneuver_mode + acceration : ", self.maneuver_mode) 
                    rate.sleep()

                else :
                    if object_status_d_back < 1.5 :
                        self.maneuver_mode = 6 # init = 6
                        self.mode_pub.publish(self.maneuver_mode)  
                        print("maneuver_mode + acceration : ", self.maneuver_mode) 
                        rate.sleep()     
                    
                    elif object_status_d_front < 1.5 and object_status_d_front != 0 :
                        self.maneuver_mode = 2 
                        self.mode_pub.publish(self.maneuver_mode)
                        print("maneuver_mode + deceleration : ", self.maneuver_mode)
                        rate.sleep()
                    else:
                        self.maneuver_mode = 7 # target_spd = 5
                        self.mode_pub.publish(self.maneuver_mode)  
                        print("maneuver_mode : ", self.maneuver_mode) 
                        rate.sleep()                   
            

            
            elif mission_number == 3: 

                if trafficlight_signal == 1:
                    self.maneuver_mode = 4
                    self.mode_pub.publish(self.maneuver_mode)

                elif trafficlight_signal == 4:
                    self.maneuver_mode = 4
                    self.mode_pub.publish(self.maneuver_mode)

                elif trafficlight_signal == 5:
                    self.maneuver_mode = 4
                    self.mode_pub.publish(self.maneuver_mode) 

                elif trafficlight_signal == 16:
                    self.maneuver_mode = 4
                    self.mode_pub.publish(self.maneuver_mode) 

                elif trafficlight_signal == 33:
                    self.maneuver_mode = 1
                    self.mode_pub.publish(self.maneuver_mode)  

            elif mission_number == 33:
                self.maneuver_mode = 7 # target_spd = 5
                self.mode_pub.publish(self.maneuver_mode)
                print('back to the 2nd lane')
                rate.sleep()

            elif mission_number == 4:
                if object_status_rel_x_1 > 0:
                    object_status_x = object_status_x_1
                    object_status_y = object_status_y_1
                    object_status_d = object_status_d_1
                    object_status_rel_x = object_status_rel_x_1
                    object_status_rel_y = object_status_rel_y_1
                            
                else:
                    if object_status_rel_x_2 < 0:
                        object_status_x = 0
                        object_status_y = 0
                        object_status_d = 0
                        object_status_rel_x = 0
                        object_status_rel_y = 0  
                    else:
                        object_status_x = object_status_x_2
                        object_status_y = object_status_y_2
                        object_status_d = object_status_d_2
                        object_status_rel_x = object_status_rel_x_2
                        object_status_rel_y = object_status_rel_y_2   
                D_nearest = math.sqrt((object_status_x-ego_status_x)**2+(object_status_y-ego_status_y)**2)
                
                risk,IVT, MSD, RISK_F                           = self.threatAssessment(D_nearest, ego_status_velocity)
                object_state                                    = self.RecognitionObject(lane_offset, lane_curvature, object_status_rel_x, object_status_rel_y)
                self.maneuver_mode                                   = self.ManeuverMode(risk, object_state, trafficlight_signal)
                if (ego_heading <= 174 and ego_heading >= 120) or (ego_heading <= -120 and ego_heading >= -174): # abs(ego_heading - old_heading) > 5:
                    self.maneuver_mode = 8
                    time.sleep(0.1)
                print("maneuver_mode : ", self.maneuver_mode,'object state : ',object_state)
                
                self.mode_pub.publish(self.maneuver_mode)
                rate.sleep()

            elif mission_number == 44: # for mission 4 at curve
                if object_status_rel_x_1 > 0:
                    object_status_x = object_status_x_1
                    object_status_y = object_status_y_1
                    object_status_d = object_status_d_1
                    object_status_rel_x = object_status_rel_x_1
                    object_status_rel_y = object_status_rel_y_1
                            
                else:
                    if object_status_rel_x_2 < 0:
                        object_status_x = 0
                        object_status_y = 0
                        object_status_d = 0
                        object_status_rel_x = 0
                        object_status_rel_y = 0  
                    else:
                        object_status_x = object_status_x_2
                        object_status_y = object_status_y_2
                        object_status_d = object_status_d_2
                        object_status_rel_x = object_status_rel_x_2
                        object_status_rel_y = object_status_rel_y_2   
                D_nearest = math.sqrt((object_status_x-ego_status_x)**2+(object_status_y-ego_status_y)**2)
                
                risk,IVT, MSD, RISK_F                           = self.threatAssessment(D_nearest, ego_status_velocity)
                object_state                                    = self.RecognitionObject(lane_offset, lane_curvature_for_curve, object_status_rel_x, object_status_rel_y)
                self.maneuver_mode                                   = self.ManeuverMode(risk, object_state, trafficlight_signal)
                if (ego_heading >= 165) or (ego_heading >= -180 and ego_heading <= -160): # abs(ego_heading - old_heading) > 5:
                    self.maneuver_mode = 8
                    time.sleep(0.1)
                print("maneuver_mode : ", self.maneuver_mode,'object state : ',object_state)
                
                self.mode_pub.publish(self.maneuver_mode)
                rate.sleep()


            elif mission_number == 5: #temp

                D_nearest = math.sqrt((object_status_x_1-ego_status_x)**2+(object_status_y_1-ego_status_y)**2)
                
                risk,IVT, MSD, RISK_F                           = self.threatAssessment(D_nearest, ego_status_velocity)
                object_state                                    = self.RecognitionObject(lane_offset_for_mission_5, lane_curvature, object_status_rel_x, object_status_rel_y)
                self.maneuver_mode                                   = self.ManeuverMode(risk, object_state, trafficlight_signal)
                if self.maneuver_mode == 8 or self.maneuver_mode == 3:
                    self.maneuver_mode = 7
                if risk == 1 and object_state == 0 and object_status_rel_x_1 > 0: # if self.maneuver_mode == 3:
                    self.maneuver_mode = 4

                print('object state : ',object_state,'risk : ',risk)
                self.mode_pub.publish(self.maneuver_mode)
                rate.sleep()


            elif mission_number == 6: # mission 2 (before Rotary) temp
                
                k_object_status_d = object_status_d_1
                if (k_object_status_d - k_1_object_status_d < 0) or (k_object_status_d == k_1_object_status_d and object_status_rel_x_1 > 0) and k_object_status_d<1.5:
                    self.maneuver_mode = 4
                    self.mode_pub.publish(self.maneuver_mode)
                    print("maneuver_mode : ", self.maneuver_mode)
                    rate.sleep()

                
                else:
                    self.maneuver_mode = 7
                    self.mode_pub.publish(self.maneuver_mode)
                    print("maneuver_mode : ", self.maneuver_mode)
                    rate.sleep()    

                
                k_1_object_status_d = object_status_d_1
                # if object_status_d < 1.8 and object_status_d != 0 and object_status_rel_y>0:
                #     self.maneuver_mode = 4
                #     self.mode_pub.publish(self.maneuver_mode)  
                #     print("maneuver_mode : ", self.maneuver_mode) 
                #     rate.sleep()
                # else:
                #     self.maneuver_mode = 2
                #     self.mode_pub.publish(self.maneuver_mode)
                #     print("maneuver_mode : ", self.maneuver_mode)
                #     rate.sleep()                

            if mission_number == 99 :
                self.maneuver_mode = 99
                self.mode_pub.publish(self.maneuver_mode)
                print('back to the 2nd lane')
                rate.sleep()



            isAliveCount_TrafficLight_old = self.isAliveCount_TrafficLight
            

if __name__ == '__main__':

    try:
        rospy.init_node('ManeuverPlanner')
        ManeuverPlanner()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass