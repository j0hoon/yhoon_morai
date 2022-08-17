from decimal import DivisionByZero
import numpy as np
import time
import math
import rospy
from std_msgs.msg import Int32
from maneuver_planner_msg.msg import maneuver_planner_msg, LC_complete_msg
#from darknet_ros_msgs.msg import BoundingBoxes
#from ypstruct import struct
from morai_msgs.msg import GetTrafficLightStatus, EgoVehicleStatus, ObjectStatusList # 신호등 신호

# 내가 받아야할 data 목록
# 신호등 data, lane 값, object 값 

########### this is parameter before getting lane value
lane_curvature = 0.01
lane_offset = 0.35


class ManeuverPlanner():
    def __init__(self):
        
        # Publish
        self.mode_pub = rospy.Publisher('/mode', maneuver_planner_msg , queue_size=1)
        # subscriber
        self.sub_ego = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.callback_ego)
        self.sub_trafficlight = rospy.Subscriber('/GetTrafficLightStatus', GetTrafficLightStatus, self.callback_traffic)
        self.sub_lccomplete = rospy.Subscriber('/lc_complete', LC_complete_msg, self.callback_lc_complete)
        # rospy.Subscriber()
        self.sub_Object = rospy.Subscriber('/Object_topic', ObjectStatusList, self.callback_object)
        self.FrameRate = 10
        # self.Traffic_signal = None
        self.isAlive = 0
        self.Ego_status = None
        self.lc_complete = None
        self.isAliveCount_TrafficLight = None
        self.distance_index=0
        # self.lane_offset = lane_offsetdeceleration mode
        # self.lane_curvature = lane_curvature

        while not rospy.is_shutdown():
            if self.Ego_status:
                break
            else:
                print('Waiting for Ego status msg')
                time.sleep(0.5)

            if self.lc_complete:
                break
            else:
                print('Waiting for lc complete msg')
                time.sleep(0.5)

        self.main()
    
    ############# sub topic ############

    def sub_egostatus(self, msg):
        
        ego_status_velocity = math.sqrt(msg.velocity.x ** 2 + msg.velocity.y ** 2 + msg.velocity.z ** 2)
        ego_status_x        = msg.position.x
        ego_status_y        = msg.position.y

        return ego_status_velocity, ego_status_x, ego_status_y

    def sub_Objectstatus(self, msg):

        object_class        = msg.obstacle_list[0].name
        object_status_x     = msg.obstacle_list[0].position.x
        object_status_y     = msg.obstacle_list[0].position.y

        object_class_2       = msg.obstacle_list[1].name
        object_status_x_2     = msg.obstacle_list[1].position.x
        object_status_y_2     = msg.obstacle_list[1].position.y

        return object_class, object_status_x, object_status_y,object_class_2, object_status_x_2, object_status_y_2

    def sub_trafficsign(self, msg):


        trafficlight_signal = msg.trafficLightStatus

        return trafficlight_signal

    def sub_lc_complete(self, msg):

        LC_complete = msg

        return LC_complete
    ############ callback ############

    def callback_ego(self, msg):
        
        self.Ego_status = msg

    def callback_traffic(self, msg):
        
        self.Traffic_signal = msg.trafficLightStatus
        self.isAliveCount_TrafficLight = msg.header.seq

    def callback_object(self, msg):

        self.Object_status = msg

    def callback_lc_complete(self, msg):

        self.lc_complete = msg
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
        
        return risk
        
        
    def RecognitionObject(self, lane_offset, lane_curvature, object_status_x, ego_status_x, object_status_y, ego_status_y):

        epsilon     = lane_offset # 0.5
        rel_x       = object_status_x - ego_status_x
        rel_y       = object_status_y - ego_status_y
        lane_radius = 1 / lane_curvature

        p_f = lane_radius - math.sqrt(rel_x ** 2 + (rel_y - lane_radius) ** 2)

        if p_f < - epsilon :
            object_state = -1 # FVL

        elif p_f > epsilon :
            object_state = 1 # FVR

        else :
            object_state = 0 # FVI
        
        return object_state

    def ManeuverMode(self, risk, object_state, traffic_signal):

        # 곡률에 따른 모드 넣으면 좋을듯

        if risk == 0 and object_state != 0 and traffic_signal == 1 : # 위험물체 없고 내차선에 아무것도 없음
            maneuver_mode = 4 # stop mode 

        elif risk == 1 and object_state != 0 and traffic_signal != 1: # 위험물체 잡혔지만 내차선에는 없고 빨간불 x 
            maneuver_mode = 1 # driving mode

        elif risk == 1 and object_state == 0 and traffic_signal == 1: 
            maneuver_mode = 4 # stop mode

        elif risk == 0 and object_state == 0 and traffic_signal == 1 : 
            maneuver_mode = 4 # stop mode

        elif risk == 1 and object_state == 0: # 위험객체 있고 내 차선에 
            maneuver_mode = 3 # lane change mode

        elif risk == 0 and object_state == 0: # 위험객체 없지만 내 차선에 객체 발견 빨간불x
            maneuver_mode = 2 # deceleration mode
        
        elif risk == 0 and object_state != 0:
            maneuver_mode = 1 # driving mode

        elif risk == 1 and object_state != 0 and traffic_signal == 1 : # 위험객체 있지만 내 차선에 객체 없고 빨간불 o
            maneuver_mode = 4 # stop mode

        return maneuver_mode

    def main(self):

        rate = rospy.Rate(self.FrameRate)
        isAliveCount_TrafficLight_old = None
        while not rospy.is_shutdown():

            ego_status_velocity, ego_status_x, ego_status_y = self.sub_egostatus(self.Ego_status)
            object_class, object_status_x, object_status_y,object_class_2, object_status_x_2, object_status_y_2  = self.sub_Objectstatus(self.Object_status)
            LC_complete = self.sub_lc_complete(self.lc_complete)
            # trafficlight_signal                             = self.sub_trafficsign(self.Traffic_signal)

            if (self.isAliveCount_TrafficLight is not None) and (self.isAliveCount_TrafficLight != isAliveCount_TrafficLight_old):

                trafficlight_signal = self.Traffic_signal
                
            else:

                trafficlight_signal = 0

            print("Traffic light status :", trafficlight_signal)
            Distance_List = []
            D_nearest_1 = math.sqrt((object_status_x-ego_status_x)**2+(object_status_y-ego_status_y)**2)
            Distance_List.append(D_nearest_1)
            D_nearest_2 = math.sqrt((object_status_x_2-ego_status_x)**2+(object_status_y_2-ego_status_y)**2)
            Distance_List.append(D_nearest_2)
            
            #Distance_index = np.argmin(Distance_List)

            D_nearest = Distance_List[self.distance_index]

            risk                                            = self.threatAssessment(D_nearest, ego_status_velocity)
            object_state                                    = self.RecognitionObject(lane_offset, lane_curvature, object_status_x, ego_status_x, object_status_y, ego_status_y)
            maneuver_mode                                   = self.ManeuverMode(risk, object_state, trafficlight_signal)

            if maneuver_mode == 3:
                Lane_changing = 1
                
                while Lane_changing:
                    rate.sleep()
                    self.mode_pub.publish(maneuver_mode)
                    print("lane changing mode")
                    LC_complete = self.sub_lc_complete(self.lc_complete)
                    if LC_complete == 1:
                        self.distance_index = 1
                        maneuver_mode = 1
                        time.sleep(1000)
                        break
                    break

            # print(f'trafficlight_signal = {trafficlight_signal}')
            # print(f'D_nearest = {D_nearest}')
            # print(f'risk = {risk}')
            # print(f'object_state = {object_state}')
            self.mode_pub.publish(maneuver_mode)
            print(f'maneuver_mode = {maneuver_mode}')
            print('risk : ',risk, 'object_state : ',object_state, 'D_nearest : ',D_nearest)

            isAliveCount_TrafficLight_old = self.isAliveCount_TrafficLight

            rate.sleep()

if __name__ == '__main__':

    try:
        rospy.init_node('ManeuverPlanner')
        ManeuverPlanner()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass



    



    

        

        

# D_nearest = 10
# V_ego = 20
# loc_ego = struct()
# loc_ego.x = 0
# loc_ego.y = 0
# loc_obj = struct()
# loc_obj.x = 3
# loc_obj.y = 4
# lane_offset = 1.5
# lane_curvature = 0.8
# a = ManeuverPlanner(V_ego, D_nearest, loc_ego, loc_obj, lane_offset, lane_curvature)
# print(a.ManeuverMode())