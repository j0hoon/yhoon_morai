#!/usr/bin/env python3

from re import X
import rospy
from  morai_msgs.msg import ObjectStatusList


class Get_npc_path:

    def __init__(self):
        self.FrameRate = 20
        npc_topic = '/Object_topic'
        rospy.Subscriber(npc_topic, ObjectStatusList, self.callback)

        self.Npc_status = None

        self.main()

    def callback(self, msg):

        for i in msg:
            self.Npc_status = i   

            print(self.Npc_status)
        
    def sub_npc(self, msg):

        npc_pos_x = msg

        return npc_pos_x

    def main(self):
        rate = rospy.Rate(self.FrameRate)
        while not rospy.is_shutdown():
            npc_pos_x = self.sub_npc(self.Npc_status)
            # print(npc_pos_x)
            rate.sleep()

if __name__ == '__main__':

    try:
        rospy.init_node('Get_npc_path')
        
        Get_npc_path()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
 
