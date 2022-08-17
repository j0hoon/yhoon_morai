cmake_minimum_required(VERSION 2.8.3)
project(maneuver_planner_msg)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  roscpp
  rospy
)


 add_message_files(
   FILES
  maneuver_planner_msg.msg

 )


 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)


