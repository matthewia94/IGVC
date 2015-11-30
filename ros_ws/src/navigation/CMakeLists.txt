cmake_minimum_required(VERSION 2.8.3)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES navigation
  CATKIN_DEPENDS rospy std_msgs
  DEPENDS system_lib
)
