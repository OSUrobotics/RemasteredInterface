cmake_minimum_required(VERSION 3.0.2)
project(RemasteredInterface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  visualization_msgs
  infrastructure_msgs
  message_generation
  rosbag
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs visualization_msgs rosbag
)

include_directories(
  $(catkin_INCLUDE_DIRS)
  $(Boost_INCLUDE_DIRS)
)
