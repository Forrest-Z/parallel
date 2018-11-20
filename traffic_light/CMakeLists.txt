cmake_minimum_required(VERSION 2.8.3)
project(traffic_light)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  msg_traffic_light.msg
  msg_traffic_light_list.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)
