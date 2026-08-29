cmake_minimum_required(VERSION 3.0.2)
project(rig_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  RadarDebug.msg
  GraphDebug.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs)
