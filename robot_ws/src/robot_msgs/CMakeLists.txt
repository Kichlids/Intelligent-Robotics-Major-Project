cmake_minimum_required(VERSION 3.0.2)
project(robot_msgs)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  coordinate.msg
  tasks.msg
)

generate_messages(
  DEPENDENCIES
  robot_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

