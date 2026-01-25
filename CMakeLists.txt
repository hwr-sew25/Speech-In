cmake_minimum_required(VERSION 3.0.2)
project(speech_in)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  custom_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
   scripts/speech_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
