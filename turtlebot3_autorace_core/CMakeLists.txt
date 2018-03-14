cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_autorace_core)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

