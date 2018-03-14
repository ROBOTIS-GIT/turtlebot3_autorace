cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_autorace_detect)

set(OpenCV_DIR /usr/share/OpenCV/)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  dynamic_reconfigure
  OpenCV
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/DetectLaneParams.cfg
  cfg/DetectLevelParams.cfg
  cfg/DetectTrafficLightParams.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure rospy std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})



