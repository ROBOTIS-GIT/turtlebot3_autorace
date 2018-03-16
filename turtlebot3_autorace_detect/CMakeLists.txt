################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_autorace_detect)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  move_base_msgs
  tf
  dynamic_reconfigure
  cv_bridge
)

find_package(OpenCV REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################
generate_dynamic_reconfigure_options(
  cfg/DetectLaneParams.cfg
  cfg/DetectLevelParams.cfg
  cfg/DetectTrafficLightParams.cfg
)

################################################################################
# Declare things to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs nav_msgs move_base_msgs tf dynamic_reconfigure cv_bridge 
  DEPENDS OpenCV
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS
  scripts/detect_lane.py
  scripts/detect_level.py
  scripts/detect_obstacle.py
  scripts/detect_parking.py
  scripts/detect_sign.py
  scripts/detect_traffic_light.py
  scripts/detect_tunnel.py
 	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY file launch param
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
