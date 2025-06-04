################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(turtlebot3_autorace_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

add_executable(undocking_node src/undocking.cpp)
target_link_libraries(undocking_node
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
add_executable(alley_mission_node src/alley_mission.cpp)
target_link_libraries(alley_mission_node
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

################################################################################
# Install
################################################################################
install(TARGETS
  alley_mission_node
  undocking_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch param map
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
