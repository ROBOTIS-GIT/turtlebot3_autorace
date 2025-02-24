cmake_minimum_required(VERSION 3.8)
project(turtlebot3_autorace_detect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

install(PROGRAMS
  src/detect_construction_sign.py
  src/detect_intersection_sign.py
  src/detect_lane.py
  src/detect_level_crossing_sign.py
  src/detect_level_crossing.py
  src/detect_parking_sign.py
  src/detect_traffic_light.py
  src/detect_tunnel_sign.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY image
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  #  DESTINATION share/${PROJECT_NAME}
  #)
endif()

ament_package()
