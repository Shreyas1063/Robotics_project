cmake_minimum_required(VERSION 3.8)
project(ur5_ibvs)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# OpenCV — Ubuntu 22.04 puts headers under /usr/include/opencv4
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV includes: ${OpenCV_INCLUDE_DIRS}")

# ViSP — exact cmake path confirmed from find command
set(VISP_DIR /opt/ros/humble/lib/x86_64-linux-gnu/cmake/visp)
find_package(VISP REQUIRED)
message(STATUS "ViSP version: ${VISP_VERSION}")
message(STATUS "ViSP includes: ${VISP_INCLUDE_DIRS}")

add_executable(ibvs_controller src/ibvs_controller.cpp)

target_include_directories(ibvs_controller
  BEFORE PUBLIC
    /opt/ros/humble/include          # ROS ViSP 3.7 headers (visp:: namespace)
    /usr/include/opencv4             # OpenCV 4.x headers on Ubuntu 22.04
    ${VISP_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(ibvs_controller
  ${VISP_LIBRARIES}
  ${OpenCV_LIBS}
)

ament_target_dependencies(ibvs_controller
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  trajectory_msgs
)

install(TARGETS ibvs_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY urdf launch models config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()