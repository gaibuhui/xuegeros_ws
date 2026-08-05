cmake_minimum_required(VERSION 3.8)
project(scan_timestamp_relay)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(scan_timestamp_relay src/scan_timestamp_relay.cpp)
ament_target_dependencies(scan_timestamp_relay rclcpp sensor_msgs)

install(TARGETS scan_timestamp_relay
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
