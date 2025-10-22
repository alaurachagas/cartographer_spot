cmake_minimum_required(VERSION 3.8)
project(spot_cartographer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pc2_time_normalizer src/pc2_time_normalizer.cpp)
ament_target_dependencies(pc2_time_normalizer rclcpp sensor_msgs)

install(TARGETS pc2_time_normalizer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
