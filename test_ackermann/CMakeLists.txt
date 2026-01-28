cmake_minimum_required(VERSION 3.5)
project(test_ackermann)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(test_ackermann src/test_ackermann.cpp)
ament_target_dependencies(test_ackermann
  rclcpp
  ackermann_msgs
  std_msgs
)

install(TARGETS test_ackermann DESTINATION lib/${PROJECT_NAME})

ament_package()
