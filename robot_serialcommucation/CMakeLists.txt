cmake_minimum_required(VERSION 3.5)
project(robot_serialcommucation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# 设置 CMAKE_PREFIX_PATH 以确保找到所有依赖
list(APPEND CMAKE_PREFIX_PATH
  /home/lierrong/serial_ros2/install
  /home/lierrong/lierrong_ws/install
  /opt/ros/foxy
)

# 查找所有需要的包
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)  # 关键：必须在 ackermann_msgs 之前
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(wheeltec_robot_msg REQUIRED)

# 特别注意：serial 包可能使用不同的名称
find_package(serial REQUIRED)

# 如果找不到 serial，尝试手动设置
if(NOT serial_FOUND)
  message(STATUS "Serial package not found via find_package, trying manual setup")
  
  # 检查 serial 头文件路径
  set(SERIAL_INCLUDE_DIRS 
    /home/lierrong/serial_ros2/install/serial/include
    /opt/ros/foxy/include
  )
  
  # 检查 serial 库路径
  set(SERIAL_LIBRARIES
    /home/lierrong/serial_ros2/install/serial/lib/libserial.so
    /opt/ros/foxy/lib/libserial.so
  )
  
  # 查找头文件
  find_path(SERIAL_INCLUDE_DIR serial/serial.h
    PATHS ${SERIAL_INCLUDE_DIRS}
    NO_DEFAULT_PATH
  )
  
  # 查找库文件
  find_library(SERIAL_LIB serial
    PATHS ${SERIAL_LIBRARIES}
    NO_DEFAULT_PATH
  )
  
  if(SERIAL_INCLUDE_DIR AND SERIAL_LIB)
    message(STATUS "Found serial manually: include=${SERIAL_INCLUDE_DIR}, lib=${SERIAL_LIB}")
    set(serial_FOUND TRUE)
  else()
    message(FATAL_ERROR "Could not find serial library")
  endif()
endif()

add_executable(robot_serialcommucation
  src/wheeltec_robot.cpp
  src/Quaternion_Solution.cpp
)
# 在 target_link_libraries 后添加
target_link_libraries(robot_serialcommucation
  # 显式链接 ackermann_msgs 的库
  ackermann_msgs::ackermann_msgs__rosidl_typesupport_cpp
)
# 添加头文件包含路径
target_include_directories(robot_serialcommucation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 添加 serial 头文件路径
if(SERIAL_INCLUDE_DIR)
  target_include_directories(robot_serialcommucation PUBLIC ${SERIAL_INCLUDE_DIR})
elseif(serial_INCLUDE_DIRS)
  target_include_directories(robot_serialcommucation PUBLIC ${serial_INCLUDE_DIRS})
endif()

# 使用 ament_target_dependencies 添加 ROS 依赖
ament_target_dependencies(robot_serialcommucation
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  ackermann_msgs
  wheeltec_robot_msg
)

# 链接 serial 库
if(SERIAL_LIB)
  target_link_libraries(robot_serialcommucation ${SERIAL_LIB})
elseif(serial_LIBRARIES)
  target_link_libraries(robot_serialcommucation ${serial_LIBRARIES})
elseif(TARGET serial::serial)
  target_link_libraries(robot_serialcommucation serial::serial)
endif()

# 确保 ackermann_msgs 正确链接
# 使用正确的方式查找库
find_library(ACKERMANN_TS_CPP_LIB ackermann_msgs__rosidl_typesupport_cpp
  PATHS /home/lierrong/lierrong_ws/install/ackermann_msgs/lib
  NO_DEFAULT_PATH
)

if(ACKERMANN_TS_CPP_LIB)
  message(STATUS "Found ackermann_msgs typesupport cpp library: ${ACKERMANN_TS_CPP_LIB}")
  target_link_libraries(robot_serialcommucation ${ACKERMANN_TS_CPP_LIB})
else()
  # 如果找不到，尝试其他可能的库名
  find_library(ACKERMANN_ANY_LIB
    NAMES 
      ackermann_msgs__rosidl_typesupport_cpp
      ackermann_msgs__rosidl_typesupport_c
      ackermann_msgs__rosidl_generator_c
    PATHS /home/lierrong/lierrong_ws/install/ackermann_msgs/lib
    NO_DEFAULT_PATH
  )
  if(ACKERMANN_ANY_LIB)
    message(STATUS "Found ackermann_msgs library: ${ACKERMANN_ANY_LIB}")
    target_link_libraries(robot_serialcommucation ${ACKERMANN_ANY_LIB})
  else()
    message(WARNING "Could not find ackermann_msgs library")
  endif()
endif()

# 安装目标
install(TARGETS
  robot_serialcommucation
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 如果还有配置文件，也可以安装
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  ackermann_msgs
  wheeltec_robot_msg
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
