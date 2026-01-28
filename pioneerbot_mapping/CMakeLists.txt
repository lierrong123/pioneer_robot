cmake_minimum_required(VERSION 3.8)
project(pioneerbot_mapping)

# 查找ament_cmake，不需要查找launch_ros（它是Python包）
find_package(ament_cmake REQUIRED)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件（可选）
# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()