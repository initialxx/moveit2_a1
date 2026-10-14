cmake_minimum_required(VERSION 3.8)
project(onero_a1_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# 许可证随包安装（rosdistro 发布与下游再分发都需要）
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
