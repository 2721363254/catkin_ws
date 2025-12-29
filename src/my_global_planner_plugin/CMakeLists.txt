cmake_minimum_required(VERSION 3.0.2)
project(my_global_planner_plugin)

# 查找catkin和所需组件
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_core
  costmap_2d
  pluginlib
  nav_msgs
  geometry_msgs
  tf
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 声明catkin包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp nav_core costmap_2d pluginlib nav_msgs geometry_msgs tf
)

# 创建插件库
add_library(${PROJECT_NAME}
  src/my_global_planner.cpp
)

# 链接库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# 安装目标
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# 安装插件描述文件
install(FILES my_global_planner.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)