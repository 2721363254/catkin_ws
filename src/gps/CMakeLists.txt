cmake_minimum_required(VERSION 2.8.3)
project(gps)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 查找GeographicLib
find_package(PkgConfig REQUIRED)
pkg_check_modules(GEOGRAPHIC REQUIRED geographiclib)

# 包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GEOGRAPHIC_INCLUDE_DIRS}
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/gps)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/gps)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/gps)

# 创建节点
add_executable(gps_converter_node src/gps_converter.cpp)
target_link_libraries(gps_converter_node
  ${catkin_LIBRARIES}
  ${GEOGRAPHIC_LIBRARIES}
)

# 安装规则
install(TARGETS gps_converter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)