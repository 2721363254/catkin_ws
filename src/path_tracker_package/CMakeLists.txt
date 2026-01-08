cmake_minimum_required(VERSION 3.0.2)
project(path_tracker_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf2
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_tracker
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs tf2
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_tracker_node src/path_tracker_node.cpp)
target_link_libraries(path_tracker_node ${catkin_LIBRARIES})
