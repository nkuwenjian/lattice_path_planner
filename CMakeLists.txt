cmake_minimum_required(VERSION 3.0.2)
project(lattice_path_planner)

set(CMAKE_BUILD_TYPE Release)
message(STATUS "BUILD TYPE: " ${CMAKE_BUILD_TYPE})

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    costmap_2d 
    geometry_msgs 
    nav_core 
    nav_msgs 
    pluginlib 
    roscpp 
    tf2_geometry_msgs 
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/grid_search.cpp 
  src/heap.cpp 
  src/lattice_environment.cpp
  src/lattice_path_planner.cpp
  src/lattice_path_planner_ros.cpp
  src/spline_interpolation.cpp
  src/utils.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})