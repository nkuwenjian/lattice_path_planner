cmake_minimum_required(VERSION 3.0.2)
project(lattice_path_planner)

set(CMAKE_BUILD_TYPE Release)
message(STATUS "BUILD TYPE: " ${CMAKE_BUILD_TYPE})

add_compile_options(-std=c++14)

# Compile ROS package
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall")

# set(CMAKE_CXX_CLANG_TIDY
#   clang-tidy;
#   -header-filter=${PROJECT_SOURCE_DIR}/include/lattice_path_planner;
#   -warnings-as-errors=*;
# )

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

# Glog
find_package(glog REQUIRED)

catkin_package(
  INCLUDE_DIRS include third_party
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
  third_party
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/common/breadth_first_search.cc
  src/common/footprint_helper.cc
  src/common/heap.cc
  src/common/utils.cc
  src/grid_search/grid_search.cc
  src/lattice_a_star/lattice_a_star.cc
  src/primitive_generator/primitive_generator.cc
  src/cubic_spline_interpolation.cpp
  src/lattice_path_planner_ros.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} glog::glog)
