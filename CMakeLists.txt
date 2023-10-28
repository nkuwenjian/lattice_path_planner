cmake_minimum_required(VERSION 3.0.2)
project(lattice_path_planner)

set(CMAKE_BUILD_TYPE Release)
message(STATUS "BUILD TYPE: " ${CMAKE_BUILD_TYPE})

add_compile_options(-std=c++14)

# Compile ROS package
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall")

# set(CMAKE_CXX_CLANG_TIDY
#   clang-tidy;
#   -header-filter=${PROJECT_SOURCE_DIR}/lattice_path_planner;
#   -warnings-as-errors=*;
# )

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  tf2_geometry_msgs
  tf2_ros
)

# Glog
find_package(glog REQUIRED)

catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}
  LIBRARIES lattice_path_planner
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    tf2_geometry_msgs
    tf2_ros
)

include_directories(
  ${PROJECT_SOURCE_DIR}
  third_party
  ${catkin_INCLUDE_DIRS}
)

add_library(lattice_path_planner
  lattice_path_planner/common/breadth_first_search.cc
  lattice_path_planner/common/footprint_helper.cc
  lattice_path_planner/common/heap.cc
  lattice_path_planner/common/utils.cc
  lattice_path_planner/grid_search/grid_search.cc
  lattice_path_planner/lattice_a_star/lattice_a_star.cc
  lattice_path_planner/primitive_generator/primitive_generator.cc
  lattice_path_planner/cubic_spline_interpolation.cc
  lattice_path_planner/lattice_path_planner_ros.cc
)
target_link_libraries(lattice_path_planner ${catkin_LIBRARIES} glog::glog)

add_executable(lattice_path_planner_test lattice_path_planner/lattice_path_planner_ros_test.cc)
target_link_libraries(lattice_path_planner_test lattice_path_planner ${catkin_LIBRARIES})

add_executable(lattice_path_planner_tf_broadcaster lattice_path_planner/tf_broadcaster.cc)
target_link_libraries(lattice_path_planner_tf_broadcaster ${catkin_LIBRARIES} glog::glog)
