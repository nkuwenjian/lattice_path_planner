/******************************************************************************
 * Copyright (c) 2023, NKU Mobile & Flying Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Jian Wen (nkuwenjian@gmail.com)
 *****************************************************************************/

#include "lattice_path_planner/lattice_path_planner_ros.h"

#include <utility>

#include "costmap_2d/inflation_layer.h"
#include "nav_msgs/Path.h"
#include "pluginlib/class_list_macros.hpp"
#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"

PLUGINLIB_EXPORT_CLASS(lattice_path_planner::LatticePathPlannerROS,
                       nav_core::BaseGlobalPlanner)

namespace lattice_path_planner {

void LatticePathPlannerROS::initialize(std::string name,
                                       costmap_2d::Costmap2DROS* costmap_ros) {
  // Check if LatticePathPlannerROS has been initialized.
  if (initialized_) {
    LOG(INFO) << "LatticePathPlannerROS has been initialized.";
    return;
  }

  // Check and update costmap.
  if (!UpdateCostmap(costmap_ros)) {
    LOG(ERROR) << "Failed to update costmap.";
    return;
  }

  // Get parameters through ROS parameter server.
  ros::NodeHandle private_nh("~/" + name);
  VLOG(4) << "Name is " << name;
  name_ = name;
  double nominalvel_mpersecs = 0.0;
  double timetoturn45degsinplace_secs = 0.0;
  double vehicle_length = 0.0;
  double vehicle_width = 0.0;
  if (!LoadRosParamFromNodeHandle(private_nh, &nominalvel_mpersecs,
                                  &timetoturn45degsinplace_secs,
                                  &vehicle_length, &vehicle_width)) {
    LOG(ERROR) << "Failed to load ROS parameters.";
    return;
  }

  // Update robot footprint.
  std::vector<common::XYPoint> footprint =
      GetFootprint(costmap_ros_->getRobotFootprint());

  uint8_t cost_possibly_circumscribed_thresh = ComputeCircumscribedCost();
  if (cost_possibly_circumscribed_thresh == 0U) {
    // Unfortunately, the inflation_radius is not taken into account by
    // inflation_layer->computeCost(). If inflation_radius is smaller than
    // the circumscribed radius, SBPL will ignore some obstacles, but we
    // cannot detect this problem. If the cost_scaling_factor is too large,
    // SBPL won't run into obstacles, but will always perform an expensive
    // footprint check, no matter how far the nearest obstacle is.
    LOG(ERROR) << std::fixed
               << "The costmap value at the robot's circumscribed radius ("
               << layered_costmap_->getCircumscribedRadius() << " m) is 0.";
    return;
  }

  // Create and initialize LatticePathPlannerROS.
  planner_ = std::make_unique<lattice_a_star::LatticeAStar>();
  planner_->Init(static_cast<int>(costmap_2d_->getSizeInCellsX()),
                 static_cast<int>(costmap_2d_->getSizeInCellsY()),
                 costmap_2d_->getResolution(), costmap_2d::LETHAL_OBSTACLE,
                 costmap_2d::INSCRIBED_INFLATED_OBSTACLE,
                 cost_possibly_circumscribed_thresh, nominalvel_mpersecs,
                 timetoturn45degsinplace_secs, footprint,
                 const_cast<char*>(primitive_filename_.c_str()));

  visualizer_ = std::make_unique<PathVisualizer>();
  visualizer_->Initialize(name, vehicle_length, vehicle_width);

  initialized_ = true;
  LOG(INFO) << "LatticePathPlannerROS is initialized successfully.";
}

bool LatticePathPlannerROS::UpdateCostmap(
    costmap_2d::Costmap2DROS* costmap_ros) {
  if (costmap_ros == nullptr) {
    LOG(ERROR) << "costmap_ros == nullptr";
    return false;
  }

  costmap_ros_ = costmap_ros;
  costmap_2d_ = costmap_ros->getCostmap();
  layered_costmap_ = costmap_ros->getLayeredCostmap();
  if (costmap_2d_ == nullptr || layered_costmap_ == nullptr) {
    LOG(ERROR) << "costmap_2d_ == nullptr || layered_costmap_ == nullptr";
    return false;
  }
  return true;
}

bool LatticePathPlannerROS::LoadRosParamFromNodeHandle(
    const ros::NodeHandle& nh, double* nominalvel_mpersecs,
    double* timetoturn45degsinplace_secs, double* vehicle_length,
    double* vehicle_width) {
  // Sanity checks.
  CHECK_NOTNULL(nominalvel_mpersecs);
  CHECK_NOTNULL(timetoturn45degsinplace_secs);
  CHECK_NOTNULL(costmap_2d_);

  nh.param("nominalvel_mpersecs", *nominalvel_mpersecs, 0.4);
  nh.param("timetoturn45degsinplace_secs", *timetoturn45degsinplace_secs, 0.6);
  nh.param("treat_unknown_as_free", treat_unknown_as_free_, true);
  if (!nh.getParam("vehicle_length", *vehicle_length)) {
    LOG(ERROR) << "Failed to set vehicle_length";
    return false;
  }
  if (!nh.getParam("vehicle_width", *vehicle_width)) {
    LOG(ERROR) << "Failed to set vehicle_width";
    return false;
  }
  if (!nh.getParam("primitive_filename", primitive_filename_)) {
    LOG(ERROR) << "Failed to set primitive_filename";
    return false;
  }

  VLOG(4) << std::fixed << "nominalvel_mpersecs: " << *nominalvel_mpersecs;
  VLOG(4) << std::fixed
          << "timetoturn45degsinplace_secs: " << *timetoturn45degsinplace_secs;
  VLOG(4) << std::boolalpha
          << "treat_unknown_as_free: " << treat_unknown_as_free_;
  VLOG(4) << std::fixed << "vehicle_length: " << *vehicle_length;
  VLOG(4) << std::fixed << "vehicle_width: " << *vehicle_width;
  VLOG(4) << "primitive_filename: " << primitive_filename_;
  return true;
}

uint8_t LatticePathPlannerROS::ComputeCircumscribedCost() const {
  // Sanity checks.
  CHECK_NOTNULL(layered_costmap_);
  CHECK_NOTNULL(costmap_2d_);

  std::vector<boost::shared_ptr<costmap_2d::Layer>>* plugins =
      layered_costmap_->getPlugins();
  if (plugins == nullptr) {
    LOG(ERROR) << "plugins == nullptr";
    return 0U;
  }

  // Check if the costmap has an inflation layer.
  for (auto& plugin : *plugins) {
    auto inflation_layer =
        boost::dynamic_pointer_cast<costmap_2d::InflationLayer>(plugin);
    if (inflation_layer != nullptr) {
      return inflation_layer->computeCost(
          layered_costmap_->getCircumscribedRadius() /
          costmap_2d_->getResolution());
    }
  }
  return 0U;
}

std::vector<common::XYPoint> LatticePathPlannerROS::GetFootprint(
    const std::vector<geometry_msgs::Point>& robot_footprint) {
  std::vector<common::XYPoint> footprint;
  footprint.reserve(robot_footprint.size());
  for (const geometry_msgs::Point& point : robot_footprint) {
    footprint.emplace_back(point.x, point.y);
  }
  return footprint;
}

void LatticePathPlannerROS::GetStartAndEndConfigurations(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, double origin_x, double origin_y,
    double* start_x, double* start_y, double* start_phi, double* end_x,
    double* end_y, double* end_phi) {
  // Sanity checks.
  CHECK_NOTNULL(start_x);
  CHECK_NOTNULL(start_y);
  CHECK_NOTNULL(start_phi);
  CHECK_NOTNULL(end_x);
  CHECK_NOTNULL(end_y);
  CHECK_NOTNULL(end_phi);

  VLOG(4) << std::fixed << "start point: " << start.pose.position.x << ","
          << start.pose.position.y;
  VLOG(4) << std::fixed << "end point: " << goal.pose.position.x << ","
          << goal.pose.position.y;

  // Start configuration.
  *start_x = start.pose.position.x - origin_x;
  *start_y = start.pose.position.y - origin_y;
  *start_phi = tf::getYaw(start.pose.orientation);

  // End configuration.
  *end_x = goal.pose.position.x - origin_x;
  *end_y = goal.pose.position.y - origin_y;
  *end_phi = tf::getYaw(goal.pose.orientation);
}

std::vector<std::vector<uint8_t>> LatticePathPlannerROS::GetGridMap(
    const uint8_t* char_map, unsigned int size_x, unsigned int size_y,
    bool treat_unknown_as_free) {
  if (char_map == nullptr) {
    LOG(ERROR) << "char_map == nullptr";
    return std::vector<std::vector<uint8_t>>();
  }
  std::vector<std::vector<uint8_t>> grid_map;
  grid_map.resize(size_x);
  for (unsigned int i = 0U; i < size_x; ++i) {
    grid_map[i].resize(size_y);
    for (unsigned int j = 0U; j < size_y; ++j) {
      grid_map[i][j] = char_map[i + j * size_x];
      if (treat_unknown_as_free &&
          grid_map[i][j] == costmap_2d::NO_INFORMATION) {
        grid_map[i][j] = costmap_2d::FREE_SPACE;
      }
    }
  }
  return grid_map;
}

bool LatticePathPlannerROS::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan) {
  // Check if LatticePathPlannerROS has been initialized.
  if (!initialized_) {
    LOG(ERROR) << "LatticePathPlannerROS has not been initialized.";
    return false;
  }

  // Get start and end configurations.
  double start_x;
  double start_y;
  double start_phi;
  double end_x;
  double end_y;
  double end_phi;
  GetStartAndEndConfigurations(start, goal, costmap_2d_->getOriginX(),
                               costmap_2d_->getOriginY(), &start_x, &start_y,
                               &start_phi, &end_x, &end_y, &end_phi);

  // Get map data from cost map.
  std::vector<std::vector<uint8_t>> grid_map =
      GetGridMap(costmap_2d_->getCharMap(), costmap_2d_->getSizeInCellsX(),
                 costmap_2d_->getSizeInCellsY(), treat_unknown_as_free_);
  if (grid_map.empty()) {
    LOG(ERROR) << "Failed to get grid map.";
    return false;
  }

  // Run lattice A star planner.
  VLOG(4) << "Run lattice A-star planner.";
  lattice_a_star::LatticeAStarResult result;
  if (!planner_->Plan(start_x, start_y, start_phi, end_x, end_y, end_phi,
                      std::move(grid_map), &result)) {
    return false;
  }

  // Populate global plan.
  PopulateGlobalPlan(result, start.header, costmap_2d_->getOriginX(),
                     costmap_2d_->getOriginY(), &plan);

  visualizer_->Visualize(plan);

  return true;
}

void LatticePathPlannerROS::PopulateGlobalPlan(
    const lattice_a_star::LatticeAStarResult& searched_path,
    const std_msgs::Header& header, double origin_x, double origin_y,
    std::vector<geometry_msgs::PoseStamped>* plan) {
  // Sanity checks.
  CHECK_NOTNULL(plan);
  CHECK_EQ(searched_path.x.size(), searched_path.y.size());
  CHECK_EQ(searched_path.x.size(), searched_path.phi.size());
  const std::size_t N = searched_path.x.size();

  plan->clear();
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = header;

  for (std::size_t i = 0U; i < N; ++i) {
    pose_stamped.pose.position.x = searched_path.x[i] + origin_x;
    pose_stamped.pose.position.y = searched_path.y[i] + origin_y;
    pose_stamped.pose.orientation =
        tf::createQuaternionMsgFromYaw(searched_path.phi[i]);
    plan->push_back(pose_stamped);
  }
}

}  // namespace lattice_path_planner
