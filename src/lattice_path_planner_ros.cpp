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

#include "costmap_2d/inflation_layer.h"
#include "nav_msgs/Path.h"
#include "pluginlib/class_list_macros.hpp"
#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"

#include "lattice_path_planner/cubic_spline_interpolation.h"

PLUGINLIB_EXPORT_CLASS(lattice_path_planner::LatticePathPlannerROS,
                       nav_core::BaseGlobalPlanner)

namespace lattice_path_planner {

LatticePathPlannerROS::LatticePathPlannerROS(
    std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(name, costmap_ros);
}

void LatticePathPlannerROS::initialize(std::string name,
                                       costmap_2d::Costmap2DROS* costmap_ros) {
  // Check whether lattice path planner has been initialized.
  if (initialized_) {
    LOG(INFO) << "Lattice path planner has been initialized.";
    return;
  }

  // Check and update costmap.
  if (!UpdateCostmap(costmap_ros)) {
    LOG(ERROR) << "Failed to update costmap.";
    return;
  }

  // Update ROS parameters.
  ros::NodeHandle private_nh("~/" + name);
  VLOG(4) << "Name is " << name;
  name_ = name;
  double nominalvel_mpersecs = 0.0;
  double timetoturn45degsinplace_secs = 0.0;
  if (!GetRosParameters(private_nh, &nominalvel_mpersecs,
                        &timetoturn45degsinplace_secs)) {
    LOG(ERROR) << "Failed to get ROS parameters.";
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
    LOG(WARNING) << std::fixed
                 << "The costmap value at the robot's circumscribed radius ("
                 << layered_costmap_->getCircumscribedRadius() << " m) is 0.";
    LOG(WARNING) << "SBPL performance will suffer.";
    LOG(WARNING) << "Please decrease the costmap's cost_scaling_factor.";
  }

  // Create and initialize lattice path planner.
  if (planner_ != nullptr) {
    planner_.reset();
  }
  planner_ = std::make_unique<lattice_a_star::LatticeAStar>();
  planner_->Init(
      costmap_2d_->getSizeInCellsX(), costmap_2d_->getSizeInCellsY(),
      costmap_2d::LETHAL_OBSTACLE, costmap_2d::INSCRIBED_INFLATED_OBSTACLE,
      cost_possibly_circumscribed_thresh, costmap_2d_->getResolution(),
      nominalvel_mpersecs, timetoturn45degsinplace_secs, footprint,
      const_cast<char*>(primitive_filename_.c_str()));

  LOG(INFO) << "Lattice path planner is initialized successfully.";
  plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
  initialized_ = true;
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

bool LatticePathPlannerROS::GetRosParameters(
    const ros::NodeHandle& nh, double* nominalvel_mpersecs,
    double* timetoturn45degsinplace_secs) {
  // Sanity checks.
  CHECK_NOTNULL(nominalvel_mpersecs);
  CHECK_NOTNULL(timetoturn45degsinplace_secs);

  nh.param("nominalvel_mpersecs", *nominalvel_mpersecs, 0.4);
  nh.param("timetoturn45degsinplace_secs", *timetoturn45degsinplace_secs, 0.6);
  nh.param("sample_step_size_m", sample_step_size_m_,
           costmap_2d_->getResolution());
  nh.param("treat_unknown_as_free", treat_unknown_as_free_, true);
  if (!nh.getParam("primitive_filename", primitive_filename_)) {
    LOG(ERROR) << "Failed to set primitive_filename";
    return false;
  }

  VLOG(4) << std::fixed << "nominalvel_mpersecs: " << *nominalvel_mpersecs;
  VLOG(4) << std::fixed
          << "timetoturn45degsinplace_secs: " << *timetoturn45degsinplace_secs;
  VLOG(4) << std::fixed << "sample_step_size_m: " << sample_step_size_m_;
  VLOG(4) << std::boolalpha
          << "treat_unknown_as_free: " << treat_unknown_as_free_;
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

  // check if the costmap has an inflation layer
  for (auto layer = plugins->begin(); layer != plugins->end(); ++layer) {
    auto inflation_layer =
        boost::dynamic_pointer_cast<costmap_2d::InflationLayer>(*layer);
    if (inflation_layer == nullptr) {
      continue;
    }

    return inflation_layer->computeCost(
        layered_costmap_->getCircumscribedRadius() /
        costmap_2d_->getResolution());
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

  VLOG(4) << std::fixed << "Start point: " << start.pose.position.x << ","
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
    const uint8_t* char_map, size_t size_x, size_t size_y,
    bool treat_unknown_as_free) {
  if (char_map == nullptr) {
    LOG(ERROR) << "char_map == nullptr";
    return std::vector<std::vector<uint8_t>>();
  }
  std::vector<std::vector<uint8_t>> grid_map;
  grid_map.resize(size_x);
  for (size_t i = 0U; i < size_x; ++i) {
    grid_map[i].resize(size_y);
    for (size_t j = 0U; j < size_y; ++j) {
      grid_map[i][j] = char_map[i + j * size_x];
      if (treat_unknown_as_free &&
          grid_map[i][j] == costmap_2d::NO_INFORMATION) {
        grid_map[i][j] = costmap_2d::FREE_SPACE;
      }
    }
  }
  return grid_map;
}

std::vector<common::XYPoint> LatticePathPlannerROS::InterpolateLatticeAStarPath(
    const lattice_a_star::LatticeAStarResult& result,
    double sample_step_size_m) {
  // Sanity checks.
  CHECK_EQ(result.x.size(), result.y.size());
  CHECK_EQ(result.x.size(), result.phi.size());
  const size_t N = result.x.size();

  // Sample raw path along lattice path.
  std::vector<common::XYPoint> raw_path;
  raw_path.emplace_back(result.x.front(), result.y.front());
  double last_x = result.x.front();
  double last_y = result.y.front();

  for (size_t i = 1U; i < N; ++i) {
    double dx = result.x[i] - last_x;
    double dy = result.y[i] - last_y;
    if (std::hypot(dx, dy) > sample_step_size_m) {
      raw_path.emplace_back(result.x[i], result.y[i]);
      last_x = result.x[i];
      last_y = result.y[i];
    }
  }
  raw_path.pop_back();
  raw_path.emplace_back(result.x.back(), result.y.back());

  // Smooth the raw path via cubic spline interpolation.
  std::vector<common::XYPoint> interpolated_path;
  if (raw_path.size() > 2U) {
    CubicSplineInterpolation::Interpolate(raw_path, &interpolated_path);
  } else {
    interpolated_path = raw_path;
  }
  return interpolated_path;
}

bool LatticePathPlannerROS::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!initialized_) {
    LOG(ERROR) << "Lattice path planner has not been initialized.";
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

  // Interpolate raw lattice A star result to get dense path points.
  std::vector<common::XYPoint> interpolated_path =
      InterpolateLatticeAStarPath(result, sample_step_size_m_);

  // Populate global plan.
  PopulateGlobalPlan(interpolated_path, start, costmap_2d_->getOriginX(),
                     costmap_2d_->getOriginY(), &plan);

  // Publish global plan.
  PublishGlobalPlan(plan);

  return true;
}

void LatticePathPlannerROS::PopulateGlobalPlan(
    const std::vector<common::XYPoint>& interpolated_path,
    const geometry_msgs::PoseStamped& start, double origin_x, double origin_y,
    std::vector<geometry_msgs::PoseStamped>* plan) {
  // Sanity checks.
  CHECK_NOTNULL(plan);

  plan->clear();
  geometry_msgs::PoseStamped pose;
  pose.header = start.header;

  if (interpolated_path.size() < 2U) {
    pose.pose = start.pose;
    plan->push_back(pose);
    return;
  }

  for (size_t i = 0U; i < interpolated_path.size(); ++i) {
    pose.pose.position.x = interpolated_path[i].x() + origin_x;
    pose.pose.position.y = interpolated_path[i].y() + origin_y;

    double dx = 0.0;
    double dy = 0.0;
    if (i == 0U) {
      dx = interpolated_path[i + 1].x() - interpolated_path[i].x();
      dy = interpolated_path[i + 1].y() - interpolated_path[i].y();
    } else if (i == interpolated_path.size() - 1) {
      dx = interpolated_path[i].x() - interpolated_path[i - 1].x();
      dy = interpolated_path[i].y() - interpolated_path[i - 1].y();
    } else {
      dx = 0.5 * (interpolated_path[i + 1].x() - interpolated_path[i - 1].x());
      dy = 0.5 * (interpolated_path[i + 1].y() - interpolated_path[i - 1].y());
    }

    double yaw = std::atan2(dy, dx);
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    plan->push_back(pose);
  }
}

void LatticePathPlannerROS::PublishGlobalPlan(
    const std::vector<geometry_msgs::PoseStamped>& plan) const {
  if (plan.empty()) {
    return;
  }

  nav_msgs::Path gui_path;
  gui_path.header = plan.front().header;
  gui_path.poses = plan;

  plan_pub_.publish(gui_path);
}

}  // namespace lattice_path_planner
