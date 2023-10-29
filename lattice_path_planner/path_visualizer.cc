/******************************************************************************
 * Copyright (c) 2022, NKU Mobile & Flying Robotics Lab
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
 *****************************************************************************/

#include "lattice_path_planner/path_visualizer.h"

#include <utility>

#include "glog/logging.h"

namespace lattice_path_planner {

void PathVisualizer::Initialize(const std::string& name, double vehicle_length,
                                double vehicle_width) {
  if (initialized_) {
    LOG(INFO) << name << " path visualizer has been initialized.";
    return;
  }

  ros::NodeHandle private_nh("~/" + name);
  name_ = name;
  vehicle_length_ = vehicle_length;
  vehicle_width_ = vehicle_width;

  path_pub_ = private_nh.advertise<nav_msgs::Path>("path", 1);
  path_nodes_pub_ =
      private_nh.advertise<visualization_msgs::MarkerArray>("path_nodes", 1);
  path_vehicles_pub_ =
      private_nh.advertise<visualization_msgs::MarkerArray>("path_vehicles", 1);

  LOG(INFO) << name << " path visualizer is initialized successfully.";
  initialized_ = true;
}

void PathVisualizer::Clear() {
  path_.poses.clear();
  path_nodes_.markers.clear();
  path_vehicles_.markers.clear();
}

void PathVisualizer::Visualize(
    const std::vector<geometry_msgs::PoseStamped>& path) {
  if (!initialized_) {
    LOG(ERROR) << name_ << " path visualizer has not been initialized.";
    return;
  }

  Clear();

  UpdatePath(path);

  PublishPath();
  PublishPathNodes();
  PublishPathVehicles();
}

void PathVisualizer::UpdatePath(
    const std::vector<geometry_msgs::PoseStamped>& path) {
  path_.header.frame_id = "map";
  path_.header.stamp = ros::Time::now();

  std::size_t index = 0U;
  for (const geometry_msgs::PoseStamped& pose_stamped : path) {
    AddSegment(pose_stamped);
    AddNode(pose_stamped, index);
    ++index;
    AddVehicle(pose_stamped, index);
    ++index;
  }
}

void PathVisualizer::AddSegment(
    const geometry_msgs::PoseStamped& pose_stamped) {
  path_.poses.push_back(pose_stamped);
}

void PathVisualizer::AddNode(const geometry_msgs::PoseStamped& pose_stamped,
                             std::size_t index) {
  visualization_msgs::Marker path_node;

  // Delete all previous markers.
  if (index == 0U) {
    path_node.action = visualization_msgs::Marker::DELETEALL;
  }

  path_node.header.frame_id = "map";
  path_node.header.stamp = ros::Time(0);
  path_node.id = index;
  path_node.type = visualization_msgs::Marker::SPHERE;
  path_node.scale.x = 0.1;
  path_node.scale.y = 0.1;
  path_node.scale.z = 0.1;
  path_node.color.a = 1.0;

  // path_node.color.r = pink.red;
  // path_node.color.g = pink.green;
  // path_node.color.b = pink.blue;
  path_node.color.r = kPurple.red;
  path_node.color.g = kPurple.green;
  path_node.color.b = kPurple.blue;

  path_node.pose = pose_stamped.pose;
  path_nodes_.markers.push_back(std::move(path_node));
}

void PathVisualizer::AddVehicle(const geometry_msgs::PoseStamped& pose_stamped,
                                std::size_t index) {
  visualization_msgs::Marker path_vehicle;

  // Delete all previous markers.
  if (index == 1U) {
    path_vehicle.action = visualization_msgs::Marker::DELETEALL;
  }

  path_vehicle.header.frame_id = "map";
  path_vehicle.header.stamp = ros::Time(0);
  path_vehicle.id = index;
  path_vehicle.type = visualization_msgs::Marker::CUBE;
  path_vehicle.scale.x = vehicle_length_;
  path_vehicle.scale.y = vehicle_width_;
  path_vehicle.scale.z = 1e-5;
  path_vehicle.color.a = 0.1;

  // path_vehicle.color.r = orange.red;
  // path_vehicle.color.g = orange.green;
  // path_vehicle.color.b = orange.blue;
  path_vehicle.color.r = kTeal.red;
  path_vehicle.color.g = kTeal.green;
  path_vehicle.color.b = kTeal.blue;

  path_vehicle.pose = pose_stamped.pose;
  path_vehicles_.markers.push_back(std::move(path_vehicle));
}

void PathVisualizer::PublishPath() const { path_pub_.publish(path_); }

void PathVisualizer::PublishPathNodes() const {
  path_nodes_pub_.publish(path_nodes_);
}

void PathVisualizer::PublishPathVehicles() const {
  path_vehicles_pub_.publish(path_vehicles_);
}

}  // namespace lattice_path_planner
