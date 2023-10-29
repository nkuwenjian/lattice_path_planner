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

#pragma once

#include <string>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "visualization_msgs/MarkerArray.h"

namespace lattice_path_planner {

struct Color {
  float red;
  float green;
  float blue;
};

static constexpr Color kTeal = {102.F / 255.F, 217.F / 255.F, 239.F / 255.F};
static constexpr Color kGreen = {166.F / 255.F, 226.F / 255.F, 46.F / 255.F};
static constexpr Color kOrange = {253.F / 255.F, 151.F / 255.F, 31.F / 255.F};
static constexpr Color kPink = {249.F / 255.F, 38.F / 255.F, 114.F / 255.F};
static constexpr Color kPurple = {174.F / 255.F, 129.F / 255.F, 255.F / 255.F};

class PathVisualizer {
 public:
  PathVisualizer() = default;
  virtual ~PathVisualizer() = default;

  void Initialize(const std::string& name, double vehicle_length,
                  double vehicle_width);
  void Visualize(const std::vector<geometry_msgs::PoseStamped>& path);

 private:
  void Clear();
  void UpdatePath(const std::vector<geometry_msgs::PoseStamped>& path);
  void AddSegment(const geometry_msgs::PoseStamped& pose_stamped);
  void AddNode(const geometry_msgs::PoseStamped& pose_stamped,
               std::size_t index);
  void AddVehicle(const geometry_msgs::PoseStamped& pose_stamped,
                  std::size_t index);
  void PublishPath() const;
  void PublishPathNodes() const;
  void PublishPathVehicles() const;

 private:
  ros::Publisher path_pub_;
  ros::Publisher path_nodes_pub_;
  ros::Publisher path_vehicles_pub_;
  nav_msgs::Path path_;
  visualization_msgs::MarkerArray path_nodes_;
  visualization_msgs::MarkerArray path_vehicles_;
  std::string name_;
  double vehicle_length_ = 0.0;
  double vehicle_width_ = 0.0;
  bool initialized_ = false;
};

}  // namespace lattice_path_planner
