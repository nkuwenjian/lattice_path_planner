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

#include "lattice_path_planner/primitive_generator/primitive_generator.h"

#include <algorithm>
#include <fstream>
#include <set>
#include <utility>

#include "lattice_path_planner/common/footprint_helper.h"
#include "lattice_path_planner/common/utils.h"

namespace lattice_path_planner {
namespace primitive_generator {

PrimitiveGenerator::PrimitiveGenerator(const common::EnvironmentConfig& env_cfg)
    : env_cfg_(env_cfg) {}

void PrimitiveGenerator::Init(const std::string& file) {
  ReadMotionPrimitives(file);
  PrecomputeActionswithCompleteMotionPrimitive();
}

bool PrimitiveGenerator::ReadMotionPrimitives(const std::string& file) {
  std::ifstream fin(file, std::ios::in);
  if (!fin.is_open()) {
    return false;
  }
  std::string line;
  std::vector<std::string> strs;

  // resolution_m:
  std::getline(fin, line);
  common::StrSplit(line, ' ', &strs);
  CHECK_EQ(strs.size(), 2);
  CHECK_EQ(strs[0], "resolution_m:");
  CHECK_NEAR(std::abs(std::stod(strs[1])),
             std::abs(env_cfg_.xy_grid_resolution), 1e-6);
  VLOG(4) << std::fixed << "resolution_m: " << std::stod(strs[1]);

  // numberofangles:
  std::getline(fin, line);
  common::StrSplit(line, ' ', &strs);
  CHECK_EQ(strs.size(), 2);
  CHECK_EQ(strs[0], "numberofangles:");
  phi_grid_resolution_ = std::stoi(strs[1]);
  VLOG(4) << "numberofangles: " << strs[1];

  // totalnumberofprimitives:
  std::getline(fin, line);
  common::StrSplit(line, ' ', &strs);
  CHECK_EQ(strs.size(), 2);
  CHECK_EQ(strs[0], "totalnumberofprimitives:");
  int total_num_of_actions = std::stoi(strs[1]);
  VLOG(4) << "totalnumberofprimitives: " << total_num_of_actions;

  // Read in motion primitive for each action
  CHECK_EQ(total_num_of_actions % phi_grid_resolution_, 0);
  action_width_ = total_num_of_actions / phi_grid_resolution_;
  VLOG(4) << "action_width: " << action_width_;
  for (int tind = 0; tind < phi_grid_resolution_; tind++) {
    std::vector<Primitive> primitives;
    for (int mind = 0; mind < action_width_; mind++) {
      Primitive primitive;
      ReadinMotionPrimitive(&fin, &primitive);
      CHECK_EQ(static_cast<int>(primitive.start_grid_phi), tind);
      primitives.push_back(std::move(primitive));
    }
    motion_primitives_.push_back(std::move(primitives));
  }

  fin.close();
  return true;
}

void PrimitiveGenerator::ReadinMotionPrimitive(std::ifstream* fin,
                                               Primitive* prim) const {
  std::string line;
  std::vector<std::string> strs;

  // primID:
  std::getline(*fin, line);
  common::StrSplit(line, ' ', &strs);
  CHECK_EQ(strs.size(), 2);
  CHECK_EQ(strs[0], "primID:");
  prim->id = std::stoi(strs[1]);

  // startangle_c:
  std::getline(*fin, line);
  common::StrSplit(line, ' ', &strs);
  CHECK_EQ(strs.size(), 2);
  CHECK_EQ(strs[0], "startangle_c:");
  prim->start_grid_phi = std::stoi(strs[1]);

  // endpose_c:
  std::getline(*fin, line);
  common::StrSplit(line, ' ', &strs);
  CHECK_EQ(strs.size(), 4);
  CHECK_EQ(strs[0], "endpose_c:");
  prim->end_grid_x = std::stoi(strs[1]);
  prim->end_grid_y = std::stoi(strs[2]);
  prim->end_grid_phi = std::stoi(strs[3]);
  // normalize the angle
  prim->end_grid_phi =
      common::NormalizeDiscTheta(prim->end_grid_phi, phi_grid_resolution_);
  CHECK_GE(prim->end_grid_phi, 0);
  CHECK_LT(prim->end_grid_phi, phi_grid_resolution_);

  // additionalactioncostmult:
  std::getline(*fin, line);
  common::StrSplit(line, ' ', &strs);
  CHECK_EQ(strs.size(), 2);
  CHECK_EQ(strs[0], "additionalactioncostmult:");
  prim->multiplier = std::stoi(strs[1]);

  // intermediateposes:
  std::getline(*fin, line);
  common::StrSplit(line, ' ', &strs);
  CHECK_EQ(strs.size(), 2);
  CHECK_EQ(strs[0], "intermediateposes:");
  int num_of_interm_poses = std::stoi(strs[1]);

  // all intermposes should be with respect to 0,0 as starting pose since it
  // will be added later and should be done after the action is rotated by
  // initial orientation
  for (int i = 0; i < num_of_interm_poses; ++i) {
    common::XYThetaPoint pose;
    ReadinPose(fin, pose.mutable_x(), pose.mutable_y(), pose.mutable_theta());
    prim->intermptV.emplace_back(std::move(pose));
  }

  // Check that the last pose of the motion matches (within lattice
  // resolution) the designated end pose of the primitive
  double source_pose_x = common::DiscXY2Cont(0, env_cfg_.xy_grid_resolution);
  double source_pose_y = common::DiscXY2Cont(0, env_cfg_.xy_grid_resolution);
  double end_pose_x = source_pose_x + prim->intermptV.back().x();
  double end_pose_y = source_pose_y + prim->intermptV.back().y();
  double end_pose_phi = prim->intermptV.back().theta();

  int end_grid_x = common::ContXY2Disc(end_pose_x, env_cfg_.xy_grid_resolution);
  int end_grid_y = common::ContXY2Disc(end_pose_y, env_cfg_.xy_grid_resolution);
  int end_grid_phi = common::ContTheta2Disc(end_pose_phi, phi_grid_resolution_);
  CHECK_EQ(end_grid_x, prim->end_grid_x);
  CHECK_EQ(end_grid_y, prim->end_grid_y);
  CHECK_EQ(end_grid_phi, prim->end_grid_phi);
}

void PrimitiveGenerator::ReadinPose(std::ifstream* fin, double* x, double* y,
                                    double* phi) {
  std::string line;
  std::vector<std::string> strs;
  std::getline(*fin, line);
  common::StrSplit(line, ' ', &strs);
  CHECK_EQ(strs.size(), 3);

  *x = std::stod(strs[0]);
  *y = std::stod(strs[1]);
  *phi = std::stod(strs[2]);
  *phi = common::NormalizeAngle(*phi);
}

void PrimitiveGenerator::PrecomputeActionswithCompleteMotionPrimitive() {
  // compute sourcepose
  const double source_pose_x =
      common::DiscXY2Cont(0, env_cfg_.xy_grid_resolution);
  const double source_pose_y =
      common::DiscXY2Cont(0, env_cfg_.xy_grid_resolution);

  // iterate over source angles
  for (int tind = 0; tind < phi_grid_resolution_; tind++) {
    // iterate over motion primitives
    for (int mind = 0; mind < action_width_; mind++) {
      CHECK_EQ(motion_primitives_[tind][mind].id, mind);

      // start angle
      CHECK_EQ(static_cast<int>(motion_primitives_[tind][mind].start_grid_phi),
               tind);

      // compute and store interm points as well as intersecting cells
      motion_primitives_[tind][mind].intersectingcellsV.clear();
      motion_primitives_[tind][mind].interm2DcellsV.clear();

      std::set<common::XYCell> interm2DcellSet;
      // Compute all the intersected cells for this action (intermptV and
      // interm3DcellsV)
      for (const common::XYThetaPoint& intermpt :
           motion_primitives_[tind][mind].intermptV) {
        // also compute the intermediate discrete cells if not there already
        double x = intermpt.x() + source_pose_x;
        double y = intermpt.y() + source_pose_y;

        // add unique cells to the list
        interm2DcellSet.insert(
            {common::ContXY2Disc(x, env_cfg_.xy_grid_resolution),
             common::ContXY2Disc(y, env_cfg_.xy_grid_resolution)});
      }

      for (const auto& cell : interm2DcellSet) {
        motion_primitives_[tind][mind].interm2DcellsV.push_back(cell);
      }

      // compute linear and angular time
      double linear_distance = 0.0;
      for (size_t i = 1; i < motion_primitives_[tind][mind].intermptV.size();
           ++i) {
        double x0 = motion_primitives_[tind][mind].intermptV[i - 1].x();
        double y0 = motion_primitives_[tind][mind].intermptV[i - 1].y();
        double x1 = motion_primitives_[tind][mind].intermptV[i].x();
        double y1 = motion_primitives_[tind][mind].intermptV[i].y();
        linear_distance += std::hypot(x1 - x0, y1 - y0);
      }
      double linear_time = linear_distance / env_cfg_.nominalvel_mpersecs;
      double angular_distance = std::abs(common::ComputeMinUnsignedAngleDiff(
          common::DiscTheta2Cont(motion_primitives_[tind][mind].end_grid_phi,
                                 phi_grid_resolution_),
          common::DiscTheta2Cont(motion_primitives_[tind][mind].start_grid_phi,
                                 phi_grid_resolution_)));

      double angular_time =
          angular_distance /
          ((M_PI / 4.0) / env_cfg_.timetoturn45degsinplace_secs);
      // make the cost the max of the two times
      motion_primitives_[tind][mind].cost = static_cast<int>(
          std::ceil(1000 * std::max(linear_time, angular_time)));
      // use any additional cost multiplier
      motion_primitives_[tind][mind].cost *=
          motion_primitives_[tind][mind].multiplier;

      // now compute the intersecting cells for this motion (including ignoring
      // the source footprint)
      polygon_bounds_.Add(common::FootprintHelper::GetMotionXYCells(
          env_cfg_.footprint, motion_primitives_[tind][mind].intermptV,
          &motion_primitives_[tind][mind].intersectingcellsV,
          env_cfg_.xy_grid_resolution));
    }
  }

  LOG(INFO) << "bounding box of the polygon: minx="
            << polygon_bounds_.minimum().x()
            << ", maxx=" << polygon_bounds_.maximum().x()
            << ", miny=" << polygon_bounds_.minimum().y()
            << ", maxy=" << polygon_bounds_.maximum().y();
}

}  // namespace primitive_generator
}  // namespace lattice_path_planner
