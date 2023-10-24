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

#include "lattice_path_planner/lattice_a_star/lattice_a_star.h"

#include <chrono>  // NOLINT

#include "glog/logging.h"

#include "lattice_path_planner/common/footprint_helper.h"
#include "lattice_path_planner/common/utils.h"

namespace lattice_path_planner {
namespace lattice_a_star {

LatticeAStar::~LatticeAStar() { open_list_->Clear(); }

void LatticeAStar::Init(int max_grid_x, int max_grid_y, uint8_t obsthresh,
                        uint8_t cost_inscribed_thresh,
                        int cost_possibly_circumscribed_thresh,
                        double xy_grid_resolution, double nominalvel_mpersecs,
                        double timetoturn45degsinplace_secs,
                        const std::vector<common::XYPoint>& footprint,
                        char* motPrimFilename) {
  env_cfg_.max_grid_x = max_grid_x;
  env_cfg_.max_grid_y = max_grid_y;
  env_cfg_.obsthresh = obsthresh;
  env_cfg_.cost_inscribed_thresh = cost_inscribed_thresh;
  env_cfg_.cost_possibly_circumscribed_thresh =
      cost_possibly_circumscribed_thresh;
  env_cfg_.xy_grid_resolution = xy_grid_resolution;
  env_cfg_.nominalvel_mpersecs = nominalvel_mpersecs;
  env_cfg_.timetoturn45degsinplace_secs = timetoturn45degsinplace_secs;
  env_cfg_.footprint = footprint;

  motion_primitive_generator_ =
      std::make_unique<primitive_generator::PrimitiveGenerator>(env_cfg_);
  motion_primitive_generator_->Init(motPrimFilename);

  grid_a_star_heuristic_generator_ = std::make_unique<grid_search::GridSearch>(
      env_cfg_.max_grid_x, env_cfg_.max_grid_y, env_cfg_.xy_grid_resolution);
  open_list_ = std::make_unique<common::Heap>();

  lattice_lookup_table_.resize(env_cfg_.phi_grid_resolution);
  for (int i = 0; i < env_cfg_.phi_grid_resolution; ++i) {
    lattice_lookup_table_[i].resize(env_cfg_.max_grid_x * env_cfg_.max_grid_y);
  }
}

bool LatticeAStar::SetStart(double start_x, double start_y, double start_phi) {
  int start_grid_x;
  int start_grid_y;
  int start_grid_phi;
  WorldToGrid(start_x, start_y, start_phi, &start_grid_x, &start_grid_y,
              &start_grid_phi);

  // create start node
  start_node_ = GetNode(start_grid_x, start_grid_y, start_grid_phi);
  if (!ValidityCheck(start_node_)) {
    LOG(ERROR) << "Invalid start node: (" << start_grid_x << "," << start_grid_y
               << "," << start_grid_phi << ")";
    return false;
  }
  return true;
}

bool LatticeAStar::SetEnd(double end_x, double end_y, double end_phi) {
  int end_grid_x;
  int end_grid_y;
  int end_grid_phi;
  WorldToGrid(end_x, end_y, end_phi, &end_grid_x, &end_grid_y, &end_grid_phi);

  // create end node.
  end_node_ = GetNode(end_grid_x, end_grid_y, end_grid_phi);
  if (!ValidityCheck(end_node_)) {
    LOG(ERROR) << "Invalid end node: (" << end_grid_x << "," << end_grid_y
               << "," << end_grid_phi << ")";
    return false;
  }
  return true;
}

void LatticeAStar::Clear() {
  // clean up heap elements in open list
  open_list_->Clear();

  // clear closed list
  closed_list_.clear();
  closed_list_.resize(env_cfg_.phi_grid_resolution);
  for (int i = 0; i < env_cfg_.phi_grid_resolution; ++i) {
    closed_list_[i].resize(env_cfg_.max_grid_x * env_cfg_.max_grid_y,
                           common::NodeStatus::OPEN);
  }

  start_node_ = nullptr;
  end_node_ = nullptr;
}

bool LatticeAStar::Plan(double start_x, double start_y, double start_phi,
                        double end_x, double end_y, double end_phi,
                        std::vector<std::vector<uint8_t>>&& grid_map,
                        LatticeAStarResult* result) {
  const auto start_timestamp = std::chrono::system_clock::now();

  // clean up previous planning result
  Clear();
  iterations_++;
  created_node_num_ = 0;
  env_cfg_.grid_map = std::move(grid_map);

  if (!SetStart(start_x, start_y, start_phi)) {
    return false;
  }
  CHECK_NOTNULL(start_node_);
  // since the goal has not been set yet, the start node's h value is set to 0
  CHECK_EQ(start_node_->h(), 0);

  if (!SetEnd(end_x, end_y, end_phi)) {
    return false;
  }
  CHECK_NOTNULL(end_node_);
  CHECK_EQ(end_node_->h(), 0);

  // update heuristic function
  grid_a_star_heuristic_generator_->GenerateGridPath(
      end_node_->grid_x(), end_node_->grid_y(), start_node_->grid_x(),
      start_node_->grid_y(), env_cfg_.grid_map, env_cfg_.cost_inscribed_thresh,
      grid_search::TerminationCondition::TERM_CONDITION_TWOTIMESOPTPATH,
      nullptr);

  // initialize start node and insert it into heap
  start_node_->set_g(0);
  start_node_->set_h(CalcHeuCost(start_node_->grid_x(), start_node_->grid_y()));
  open_list_->Insert(start_node_, start_node_->g() + start_node_->h());

  // Lattice A* begins
  std::size_t explored_node_num = 0U;
  while (!open_list_->Empty() && end_node_->g() > open_list_->GetMinKey()) {
    // get the state
    auto* node = dynamic_cast<Node3d*>(open_list_->Pop());
    CHECK_NOTNULL(node);
    CHECK_NE(node->g(), common::kInfiniteCost);
    closed_list_[node->grid_phi()][CalcGridXYIndex(
        node->grid_x(), node->grid_y())] = common::NodeStatus::CLOSED;
    // new expand
    ++explored_node_num;
    UpdateSuccs(node);

    if (explored_node_num % 100000 == 0 && explored_node_num > 0) {
      LOG(INFO) << "expands so far=" << explored_node_num;
    }
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  const std::chrono::duration<double> time_diff =
      end_timestamp - start_timestamp;
  LOG(INFO) << "total expands this call=" << explored_node_num
            << ", planning time=" << time_diff.count() * 1e3
            << " msecs, solution cost=" << end_node_->g()
            << ", heap size=" << open_list_->Size()
            << ", num of newly created nodes=" << created_node_num_;

  if (end_node_->g() == common::kInfiniteCost) {
    LOG(INFO) << "Lattice A* searching failed.";
    return false;
  }

  LoadLatticeAStarResult(result);
  return true;
}

int LatticeAStar::CalcGridXYIndex(const int grid_x, const int grid_y) const {
  DCHECK(IsWithinMap(grid_x, grid_y));
  return grid_x + grid_y * env_cfg_.max_grid_x;
}

int LatticeAStar::CalcHeuCost(const int grid_x, const int grid_y) const {
  if (end_node_ == nullptr) {
    return 0;
  }

  // computes distances from start state that is grid2D, so it is
  // EndX_c EndY_c
  int h2D = grid_a_star_heuristic_generator_->CheckDpMap(grid_x, grid_y);
  int hEuclid =
      static_cast<int>(1000 * EuclidHeuCost(grid_x, grid_y, end_node_->grid_x(),
                                            end_node_->grid_y()));

  // define this function if it is used in the planner (heuristic backward
  // search would use it)
  return static_cast<int>(static_cast<double>(std::max(h2D, hEuclid)) /
                          env_cfg_.nominalvel_mpersecs);
}

double LatticeAStar::EuclidHeuCost(const int x1, const int y1, const int x2,
                                   const int y2) const {
  int sqdist = ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  return env_cfg_.xy_grid_resolution * std::sqrt(static_cast<double>(sqdist));
}

bool LatticeAStar::WorldToGrid(const double x, const double y, const double phi,
                               int* grid_x, int* grid_y, int* grid_phi) const {
  *grid_x = common::ContXY2Disc(x, env_cfg_.xy_grid_resolution);
  *grid_y = common::ContXY2Disc(y, env_cfg_.xy_grid_resolution);
  *grid_phi = common::ContTheta2Disc(phi, env_cfg_.phi_grid_resolution);
  return true;
}

bool LatticeAStar::GridToWorld(const int grid_x, const int grid_y,
                               const int grid_phi, double* x, double* y,
                               double* phi) const {
  if (!IsWithinMap(grid_x, grid_y)) {
    return false;
  }
  if (grid_phi < 0 || grid_phi >= env_cfg_.phi_grid_resolution) {
    return false;
  }

  *x = common::DiscXY2Cont(grid_x, env_cfg_.xy_grid_resolution);
  *y = common::DiscXY2Cont(grid_y, env_cfg_.xy_grid_resolution);
  *phi = common::DiscTheta2Cont(grid_phi, env_cfg_.phi_grid_resolution);
  return true;
}

bool LatticeAStar::IsWithinMap(const int grid_x, const int grid_y) const {
  return grid_x >= 0 && grid_x < env_cfg_.max_grid_x && grid_y >= 0 &&
         grid_y < env_cfg_.max_grid_y;
}

bool LatticeAStar::ValidityCheck(const Node3d* node) const {
  CHECK_NOTNULL(node);
  int grid_x = node->grid_x();
  int grid_y = node->grid_y();
  int grid_phi = node->grid_phi();
  if (!IsValidCell(grid_x, grid_y)) {
    return false;
  }

  // compute continuous pose
  common::XYThetaPoint pose;
  CHECK(GridToWorld(grid_x, grid_y, grid_phi, pose.mutable_x(),
                    pose.mutable_y(), pose.mutable_theta()));

  // compute covered cells
  std::vector<common::XYCell> covered_cells;
  common::FootprintHelper::GetFootprintXYCells(
      env_cfg_.footprint, &covered_cells, pose, env_cfg_.xy_grid_resolution);

  // iterate over all covered cells
  for (const common::XYCell& cell : covered_cells) {
    int x = cell.x();
    int y = cell.y();
    if (!IsWithinMap(x, y) || env_cfg_.grid_map[x][y] >= env_cfg_.obsthresh) {
      return false;
    }
  }
  return true;
}

void LatticeAStar::UpdateSuccs(const Node3d* curr_node) {
  CHECK_NOTNULL(curr_node);
  const int curr_x = curr_node->grid_x();
  const int curr_y = curr_node->grid_y();
  const int curr_phi = curr_node->grid_phi();
  const std::vector<primitive_generator::Primitive>& actions =
      motion_primitive_generator_->motion_primitives()[curr_phi];

  // iterate through every primitive
  for (const primitive_generator::Primitive& action : actions) {
    const int succ_x = curr_x + action.end_grid_x;
    const int succ_y = curr_y + action.end_grid_y;
    const int succ_phi = action.end_grid_phi;
    if (!IsValidCell(succ_x, succ_y)) {
      continue;
    }
    if (closed_list_[succ_phi][CalcGridXYIndex(succ_x, succ_y)] ==
        common::NodeStatus::CLOSED) {
      continue;
    }
    // get action cost
    int action_cost = GetActionCost(curr_x, curr_y, action);
    if (action_cost == common::kInfiniteCost) {
      continue;
    }

    Node3d* succ_node = GetNode(succ_x, succ_y, succ_phi);
    // see if we can decrease the value of successive node taking into account
    // the cost of action
    if (succ_node->g() > curr_node->g() + action_cost) {
      succ_node->set_g(curr_node->g() + action_cost);
      succ_node->set_pre_node(curr_node);
      succ_node->set_action_idx({curr_phi, action.id});

      // re-insert into heap if not closed yet
      if (succ_node->heap_index() == 0) {
        open_list_->Insert(succ_node, succ_node->g() + succ_node->h());
      } else {
        open_list_->Update(succ_node, succ_node->g() + succ_node->h());
      }
    }
  }
}

bool LatticeAStar::IsValidCell(const int grid_x, const int grid_y) const {
  if (!IsWithinMap(grid_x, grid_y)) {
    return false;
  }
  if (env_cfg_.grid_map[grid_x][grid_y] >= env_cfg_.cost_inscribed_thresh) {
    return false;
  }
  return true;
}

int LatticeAStar::GetActionCost(
    const int curr_x, const int curr_y,
    const primitive_generator::Primitive& action) const {
  // TODO(all) - go over bounding box (minpt and maxpt) to test validity and
  // skip testing boundaries below, also order intersect cells so that the four
  // farthest pts go first
  CHECK(IsValidCell(curr_x, curr_y));
  const int succ_x = curr_x + action.end_grid_x;
  const int succ_y = curr_y + action.end_grid_y;
  CHECK(IsValidCell(succ_x, succ_y));

  // need to iterate over discretized center cells and compute cost based on
  // them
  uint8_t maxcellcost = 0;
  for (const common::XYCell& cell : action.interm2DcellsV) {
    const int x = curr_x + cell.x();
    const int y = curr_y + cell.y();
    if (!IsValidCell(x, y)) {
      return common::kInfiniteCost;
    }
    maxcellcost = std::max(maxcellcost, env_cfg_.grid_map[x][y]);
  }

  // check collisions that for the particular footprint orientation along the
  // action
  if (env_cfg_.footprint.size() > 1 &&
      static_cast<int>(maxcellcost) >=
          env_cfg_.cost_possibly_circumscribed_thresh) {
    for (const common::XYCell& cell : action.intersectingcellsV) {
      // get the cell in the map
      int x = curr_x + cell.x();
      int y = curr_y + cell.y();

      // check validity
      if (!IsWithinMap(x, y) || env_cfg_.grid_map[x][y] >= env_cfg_.obsthresh) {
        return common::kInfiniteCost;
      }
    }
  }

  // to ensure consistency of h2D:
  int currentmaxcost =
      static_cast<int>(std::max({maxcellcost, env_cfg_.grid_map[curr_x][curr_y],
                                 env_cfg_.grid_map[succ_x][succ_y]}));

  // use cell cost as multiplicative factor
  return static_cast<int>(action.cost) * (currentmaxcost + 1);
  // return action.cost;
}

Node3d* LatticeAStar::GetNode(const int grid_x, const int grid_y,
                              const int grid_phi) {
  int xy_index = CalcGridXYIndex(grid_x, grid_y);

  const Node3dPtr& node = lattice_lookup_table_[grid_phi][xy_index];
  if (node == nullptr) {
    lattice_lookup_table_[grid_phi][xy_index] =
        std::make_unique<Node3d>(grid_x, grid_y, grid_phi);
    node->set_h(CalcHeuCost(grid_x, grid_y));
    node->set_iterations(iterations_);
    created_node_num_++;
  }
  // reset node created in previous planning cycle
  if (node->iterations() != iterations_) {
    node->set_h(CalcHeuCost(grid_x, grid_y));
    node->set_g(common::kInfiniteCost);
    node->set_action_idx({0, 0});
    node->set_pre_node(nullptr);
    node->set_heap_index(0);
    node->set_iterations(iterations_);
  }
  return node.get();
}

void LatticeAStar::LoadLatticeAStarResult(LatticeAStarResult* result) {
  if (result == nullptr) {
    return;
  }
  result->x.clear();
  result->y.clear();
  result->phi.clear();

  const Node3d* node = end_node_;
  std::vector<const Node3d*> astar_result;
  while (node != nullptr) {
    astar_result.push_back(node);
    node = node->pre_node();
  }
  std::reverse(astar_result.begin(), astar_result.end());
  if (astar_result.size() < 2) {
    LOG(ERROR) << "A-star result contains less than 2 nodes";
    return;
  }

  const std::vector<std::vector<primitive_generator::Primitive>>&
      motion_primitives = motion_primitive_generator_->motion_primitives();
  for (std::size_t i = 0U; i < astar_result.size() - 1U; ++i) {
    double curr_x = 0.0;
    double curr_y = 0.0;
    double curr_phi = 0.0;
    CHECK(GridToWorld(astar_result[i]->grid_x(), astar_result[i]->grid_y(),
                      astar_result[i]->grid_phi(), &curr_x, &curr_y,
                      &curr_phi));
    result->x.push_back(curr_x);
    result->y.push_back(curr_y);
    result->phi.push_back(curr_phi);
    double succ_x = 0.0;
    double succ_y = 0.0;
    double succ_phi = 0.0;
    CHECK(GridToWorld(
        astar_result[i + 1]->grid_x(), astar_result[i + 1]->grid_y(),
        astar_result[i + 1]->grid_phi(), &succ_x, &succ_y, &succ_phi));

    const primitive_generator::Primitive& action =
        motion_primitives[astar_result[i + 1]->action_idx().first]
                         [astar_result[i + 1]->action_idx().second];
    CHECK_EQ(astar_result[i]->grid_phi(), action.start_grid_phi);
    CHECK_EQ(astar_result[i + 1]->grid_phi(), action.end_grid_phi);
    for (std::size_t i = 0U; i < action.intermptV.size(); ++i) {
      const double intermpt_x = curr_x + action.intermptV[i].x();
      const double intermpt_y = curr_y + action.intermptV[i].y();
      const double intermpt_phi = action.intermptV[i].theta();
      if (i > 0 && i < action.intermptV.size() - 1) {
        result->x.push_back(intermpt_x);
        result->y.push_back(intermpt_y);
        result->phi.push_back(intermpt_phi);
      }
    }
  }

  common::XYThetaPoint last_point;
  CHECK(GridToWorld(astar_result.back()->grid_x(),
                    astar_result.back()->grid_y(),
                    astar_result.back()->grid_phi(), last_point.mutable_x(),
                    last_point.mutable_y(), last_point.mutable_theta()));
  result->x.push_back(last_point.x());
  result->y.push_back(last_point.y());
  result->phi.push_back(last_point.theta());
}

}  // namespace lattice_a_star
}  // namespace lattice_path_planner
