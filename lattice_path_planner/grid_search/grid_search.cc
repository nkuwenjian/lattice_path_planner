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

#include "lattice_path_planner/grid_search/grid_search.h"

#include <algorithm>
#include <chrono>  // NOLINT
#include <cmath>

namespace lattice_path_planner {
namespace grid_search {

void GridSearch::Init(int max_grid_x, int max_grid_y, double xy_grid_resolution,
                      uint8_t obsthresh,
                      TerminationCondition termination_condition) {
  if (initialized_) {
    LOG(INFO) << "GridSearch has been initialized.";
    return;
  }

  max_grid_x_ = max_grid_x;
  max_grid_y_ = max_grid_y;
  xy_grid_resolution_ = xy_grid_resolution;
  obsthresh_ = obsthresh;
  termination_condition_ = termination_condition;

  open_list_ = std::make_unique<common::Heap>();

  dp_lookup_table_.resize(max_grid_x_);
  for (int grid_x = 0; grid_x < max_grid_x; ++grid_x) {
    for (int grid_y = 0; grid_y < max_grid_y; ++grid_y) {
      dp_lookup_table_[grid_x].emplace_back(grid_x, grid_y);
    }
  }

  ComputeGridSearchActions();
  initialized_ = true;
  LOG(INFO) << "GridSearch is initialized successfully.";
}

GridSearch::~GridSearch() { open_list_->Clear(); }

void GridSearch::Clear() {
  // clean up heap elements in open list
  open_list_->Clear();

  // clear closed list
  closed_list_.clear();
  closed_list_.resize(max_grid_x_ * max_grid_y_, common::NodeStatus::OPEN);

  start_node_ = nullptr;
  end_node_ = nullptr;
}

bool GridSearch::GenerateGridPath(
    int sx, int sy, int ex, int ey,
    const std::vector<std::vector<uint8_t>>& grid_map,
    GridAStarResult* result) {
  if (!initialized_) {
    LOG(ERROR) << "GridSearch has not been initialized.";
    return false;
  }

  const auto start_timestamp = std::chrono::system_clock::now();

  // clean up previous planning result
  Clear();
  grid_map_ = grid_map;
  iterations_++;

  // check the validity of start/goal
  if (!SetStartAndEndConfiguration(sx, sy, ex, ey)) {
    return false;
  }

  // initialize start node and insert it into heap
  start_node_->set_g(0);
  start_node_->set_h(CalcHeuCost(sx, sy));
  open_list_->Insert(start_node_, GetKey(start_node_));

  float term_factor = GetTerminationFactor(termination_condition_);

  // grid search begins
  std::size_t explored_node_num = 0U;
  while (!open_list_->Empty() &&
         end_node_->g() >
             static_cast<int>(term_factor *
                              static_cast<float>(open_list_->GetMinKey()))) {
    auto* node = dynamic_cast<Node2d*>(open_list_->Pop());
    CHECK_NOTNULL(node);
    CHECK_NE(node->g(), common::kInfiniteCost);
    closed_list_[CalcGridXYIndex(node->grid_x(), node->grid_y())] =
        common::NodeStatus::CLOSED;

    // new expand
    ++explored_node_num;
    UpdateSuccs(node);
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  const std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  VLOG(4) << "time used is " << diff.count() * 1e3 << " ms";
  VLOG(4) << "explored node num is " << explored_node_num;
  VLOG(4) << "2Dsolcost_inms = " << end_node_->g();
  VLOG(4) << "largestoptfval = " << open_list_->GetMinKey();
  VLOG(4) << "heap size = " << open_list_->Size();

  if (end_node_->g() == common::kInfiniteCost) {
    LOG(ERROR) << "Grid searching return infinite cost (open_list ran out)";
    return false;
  }
  if (termination_condition_ ==
      TerminationCondition::TERM_CONDITION_OPTPATHFOUND) {
    LoadGridAStarResult(result);
  }
  return true;
}

bool GridSearch::IsWithinMap(const int grid_x, const int grid_y) const {
  return grid_x >= 0 && grid_x < max_grid_x_ && grid_y >= 0 &&
         grid_y < max_grid_y_;
}

bool GridSearch::IsValidCell(const int grid_x, const int grid_y) const {
  if (!IsWithinMap(grid_x, grid_y)) {
    return false;
  }
  if (grid_map_[grid_x][grid_y] >= obsthresh_) {
    return false;
  }
  return true;
}

int GridSearch::CalcGridXYIndex(const int grid_x, const int grid_y) const {
  DCHECK(IsWithinMap(grid_x, grid_y));
  return grid_x + grid_y * max_grid_x_;
}

int GridSearch::GetKey(const Node2d* node) const {
  CHECK_NOTNULL(node);
  return termination_condition_ ==
                 TerminationCondition::TERM_CONDITION_OPTPATHFOUND
             ? node->g() + node->h()
             : node->g();
}

int GridSearch::GetActionCost(int curr_x, int curr_y, int action_id) const {
  CHECK(IsValidCell(curr_x, curr_y));
  const int succ_x = curr_x + actions_.dx[action_id];
  const int succ_y = curr_y + actions_.dy[action_id];
  CHECK(IsValidCell(succ_x, succ_y));

  uint8_t cost = std::max(grid_map_[curr_x][curr_y], grid_map_[succ_x][succ_y]);
  if (common::kNumOfGridSearchActions > 8) {
    if (action_id > 7) {
      int x = curr_x + actions_.dx0intersects[action_id];
      int y = curr_y + actions_.dy0intersects[action_id];
      CHECK(IsWithinMap(x, y));
      cost = std::max(cost, grid_map_[x][y]);
      x = curr_x + actions_.dx1intersects[action_id];
      y = curr_y + actions_.dy1intersects[action_id];
      CHECK(IsWithinMap(x, y));
      cost = std::max(cost, grid_map_[x][y]);
    }
  }
  if (cost >= obsthresh_) {
    return common::kInfiniteCost;
  }
  return static_cast<int>(cost + 1) * actions_.dxy_distance_mm[action_id];
}

void GridSearch::ComputeGridSearchActions() {
  // initialize some constants for 2D search
  // up
  actions_.dx[0] = 0;
  actions_.dy[0] = 1;
  actions_.dx0intersects[0] = -1;
  actions_.dy0intersects[0] = -1;
  // up right
  actions_.dx[1] = 1;
  actions_.dy[1] = 1;
  actions_.dx0intersects[1] = -1;
  actions_.dy0intersects[1] = -1;
  // right
  actions_.dx[2] = 1;
  actions_.dy[2] = 0;
  actions_.dx0intersects[2] = -1;
  actions_.dy0intersects[2] = -1;
  // down right
  actions_.dx[3] = 1;
  actions_.dy[3] = -1;
  actions_.dx0intersects[3] = -1;
  actions_.dy0intersects[3] = -1;
  // down
  actions_.dx[4] = 0;
  actions_.dy[4] = -1;
  actions_.dx0intersects[4] = -1;
  actions_.dy0intersects[4] = -1;
  // down left
  actions_.dx[5] = -1;
  actions_.dy[5] = -1;
  actions_.dx0intersects[5] = -1;
  actions_.dy0intersects[5] = -1;
  // left
  actions_.dx[6] = -1;
  actions_.dy[6] = 0;
  actions_.dx0intersects[6] = -1;
  actions_.dy0intersects[6] = -1;
  // up left
  actions_.dx[7] = -1;
  actions_.dy[7] = 1;
  actions_.dx0intersects[7] = -1;
  actions_.dy0intersects[7] = -1;

  // Note: these actions have to be starting at 8 and through 15, since they
  // get multiplied correspondingly in Dijkstra's search based on index
  if (common::kNumOfGridSearchActions == 16) {
    actions_.dx[8] = 1;
    actions_.dy[8] = 2;
    actions_.dx0intersects[8] = 0;
    actions_.dy0intersects[8] = 1;
    actions_.dx1intersects[8] = 1;
    actions_.dy1intersects[8] = 1;
    actions_.dx[9] = 2;
    actions_.dy[9] = 1;
    actions_.dx0intersects[9] = 1;
    actions_.dy0intersects[9] = 0;
    actions_.dx1intersects[9] = 1;
    actions_.dy1intersects[9] = 1;
    actions_.dx[10] = 2;
    actions_.dy[10] = -1;
    actions_.dx0intersects[10] = 1;
    actions_.dy0intersects[10] = 0;
    actions_.dx1intersects[10] = 1;
    actions_.dy1intersects[10] = -1;
    actions_.dx[11] = 1;
    actions_.dy[11] = -2;
    actions_.dx0intersects[11] = 0;
    actions_.dy0intersects[11] = -1;
    actions_.dx1intersects[11] = 1;
    actions_.dy1intersects[11] = -1;
    actions_.dx[12] = -1;
    actions_.dy[12] = -2;
    actions_.dx0intersects[12] = 0;
    actions_.dy0intersects[12] = -1;
    actions_.dx1intersects[12] = -1;
    actions_.dy1intersects[12] = -1;
    actions_.dx[13] = -2;
    actions_.dy[13] = -1;
    actions_.dx0intersects[13] = -1;
    actions_.dy0intersects[13] = 0;
    actions_.dx1intersects[13] = -1;
    actions_.dy1intersects[13] = -1;
    actions_.dx[14] = -2;
    actions_.dy[14] = 1;
    actions_.dx0intersects[14] = -1;
    actions_.dy0intersects[14] = 0;
    actions_.dx1intersects[14] = -1;
    actions_.dy1intersects[14] = 1;
    actions_.dx[15] = -1;
    actions_.dy[15] = 2;
    actions_.dx0intersects[15] = 0;
    actions_.dy0intersects[15] = 1;
    actions_.dx1intersects[15] = -1;
    actions_.dy1intersects[15] = 1;
  }

  // compute distances
  for (int dind = 0; dind < common::kNumOfGridSearchActions; dind++) {
    if (actions_.dx[dind] != 0 && actions_.dy[dind] != 0) {
      if (dind <= 7) {
        // the cost of a diagonal move in millimeters
        actions_.dxy_distance_mm[dind] =
            static_cast<int>(xy_grid_resolution_ * 1414);
      } else {
        // the cost of a move to 1,2 or 2,1 or so on in millimeters
        actions_.dxy_distance_mm[dind] =
            static_cast<int>(xy_grid_resolution_ * 2236);
      }
    } else {
      // the cost of a horizontal move in millimeters
      actions_.dxy_distance_mm[dind] =
          static_cast<int>(xy_grid_resolution_ * 1000);
    }
  }
}

bool GridSearch::SetStart(const int start_x, const int start_y) {
  if (!IsValidCell(start_x, start_y)) {
    return false;
  }
  start_node_ = GetNode(start_x, start_y);
  return true;
}

bool GridSearch::SetEnd(const int end_x, const int end_y) {
  if (!IsValidCell(end_x, end_y)) {
    return false;
  }
  end_node_ = GetNode(end_x, end_y);
  return true;
}

bool GridSearch::SetStartAndEndConfiguration(int sx, int sy, int ex, int ey) {
  if (!SetStart(sx, sy)) {
    LOG(ERROR) << "VoronoiPlanner is called on invalid start (" << sx << ","
               << sy << ")";
    return false;
  }
  CHECK_NOTNULL(start_node_);
  // since the goal has not been set yet, the start node's h value is set to 0
  CHECK_EQ(start_node_->h(), 0);

  if (!SetEnd(ex, ey)) {
    LOG(ERROR) << "VoronoiPlanner is called on invalid end (" << ex << "," << ey
               << ")";
    return false;
  }
  CHECK_NOTNULL(end_node_);
  CHECK_EQ(end_node_->h(), 0);
  return true;
}

Node2d* GridSearch::GetNode(const int grid_x, const int grid_y) {
  DCHECK(IsWithinMap(grid_x, grid_y));
  Node2d* node = &dp_lookup_table_[grid_x][grid_y];
  if (node->iterations() != iterations_) {
    node->set_h(CalcHeuCost(grid_x, grid_y));
    node->set_g(common::kInfiniteCost);
    node->set_pre_node(nullptr);
    node->set_heap_index(0);
    node->set_iterations(iterations_);
  }
  return node;
}

void GridSearch::UpdateSuccs(const Node2d* curr_node) {
  CHECK_NOTNULL(curr_node);
  const int curr_x = curr_node->grid_x();
  const int curr_y = curr_node->grid_y();
  for (int action_id = 0; action_id < common::kNumOfGridSearchActions;
       ++action_id) {
    const int succ_x = curr_x + actions_.dx[action_id];
    const int succ_y = curr_y + actions_.dy[action_id];
    if (!IsValidCell(succ_x, succ_y)) {
      continue;
    }
    if (closed_list_[CalcGridXYIndex(succ_x, succ_y)] ==
        common::NodeStatus::CLOSED) {
      continue;
    }
    // get action cost
    int action_cost = GetActionCost(curr_x, curr_y, action_id);
    if (action_cost == common::kInfiniteCost) {
      continue;
    }

    Node2d* succ_node = GetNode(succ_x, succ_y);
    // see if we can decrease the value of successive node taking into account
    // the cost of action
    if (succ_node->g() > curr_node->g() + action_cost) {
      succ_node->set_g(curr_node->g() + action_cost);
      succ_node->set_pre_node(curr_node);

      // re-insert into heap if not closed yet
      if (succ_node->heap_index() == 0) {
        open_list_->Insert(succ_node, GetKey(succ_node));
      } else {
        open_list_->Update(succ_node, GetKey(succ_node));
      }
    }
  }
}

int GridSearch::CalcHeuCost(const int grid_x, const int grid_y) const {
  if (end_node_ == nullptr) {
    return 0;
  }
  return static_cast<int>(1000 * xy_grid_resolution_ *
                          std::max(std::abs(grid_x - end_node_->grid_x()),
                                   std::abs(grid_y - end_node_->grid_y())));
}

void GridSearch::LoadGridAStarResult(GridAStarResult* result) const {
  if (result == nullptr) {
    return;
  }
  result->path_cost = end_node_->g();
  const Node2d* node = end_node_;
  std::vector<int> grid_a_x;
  std::vector<int> grid_a_y;
  while (node != nullptr) {
    grid_a_x.push_back(node->grid_x());
    grid_a_y.push_back(node->grid_y());
    node = node->pre_node();
  }
  std::reverse(grid_a_x.begin(), grid_a_x.end());
  std::reverse(grid_a_y.begin(), grid_a_y.end());
  result->x = std::move(grid_a_x);
  result->y = std::move(grid_a_y);
}

int GridSearch::CheckDpMap(const int grid_x, const int grid_y) {
  if (!initialized_) {
    LOG(ERROR) << "GridSearch has not been initialized.";
    return common::kInfiniteCost;
  }

  const Node2d* node = GetNode(grid_x, grid_y);
  CHECK_NOTNULL(node);
  CHECK_EQ(node->iterations(), iterations_);
  return node->g();
}

float GridSearch::GetTerminationFactor(
    TerminationCondition termination_condition) {
  float term_factor = 0.0F;
  switch (termination_condition) {
    case TerminationCondition::TERM_CONDITION_OPTPATHFOUND:
      term_factor = 1.0F;
      break;
    case TerminationCondition::TERM_CONDITION_20PERCENTOVEROPTPATH:
      term_factor = 1.0F / 1.2F;
      break;
    case TerminationCondition::TERM_CONDITION_TWOTIMESOPTPATH:
      term_factor = 0.5F;
      break;
    case TerminationCondition::TERM_CONDITION_THREETIMESOPTPATH:
      term_factor = 1.0F / 3.0F;
      break;
    case TerminationCondition::TERM_CONDITION_ALLCELLS:
      term_factor = 0.0F;
      break;
    default:
      term_factor = 0.0F;
  }
  return term_factor;
}

}  // namespace grid_search
}  // namespace lattice_path_planner
