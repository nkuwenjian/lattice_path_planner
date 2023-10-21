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

#include <array>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "glog/logging.h"

#include "lattice_path_planner/common/heap.h"
#include "lattice_path_planner/grid_search/node2d.h"

namespace lattice_path_planner {
namespace grid_search {

enum class TerminationCondition : int {
  TERM_CONDITION_OPTPATHFOUND,
  TERM_CONDITION_20PERCENTOVEROPTPATH,
  TERM_CONDITION_TWOTIMESOPTPATH,
  TERM_CONDITION_THREETIMESOPTPATH,
  TERM_CONDITION_ALLCELLS
};

struct GridAStarResult {
  std::vector<int> x;
  std::vector<int> y;
  int path_cost = 0;
};

struct GridSearchPrimitives {
  std::array<int, common::kNumOfGridSearchActions> dx;
  std::array<int, common::kNumOfGridSearchActions> dy;
  // the intermediate cells through which the actions go
  // for all the horizontal moving actions, they go to direct neighbors, so
  // initialize these intermediate cells to 0
  std::array<int, common::kNumOfGridSearchActions> dx0intersects;
  std::array<int, common::kNumOfGridSearchActions> dy0intersects;
  std::array<int, common::kNumOfGridSearchActions> dx1intersects;
  std::array<int, common::kNumOfGridSearchActions> dy1intersects;
  // distances of transitions
  std::array<int, common::kNumOfGridSearchActions> dxy_distance_mm;
};

class GridSearch {
 public:
  GridSearch(int max_grid_x, int max_grid_y, double xy_grid_resolution);
  virtual ~GridSearch();
  bool GenerateGridPath(int sx, int sy, int ex, int ey,
                        const std::vector<std::vector<uint8_t>>& grid_map,
                        uint8_t obsthresh,
                        TerminationCondition termination_condition,
                        GridAStarResult* result);
  int CheckDpMap(int grid_x, int grid_y);

 private:
  bool SetStart(int start_x, int start_y);
  bool SetEnd(int end_x, int end_y);
  Node2d* GetNode(int grid_x, int grid_y);
  int CalcHeuCost(int grid_x, int grid_y) const;
  bool IsWithinMap(int grid_x, int grid_y) const;
  bool IsValidCell(int grid_x, int grid_y) const;
  int CalcGridXYIndex(int grid_x, int grid_y) const;
  int GetKey(Node2d* node) const;
  void UpdateSuccs(const Node2d& curr_node);
  void ComputeGridSearchActions();
  int GetActionCost(int curr_x, int curr_y, int action_id) const;
  void LoadGridAStarResult(GridAStarResult* result) const;
  void Clear();
  static float GetTerminationFactor(TerminationCondition termination_condition);

  int max_grid_x_ = 0;
  int max_grid_y_ = 0;
  double xy_grid_resolution_ = 0.0;
  std::vector<std::vector<uint8_t>> grid_map_;
  uint8_t obsthresh_;
  Node2d* start_node_ = nullptr;
  Node2d* end_node_ = nullptr;
  TerminationCondition termination_condition_;

  std::vector<std::vector<Node2d>> dp_lookup_table_;
  std::unique_ptr<common::Heap> open_list_ = nullptr;
  std::vector<common::NodeStatus> closed_list_;
  std::size_t iterations_ = 0U;

  GridSearchPrimitives actions_;
};

}  // namespace grid_search
}  // namespace lattice_path_planner
