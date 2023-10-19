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

#include <queue>
#include <vector>

#include "glog/logging.h"

#include "lattice_path_planner/common/utils.h"

namespace lattice_path_planner {
namespace common {

class BreadthFirstSearch {
 public:
  BreadthFirstSearch(int size_x, int size_y, int obs_thresh);
  virtual ~BreadthFirstSearch() = default;

  bool ComputeDistanceFromPoint(const std::vector<std::vector<int>>& grid,
                                int x, int y);

  bool ComputeDistanceFromPoints(const std::vector<std::vector<int>>& grid,
                                 const std::vector<int>& x,
                                 const std::vector<int>& y);

  void ComputeDistanceFromObstacles(const std::vector<std::vector<int>>& grid);

  int GetDistance(int x, int y) const { return dist_[x][y]; }

 private:
  void ClearDistances();

  void ComputeDistances(const std::vector<std::vector<int>>& grid);

  std::vector<std::vector<int>> dist_;
  std::queue<XYCell> queue_;
  int size_x_;
  int size_y_;
  int obs_thresh_;

  const size_t num_of_actions_ = 8;
  std::vector<int> dx_;
  std::vector<int> dy_;
};

}  // namespace common
}  // namespace lattice_path_planner
