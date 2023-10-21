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

#include "lattice_path_planner/common/breadth_first_search.h"

#include <utility>

namespace lattice_path_planner {
namespace common {

BreadthFirstSearch::BreadthFirstSearch(int size_x, int size_y, int obs_thresh)
    : size_x_(size_x), size_y_(size_y), obs_thresh_(obs_thresh) {
  // initialize the distance grid
  dist_.resize(size_x_);
  for (int x = 0; x < size_x_; x++) {
    dist_[x].resize(size_y_);
  }

  // initialize the actions
  dx_.resize(num_of_actions_);
  dy_.resize(num_of_actions_);
  int idx = 0;
  for (int x = -1; x <= 1; x++) {
    for (int y = -1; y <= 1; y++) {
      if (x == 0 && y == 0) {
        continue;
      }
      dx_[idx] = x;
      dy_[idx] = y;
      idx++;
    }
  }
}

bool BreadthFirstSearch::ComputeDistanceFromPoint(
    const std::vector<std::vector<int>>& grid, int x, int y) {
  if (x < 0 || x >= size_x_ || y < 0 || y >= size_y_) {
    LOG(ERROR) << "point is out of bounds!";
    return false;
  }

  ClearDistances();

  queue_.emplace(x, y);
  dist_[x][y] = 0;

  ComputeDistances(grid);
  return true;
}

bool BreadthFirstSearch::ComputeDistanceFromPoints(
    const std::vector<std::vector<int>>& grid, const std::vector<int>& x,
    const std::vector<int>& y) {
  if (x.size() != y.size()) {
    LOG(ERROR) << "size of x and y coordinates must agree!";
    return false;
  }

  ClearDistances();

  for (std::size_t i = 0U; i < x.size(); ++i) {
    if (x[i] < 0 || x[i] >= size_x_ || y[i] < 0 || y[i] >= size_y_) {
      LOG(ERROR) << "point is out of bounds!";
      return false;
    }
    queue_.emplace(x[i], y[i]);
    dist_[x[i]][y[i]] = 0;
  }

  ComputeDistances(grid);
  return true;
}

void BreadthFirstSearch::ComputeDistanceFromObstacles(
    const std::vector<std::vector<int>>& grid) {
  ClearDistances();

  for (int x = 0; x < size_x_; x++) {
    for (int y = 0; y < size_y_; y++) {
      if (grid[x][y] >= obs_thresh_) {
        queue_.emplace(x, y);
        dist_[x][y] = 0;
      }
    }
  }

  ComputeDistances(grid);
}

void BreadthFirstSearch::ComputeDistances(
    const std::vector<std::vector<int>>& grid) {
  XYCell cell;
  while (!queue_.empty()) {
    cell = queue_.front();
    queue_.pop();
    int cost = dist_[cell.x()][cell.y()] + 1;
    if (cell.x() == 0 || cell.x() == size_x_ - 1 || cell.y() == 0 ||
        cell.y() == size_y_ - 1) {
      // we are on a boundary so we have to bounds check each successor
      for (std::size_t i = 0U; i < num_of_actions_; ++i) {
        int x = cell.x() + dx_[i];
        int y = cell.y() + dy_[i];
        if (x < 0 || x >= size_x_ || y < 0 || y >= size_y_) {
          continue;
        }
        if (dist_[x][y] < 0 && grid[x][y] < obs_thresh_) {
          dist_[x][y] = cost;
          queue_.emplace(x, y);
        }
      }
    } else {
      // we are not near a boundary so no bounds check is required
      for (std::size_t i = 0U; i < num_of_actions_; ++i) {
        int x = cell.x() + dx_[i];
        int y = cell.y() + dy_[i];
        if (dist_[x][y] < 0 && grid[x][y] < obs_thresh_) {
          dist_[x][y] = cost;
          queue_.emplace(x, y);
        }
      }
    }
  }
}

void BreadthFirstSearch::ClearDistances() {
  std::queue<XYCell> empty_queue;
  swap(queue_, empty_queue);

  for (int x = 0; x < size_x_; x++) {
    for (int y = 0; y < size_y_; y++) {
      dist_[x][y] = -1;
    }
  }
}

}  // namespace common
}  // namespace lattice_path_planner
