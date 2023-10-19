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

#include "lattice_path_planner/common/utils.h"

#include <fstream>

#include "lattice_path_planner/common/breadth_first_search.h"

namespace lattice_path_planner {
namespace common {

void ReadConfiguration(const std::string& file, int* max_grid_x,
                       int* max_grid_y, uint8_t* obsthresh,
                       uint8_t* cost_inscribed_thresh,
                       int* cost_possibly_circumscribed_thresh,
                       double* xy_grid_resolution, double* nominalvel_mpersecs,
                       double* timetoturn45degsinplace_secs,
                       std::vector<std::vector<uint8_t>>* grid_map) {
  std::ifstream fin(file, std::ios::in);
  if (!fin.is_open()) {
    return;
  }
  std::string line;
  std::vector<std::string> strs;

  // discretization(cells):
  std::getline(fin, line);
  StrSplit(line, ' ', &strs);
  CHECK_EQ(strs.size(), 3);
  CHECK_EQ(strs[0], "discretization(cells):");
  *max_grid_x = std::stoi(strs[1]);
  *max_grid_y = std::stoi(strs[2]);
  LOG(INFO) << "discretization(cells): " << *max_grid_x << " " << *max_grid_y;

  // obsthresh:
  std::getline(fin, line);
  StrSplit(line, ' ', &strs);
  CHECK_EQ(strs.size(), 2);
  CHECK_EQ(strs[0], "obsthresh:");
  *obsthresh = static_cast<uint8_t>(std::stoi(strs[1]));
  LOG(INFO) << "obsthresh: " << std::to_string(*obsthresh);

  // cost_inscribed_thresh:
  std::getline(fin, line);
  StrSplit(line, ' ', &strs);
  CHECK_EQ(strs.size(), 2);
  CHECK_EQ(strs[0], "cost_inscribed_thresh:");
  *cost_inscribed_thresh = static_cast<uint8_t>(std::stoi(strs[1]));
  LOG(INFO) << "cost_inscribed_thresh: "
            << std::to_string(*cost_inscribed_thresh);

  // cost_possibly_circumscribed_thresh:
  std::getline(fin, line);
  StrSplit(line, ' ', &strs);
  CHECK_EQ(strs.size(), 2);
  CHECK_EQ(strs[0], "cost_possibly_circumscribed_thresh:");
  *cost_possibly_circumscribed_thresh = std::stoi(strs[1]);
  LOG(INFO) << "cost_possibly_circumscribed_thresh: "
            << *cost_possibly_circumscribed_thresh;

  // cellsize(meters):
  std::getline(fin, line);
  StrSplit(line, ' ', &strs);
  CHECK_EQ(strs.size(), 2);
  CHECK_EQ(strs[0], "cellsize(meters):");
  *xy_grid_resolution = std::stod(strs[1]);
  LOG(INFO) << "cellsize(meters): " << *xy_grid_resolution;

  // nominalvel(mpersecs):
  std::getline(fin, line);
  StrSplit(line, ' ', &strs);
  CHECK_EQ(strs.size(), 2);
  CHECK_EQ(strs[0], "nominalvel(mpersecs):");
  *nominalvel_mpersecs = std::stod(strs[1]);
  LOG(INFO) << "nominalvel(mpersecs): " << *nominalvel_mpersecs;

  // timetoturn45degsinplace(secs):
  std::getline(fin, line);
  StrSplit(line, ' ', &strs);
  CHECK_EQ(strs.size(), 2);
  CHECK_EQ(strs[0], "timetoturn45degsinplace(secs):");
  *timetoturn45degsinplace_secs = std::stod(strs[1]);
  LOG(INFO) << "timetoturn45degsinplace(secs): "
            << *timetoturn45degsinplace_secs;

  // environment:
  std::getline(fin, line);
  CHECK_EQ(line, "environment:");

  // allocate the 2D environment
  grid_map->resize(*max_grid_x);
  for (int x = 0; x < *max_grid_x; x++) {
    grid_map->at(x).resize(*max_grid_y);
  }

  size_t col = 0;
  while (std::getline(fin, line)) {
    StrSplit(line, ' ', &strs);
    CHECK_EQ(strs.size(), *max_grid_x);
    for (int row = 0; row < *max_grid_x; row++) {
      grid_map->at(row)[col] = static_cast<uint8_t>(std::stoi(strs[row]));
    }
    ++col;
  }
  CHECK_EQ(static_cast<int>(col), *max_grid_y);
  fin.close();
}

bool WriteConfiguration(const std::string& file, int max_grid_x, int max_grid_y,
                        uint8_t obsthresh, uint8_t cost_inscribed_thresh,
                        int cost_possibly_circumscribed_thresh,
                        double xy_grid_resolution, double nominalvel_mpersecs,
                        double timetoturn45degsinplace_secs,
                        const std::vector<std::vector<uint8_t>>& grid_map) {
  std::ofstream fout;
  fout.open(file, std::ios::trunc);
  if (!fout.is_open()) {
    LOG(ERROR) << "Failed to open " << file;
    return false;
  }

  fout << "discretization(cells): " << max_grid_x << " " << max_grid_y << "\n";
  fout << "obsthresh: " << std::to_string(obsthresh) << "\n";
  fout << "cost_inscribed_thresh: " << std::to_string(cost_inscribed_thresh)
       << "\n";
  fout << "cost_possibly_circumscribed_thresh: "
       << cost_possibly_circumscribed_thresh << "\n";
  fout << std::fixed << "cellsize(meters): " << xy_grid_resolution << "\n";
  fout << std::fixed << "nominalvel(mpersecs): " << nominalvel_mpersecs << "\n";
  fout << std::fixed
       << "timetoturn45degsinplace(secs): " << timetoturn45degsinplace_secs
       << "\n";
  fout << "environment:\n";
  for (int y = 0; y < max_grid_y; y++) {
    for (int x = 0; x < max_grid_x; x++) {
      fout << std::to_string(grid_map[x][y]) << " ";
    }
    fout << "\n";
  }

  fout.close();
  return true;
}

}  // namespace common
}  // namespace lattice_path_planner
