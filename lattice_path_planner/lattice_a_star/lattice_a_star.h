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

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "lattice_path_planner/common/utils.h"
#include "lattice_path_planner/grid_search/grid_search.h"
#include "lattice_path_planner/lattice_a_star/node3d.h"
#include "lattice_path_planner/primitive_generator/primitive_generator.h"

namespace lattice_path_planner {
namespace lattice_a_star {

struct LatticeAStarResult {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  std::vector<double> v;
  std::vector<double> a;
  std::vector<double> steer;
  std::vector<double> accumulated_s;
};

class LatticeAStar {
  using Node3dPtr = std::unique_ptr<Node3d>;

 public:
  LatticeAStar() = default;
  virtual ~LatticeAStar();
  void Init(int max_grid_x, int max_grid_y, double xy_grid_resolution,
            uint8_t obsthresh, uint8_t cost_inscribed_thresh,
            int cost_possibly_circumscribed_thresh, double nominalvel_mpersecs,
            double timetoturn45degsinplace_secs,
            const std::vector<common::XYPoint>& footprint,
            char* motPrimFilename);
  bool Plan(double start_x, double start_y, double start_phi, double end_x,
            double end_y, double end_phi,
            std::vector<std::vector<uint8_t>>&& grid_map,
            LatticeAStarResult* result);

 private:
  bool SetStart(double start_x, double start_y, double start_phi);
  bool SetEnd(double end_x, double end_y, double end_phi);
  void LoadLatticeAStarResult(LatticeAStarResult* result);
  void Clear();
  int CalcGridXYIndex(int grid_x, int grid_y) const;
  int CalcHeuCost(int grid_x, int grid_y) const;
  double EuclidHeuCost(int x1, int y1, int x2, int y2) const;
  bool ValidityCheck(const Node3d* node) const;
  bool IsValidCell(int grid_x, int grid_y) const;
  bool WorldToGrid(double x, double y, double phi, int* grid_x, int* grid_y,
                   int* grid_phi) const;
  bool GridToWorld(int grid_x, int grid_y, int grid_phi, double* x, double* y,
                   double* phi) const;
  bool IsWithinMap(int grid_x, int grid_y) const;
  void UpdateSuccs(const Node3d* curr_node);
  int GetActionCost(int curr_x, int curr_y,
                    const primitive_generator::Primitive& action) const;
  Node3d* GetNode(int grid_x, int grid_y, int grid_phi);

  std::unique_ptr<common::Heap> open_list_ = nullptr;
  std::vector<std::vector<Node3dPtr>> lattice_lookup_table_;
  std::vector<std::vector<common::Node::NodeStatus>> closed_list_;
  std::unique_ptr<grid_search::GridSearch> grid_a_star_heuristic_generator_ =
      nullptr;
  std::unique_ptr<primitive_generator::PrimitiveGenerator>
      motion_primitive_generator_ = nullptr;
  common::EnvironmentConfig env_cfg_;

  Node3d* start_node_ = nullptr;
  Node3d* end_node_ = nullptr;
  std::size_t iterations_ = 0U;
  std::size_t created_node_num_ = 0U;
  bool initialized_ = false;
};

}  // namespace lattice_a_star
}  // namespace lattice_path_planner
