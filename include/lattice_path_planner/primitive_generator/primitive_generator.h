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

#include "lattice_path_planner/common/utils.h"

namespace lattice_path_planner {
namespace primitive_generator {

// data read from mprim file
struct Primitive {
  int id;
  uint8_t start_grid_phi;
  int multiplier;
  unsigned int cost;
  int end_grid_x;
  int end_grid_y;
  int end_grid_phi;  // end_grid_phi is in the range of [0, 16)

  std::vector<common::XYCell> intersectingcellsV;

  // intermptV start at 0,0,starttheta and end at endcell in continuous
  // domain with half-bin less to account for 0,0 start
  std::vector<common::XYThetaPoint> intermptV;

  // start at 0,0,starttheta and end at endcell in discrete domain
  std::vector<common::XYCell> interm2DcellsV;
};

class PrimitiveGenerator {
 public:
  explicit PrimitiveGenerator(const common::EnvironmentConfig& env_cfg);
  virtual ~PrimitiveGenerator() = default;

  void Init(const std::string& file);
  const std::vector<std::vector<Primitive>>& motion_primitives() const {
    return motion_primitives_;
  }

 private:
  bool ReadMotionPrimitives(const std::string& file);
  void ReadinMotionPrimitive(std::ifstream* fin, Primitive* prim) const;
  static void ReadinPose(std::ifstream* fin, double* x, double* y, double* phi);
  void PrecomputeActionswithCompleteMotionPrimitive();

  const common::EnvironmentConfig& env_cfg_;
  int phi_grid_resolution_;
  int action_width_;
  std::vector<std::vector<Primitive>> motion_primitives_;
  common::XYCellBounds polygon_bounds_;
};

}  // namespace primitive_generator
}  // namespace lattice_path_planner
