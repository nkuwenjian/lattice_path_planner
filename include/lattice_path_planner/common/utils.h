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

#include <cmath>
#include <limits>
#include <set>
#include <string>
#include <vector>

#include "glog/logging.h"

#include "lattice_path_planner/common/constants.h"

namespace lattice_path_planner {
namespace common {

// input angle should be in radians
// counterclockwise is positive
// output is an angle in the range of from 0 to 2*PI
inline double NormalizeAngle(double angle) {
  double retangle = angle;

  // get to the range from -2PI, 2PI
  if (std::abs(retangle) > 2 * M_PI) {
    retangle = retangle - static_cast<int>(retangle / (2 * M_PI)) * 2 * M_PI;
  }

  // get to the range 0, 2PI
  if (retangle < 0) {
    retangle += 2 * M_PI;
  }

  CHECK(retangle >= 0 && retangle < 2 * M_PI)
      << "after normalization of angle=" << angle
      << " we get angle=" << retangle;
  return retangle;
}

inline int NormalizeDiscTheta(int nTheta, int THETADIRS) {
  return nTheta >= 0 ? nTheta % THETADIRS
                     : (nTheta % THETADIRS + THETADIRS) % THETADIRS;
}

/**
 * \brief computes minimum unsigned difference between two angles in radians
 */
inline double ComputeMinUnsignedAngleDiff(double angle1, double angle2) {
  // get the angles into 0-2*PI range
  angle1 = NormalizeAngle(angle1);
  angle2 = NormalizeAngle(angle2);

  double anglediff = std::abs(angle1 - angle2);

  // see if we can take a shorter route
  if (anglediff > M_PI) {
    anglediff = std::abs(anglediff - 2 * M_PI);
  }

  return anglediff;
}

// converts discretized version of angle into continuous (radians)
// maps 0->0, 1->delta, 2->2*delta, ...
inline double DiscTheta2Cont(int nTheta, int NUMOFANGLEVALS) {
  double thetaBinSize = 2.0 * M_PI / NUMOFANGLEVALS;
  return nTheta * thetaBinSize;
}

// converts continuous (radians) version of angle into discrete
// maps 0->0, [delta/2, 3/2*delta)->1, [3/2*delta, 5/2*delta)->2,...
inline int ContTheta2Disc(double fTheta, int NUMOFANGLEVALS) {
  double thetaBinSize = 2.0 * M_PI / NUMOFANGLEVALS;
  return static_cast<int>(NormalizeAngle(fTheta + thetaBinSize / 2.0) /
                          (2.0 * M_PI) * (NUMOFANGLEVALS));
}

inline double DiscXY2Cont(int X, double CELLSIZE) {
  return X * CELLSIZE + CELLSIZE / 2.0;
}

inline int ContXY2Disc(double X, double CELLSIZE) {
  return X >= 0.0 ? static_cast<int>(X / CELLSIZE)
                  : static_cast<int>(X / CELLSIZE) - 1;
}

class XYCell {
 public:
  XYCell() = default;
  XYCell(int x, int y) : x_(x), y_(y) {}
  virtual ~XYCell() = default;

  void set_x(int x) { x_ = x; }
  void set_y(int y) { y_ = y; }
  int x() const { return x_; }
  int y() const { return y_; }
  int* mutable_x() { return &x_; }
  int* mutable_y() { return &y_; }

  void MakeFloor(const XYCell& cell) {
    if (cell.x_ < x_) {
      x_ = cell.x_;
    }
    if (cell.y_ < y_) {
      y_ = cell.y_;
    }
  }

  void MakeCeil(const XYCell& cell) {
    if (cell.x_ > x_) {
      x_ = cell.x_;
    }
    if (cell.y_ > y_) {
      y_ = cell.y_;
    }
  }

  bool operator==(const XYCell& rhs) const {
    return x_ == rhs.x_ && y_ == rhs.y_;
  }

  bool operator<(const XYCell& rhs) const {
    return x_ < rhs.x_ || (x_ == rhs.x_ && y_ < rhs.y_);
  }

 private:
  int x_ = 0;
  int y_ = 0;
};

class XYPoint {
 public:
  XYPoint() = default;
  XYPoint(double x, double y) : x_(x), y_(y) {}
  virtual ~XYPoint() = default;

  void set_x(double x) { x_ = x; }
  void set_y(double y) { y_ = y; }
  double x() const { return x_; }
  double y() const { return y_; }
  double* mutable_x() { return &x_; }
  double* mutable_y() { return &y_; }

 private:
  double x_ = 0.0;
  double y_ = 0.0;
};

class XYThetaPoint {
 public:
  XYThetaPoint() = default;
  XYThetaPoint(double x, double y, double theta)
      : x_(x), y_(y), theta_(theta) {}
  virtual ~XYThetaPoint() = default;

  void set_x(double x) { x_ = x; }
  void set_y(double y) { y_ = y; }
  void set_theta(double theta) { theta_ = theta; }
  double x() const { return x_; }
  double y() const { return y_; }
  double theta() const { return theta_; }
  double* mutable_x() { return &x_; }
  double* mutable_y() { return &y_; }
  double* mutable_theta() { return &theta_; }

 private:
  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
};

// configuration parameters
struct EnvironmentConfig {
  int max_grid_x;
  int max_grid_y;
  std::vector<std::vector<uint8_t>> grid_map;

  // the value at which and above which cells are obstacles in the maps sent
  // from outside the default is defined above
  uint8_t obsthresh;

  // the value at which and above which until obsthresh (not including it)
  // cells have the nearest obstacle at distance smaller than or equal to
  // the inner circle of the robot. In other words, the robot is definitely
  // colliding with the obstacle, independently of its orientation
  // if no such cost is known, then it should be set to obsthresh (if center
  // of the robot collides with obstacle, then the whole robot collides with
  // it independently of its rotation)
  uint8_t cost_inscribed_thresh;

  // the value at which and above which until cost_inscribed_thresh (not
  // including it) cells
  // **may** have a nearest osbtacle within the distance that is in between
  // the robot inner circle and the robot outer circle
  // any cost below this value means that the robot will NOT collide with any
  // obstacle, independently of its orientation
  // if no such cost is known, then it should be set to 0 or -1 (then no cell
  // cost will be lower than it, and therefore the robot's footprint will
  // always be checked)
  int cost_possibly_circumscribed_thresh;  // it has to be integer, because -1
                                           // means that it is not provided.

  double nominalvel_mpersecs;

  double timetoturn45degsinplace_secs;

  double xy_grid_resolution;
  int phi_grid_resolution = kPhiGridResolution;

  std::vector<XYPoint> footprint;
};

/**
 * Defines a bounding box in 2-dimensional real space.
 */
class XYCellBounds {
 public:
  XYCellBounds()
      : minimum_(std::numeric_limits<int>::max(),
                 std::numeric_limits<int>::max()),
        maximum_(-std::numeric_limits<int>::max(),
                 -std::numeric_limits<int>::max()) {}

  virtual ~XYCellBounds() = default;

  const XYCell& minimum() const { return minimum_; }
  const XYCell& maximum() const { return maximum_; }

  void set_minimum(const XYCell& minimum) { minimum_ = minimum; }
  void set_maximum(const XYCell& maximum) { maximum_ = maximum; }

  void Add(const XYCell& rhs) {
    minimum_.MakeFloor(rhs);
    maximum_.MakeCeil(rhs);
  }

  void Add(const XYCellBounds& bounds) {
    Add(bounds.minimum_);
    Add(bounds.maximum_);
  }

  bool IsInBounds(const XYCell& rhs) const {
    return rhs.x() >= minimum_.x() && rhs.x() <= maximum_.x() &&
           rhs.y() >= minimum_.y() && rhs.y() <= maximum_.y();
  }

 private:
  XYCell minimum_;
  XYCell maximum_;
};

inline void StrSplit(const std::string& str, char spliter,
                     std::vector<std::string>* out) {
  if (out == nullptr) {
    return;
  }
  out->clear();
  std::size_t posBegin(0);
  std::size_t posFind = str.find(spliter, posBegin);
  while (posFind != std::string::npos) {
    std::string tmp = str.substr(posBegin, posFind - posBegin);
    out->push_back(tmp);
    posBegin = posFind + 1;
    posFind = str.find(spliter, posBegin);
  }

  std::string tmp = str.substr(posBegin, str.length() - posBegin);
  if (!tmp.empty()) {
    out->push_back(tmp);
  }
}

void ReadConfiguration(const std::string& file, int* max_grid_x,
                       int* max_grid_y, uint8_t* obsthresh,
                       uint8_t* cost_inscribed_thresh,
                       int* cost_possibly_circumscribed_thresh,
                       double* xy_grid_resolution, double* nominalvel_mpersecs,
                       double* timetoturn45degsinplace_secs,
                       std::vector<std::vector<uint8_t>>* grid_map);

bool WriteConfiguration(const std::string& file, int max_grid_x, int max_grid_y,
                        uint8_t obsthresh, uint8_t cost_inscribed_thresh,
                        int cost_possibly_circumscribed_thresh,
                        double xy_grid_resolution, double nominalvel_mpersecs,
                        double timetoturn45degsinplace_secs,
                        const std::vector<std::vector<uint8_t>>& grid_map);

}  // namespace common
}  // namespace lattice_path_planner
