/******************************************************************************
 * Copyright (c) 2023, NKU Mobile & Flying Robotics Lab
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
 *
 * Author: Jian Wen (nkuwenjian@gmail.com)
 *****************************************************************************/

#include "lattice_path_planner/cubic_spline_interpolation.h"

#include "glog/logging.h"
#include "spline/spline.h"

namespace lattice_path_planner {

void CubicSplineInterpolation::Interpolate(
    const std::vector<common::XYPoint>& path,
    std::vector<common::XYPoint>* spline) {
  CHECK_NOTNULL(spline);
  if (path.size() < 3U) {
    return;
  }

  // setup auxiliary "time grid"
  double tmin = 0.0;
  double tmax = 0.0;
  std::vector<double> T;
  std::vector<double> X;
  std::vector<double> Y;

  for (const common::XYPoint& point : path) {
    X.push_back(point.x());
    Y.push_back(point.y());
  }

  CreateTimeGrid(&T, &tmin, &tmax, X, Y);

  // define a spline for each coordinate x, y
  tk::spline sx, sy;
  sx.set_points(T, X);
  sy.set_points(T, Y);

  spline->clear();
  // evaluates spline and outputs data to be used with gnuplot
  size_t n = 1000U;  // number of grid points to plot the spline
  for (size_t i = 0U; i < n; ++i) {
    double t = tmin + static_cast<double>(i) * (tmax - tmin) /
                          static_cast<double>(n - 1);
    spline->emplace_back(sx(t), sy(t));
  }
}

void CubicSplineInterpolation::CreateTimeGrid(std::vector<double>* T,
                                              double* tmin, double* tmax,
                                              const std::vector<double>& X,
                                              const std::vector<double>& Y) {
  // Sanity checks.
  CHECK_NOTNULL(T);
  CHECK_NOTNULL(tmin);
  CHECK_NOTNULL(tmax);
  CHECK_EQ(X.size(), Y.size());
  CHECK_GT(X.size(), 2U);

  // setup a "time variable" so that we can interpolate x and y
  // coordinates as a function of time: (X(t), Y(t))
  T->resize(X.size());
  T->front() = 0.0;
  for (size_t i = 1U; i < T->size(); ++i) {
    // time is proportional to the distance, i.e. we go at a const speed
    T->at(i) = T->at(i - 1) + std::hypot(X[i] - X[i - 1], Y[i] - Y[i - 1]);
  }

  *tmin = T->front();
  *tmax = T->back();
}

}  // namespace lattice_path_planner
