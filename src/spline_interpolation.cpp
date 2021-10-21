#include <chrono>
#include "lattice_path_planner/spline_interpolation.h"

namespace lattice_path_planner
{
SplineInterpolation::SplineInterpolation()
{
}

SplineInterpolation::~SplineInterpolation()
{
}

void SplineInterpolation::interpolate(const std::vector<sbpl_2Dpt_t>& path, std::vector<sbpl_2Dpt_t>& spline)
{
  const auto start_t = std::chrono::high_resolution_clock::now();

  // setup auxiliary "time grid"
  double tmin = 0.0, tmax = 0.0;
  std::vector<double> T, X, Y;

  for (int i = 0; i < (int)path.size(); ++i)
  {
    X.push_back(path[i].x);
    Y.push_back(path[i].y);
  }

  createTimeGrid(T, tmin, tmax, X, Y);

  // define a spline for each coordinate x, y
  tk::spline sx, sy;
  sx.set_points(T, X);
  sy.set_points(T, Y);

  spline.clear();
  sbpl_2Dpt_t point;
  // evaluates spline and outputs data to be used with gnuplot
  int n = 1000;  // number of grid points to plot the spline
  for (int i = 0; i < n; i++)
  {
    double t = tmin + (double)i * (tmax - tmin) / (n - 1);
    point.x = sx(t);
    point.y = sy(t);
    spline.push_back(point);
  }

  const auto end_t = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff = end_t - start_t;
  // printf("Cubic spline interpolation costs = %f ms\n", diff.count() * 1e3);
}

void SplineInterpolation::createTimeGrid(std::vector<double>& T, double& tmin, double& tmax, std::vector<double>& X,
                                         std::vector<double>& Y)
{
  assert(X.size() == Y.size() && X.size() > 2);

  // setup a "time variable" so that we can interpolate x and y
  // coordinates as a function of time: (X(t), Y(t))
  T.resize(X.size());
  T[0] = 0.0;
  for (size_t i = 1; i < T.size(); i++)
  {
    // time is proportional to the distance, i.e. we go at a const speed
    T[i] = T[i - 1] + hypot(X[i] - X[i - 1], Y[i] - Y[i - 1]);
  }

  tmin = T[0] - 0.0;
  tmax = T.back() + 0.0;
}

}  // namespace lattice_path_planner