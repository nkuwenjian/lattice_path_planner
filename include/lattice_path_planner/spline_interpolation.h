#ifndef _SPLINE_INTERPOLATION_H_
#define _SPLINE_INTERPOLATION_H_

#include <vector>
#include "lattice_path_planner/utils.h"
#include "lattice_path_planner/spline.h"

namespace lattice_path_planner
{
class SplineInterpolation
{
public:
  SplineInterpolation();
  virtual ~SplineInterpolation();

  void interpolate(const std::vector<sbpl_2Dpt_t>& path, std::vector<sbpl_2Dpt_t>& spline);

private:
  void createTimeGrid(std::vector<double>& T, double& tmin, double& tmax, std::vector<double>& X,
                      std::vector<double>& Y);
};

}  // namespace lattice_path_planner

#endif  // _SPLINE_INTERPOLATION_H_