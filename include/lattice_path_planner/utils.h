#ifndef LATTICE_PATH_PLANNER_UTILS_H
#define LATTICE_PATH_PLANNER_UTILS_H

#include <ros/ros.h>
#include <cmath>
#include <cstdio>
#include <vector>
#include <set>

#define NORMALIZEDISCTHETA(THETA, THETADIRS) (((THETA >= 0) ?\
            ((THETA) % (THETADIRS)) :\
            (((THETA) % (THETADIRS) + THETADIRS) % THETADIRS)))
#define CONTXY2DISC(X, CELLSIZE) (((X)>=0)?((int)((X)/(CELLSIZE))):((int)((X)/(CELLSIZE))-1))
#define DISCXY2CONT(X, CELLSIZE) ((X)*(CELLSIZE) + (CELLSIZE)/2.0)

#define PI_CONST 3.141592653589793238462643383279502884
#define ERR_EPS 0.000001
#define INFINITECOST 1000000000

#define DEBUG 0

namespace lattice_path_planner
{
//input angle should be in radians
//counterclockwise is positive
//output is an angle in the range of from 0 to 2*PI
inline double normalizeAngle(double angle)
{
  double retangle = angle;

  //get to the range from -2PI, 2PI
  if (fabs(retangle) > 2 * PI_CONST) retangle = retangle - ((int)(retangle / (2 * PI_CONST))) * 2 * PI_CONST;

  //get to the range 0, 2PI
  if (retangle < 0) retangle += 2 * PI_CONST;

  if (retangle < 0 || retangle > 2 * PI_CONST) {
    printf("ERROR: after normalization of angle=%f we get angle=%f\n", angle, retangle);
  }

  return retangle;
}

//converts discretized version of angle into continuous (radians)
//maps 0->0, 1->delta, 2->2*delta, ...
inline double DiscTheta2Cont(int nTheta, int NUMOFANGLEVALS)
{
  double thetaBinSize = 2.0 * PI_CONST / NUMOFANGLEVALS;
  return nTheta * thetaBinSize;
}

//converts continuous (radians) version of angle into discrete
//maps 0->0, [delta/2, 3/2*delta)->1, [3/2*delta, 5/2*delta)->2,...
inline int ContTheta2Disc(double fTheta, int NUMOFANGLEVALS)
{
  double thetaBinSize = 2.0 * PI_CONST / NUMOFANGLEVALS;
  return (int)(normalizeAngle(fTheta + thetaBinSize / 2.0) / (2.0 * PI_CONST) * (NUMOFANGLEVALS));
}

/**
 * \brief computes minimum unsigned difference between two angles in radians
 */
inline double computeMinUnsignedAngleDiff(double angle1, double angle2)
{
  //get the angles into 0-2*PI range
  angle1 = normalizeAngle(angle1);
  angle2 = normalizeAngle(angle2);

  double anglediff = fabs(angle1 - angle2);

  //see if we can take a shorter route
  if (anglediff > PI_CONST) {
    anglediff = fabs(anglediff - 2 * PI_CONST);
  }

  return anglediff;
}

class sbpl_2Dcell_t
{
public:
  sbpl_2Dcell_t()
  {
    x = 0;
    y = 0;
  }

  sbpl_2Dcell_t(int x_, int y_)
  {
    x = x_;
    y = y_;
  }

  bool operator == (const sbpl_2Dcell_t& cell) const
  {
    return x == cell.x && y == cell.y;
  }

  bool operator < (const sbpl_2Dcell_t& cell) const
  {
    return x < cell.x || (x == cell.x && y < cell.y);
  }

  /**
   * Floor point operator
   * @param rOther
   */
  void MakeFloor(const sbpl_2Dcell_t& cell)
  {
    if (cell.x < x)
      x = cell.x;
    if (cell.y < y)
      y = cell.y;
  }

  /**
   * Ceiling point operator
   * @param rOther
   */
  void MakeCeil(const sbpl_2Dcell_t& cell)
  {
    if (cell.x > x)
      x = cell.x;
    if (cell.y > y)
      y = cell.y;
  }

  int x;
  int y;
};

/**
 * Defines a bounding box in 2-dimensional real space.
 */
class BoundingBox2
{
public:
  /*
   * Default constructor
   */
  BoundingBox2()
    : m_Minimum(INFINITECOST, INFINITECOST)
    , m_Maximum(-INFINITECOST, -INFINITECOST)
  {
  }

public:
  /**
   * Get bounding box minimum
   */
  const sbpl_2Dcell_t &GetMinimum() const
  {
    return m_Minimum;
  }

  /**
   * Set bounding box minimum
   */
  void SetMinimum(const sbpl_2Dcell_t &mMinimum)
  {
    m_Minimum = mMinimum;
  }

  /**
   * Get bounding box maximum
   */
  const sbpl_2Dcell_t &GetMaximum() const
  {
    return m_Maximum;
  }

  /**
   * Set bounding box maximum
   */
  void SetMaximum(const sbpl_2Dcell_t &rMaximum)
  {
    m_Maximum = rMaximum;
  }

  /**
   * Add vector to bounding box
   */
  void Add(const sbpl_2Dcell_t &rPoint)
  {
    m_Minimum.MakeFloor(rPoint);
    m_Maximum.MakeCeil(rPoint);
  }

  /**
   * Add other bounding box to bounding box
   */
  void Add(const BoundingBox2 &rBoundingBox)
  {
    Add(rBoundingBox.GetMinimum());
    Add(rBoundingBox.GetMaximum());
  }

  /**
   * Whether the given point is in the bounds of this box
   * @param rPoint
   * @return in bounds?
   */
  bool IsInBounds(const sbpl_2Dcell_t &rPoint) const
  {
    return rPoint.x >= m_Minimum.x && rPoint.x <= m_Maximum.x &&
            rPoint.y >= m_Minimum.y && rPoint.y <= m_Maximum.y;
  }

private:
  sbpl_2Dcell_t m_Minimum;
  sbpl_2Dcell_t m_Maximum;
}; // BoundingBox2

class sbpl_2Dpt_t
{
public:
  sbpl_2Dpt_t()
  {
    x = 0;
    y = 0;
  }

  sbpl_2Dpt_t(double x_, double y_)
  {
    x = x_;
    y = y_;
  }

  bool operator == (const sbpl_2Dpt_t& p) const
  {
    return x == p.x && y == p.y;
  }

  bool operator < (const sbpl_2Dpt_t& p) const
  {
    return x < p.x || (x == p.x && y < p.y);
  }

  double x;
  double y;
};

class sbpl_xy_theta_cell_t
{
public:
  sbpl_xy_theta_cell_t()
  {
    x = 0;
    y = 0;
    theta = 0;
  }

  sbpl_xy_theta_cell_t(int x_, int y_, int theta_)
  {
    x = x_;
    y = y_;
    theta = theta_;
  }

  bool operator == (const sbpl_xy_theta_cell_t& cell) const
  {
    return x == cell.x && y == cell.y && theta == cell.theta;
  }

  bool operator < (const sbpl_xy_theta_cell_t& cell) const
  {
    return x < cell.x || (x == cell.x && (y < cell.y || (y == cell.y && theta < cell.theta)));
  }

  int x;
  int y;
  int theta;
};

class sbpl_xy_theta_pt_t
{
public:
  sbpl_xy_theta_pt_t()
  {
    x = 0;
    y = 0;
    theta = 0;
  }

  sbpl_xy_theta_pt_t(double x_, double y_, double theta_)
  {
    x = x_;
    y = y_;
    theta = theta_;
  }

  bool operator == (const sbpl_xy_theta_pt_t& p) const
  {
    return x == p.x && y == p.y && theta == p.theta;
  }

  bool operator < (const sbpl_xy_theta_pt_t& p) const
  {
    return x < p.x || (x == p.x && (y < p.y || (y == p.y && theta < p.theta)));
  }

  double x;
  double y;
  double theta;
};

// Jian Wen
BoundingBox2 get_2d_motion_cells(const std::vector<sbpl_2Dpt_t>& polygon, const std::vector<sbpl_xy_theta_pt_t>& poses,
  std::vector<sbpl_2Dcell_t>* cells, double res);

void get_2d_footprint_cells(std::vector<sbpl_2Dpt_t> polygon, std::vector<sbpl_2Dcell_t>* cells,
  sbpl_xy_theta_pt_t pose, double res);

BoundingBox2 get_2d_footprint_cells(const std::vector<sbpl_2Dpt_t>& polygon, std::set<sbpl_2Dcell_t>* cells,
  const sbpl_xy_theta_pt_t& pose, double res);

}  // namespace lattice_path_planner

#endif  // LATTICE_PATH_PLANNER_UTILS_H