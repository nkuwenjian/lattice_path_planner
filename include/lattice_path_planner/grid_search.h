#ifndef LATTICE_PATH_PLANNER_GRID_SEARCH_2D_H
#define LATTICE_PATH_PLANNER_GRID_SEARCH_2D_H

#include <ros/ros.h>
#include <algorithm>
#include <cstring>
#include <cmath>
#include <cstdio>
#include "lattice_path_planner/heap.h"

#define SBPL_2DGRIDSEARCH_NUMOF2DDIRS 16

namespace lattice_path_planner
{
enum SBPL_2DGRIDSEARCH_TERM_CONDITION
{
  SBPL_2DGRIDSEARCH_TERM_CONDITION_OPTPATHFOUND,
  SBPL_2DGRIDSEARCH_TERM_CONDITION_20PERCENTOVEROPTPATH,
  SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH,
  SBPL_2DGRIDSEARCH_TERM_CONDITION_THREETIMESOPTPATH,
  SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS
};

class Grid2DSearchState : public SearchStateBase
{
public:
  int x, y;
  int g;

  /**
   * \brief iteration at which the state was accessed (generated) last
   */
  int iterationaccessed;
public:
  Grid2DSearchState() { iterationaccessed = 0; }
  virtual ~Grid2DSearchState() {}
};

class Grid2DSearch
{
public:
  Grid2DSearch(int width, int height, double cellSize_m, double nominalvel_mpersecs);
  ~Grid2DSearch();

public:
  /** \brief performs search itself. All costs are given as cost(cell1,
   *         cell2) = Euclidean distance*1000*(max(cell1,cell2)+1) for adjacent
   *         cells.
   * \note It is infinite if max(cell1,cell2) >= obsthresh For cells that are
   *       not adjacent (which may happen if 16 connected grid is ON), then max is
   *       taken over all the cells center line goes through Search is done from
   *       start to goal. termination_condition specifies when to stop the search.
   *       In particular, one may specify to run it until OPEN is empty. In this
   *       case, the values of all states will be computed.
   */
  bool search(unsigned char** Grid2D, unsigned char obsthresh, 
              int startx_c, int starty_c, int goalx_c, int goaly_c,
              SBPL_2DGRIDSEARCH_TERM_CONDITION termination_condition);

  /**
   * \brief returns the computed time cost from the start to <x,y>. If not computed, then returns lower bound on it.
   */
  int getlowerboundoncostfromstart_inms(int x, int y) const
  {
    //Dijkstra's search
    //the logic is that if s wasn't expanded, then g(s) >= maxcomputed_fval => g(s) >= maxcomputed_fval - h(s)
    return ((searchStates2D_[x][y].iterationaccessed == iteration_ && 
            searchStates2D_[x][y].g <= largestcomputedoptf_) ? searchStates2D_[x][y].g : largestcomputedoptf_);
  }

private:
  void computedxy();
  bool createSearchStates2D();
  void initializeSearchState2D(Grid2DSearchState* state2D);

  bool withinMap(int x, int y) const
  {
    return (x >= 0 && y >= 0 && x < width_ && y < height_);
  }

private:
  Heap* OPEN2D_;
  Grid2DSearchState** searchStates2D_;
  int dx_[SBPL_2DGRIDSEARCH_NUMOF2DDIRS];
  int dy_[SBPL_2DGRIDSEARCH_NUMOF2DDIRS];
  //the intermediate cells through which the actions go 
  //for all the horizontal moving actions, they go to direct neighbors, so initialize these intermediate cells to 0
  int dx0intersects_[SBPL_2DGRIDSEARCH_NUMOF2DDIRS];
  int dy0intersects_[SBPL_2DGRIDSEARCH_NUMOF2DDIRS];
  int dx1intersects_[SBPL_2DGRIDSEARCH_NUMOF2DDIRS];
  int dy1intersects_[SBPL_2DGRIDSEARCH_NUMOF2DDIRS];
  //time cost of transitions
  int dxy_timecost_ms_[SBPL_2DGRIDSEARCH_NUMOF2DDIRS];

  //map parameters
  int width_, height_;
  double cellSize_m_;
  double nominalvel_mpersecs_;

  //search iteration
  int iteration_;

  //largest optimal g-value computed by search
  int largestcomputedoptf_;

  //termination criterion used in the search
  SBPL_2DGRIDSEARCH_TERM_CONDITION term_condition_usedlast;
};

}  // namespace lattice_path_planner

#endif  // LATTICE_PATH_PLANNER_GRID_SEARCH_2D_H