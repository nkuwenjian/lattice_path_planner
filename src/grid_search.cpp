#include <chrono>
#include "lattice_path_planner/grid_search.h"
#include "lattice_path_planner/sbpl_exception.h"

namespace lattice_path_planner
{
Grid2DSearch::Grid2DSearch(int width, int height, double cellSize_m, double nominalvel_mpersecs):
  width_(width),
  height_(height),
  cellSize_m_(cellSize_m),
  nominalvel_mpersecs_(nominalvel_mpersecs),
  OPEN2D_(NULL),
  searchStates2D_(NULL)
{
  OPEN2D_ = new Heap(width_ * height_);

  largestcomputedoptf_ = 0;
  iteration_ = 0;

  computedxy();

  if (!createSearchStates2D()) {
    throw SBPL_Exception("ERROR: failed to create searchstatespace2D");
  }
}

Grid2DSearch::~Grid2DSearch()
{
  // destroy the OPEN list:
  if (OPEN2D_ != NULL) {
    OPEN2D_->makeEmptyHeap();
    delete OPEN2D_;
    OPEN2D_ = NULL;
  }

  // destroy the 2D states:
  if (searchStates2D_ != NULL) {
    for (int x = 0; x < width_; x++) {
      delete[] searchStates2D_[x];
    }
    delete[] searchStates2D_;
    searchStates2D_ = NULL;
  }
}

void Grid2DSearch::computedxy()
{
  //initialize some constants for 2D search
  // 0 degrees: x = 1, y = 0
  dx_[0] = 1; dy_[0] = 0;
  dx0intersects_[0] = 0; dy0intersects_[0] = 0; dx1intersects_[0] = 0; dy1intersects_[0] = 0;
  dxy_timecost_ms_[0] = (int)(cellSize_m_ * 1000 / nominalvel_mpersecs_);

  // 22.5 degrees: x = 2, y = 1
  dx_[1] = 2; dy_[1] = 1;
  dx0intersects_[1] = 1; dy0intersects_[1] = 0; dx1intersects_[1] = 1; dy1intersects_[1] = 1;
  dxy_timecost_ms_[1] = (int)(cellSize_m_ * 1000 * sqrt(5) / nominalvel_mpersecs_);

  // 45 degrees: x = 1, y = 1
  dx_[2] = 1; dy_[2] = 1;
  dx0intersects_[2] = 1; dy0intersects_[2] = 0; dx1intersects_[2] = 0; dy1intersects_[2] = 1;
  dxy_timecost_ms_[2] = (int)(cellSize_m_ * 1000 * sqrt(2) / nominalvel_mpersecs_);

  // 67.5 degrees
  dx_[3] = dy_[1]; dy_[3] = dx_[1];
  dx0intersects_[3] = dy0intersects_[1]; dy0intersects_[3] = dx0intersects_[1];
  dx1intersects_[3] = dy1intersects_[1]; dy1intersects_[3] = dx1intersects_[1];
  dxy_timecost_ms_[3] = dxy_timecost_ms_[1];

  for (int i = 1; i <= 3; i++) {
    double theta = i * PI_CONST /2;

    for (int j = 0; j < SBPL_2DGRIDSEARCH_NUMOF2DDIRS / 4; j++) {
      dx_[i * SBPL_2DGRIDSEARCH_NUMOF2DDIRS / 4 + j] = dx_[j] * (int)cos(theta) - dy_[j] * (int)sin(theta);
      dy_[i * SBPL_2DGRIDSEARCH_NUMOF2DDIRS / 4 + j] = dx_[j] * (int)sin(theta) + dy_[j] * (int)cos(theta);

      dx0intersects_[i * SBPL_2DGRIDSEARCH_NUMOF2DDIRS / 4 + j] = dx0intersects_[j] * (int)cos(theta) - dy0intersects_[j] * (int)sin(theta);
      dy0intersects_[i * SBPL_2DGRIDSEARCH_NUMOF2DDIRS / 4 + j] = dx0intersects_[j] * (int)sin(theta) + dy0intersects_[j] * (int)cos(theta);
      dx1intersects_[i * SBPL_2DGRIDSEARCH_NUMOF2DDIRS / 4 + j] = dx1intersects_[j] * (int)cos(theta) - dy1intersects_[j] * (int)sin(theta);
      dy1intersects_[i * SBPL_2DGRIDSEARCH_NUMOF2DDIRS / 4 + j] = dx1intersects_[j] * (int)sin(theta) + dy1intersects_[j] * (int)cos(theta);

      dxy_timecost_ms_[i * SBPL_2DGRIDSEARCH_NUMOF2DDIRS / 4 + j] = dxy_timecost_ms_[j];
    }
  }
}

bool Grid2DSearch::createSearchStates2D()
{
  int x, y;

  if (searchStates2D_ != NULL) {
    ROS_ERROR("ERROR: We already have a non-NULL search states array");
    return false;
  }

  searchStates2D_ = new Grid2DSearchState*[width_];
  for (x = 0; x < width_; x++) {
    searchStates2D_[x] = new Grid2DSearchState[height_];
    for (y = 0; y < height_; y++) {
      searchStates2D_[x][y].x = x;
      searchStates2D_[x][y].y = y;
      initializeSearchState2D(&searchStates2D_[x][y]);
    }
  }
  return true;
}

void Grid2DSearch::initializeSearchState2D(Grid2DSearchState* state2D)
{
  state2D->g = INFINITECOST;
  state2D->heap_index = 0;
  state2D->iterationaccessed = iteration_;
}

bool Grid2DSearch::search(unsigned char** Grid2D, unsigned char obsthresh, 
                          int startx_c, int starty_c, int goalx_c, int goaly_c,
                          SBPL_2DGRIDSEARCH_TERM_CONDITION termination_condition)
{
  //check the validity of start/goal
  if (!withinMap(startx_c, starty_c) || !withinMap(goalx_c, goaly_c)) {
    ROS_ERROR("ERROR: grid2Dsearch is called on invalid start (%d %d) or goal(%d %d)", startx_c, starty_c,
                   goalx_c, goaly_c);
    return false;
  }

  //closed = 0
  iteration_++;

  Grid2DSearchState *searchExpState = NULL;
  Grid2DSearchState *searchPredState = NULL;
  int numofExpands = 0;
  int key;

  const auto start_t = std::chrono::high_resolution_clock::now();

  //clear the heap
  OPEN2D_->makeEmptyHeap();

  //set the term. condition
  term_condition_usedlast = termination_condition;

  // initialize the start and goal states
  searchExpState = &searchStates2D_[startx_c][starty_c];
  initializeSearchState2D(searchExpState);
  initializeSearchState2D(&searchStates2D_[goalx_c][goaly_c]);
  Grid2DSearchState* search2DGoalState = &searchStates2D_[goalx_c][goaly_c];

  //seed the search
  searchExpState->g = 0;
  key = searchExpState->g;

  OPEN2D_->insertHeap(searchExpState, key);

  //set the termination condition
  float term_factor = 0.0;
  switch (termination_condition) {
  case SBPL_2DGRIDSEARCH_TERM_CONDITION_OPTPATHFOUND:
    term_factor = 1;
    break;
  case SBPL_2DGRIDSEARCH_TERM_CONDITION_20PERCENTOVEROPTPATH:
    term_factor = (float)(1.0 / 1.2);
    break;
  case SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH:
    term_factor = 0.5;
    break;
  case SBPL_2DGRIDSEARCH_TERM_CONDITION_THREETIMESOPTPATH:
    term_factor = (float)(1.0 / 3.0);
    break;
  case SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS:
    term_factor = 0.0;
    break;
  default:
    ROS_ERROR("ERROR: incorrect termination factor for grid2Dsearch");
    term_factor = 0.0;
  };

  char *pbClosed = new char[width_ * height_];
  memset(pbClosed, 0, width_ * height_ * sizeof(char));

  //the main repetition of expansions
  while (!OPEN2D_->emptyHeap() && 
    search2DGoalState->g > term_factor * OPEN2D_->getMinKey())
  {
    //get the next state for expansion
    searchExpState = dynamic_cast<Grid2DSearchState*>(OPEN2D_->deleteMinHeap());
    numofExpands++;

    int exp_x = searchExpState->x;
    int exp_y = searchExpState->y;

    //close the state
    pbClosed[exp_x + width_ * exp_y] = 1;

    //iterate over successors
    unsigned char expcost = Grid2D[exp_x][exp_y];
    for (int dir = 0; dir < SBPL_2DGRIDSEARCH_NUMOF2DDIRS; dir++) {
      int newx = exp_x + dx_[dir];
      int newy = exp_y + dy_[dir];

      //make sure it is inside the map and has no obstacle
      if (!withinMap(newx, newy)) continue;

      if (pbClosed[newx + width_ * newy] == 1) continue;

      //compute the cost
      unsigned char mapcost = std::max(Grid2D[newx][newy], expcost);
      if (dx_[dir] != 0 && dy_[dir] != 0) {
        //check two more cells through which the action goes
        mapcost = std::max(mapcost, Grid2D[exp_x + dx0intersects_[dir]][exp_y + dy0intersects_[dir]]);
        mapcost = std::max(mapcost, Grid2D[exp_x + dx1intersects_[dir]][exp_y + dy1intersects_[dir]]);
      }

      //obstacle encountered
      if (mapcost >= obsthresh) continue;

      int cost = (mapcost + 1) * dxy_timecost_ms_[dir];
      // int cost = dxy_timecost_ms_[dir];

      //get the predecessor
      searchPredState = &searchStates2D_[newx][newy];

      //update predecessor if necessary
      if (searchPredState->iterationaccessed != iteration_ || searchPredState->g > cost + searchExpState->g) {
        searchPredState->iterationaccessed = iteration_;
        searchPredState->g = cost + searchExpState->g;

        key = searchPredState->g;
        if (searchPredState->heap_index == 0)
          OPEN2D_->insertHeap(searchPredState, key);
        else
          OPEN2D_->updateHeap(searchPredState, key);
      }
    } //over successors
  }//while

   //set lower bounds for the remaining states
  if (!OPEN2D_->emptyHeap())
    largestcomputedoptf_ = OPEN2D_->getMinKey();
  else
    largestcomputedoptf_ = INFINITECOST;

  delete[] pbClosed;

  const auto end_t = std::chrono::high_resolution_clock::now();
  const std::chrono::duration<double> timediff = end_t - start_t;

  ROS_DEBUG("# of expands during 2dgridsearch=%d, time=%d msecs, 2Dsolcost_inms=%d, "
    "largestoptfval=%d (start=%d %d goal=%d %d)\n",
    numofExpands, (int)(timediff.count() * 1000),
    searchStates2D_[goalx_c][goaly_c].g, largestcomputedoptf_, startx_c, starty_c, goalx_c, goaly_c);

  return true;
}

}  // namespace lattice_path_planner