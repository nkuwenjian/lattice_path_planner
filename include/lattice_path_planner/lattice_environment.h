#ifndef LATTICE_PATH_PLANNER_LATTICE_ENVIRONMENT_H
#define LATTICE_PATH_PLANNER_LATTICE_ENVIRONMENT_H

#include <vector>
#include <set>
#include <cstdio>
#include <algorithm>
#include <sstream>
#include "lattice_path_planner/utils.h"
#include "lattice_path_planner/grid_search.h"

#define ENVNAVXYTHETALAT_DEFAULTOBSTHRESH 254	//see explanation of the value below
//maximum number of states for storing them into lookup (as opposed to hash)
#define SBPL_XYTHETALAT_MAXSTATESFORLOOKUP 100000000
//definition of theta orientations
//0 - is aligned with X-axis in the positive direction (1,0 in polar coordinates)
//theta increases as we go counterclockwise
//number of theta values - should be power of 2
#define NAVXYTHETALAT_THETADIRS 16
//number of actions per x,y,theta state
//decrease, increase, same angle while moving plus decrease, increase angle while standing.
#define NAVXYTHETALAT_DEFAULT_ACTIONWIDTH 5
#define NAVXYTHETALAT_COSTMULT_MTOMM 1000

namespace lattice_path_planner
{
struct EnvNAVXYTHETALATAction_t
{
  unsigned char aind; //index of the action (unique for given starttheta)
  char starttheta;
  char dX;
  char dY;
  char endtheta;
  unsigned int cost;
  std::vector<sbpl_2Dcell_t> intersectingcellsV;
  //start at 0,0,starttheta and end at endcell in continuous domain with half-bin less to account for 0,0 start
  std::vector<sbpl_xy_theta_pt_t> intermptV;
  //start at 0,0,starttheta and end at endcell in discrete domain
  std::vector<sbpl_2Dcell_t> interm2DcellsV;
};

// data read from mprim file
struct SBPL_xytheta_mprimitive
{
  int motprimID;
  unsigned char starttheta_c;
  int additionalactioncostmult;
  sbpl_xy_theta_cell_t endcell;

  //intermptV start at 0,0,starttheta and end at endcell in continuous
  //domain with half-bin less to account for 0,0 start
  std::vector<sbpl_xy_theta_pt_t> intermptV;
};

//configuration parameters
struct EnvNAVXYTHETALATConfig_t
{
  int gridMapWidth;
  int gridMapHeight;
  int NumThetaDirs;
  int startGridX;
  int startGridY;
  int startGridTheta;
  int goalGridX;
  int goalGridY;
  int goalGridTheta;
  unsigned char** Grid2D;

  // the value at which and above which cells are obstacles in the maps sent from outside
  // the default is defined above
  unsigned char obsthresh;

  // the value at which and above which until obsthresh (not including it)
  // cells have the nearest obstacle at distance smaller than or equal to
  // the inner circle of the robot. In other words, the robot is definitely
  // colliding with the obstacle, independently of its orientation
  // if no such cost is known, then it should be set to obsthresh (if center
  // of the robot collides with obstacle, then the whole robot collides with
  // it independently of its rotation)
  unsigned char cost_inscribed_thresh;

  // the value at which and above which until cost_inscribed_thresh (not including it) cells
  // **may** have a nearest osbtacle within the distance that is in between
  // the robot inner circle and the robot outer circle
  // any cost below this value means that the robot will NOT collide with any
  // obstacle, independently of its orientation
  // if no such cost is known, then it should be set to 0 or -1 (then no cell
  // cost will be lower than it, and therefore the robot's footprint will
  // always be checked)
  int cost_possibly_circumscribed_thresh; // it has to be integer, because -1 means that it is not provided.

  double nominalvel_mpersecs;

  double timetoturn45degsinplace_secs;

  double cellsize_m;

  //array of actions, ActionsV[i][j] - jth action for sourcetheta = i
  EnvNAVXYTHETALATAction_t** ActionsV;

  int actionwidth; //number of motion primitives
  std::vector<SBPL_xytheta_mprimitive> mprimV;

  std::vector<sbpl_2Dpt_t> FootprintPolygon;
  BoundingBox2 PolygonBoundingBox;
};

struct EnvNAVXYTHETALATHashEntry_t
{
  int stateID;
  int gridX;
  int gridY;
  char gridTheta;
};

//variables that dynamically change (e.g., array of states, ...)
struct EnvironmentNAVXYTHETALAT_t
{
  int startStateID;
  int goalStateID;

  bool bInitialized;
};

class LatticeEnvironment
{
public:
  LatticeEnvironment();
  ~LatticeEnvironment();

public:
  /**
   * \brief initialization of environment from file. See .cfg files for
   *        examples it also takes the perimeter of the robot with respect to some
   *        reference point centered at x=0,y=0 and orientation = 0 (along x axis).
   *        The perimeter is defined in meters as a sequence of vertices of a
   *        polygon defining the perimeter. If vector is of zero size, then robot
   *        is assumed to be point robot (you may want to inflate all obstacles by
   *        its actual radius) Motion primitives file defines the motion primitives
   *        available to the robot
   */
  bool InitializeEnv(const char* sEnvFile, const std::vector<sbpl_2Dpt_t>& perimeterptsV,
                      const char* sMotPrimFile);

  /**
   * \brief initialize environment. Gridworld is defined as matrix A of size width by height.
   *        So, internally, it is accessed as A[x][y] with x ranging from 0 to width-1 and and y from 0 to height-1
   *        Each element in A[x][y] is unsigned char. A[x][y] = 0 corresponds to
   *        fully traversable and cost is just Euclidean distance
   *        The cost of transition between two neighboring cells is
   *        EuclideanDistance*(max(A[sourcex][sourcey],A[targetx][targety])+1)
   *        f A[x][y] >= obsthresh, then in the above equation it is assumed to be infinite.
   *        The cost also incorporates the length of a motion primitive and its cost_multiplier (see getcost function)
   *        mapdata is a pointer to the values of A. If it is null, then A is
   *        initialized to all zeros. Mapping is: A[x][y] = mapdata[x+y*width]
   *        finally obsthresh defined obstacle threshold, as mentioned above
   *        for explanation of perimeter, see comments for InitializeEnv function that reads all from file
   *        cellsize is discretization in meters
   *        nominalvel_mpersecs is assumed velocity of vehicle while moving forward in m/sec
   *        timetoturn45degsinplace_secs is rotational velocity in secs/45 degrees turn
   */
  bool InitializeEnv(int width, int height,
                    /** if mapdata is NULL the grid is initialized to all freespace */
                    const unsigned char* mapdata,
                    const std::vector<sbpl_2Dpt_t>& perimeterptsV, double cellsize_m,
                    double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
                    unsigned char obsthresh, const char* sMotPrimFile);

  /**
   * \brief way to set up various parameters. For a list of parameters, see
   *        the body of the function - it is pretty straightforward
   */
  bool SetEnvParameter(const char* parameter, int value);

  /**
   * \brief sets start in meters/radians
   */
  int SetStart(double worldX, double worldY, double worldTheta);

  /**
   * \brief sets goal in meters/radians
   */
  int SetGoal(double worldX, double worldY, double worldTheta);

  /**
   * \brief re-setting the whole 2D map
   *        transform from linear array mapdata to the 2D matrix used internally: Grid2D[x][y] = mapdata[x+y*width]
   */
  bool SetMap(const unsigned char* mapdata);

  void EnsureHeuristicsUpdated();
  int GetGoalHeuristic(int stateID) const;

  void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, 
                std::vector<EnvNAVXYTHETALATAction_t*>* actionV);

  void ConvertStateIDPathintoXYThetaPath(std::vector<int>* stateIDPath,
                                        std::vector<sbpl_xy_theta_pt_t>* xythetaPath);

  void GetCoordFromState(int stateID, int& gridX, int& gridY, int& gridTheta) const;

  void PrintTimeStat() const;

private:
  void ReadConfiguration(FILE* fCfg);

  void SetConfiguration(int width, int height,
                        /** if mapdata is NULL the grid is initialized to all freespace */
                        const unsigned char* mapdata,
                        double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
                        const std::vector<sbpl_2Dpt_t>& robot_perimeterV);

  void InitGeneral(std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV);
  void InitializeEnvConfig(std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV);
  void PrecomputeActionswithCompleteMotionPrimitive(std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV);
  void InitializeEnvironment();
  void ComputeHeuristicValues();

  int GetGridIndex(int gridX, int gridY, int gridTheta) const
  {
    return gridX + gridY * EnvNAVXYTHETALATCfg.gridMapWidth + 
        gridTheta * EnvNAVXYTHETALATCfg.gridMapWidth * EnvNAVXYTHETALATCfg.gridMapHeight;
  }

  /**
   * \brief returns true if cell is within map
   */
  bool IsWithinMapCell(int gridX, int gridY) const
  {
    return (gridX >= 0 && gridX < EnvNAVXYTHETALATCfg.gridMapWidth &&
            gridY >= 0 && gridY < EnvNAVXYTHETALATCfg.gridMapHeight);
  }

  /**
   * \brief returns false if robot intersects obstacles or lies outside of
   *        the map. Note this is pretty expensive operation since it computes the
   *        footprint of the robot based on its x,y,theta
   */
  bool IsValidConfiguration(int gridX, int gridY, int gridTheta) const;

  int GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action);

  EnvNAVXYTHETALATHashEntry_t* GetHashEntry_lookup(int gridX, int gridY, int gridTheta) const;
  EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_lookup(int gridX, int gridY, int gridTheta);

  bool ReadMotionPrimitives(FILE* fMotPrims);
  bool ReadinMotionPrimitive(SBPL_xytheta_mprimitive* pMotPrim, FILE* fIn);
  bool ReadinCell(sbpl_xy_theta_cell_t* cell, FILE* fIn);
  bool ReadinPose(sbpl_xy_theta_pt_t* pose, FILE* fIn);

public:
  EnvNAVXYTHETALATConfig_t EnvNAVXYTHETALATCfg;
  EnvironmentNAVXYTHETALAT_t EnvNAVXYTHETALAT;

  //vector that maps from stateID to coords
  std::vector<EnvNAVXYTHETALATHashEntry_t*> StateID2CoordTable;

  EnvNAVXYTHETALATHashEntry_t** Coord2StateIDHashTable_lookup;
  std::vector<int> StateID2IndexMapping;

  //computes h-values that estimate distances to goal x,y from all cells
  Grid2DSearch* grid2Dsearchfromgoal; 
  bool bNeedtoRecomputeGoalHeuristics;
};

}  // namespace lattice_path_planner

#endif  // LATTICE_PATH_PLANNER_LATTICE_ENVIRONMENT_H