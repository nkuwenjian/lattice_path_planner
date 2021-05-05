#ifndef LATTICE_PATH_PLANNER_LATTICE_PATH_PLANNER_H
#define LATTICE_PATH_PLANNER_LATTICE_PATH_PLANNER_H

#include <ros/ros.h>
#include "lattice_path_planner/heap.h"
#include "lattice_path_planner/lattice_environment.h"

namespace lattice_path_planner
{
class AStarSearchState : public SearchStateBase
{
public:
  AStarSearchState(int stateID) 
  {
    stateid = stateID;
  }
  virtual ~AStarSearchState() {}

public:
  int stateid;

  int g;
  int h;

  short unsigned int iterationclosed;
  short unsigned int callnumberaccessed;

  AStarSearchState* bestpredstate;
  AStarSearchState* bestnextstate;
};

/**
 * \brief the statespace of A*
 */
typedef struct ASTARSEARCHSTATESPACE
{
  Heap* heap;
  short unsigned int searchiteration;
  short unsigned int callnumber;
  
  AStarSearchState* searchstartstate;
  AStarSearchState* searchgoalstate;

  std::vector<AStarSearchState*> searchStatesV;

} AStarSearchStateSpace;

class LatticePathPlanner
{
public:
  LatticePathPlanner(LatticeEnvironment* environment);
  ~LatticePathPlanner();

public:
  bool SetSearchStartState(int SearchStartStateID);
  bool SetSearchGoalState(int SearchGoalStateID);

  bool replan(std::vector<int>* solution_stateIDs_V, int* psolcost);

private:
  bool CreateSearchStateSpace();
  void DeleteSearchStateSpace();

  bool InitializeSearchStateSpace();
  void ReInitializeSearchStateSpace();
  //initialization of a state
  void InitializeSearchStateInfo(AStarSearchState* state);
  void ReInitializeSearchStateInfo(AStarSearchState* state);

  AStarSearchState* CreateState(int stateID);
  AStarSearchState* GetState(int stateID);

  bool Search(std::vector<int>& pathIds, int& PathCost);
  int ImprovePath();
  void UpdateSuccs(AStarSearchState* state);

  std::vector<int> GetSearchPath(int& solcost);
  void ReconstructPath();

  int ComputeHeuristic(AStarSearchState* state) const;

private:
  AStarSearchStateSpace* pSearchStateSpace_;
  LatticeEnvironment* environment_;

  unsigned int searchexpands;
};

}  // namespace lattice_path_planner

#endif  // LATTICE_PATH_PLANNER_LATTICE_PATH_PLANNER_H