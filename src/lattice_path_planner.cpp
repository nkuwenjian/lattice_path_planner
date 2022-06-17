#include <chrono>
#include "lattice_path_planner/lattice_path_planner.h"
#include "lattice_path_planner/sbpl_exception.h"

namespace lattice_path_planner
{
LatticePathPlanner::LatticePathPlanner(LatticeEnvironment* environment)
{
  environment_ = environment;

  pSearchStateSpace_ = new AStarSearchStateSpace;

  if (CreateSearchStateSpace() == false) {
    ROS_ERROR("ERROR: Failed to create statespace");
    return;
  }
  
  if (InitializeSearchStateSpace() == false) {
    ROS_ERROR("ERROR: Failed to create statespace");
    return;
  }
}

LatticePathPlanner::~LatticePathPlanner()
{
  if (pSearchStateSpace_) {
    DeleteSearchStateSpace();
    delete pSearchStateSpace_;
    pSearchStateSpace_ = NULL;
  }
}

bool LatticePathPlanner::CreateSearchStateSpace()
{
  //create a heap
  pSearchStateSpace_->heap = new Heap;

  pSearchStateSpace_->searchstartstate = NULL;
  pSearchStateSpace_->searchgoalstate = NULL;

  searchexpands = 0;

  return true;
}

void LatticePathPlanner::DeleteSearchStateSpace()
{
  if (pSearchStateSpace_->heap != NULL) {
    pSearchStateSpace_->heap->makeEmptyHeap();
    delete pSearchStateSpace_->heap;
    pSearchStateSpace_->heap = NULL;
  }

  for (int i = 0; i < (int)pSearchStateSpace_->searchStatesV.size(); i++) {
    AStarSearchState* state = pSearchStateSpace_->searchStatesV[i];
    delete state;
    state = NULL;
  }

  pSearchStateSpace_->searchStatesV.clear();
}

bool LatticePathPlanner::InitializeSearchStateSpace()
{
  if (pSearchStateSpace_->heap->heap_size != 0) {
    throw SBPL_Exception("ERROR in InitializeSearchStateSpace: heap or list is not empty");
  }

  pSearchStateSpace_->searchiteration = 0;
  pSearchStateSpace_->callnumber = 0;

  //create and set the search start state
  pSearchStateSpace_->searchstartstate = NULL;
  pSearchStateSpace_->searchgoalstate = NULL;

  return true;
}

void LatticePathPlanner::ReInitializeSearchStateSpace()
{
  int key;

  //increase callnumber
  pSearchStateSpace_->callnumber++;

  //reset iteration
  pSearchStateSpace_->searchiteration = 0;

  pSearchStateSpace_->heap->makeEmptyHeap();


  //initialize start state
  if (pSearchStateSpace_->searchstartstate->callnumberaccessed != pSearchStateSpace_->callnumber) {
    ReInitializeSearchStateInfo(pSearchStateSpace_->searchstartstate);
  }
  pSearchStateSpace_->searchstartstate->g = 0;

  //initialize goal state
  if (pSearchStateSpace_->searchgoalstate->callnumberaccessed != pSearchStateSpace_->callnumber) {
    ReInitializeSearchStateInfo(pSearchStateSpace_->searchgoalstate);
  }

  //insert start state into the heap
  key = pSearchStateSpace_->searchstartstate->g + pSearchStateSpace_->searchstartstate->h;
  pSearchStateSpace_->heap->insertHeap(pSearchStateSpace_->searchstartstate, key);
}

bool LatticePathPlanner::SetSearchStartState(int SearchStartStateID)
{
  ROS_DEBUG("planner: setting start to %d", SearchStartStateID);

  AStarSearchState* state = GetState(SearchStartStateID);

  if (state != pSearchStateSpace_->searchstartstate) {
    pSearchStateSpace_->searchstartstate = state;
  }

  return true;
}

bool LatticePathPlanner::SetSearchGoalState(int SearchGoalStateID)
{
  ROS_DEBUG("planner: setting goal to %d", SearchGoalStateID);

  if (pSearchStateSpace_->searchgoalstate == NULL ||
      pSearchStateSpace_->searchgoalstate->stateid != SearchGoalStateID)
  {
    pSearchStateSpace_->searchgoalstate = GetState(SearchGoalStateID);
  }

  return true;
}

AStarSearchState* LatticePathPlanner::GetState(int stateID)
{
  if (stateID >= (int)environment_->StateID2IndexMapping.size()) {
    std::stringstream ss("ERROR int GetState: stateID ");
    ss << stateID << " is invalid";
    throw SBPL_Exception(ss.str());
  }

  if (environment_->StateID2IndexMapping[stateID] == -1)
    return CreateState(stateID);
  else
    return pSearchStateSpace_->searchStatesV[environment_->StateID2IndexMapping[stateID]];
}

AStarSearchState* LatticePathPlanner::CreateState(int stateID)
{
  if (environment_->StateID2IndexMapping[stateID] != -1) {
    throw SBPL_Exception("ERROR in CreateState: state already created");
  }

  AStarSearchState* state = new AStarSearchState(stateID);
  //adds to the tail a state
  pSearchStateSpace_->searchStatesV.push_back(state);

  //remember the index of the state
  environment_->StateID2IndexMapping[stateID] = 
    (int)pSearchStateSpace_->searchStatesV.size() - 1;

  //create search specific info
  InitializeSearchStateInfo(state);

  return state;
}

int LatticePathPlanner::ComputeHeuristic(AStarSearchState* state) const
{
  //forward search: heur = distance from state to searchgoal which is Goal ARAState
  int retv = environment_->GetGoalHeuristic(state->stateid);

  return retv;
}

void LatticePathPlanner::InitializeSearchStateInfo(AStarSearchState* state)
{
  state->g = INFINITECOST;
  state->iterationclosed = 0;
  state->callnumberaccessed = pSearchStateSpace_->callnumber;
  state->heap_index = 0;
  state->bestpredstate = NULL;
  state->bestnextstate = NULL;

  //compute heuristics
  if (pSearchStateSpace_->searchgoalstate != NULL)
    state->h = ComputeHeuristic(state);
  else
    state->h = 0;
}

void LatticePathPlanner::ReInitializeSearchStateInfo(AStarSearchState* state)
{
  state->g = INFINITECOST;
  state->iterationclosed = 0;
  state->callnumberaccessed = pSearchStateSpace_->callnumber;
  state->heap_index = 0;
  state->bestpredstate = NULL;
  state->bestnextstate = NULL;

  //compute heuristics
  if (pSearchStateSpace_->searchgoalstate != NULL)
    state->h = ComputeHeuristic(state);
  else
    state->h = 0;
}

bool LatticePathPlanner::replan(std::vector<int>* solution_stateIDs_V, int* psolcost)
{
  std::vector<int> pathIds;
  bool bFound = false;
  int PathCost;
  *psolcost = 0;

  //plan
  bFound = Search(pathIds, PathCost);
  if (!bFound)
  {
    ROS_INFO("Failed to find a solution");
  }

  //copy the solution
  *solution_stateIDs_V = pathIds;
  *psolcost = PathCost;

  return bFound;
}

bool LatticePathPlanner::Search(std::vector<int>& pathIds, int& PathCost)
{
  searchexpands = 0;
  const auto start_t = std::chrono::high_resolution_clock::now();

  // update heuristic function
  environment_->EnsureHeuristicsUpdated();

  ReInitializeSearchStateSpace();

  pSearchStateSpace_->searchiteration++;

  // actually graph search
  ImprovePath();

  PathCost = pSearchStateSpace_->searchgoalstate->g;
  pathIds.clear();

  int solcost = INFINITECOST;
  bool bFound = false;
  if (PathCost == INFINITECOST) {
    ROS_INFO("Could not find a solution");
    bFound = false;
  }
  else {
    ROS_DEBUG("Solution is found");
    pathIds = GetSearchPath(solcost);
    bFound = true;
  }

  const auto end_t = std::chrono::high_resolution_clock::now();
  const std::chrono::duration<double> timediff = end_t - start_t;

  ROS_DEBUG("total expands this call=%d, planning time=%.3f secs, solution cost=%d", searchexpands, timediff.count(), solcost);

  return bFound;
}

int LatticePathPlanner::ImprovePath()
{
  int expands = 0;
  AStarSearchState *state = NULL;
  AStarSearchState* searchgoalstate = NULL;

  if (pSearchStateSpace_->searchgoalstate == NULL) {
    throw SBPL_Exception("ERROR searching: no goal state is set");
  }

  //goal state
  searchgoalstate = pSearchStateSpace_->searchgoalstate;

  while (!pSearchStateSpace_->heap->emptyHeap() && searchgoalstate->g > pSearchStateSpace_->heap->getMinKey())
  {
    //get the state
    state = dynamic_cast<AStarSearchState*>(pSearchStateSpace_->heap->deleteMinHeap());

    //recompute state value
    state->iterationclosed = pSearchStateSpace_->searchiteration;

    //new expand
    expands++;

    UpdateSuccs(state);

    if (expands % 100000 == 0 && expands > 0) {
      ROS_DEBUG("expands so far=%u", expands);
    }
  }

  int retv = 1;
  if (searchgoalstate->g == INFINITECOST && pSearchStateSpace_->heap->emptyHeap()) {
    ROS_INFO("Solution does not exist: search exited because heap is empty");
    retv = 0;
  }
  else if (searchgoalstate->g == INFINITECOST && !pSearchStateSpace_->heap->emptyHeap()) {
    ROS_INFO("Solution does not exist: search exited because all candidates for expansion have infinite heuristics");
    retv = 0;
  }
  else {
    retv = 1;
  }

  searchexpands += expands;

  return retv;
}

void LatticePathPlanner::UpdateSuccs(AStarSearchState* state)
{
  std::vector<int> SuccIDV;
  std::vector<int> CostV;
  int key;
  AStarSearchState* succState = NULL;

  environment_->GetSuccs(state->stateid, &SuccIDV, &CostV, NULL);

  //iterate through predecessors of s
  for (int sind = 0; sind < (int)SuccIDV.size(); sind++) {
    succState = GetState(SuccIDV[sind]);
    int cost = CostV[sind];

    if (succState->callnumberaccessed != pSearchStateSpace_->callnumber) {
      ReInitializeSearchStateInfo(succState);
    }

    //see if we can improve the value of succstate
    //taking into account the cost of action
    if (succState->g > state->g + cost) {
      int delta_g = succState->g - state->g - cost;
      succState->g = state->g + cost;
      succState->bestpredstate = state;

      //re-insert into heap if not closed yet
      if (succState->iterationclosed != pSearchStateSpace_->searchiteration) {
        key = succState->g + succState->h;

        if (succState->heap_index != 0)
          pSearchStateSpace_->heap->updateHeap(succState, key);
        else
          pSearchStateSpace_->heap->insertHeap(succState, key);
      }
    } //check for cost improvement
  } //for actions
}

std::vector<int> LatticePathPlanner::GetSearchPath(int& solcost)
{
  ReconstructPath();

  std::vector<int> SuccIDV;
  std::vector<int> CostV;
  std::vector<int> wholePathIds;
  AStarSearchState* state = NULL;
  AStarSearchState* goalstate = pSearchStateSpace_->searchgoalstate;
  AStarSearchState* startstate = pSearchStateSpace_->searchstartstate;

  state = startstate;

  wholePathIds.push_back(state->stateid);
  solcost = 0;

  while (state->stateid != goalstate->stateid) {

    if (state->bestnextstate == NULL) {
      ROS_INFO("Path does not exist since bestnextstate == NULL");
      break;
    }
    if (state->g == INFINITECOST) {
      ROS_INFO("Path does not exist since bestnextstate == NULL");
      break;
    }

    SuccIDV.clear();
    CostV.clear();
    environment_->GetSuccs(state->stateid, &SuccIDV, &CostV, NULL);
    int actioncost = INFINITECOST;
    for (int i = 0; i < (int)SuccIDV.size(); i++) {
      if (SuccIDV.at(i) == state->bestnextstate->stateid && CostV.at(i) < actioncost) {
        actioncost = CostV.at(i);
      }
    }
    if (actioncost == INFINITECOST) ROS_WARN("WARNING: actioncost = %d", actioncost);

    solcost += actioncost;

    AStarSearchState* nextstate = state->bestnextstate;
    if (actioncost != abs(state->g - nextstate->g))
    {
      ROS_DEBUG("ERROR: actioncost=%d is not matching the difference in g-values of %d",
        actioncost, abs(state->g - nextstate->g));
    }

    state = nextstate;

    wholePathIds.push_back(state->stateid);
  }

  return wholePathIds;
}

void LatticePathPlanner::ReconstructPath()
{
  AStarSearchState* state = pSearchStateSpace_->searchgoalstate;
  AStarSearchState* predstate;

  while (state != pSearchStateSpace_->searchstartstate) {

    if (state->g == INFINITECOST) {
      throw SBPL_Exception("ERROR in ReconstructPath: g of the state on the path is INFINITE");
    }

    if (state->bestpredstate == NULL) {
      ROS_ERROR("ERROR in ReconstructPath: bestpred is NULL");
      throw SBPL_Exception("ERROR in ReconstructPath: bestpred is NULL");
    }

    //get the parent state
    predstate = state->bestpredstate;

    //set its best next info
    predstate->bestnextstate = state;

    //check the decrease of g-values along the path
    if (predstate->g >= state->g) {
      ROS_ERROR("ERROR in ReconstructPath: g-values are non-decreasing");
      throw SBPL_Exception("ERROR in ReconstructPath: g-values are non-decreasing");
    }

    //transition back
    state = predstate;
  }
}

}  // namespace lattice_path_planner