#include "lattice_path_planner/lattice_environment.h"
#include "lattice_path_planner/sbpl_exception.h"

namespace lattice_path_planner
{
LatticeEnvironment::LatticeEnvironment()
{
  EnvNAVXYTHETALATCfg.obsthresh = ENVNAVXYTHETALAT_DEFAULTOBSTHRESH;
  // the value that pretty much makes it disabled
  EnvNAVXYTHETALATCfg.cost_inscribed_thresh = EnvNAVXYTHETALATCfg.obsthresh;
  // the value that pretty much makes it disabled
  EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh = -1;

  EnvNAVXYTHETALATCfg.actionwidth = NAVXYTHETALAT_DEFAULT_ACTIONWIDTH;

  EnvNAVXYTHETALATCfg.NumThetaDirs = NAVXYTHETALAT_THETADIRS;

  // no memory allocated in cfg yet
  EnvNAVXYTHETALATCfg.Grid2D = NULL;
  EnvNAVXYTHETALATCfg.ActionsV = NULL;

  grid2Dsearchfromgoal = NULL;
  bNeedtoRecomputeGoalHeuristics = true;
  Coord2StateIDHashTable_lookup = NULL;

  EnvNAVXYTHETALAT.bInitialized = false;
  EnvNAVXYTHETALAT.startStateID = -1;
  EnvNAVXYTHETALAT.goalStateID = -1;
}

LatticeEnvironment::~LatticeEnvironment()
{
  if (EnvNAVXYTHETALATCfg.Grid2D != NULL) {
    for (int x = 0; x < EnvNAVXYTHETALATCfg.gridMapWidth; x++) {
      delete[] EnvNAVXYTHETALATCfg.Grid2D[x];
    }
    delete[] EnvNAVXYTHETALATCfg.Grid2D;
    EnvNAVXYTHETALATCfg.Grid2D = NULL;
  }

  //delete actions
  if (EnvNAVXYTHETALATCfg.ActionsV != NULL) {
    for (int tind = 0; tind < EnvNAVXYTHETALATCfg.NumThetaDirs; tind++) {
      delete[] EnvNAVXYTHETALATCfg.ActionsV[tind];
    }
    delete[] EnvNAVXYTHETALATCfg.ActionsV;
    EnvNAVXYTHETALATCfg.ActionsV = NULL;
  }

  //delete grid2Dsearch
  if (grid2Dsearchfromgoal != NULL) {
    delete grid2Dsearchfromgoal;
    grid2Dsearchfromgoal = NULL;
  }

  // delete the states themselves first
  for (int i = 0; i < (int)StateID2CoordTable.size(); i++) {
    delete StateID2CoordTable.at(i);
    StateID2CoordTable.at(i) = NULL;
  }
  StateID2CoordTable.clear();

  if (Coord2StateIDHashTable_lookup != NULL) {
    delete[] Coord2StateIDHashTable_lookup;
    Coord2StateIDHashTable_lookup = NULL;
  }
}

void LatticeEnvironment::ReadConfiguration(FILE* fCfg)
{
  // read in the configuration of environment and initialize
  // EnvNAVXYTHETALATCfg structure
  char sTemp[1024], sTemp1[1024];
  int dTemp;

  // discretization(cells)
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    throw SBPL_Exception("ERROR: ran out of env file early (discretization)");
  }
  strcpy(sTemp1, "discretization(cells):");
  if (strcmp(sTemp1, sTemp) != 0) {
    std::stringstream ss;
    ss << "ERROR: configuration file has incorrect format (discretization)" <<
          " Expected " << sTemp1 << " got " << sTemp;
    throw SBPL_Exception(ss.str());
  }
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    throw SBPL_Exception("ERROR: ran out of env file early (discretization)");
  }
  EnvNAVXYTHETALATCfg.gridMapWidth = atoi(sTemp);
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    throw SBPL_Exception("ERROR: ran out of env file early (discretization)");
  }
  EnvNAVXYTHETALATCfg.gridMapHeight = atoi(sTemp);

  // Scan for optional NumThetaDirs parameter. Check for following obsthresh.
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    throw SBPL_Exception("ERROR: ran out of env file early");
  }
  strcpy(sTemp1, "NumThetaDirs:");
  if (strcmp(sTemp1, sTemp) != 0) {
    // optional NumThetaDirs not available; default is NAVXYTHETALAT_THETADIRS (16)
    strcpy(sTemp1, "obsthresh:");
    if (strcmp(sTemp1, sTemp) != 0) {
      std::stringstream ss;
      ss << "ERROR: configuration file has incorrect format" <<
            " Expected " << sTemp1 << " got " << sTemp;
      throw SBPL_Exception(ss.str());
    }
    else {
      EnvNAVXYTHETALATCfg.NumThetaDirs = NAVXYTHETALAT_THETADIRS;
    }
  }
  else {
    if (fscanf(fCfg, "%s", sTemp) != 1) {
      throw SBPL_Exception("ERROR: ran out of env file early (NumThetaDirs)");
    }
    EnvNAVXYTHETALATCfg.NumThetaDirs = atoi(sTemp);

    //obsthresh:
    if (fscanf(fCfg, "%s", sTemp) != 1) {
      throw SBPL_Exception("ERROR: ran out of env file early (obsthresh)");
    }
    strcpy(sTemp1, "obsthresh:");
    if (strcmp(sTemp1, sTemp) != 0) {
      std::stringstream ss;
      ss << "ERROR: configuration file has incorrect format" <<
            " Expected " << sTemp1 << " got " << sTemp <<
            " see existing examples of env files for the right format of heading";
      throw SBPL_Exception(ss.str());
    }
  }

  // obsthresh
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    throw SBPL_Exception("ERROR: ran out of env file early");
  }
  EnvNAVXYTHETALATCfg.obsthresh = atoi(sTemp);
  ROS_INFO("obsthresh = %d", EnvNAVXYTHETALATCfg.obsthresh);

  //cost_inscribed_thresh:
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    throw SBPL_Exception("ERROR: ran out of env file early");
  }
  strcpy(sTemp1, "cost_inscribed_thresh:");
  if (strcmp(sTemp1, sTemp) != 0) {
    std::stringstream ss;
    ss << "ERROR: configuration file has incorrect format" <<
          " Expected " << sTemp1 << " got " << sTemp <<
          " see existing examples of env files for the right format of heading";
    throw SBPL_Exception(ss.str());
  }
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    throw SBPL_Exception("ERROR: ran out of env file early");
  }
  EnvNAVXYTHETALATCfg.cost_inscribed_thresh = atoi(sTemp);
  ROS_INFO("cost_inscribed_thresh = %d", EnvNAVXYTHETALATCfg.cost_inscribed_thresh);

  //cost_possibly_circumscribed_thresh:
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    throw SBPL_Exception("ERROR: ran out of env file early");
  }
  strcpy(sTemp1, "cost_possibly_circumscribed_thresh:");
  if (strcmp(sTemp1, sTemp) != 0) {
    std::stringstream ss;
    ss << "ERROR: configuration file has incorrect format" <<
          " Expected " << sTemp1 << " got " << sTemp <<
          " see existing examples of env files for the right format of heading";
    throw SBPL_Exception(ss.str());
  }
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    throw SBPL_Exception("ERROR: ran out of env file early");
  }
  EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh = atoi(sTemp);
  ROS_INFO("cost_possibly_circumscribed_thresh = %d", EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh);

  //cellsize(meters):
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    throw SBPL_Exception("ERROR: ran out of env file early");
  }
  strcpy(sTemp1, "cellsize(meters):");
  if (strcmp(sTemp1, sTemp) != 0) {
    std::stringstream ss;
    ss << "ERROR: configuration file has incorrect format" <<
          " Expected " << sTemp1 << " got " << sTemp;
    throw SBPL_Exception(ss.str());
  }
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    throw SBPL_Exception("ERROR: ran out of env file early");
  }
  EnvNAVXYTHETALATCfg.cellsize_m = atof(sTemp);
  ROS_INFO("cellsize_m = %f", EnvNAVXYTHETALATCfg.cellsize_m);

  // nominalvel(mpersecs):
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    throw SBPL_Exception("ERROR: ran out of env file early");
  }
  strcpy(sTemp1, "nominalvel(mpersecs):");
  if (strcmp(sTemp1, sTemp) != 0) {
    std::stringstream ss;
    ss << "ERROR: configuration file has incorrect format" <<
          " Expected " << sTemp1 << " got " << sTemp;
    throw SBPL_Exception(ss.str());
  }
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    throw SBPL_Exception("ERROR: ran out of env file early");
  }
  EnvNAVXYTHETALATCfg.nominalvel_mpersecs = atof(sTemp);
  ROS_INFO("nominalvel_mpersecs = %f", EnvNAVXYTHETALATCfg.nominalvel_mpersecs);

  // timetoturn45degsinplace(secs):
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    throw SBPL_Exception("ERROR: ran out of env file early");
  }
  strcpy(sTemp1, "timetoturn45degsinplace(secs):");
  if (strcmp(sTemp1, sTemp) != 0) {
    std::stringstream ss;
    ss << "ERROR: configuration file has incorrect format" <<
          " Expected " << sTemp1 << " got " << sTemp;
    throw SBPL_Exception(ss.str());
  }
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    throw SBPL_Exception("ERROR: ran out of env file early");
  }
  EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs = atof(sTemp);
  ROS_INFO("timetoturn45degsinplace_secs = %f", EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs);

  // unallocate the 2d environment
  if (EnvNAVXYTHETALATCfg.Grid2D != NULL) {
    for (int x = 0; x < EnvNAVXYTHETALATCfg.gridMapWidth; x++) {
      delete[] EnvNAVXYTHETALATCfg.Grid2D[x];
    }
    delete[] EnvNAVXYTHETALATCfg.Grid2D;
    EnvNAVXYTHETALATCfg.Grid2D = NULL;
  }

  // allocate the 2D environment
  EnvNAVXYTHETALATCfg.Grid2D = new unsigned char*[EnvNAVXYTHETALATCfg.gridMapWidth];
  for (int x = 0; x < EnvNAVXYTHETALATCfg.gridMapWidth; x++) {
    EnvNAVXYTHETALATCfg.Grid2D[x] = new unsigned char[EnvNAVXYTHETALATCfg.gridMapHeight];
  }

  // environment:
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    throw SBPL_Exception("ERROR: ran out of env file early");
  }
  strcpy(sTemp1, "environment:");
  if (strcmp(sTemp1, sTemp) != 0) {
    std::stringstream ss;
    ss << "ERROR: configuration file has incorrect format" <<
          " Expected " << sTemp1 << " got " << sTemp;
    throw SBPL_Exception(ss.str());
  }
  for (int y = 0; y < EnvNAVXYTHETALATCfg.gridMapHeight; y++) {
    for (int x = 0; x < EnvNAVXYTHETALATCfg.gridMapWidth; x++) {
      if (fscanf(fCfg, "%d", &dTemp) != 1) {
        throw SBPL_Exception("ERROR: incorrect format of config file");
      }
      EnvNAVXYTHETALATCfg.Grid2D[x][y] = dTemp;
    }
  }
}

bool LatticeEnvironment::InitializeEnv(
  const char* sEnvFile,
  const std::vector<sbpl_2Dpt_t>& perimeterptsV,
  const char* sMotPrimFile)
{
  if (sEnvFile == NULL || sMotPrimFile == NULL) return false;

  EnvNAVXYTHETALATCfg.FootprintPolygon = perimeterptsV;

  ROS_INFO("InitializeEnv start: sEnvFile=%s sMotPrimFile=%s", sEnvFile, sMotPrimFile);

  FILE* fCfg = fopen(sEnvFile, "r");
  if (fCfg == NULL) {
    std::stringstream ss;
    ss << "ERROR: unable to open " << sEnvFile;
    throw SBPL_Exception(ss.str());
  }

  ReadConfiguration(fCfg);
  fclose(fCfg);

  FILE* fMotPrim = fopen(sMotPrimFile, "r");
  if (fMotPrim == NULL) {
    std::stringstream ss;
    ss << "ERROR: unable to open " << sMotPrimFile;
    throw SBPL_Exception(ss.str());
  }
  if (ReadMotionPrimitives(fMotPrim) == false) {
    throw SBPL_Exception("ERROR: failed to read in motion primitive file");
  }

  InitGeneral(&EnvNAVXYTHETALATCfg.mprimV);
  fclose(fMotPrim);

  ROS_INFO("size of env: %d by %d", EnvNAVXYTHETALATCfg.gridMapWidth, EnvNAVXYTHETALATCfg.gridMapHeight);

  return true;
}

bool LatticeEnvironment::InitializeEnv(
  int width, int height, const unsigned char* mapdata,
  const std::vector<sbpl_2Dpt_t>& perimeterptsV,
  double cellsize_m,
  double nominalvel_mpersecs,
  double timetoturn45degsinplace_secs,
  unsigned char obsthresh,
  const char* sMotPrimFile)
{
  if (sMotPrimFile == NULL) return false;

  EnvNAVXYTHETALATCfg.obsthresh = obsthresh;
  EnvNAVXYTHETALATCfg.cellsize_m = cellsize_m;

  FILE* fMotPrim = fopen(sMotPrimFile, "r");
  if (fMotPrim == NULL) {
    std::stringstream ss;
    ss << "ERROR: unable to open " << sMotPrimFile;
    throw SBPL_Exception(ss.str());
  }

  if (ReadMotionPrimitives(fMotPrim) == false) {
    throw SBPL_Exception("ERROR: failed to read in motion primitive file");
  }
  fclose(fMotPrim);

  SetConfiguration(
    width, height, mapdata,
    nominalvel_mpersecs, timetoturn45degsinplace_secs,
    perimeterptsV);

  InitGeneral(&EnvNAVXYTHETALATCfg.mprimV);

  return true;
}

void LatticeEnvironment::SetConfiguration(
  int width, int height, const unsigned char* mapdata,
  double nominalvel_mpersecs,
  double timetoturn45degsinplace_secs,
  const std::vector<sbpl_2Dpt_t>& robot_perimeterV)
{
  EnvNAVXYTHETALATCfg.gridMapWidth = width;
  EnvNAVXYTHETALATCfg.gridMapHeight = height;

  EnvNAVXYTHETALATCfg.nominalvel_mpersecs = nominalvel_mpersecs;
  EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs = timetoturn45degsinplace_secs;
  EnvNAVXYTHETALATCfg.FootprintPolygon = robot_perimeterV;

  // unallocate the 2D environment
  if (EnvNAVXYTHETALATCfg.Grid2D != NULL) {
    for (int x = 0; x < EnvNAVXYTHETALATCfg.gridMapWidth; x++) {
      delete[] EnvNAVXYTHETALATCfg.Grid2D[x];
    }
    delete[] EnvNAVXYTHETALATCfg.Grid2D;
    EnvNAVXYTHETALATCfg.Grid2D = NULL;
  }

  // allocate the 2D environment
  EnvNAVXYTHETALATCfg.Grid2D = new unsigned char*[EnvNAVXYTHETALATCfg.gridMapWidth];
  for (int x = 0; x < EnvNAVXYTHETALATCfg.gridMapWidth; x++) {
    EnvNAVXYTHETALATCfg.Grid2D[x] = new unsigned char[EnvNAVXYTHETALATCfg.gridMapHeight];
  }

  // environment:
  if (0 == mapdata) {
    for (int y = 0; y < EnvNAVXYTHETALATCfg.gridMapHeight; y++) {
      for (int x = 0; x < EnvNAVXYTHETALATCfg.gridMapWidth; x++) {
        EnvNAVXYTHETALATCfg.Grid2D[x][y] = 0;
      }
    }
  }
  else {
    for (int y = 0; y < EnvNAVXYTHETALATCfg.gridMapHeight; y++) {
      for (int x = 0; x < EnvNAVXYTHETALATCfg.gridMapWidth; x++) {
        EnvNAVXYTHETALATCfg.Grid2D[x][y] = mapdata[x + y * width];
      }
    }
  }
}

void LatticeEnvironment::InitGeneral(std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV)
{
  // Initialize other parameters of the environment
  InitializeEnvConfig(motionprimitiveV);

  // initialize Environment
  InitializeEnvironment();

  // pre-compute heuristics
  ComputeHeuristicValues();
}

void LatticeEnvironment::InitializeEnvConfig(std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV)
{
  sbpl_xy_theta_pt_t temppose;
  temppose.x = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
  temppose.y = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
  std::vector<sbpl_2Dcell_t> footprint;
  get_2d_footprint_cells(
    EnvNAVXYTHETALATCfg.FootprintPolygon,
    &footprint,
    temppose,
    EnvNAVXYTHETALATCfg.cellsize_m);
  ROS_DEBUG("number of cells in footprint of the robot=%d", (unsigned int)footprint.size());
#if DEBUG
  for (std::vector<sbpl_2Dcell_t>::iterator it = footprint.begin(); it != footprint.end(); ++it) {
    ROS_DEBUG("Footprint cell at (%d, %d)", it->x, it->y);
  }
#endif
  PrecomputeActionswithCompleteMotionPrimitive(motionprimitiveV);
}

void LatticeEnvironment::PrecomputeActionswithCompleteMotionPrimitive(std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV)
{
  ROS_DEBUG("Pre-computing action data using motion primitives for every angle...");
  EnvNAVXYTHETALATCfg.ActionsV = new EnvNAVXYTHETALATAction_t*[EnvNAVXYTHETALATCfg.NumThetaDirs];

  if (motionprimitiveV->size() % EnvNAVXYTHETALATCfg.NumThetaDirs != 0) {
    throw SBPL_Exception("ERROR: motionprimitives should be uniform across actions");
  }

  EnvNAVXYTHETALATCfg.actionwidth = ((int)motionprimitiveV->size()) / EnvNAVXYTHETALATCfg.NumThetaDirs;

  // compute sourcepose
  sbpl_2Dpt_t sourcepose;
  sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
  sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);

  // iterate over source angles
  int maxnumofactions = 0;
  for (int tind = 0; tind < EnvNAVXYTHETALATCfg.NumThetaDirs; tind++) {
    ROS_DEBUG("pre-computing for angle %d out of %d angles", tind, EnvNAVXYTHETALATCfg.NumThetaDirs);

    EnvNAVXYTHETALATCfg.ActionsV[tind] = new EnvNAVXYTHETALATAction_t[EnvNAVXYTHETALATCfg.actionwidth];

    // iterate over motion primitives
    int numofactions = 0;
    int aind = -1;
    for (int mind = tind * EnvNAVXYTHETALATCfg.actionwidth; mind < (tind + 1) * EnvNAVXYTHETALATCfg.actionwidth; mind++) {
      //find a motion primitive for this angle
      if (motionprimitiveV->at(mind).starttheta_c != tind) {
        throw SBPL_Exception("ERROR: starttheta_c should be consistent with angle");
      }

      aind++;
      numofactions++;

      if (motionprimitiveV->at(mind).motprimID != aind) {
        throw SBPL_Exception("ERROR: motprimID should be consistent with aind");
      }

      // action index
      EnvNAVXYTHETALATCfg.ActionsV[tind][aind].aind = aind;

      // start angle
      EnvNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta = tind;

      // compute dislocation
      EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta = motionprimitiveV->at(mind).endcell.theta;  // the endtheta is in the range of [0, 16)
      EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX = motionprimitiveV->at(mind).endcell.x;
      EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY = motionprimitiveV->at(mind).endcell.y;

      // compute and store interm points as well as intersecting cells
      EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
      EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.clear();
      EnvNAVXYTHETALATCfg.ActionsV[tind][aind].interm2DcellsV.clear();

      std::set<sbpl_2Dcell_t> intermediate2DcellSet;
      // Compute all the intersected cells for this action (intermptV and interm3DcellsV)
      for (int pind = 0; pind < (int)motionprimitiveV->at(mind).intermptV.size(); pind++) {
        sbpl_xy_theta_pt_t intermpt = motionprimitiveV->at(mind).intermptV[pind];
        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.push_back(intermpt);

        // also compute the intermediate discrete cells if not there already
        sbpl_2Dpt_t pose;
        pose.x = intermpt.x + sourcepose.x;
        pose.y = intermpt.y + sourcepose.y;

        sbpl_2Dcell_t intermediate2Dcell;
        intermediate2Dcell.x = CONTXY2DISC(pose.x, EnvNAVXYTHETALATCfg.cellsize_m);
        intermediate2Dcell.y = CONTXY2DISC(pose.y, EnvNAVXYTHETALATCfg.cellsize_m);

        // add unique cells to the list
        intermediate2DcellSet.insert(intermediate2Dcell);
      }

      for (std::set<sbpl_2Dcell_t>::iterator iter = intermediate2DcellSet.begin(); iter != intermediate2DcellSet.end(); ++iter) {
        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].interm2DcellsV.push_back(*iter);
      }

      // compute linear and angular time
      double linear_distance = 0.0;
      for (int i = 1; i < (int)EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.size(); i++) {
        double x0 = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[i - 1].x;
        double y0 = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[i - 1].y;
        double x1 = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[i].x;
        double y1 = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[i].y;
        linear_distance += hypot(x1 - x0, y1 - y0);
      }
      double linear_time = linear_distance / EnvNAVXYTHETALATCfg.nominalvel_mpersecs;
      double angular_distance = fabs(computeMinUnsignedAngleDiff(
        DiscTheta2Cont(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs),
        DiscTheta2Cont(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta, EnvNAVXYTHETALATCfg.NumThetaDirs)));

      double angular_time = angular_distance / ((PI_CONST / 4.0) /
        EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs);
      // make the cost the max of the two times
      EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost = 
        (int)(ceil(NAVXYTHETALAT_COSTMULT_MTOMM * std::max(linear_time, angular_time)));
      // use any additional cost multiplier
      EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost *= motionprimitiveV->at(mind).additionalactioncostmult;

      // now compute the intersecting cells for this motion (including ignoring the source footprint)
      EnvNAVXYTHETALATCfg.PolygonBoundingBox.Add(get_2d_motion_cells(EnvNAVXYTHETALATCfg.FootprintPolygon,
                                                    motionprimitiveV->at(mind).intermptV, 
                                                    &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV, 
                                                    EnvNAVXYTHETALATCfg.cellsize_m));
    }

    if (maxnumofactions < numofactions) {
      maxnumofactions = numofactions;
    }
  }

  ROS_DEBUG("bounding box of the polygon: minx=%d, maxx=%d, miny=%d, maxy=%d", 
    EnvNAVXYTHETALATCfg.PolygonBoundingBox.GetMinimum().x,
    EnvNAVXYTHETALATCfg.PolygonBoundingBox.GetMaximum().x,
    EnvNAVXYTHETALATCfg.PolygonBoundingBox.GetMinimum().y,
    EnvNAVXYTHETALATCfg.PolygonBoundingBox.GetMaximum().y);

  // at this point we don't allow nonuniform number of actions
  if (motionprimitiveV->size() != (size_t)(EnvNAVXYTHETALATCfg.NumThetaDirs * maxnumofactions)) {
    std::stringstream ss;
    ss << "ERROR: nonuniform number of actions is not supported" <<
      " (maxnumofactions=" << maxnumofactions << " while motprims=" <<
      motionprimitiveV->size() << " thetas=" <<
      EnvNAVXYTHETALATCfg.NumThetaDirs;
    throw SBPL_Exception(ss.str());
  }

  ROS_DEBUG("done pre-computing action data based on motion primitives");
}

void LatticeEnvironment::InitializeEnvironment()
{
  EnvNAVXYTHETALATHashEntry_t* HashEntry;
  int maxsize = EnvNAVXYTHETALATCfg.gridMapWidth * EnvNAVXYTHETALATCfg.gridMapHeight * EnvNAVXYTHETALATCfg.NumThetaDirs;

  Coord2StateIDHashTable_lookup = new EnvNAVXYTHETALATHashEntry_t*[maxsize];
  for (int i = 0; i < maxsize; i++) {
    Coord2StateIDHashTable_lookup[i] = NULL;
  }

  // initialize the map from StateID to Coord
  StateID2CoordTable.clear();
  StateID2IndexMapping.clear();

  EnvNAVXYTHETALAT.bInitialized = true;
}

void LatticeEnvironment::ComputeHeuristicValues()
{
  // whatever necessary pre-computation of heuristic values is done here
  ROS_DEBUG("Pre-computing heuristics...");

  // allocated 2D grid searches
  grid2Dsearchfromgoal = new Grid2DSearch(
    EnvNAVXYTHETALATCfg.gridMapWidth, EnvNAVXYTHETALATCfg.gridMapHeight, 
    EnvNAVXYTHETALATCfg.cellsize_m, EnvNAVXYTHETALATCfg.nominalvel_mpersecs);

  ROS_DEBUG("done pre-computing heuristics");
}

EnvNAVXYTHETALATHashEntry_t* LatticeEnvironment::GetHashEntry_lookup(int gridX, int gridY, int gridTheta) const
{
  if (gridX < 0 || gridX >= EnvNAVXYTHETALATCfg.gridMapWidth ||
      gridY < 0 || gridY >= EnvNAVXYTHETALATCfg.gridMapHeight ||
      gridTheta < 0 || gridTheta >= EnvNAVXYTHETALATCfg.NumThetaDirs)
  {
    return NULL;
  }
  int index = GetGridIndex(gridX, gridY, gridTheta);
  return Coord2StateIDHashTable_lookup[index];
}

EnvNAVXYTHETALATHashEntry_t* LatticeEnvironment::CreateNewHashEntry_lookup(int gridX, int gridY, int gridTheta)
{
  EnvNAVXYTHETALATHashEntry_t* HashEntry = new EnvNAVXYTHETALATHashEntry_t;

  HashEntry->gridX = gridX;
  HashEntry->gridY = gridY;
  HashEntry->gridTheta = gridTheta;

  HashEntry->stateID = (int)StateID2CoordTable.size();

  // insert into the tables
  StateID2CoordTable.push_back(HashEntry);

  int index = GetGridIndex(gridX, gridY, gridTheta);

  if (Coord2StateIDHashTable_lookup[index] != NULL) {
    throw SBPL_Exception("ERROR: creating hash entry for non-NULL hashentry");
  }

  Coord2StateIDHashTable_lookup[index] = HashEntry;

  // insert into and initialize the mappings
  StateID2IndexMapping.push_back(-1);

  if (HashEntry->stateID != (int)StateID2IndexMapping.size() - 1) {
    throw SBPL_Exception("ERROR in Env... function: last state has incorrect stateID");
  }

  return HashEntry;
}

bool LatticeEnvironment::ReadMotionPrimitives(FILE* fMotPrims)
{
  char sTemp[1024], sExpected[1024];
  float fTemp;
  int dTemp;
  int totalNumofActions = 0;

  ROS_DEBUG("Reading in motion primitives...");

  // resolution_m:
  strcpy(sExpected, "resolution_m:");
  if (fscanf(fMotPrims, "%s", sTemp) == 0) {
    return false;
  }
  if (strcmp(sTemp, sExpected) != 0) {
    ROS_ERROR("ERROR: expected %s but got %s", sExpected, sTemp);
    return false;
  }
  if (fscanf(fMotPrims, "%f", &fTemp) == 0) {
    return false;
  }
  if (fabs(fTemp - EnvNAVXYTHETALATCfg.cellsize_m) > ERR_EPS) {
    ROS_ERROR("ERROR: invalid resolution %f (instead of %f) in the dynamics file", fTemp, EnvNAVXYTHETALATCfg.cellsize_m);
    return false;
  }
  ROS_INFO("resolution_m: %f", fTemp);

  // numberofangles:
  strcpy(sExpected, "numberofangles:");
  if (fscanf(fMotPrims, "%s", sTemp) == 0) {
    return false;
  }
  if (strcmp(sTemp, sExpected) != 0) {
    ROS_ERROR("ERROR: expected %s but got %s", sExpected, sTemp);
    return false;
  }
  if (fscanf(fMotPrims, "%d", &dTemp) == 0) {
    return false;
  }
  EnvNAVXYTHETALATCfg.NumThetaDirs = dTemp;
  ROS_INFO("numberofangles: %d", dTemp);
  
  // totalnumberofprimitives:
  strcpy(sExpected, "totalnumberofprimitives:");
  if (fscanf(fMotPrims, "%s", sTemp) == 0) {
    return false;
  }
  if (strcmp(sTemp, sExpected) != 0) {
    ROS_ERROR("ERROR: expected %s but got %s", sExpected, sTemp);
    return false;
  }
  if (fscanf(fMotPrims, "%d", &totalNumofActions) == 0) {
    return false;
  }
  ROS_INFO("totalnumberofprimitives: %d", totalNumofActions);

  // Read in motion primitive for each action
  for (int i = 0; i < totalNumofActions; i++) {
    SBPL_xytheta_mprimitive motprim;

    if (!ReadinMotionPrimitive(&motprim, fMotPrims)) {
      return false;
    }

    EnvNAVXYTHETALATCfg.mprimV.push_back(motprim);
  }
  ROS_DEBUG("done reading in motion primitives");
  return true;
}

bool LatticeEnvironment::ReadinMotionPrimitive(SBPL_xytheta_mprimitive* pMotPrim, FILE* fIn)
{
  char sTemp[1024];
  int dTemp;
  char sExpected[1024];
  int numofIntermPoses;

  // primID:
  strcpy(sExpected, "primID:");
  if (fscanf(fIn, "%s", sTemp) == 0) {
    return false;
  }
  if (strcmp(sTemp, sExpected) != 0) {
    ROS_ERROR("ERROR: expected %s but got %s", sExpected, sTemp);
    return false;
  }
  if (fscanf(fIn, "%d", &pMotPrim->motprimID) != 1) {
    return false;
  }

  // startangle_c:
  strcpy(sExpected, "startangle_c:");
  if (fscanf(fIn, "%s", sTemp) == 0) {
    return false;
  }
  if (strcmp(sTemp, sExpected) != 0) {
    ROS_ERROR("ERROR: expected %s but got %s", sExpected, sTemp);
    return false;
  }
  if (fscanf(fIn, "%d", &dTemp) == 0) {
    ROS_ERROR("ERROR: reading startangle");
    return false;
  }
  pMotPrim->starttheta_c = dTemp;

  // endpose_c:
  strcpy(sExpected, "endpose_c:");
  if (fscanf(fIn, "%s", sTemp) == 0) {
    return false;
  }
  if (strcmp(sTemp, sExpected) != 0) {
    ROS_ERROR("ERROR: expected %s but got %s", sExpected, sTemp);
    return false;
  }

  if (ReadinCell(&pMotPrim->endcell, fIn) == false) {
    ROS_ERROR("ERROR: failed to read in endsearchpose");
    return false;
  }

  // additionalactioncostmult:
  strcpy(sExpected, "additionalactioncostmult:");
  if (fscanf(fIn, "%s", sTemp) == 0) {
    return false;
  }
  if (strcmp(sTemp, sExpected) != 0) {
    ROS_ERROR("ERROR: expected %s but got %s", sExpected, sTemp);
    return false;
  }
  if (fscanf(fIn, "%d", &dTemp) != 1) {
    return false;
  }
  pMotPrim->additionalactioncostmult = dTemp;

  // intermediateposes:
  strcpy(sExpected, "intermediateposes:");
  if (fscanf(fIn, "%s", sTemp) == 0) {
    return false;
  }
  if (strcmp(sTemp, sExpected) != 0) {
    ROS_ERROR("ERROR: expected %s but got %s", sExpected, sTemp);
    return false;
  }
  if (fscanf(fIn, "%d", &numofIntermPoses) != 1) {
    return false;
  }
  // all intermposes should be with respect to 0,0 as starting pose since it
  // will be added later and should be done after the action is rotated by
  // initial orientation
  for (int i = 0; i < numofIntermPoses; i++) {
    sbpl_xy_theta_pt_t intermpose;
    if (ReadinPose(&intermpose, fIn) == false) {
      ROS_ERROR("ERROR: failed to read in intermediate poses");
      return false;
    }
    pMotPrim->intermptV.push_back(intermpose);
  }

  // Check that the last pose of the motion matches (within lattice
  // resolution) the designated end pose of the primitive
  sbpl_xy_theta_pt_t sourcepose;
  sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
  sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
  sourcepose.theta = DiscTheta2Cont(pMotPrim->starttheta_c, EnvNAVXYTHETALATCfg.NumThetaDirs);
  double mp_endx_m = sourcepose.x + pMotPrim->intermptV.back().x;
  double mp_endy_m = sourcepose.y + pMotPrim->intermptV.back().y;
  double mp_endtheta_rad = pMotPrim->intermptV.back().theta;

  int endx_c = CONTXY2DISC(mp_endx_m, EnvNAVXYTHETALATCfg.cellsize_m);
  int endy_c = CONTXY2DISC(mp_endy_m, EnvNAVXYTHETALATCfg.cellsize_m);
  int endtheta_c = ContTheta2Disc(mp_endtheta_rad, EnvNAVXYTHETALATCfg.NumThetaDirs);
  if (endx_c != pMotPrim->endcell.x ||
    endy_c != pMotPrim->endcell.y ||
    endtheta_c != pMotPrim->endcell.theta)
  {
    ROS_ERROR("ERROR: incorrect primitive %d with startangle=%d "
      "last interm point %f %f %f does not match end pose %d %d %d",
      pMotPrim->motprimID, pMotPrim->starttheta_c,
      pMotPrim->intermptV.back().x,
      pMotPrim->intermptV.back().y,
      pMotPrim->intermptV.back().theta,
      pMotPrim->endcell.x, pMotPrim->endcell.y,
      pMotPrim->endcell.theta);
    return false;
  }

  return true;
}

bool LatticeEnvironment::ReadinCell(sbpl_xy_theta_cell_t* cell, FILE* fIn)
{
  char sTemp[60];

  if (fscanf(fIn, "%s", sTemp) == 0) {
    return false;
  }
  cell->x = atoi(sTemp);
  if (fscanf(fIn, "%s", sTemp) == 0) {
    return false;
  }
  cell->y = atoi(sTemp);
  if (fscanf(fIn, "%s", sTemp) == 0) {
    return false;
  }
  cell->theta = atoi(sTemp);

  // normalize the angle
  cell->theta = NORMALIZEDISCTHETA(cell->theta, EnvNAVXYTHETALATCfg.NumThetaDirs);
  if (cell->theta < 0 || cell->theta >= EnvNAVXYTHETALATCfg.NumThetaDirs) {
    ROS_ERROR("ERROR: the discrete angle should be greater than or equal to 0 and less than EnvNAVXYTHETALATCfg.NumThetaDirs");
    return false;
  }

  return true;
}

bool LatticeEnvironment::ReadinPose(sbpl_xy_theta_pt_t* pose, FILE* fIn)
{
  char sTemp[60];

  if (fscanf(fIn, "%s", sTemp) == 0) {
    return false;
  }
  pose->x = atof(sTemp);
  if (fscanf(fIn, "%s", sTemp) == 0) {
    return false;
  }
  pose->y = atof(sTemp);
  if (fscanf(fIn, "%s", sTemp) == 0) {
    return false;
  }
  pose->theta = atof(sTemp);

  pose->theta = normalizeAngle(pose->theta);

  return true;
}

bool LatticeEnvironment::SetEnvParameter(const char* parameter, int value)
{
  if (EnvNAVXYTHETALAT.bInitialized) {
    ROS_ERROR("ERROR: all parameters must be set before initialization of the environment");
    return false;
  }

  ROS_INFO("setting parameter %s to %d", parameter, value);

  if (strcmp(parameter, "cost_inscribed_thresh") == 0) {
    if (value < 0 || value > 255) {
      ROS_ERROR("ERROR: invalid value %d for parameter %s", value, parameter);
      return false;
    }
    EnvNAVXYTHETALATCfg.cost_inscribed_thresh = (unsigned char)value;
  }
  else if (strcmp(parameter, "cost_possibly_circumscribed_thresh") == 0) {
    if (value < 0 || value > 255) {
      ROS_ERROR("ERROR: invalid value %d for parameter %s", value, parameter);
      return false;
    }
    EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh = value;
  }
  else if (strcmp(parameter, "cost_obsthresh") == 0) {
    if (value < 0 || value > 255) {
      ROS_ERROR("ERROR: invalid value %d for parameter %s", value, parameter);
      return false;
    }
    EnvNAVXYTHETALATCfg.obsthresh = (unsigned char)value;
  }
  else {
    ROS_ERROR("ERROR: invalid parameter %s", parameter);
    return false;
  }

  return true;
}

bool LatticeEnvironment::IsValidConfiguration(int gridX, int gridY, int gridTheta) const
{
  std::vector<sbpl_2Dcell_t> footprint;
  sbpl_xy_theta_pt_t pose;

  // compute continuous pose
  pose.x = DISCXY2CONT(gridX, EnvNAVXYTHETALATCfg.cellsize_m);
  pose.y = DISCXY2CONT(gridY, EnvNAVXYTHETALATCfg.cellsize_m);
  pose.theta = DiscTheta2Cont(gridTheta, EnvNAVXYTHETALATCfg.NumThetaDirs);

  // compute footprint cells
  get_2d_footprint_cells(
    EnvNAVXYTHETALATCfg.FootprintPolygon,
    &footprint,
    pose,
    EnvNAVXYTHETALATCfg.cellsize_m);

  // iterate over all footprint cells
  for (int find = 0; find < (int)footprint.size(); find++) {
    int x = footprint.at(find).x;
    int y = footprint.at(find).y;

    if (!IsWithinMapCell(x, y) ||
        EnvNAVXYTHETALATCfg.Grid2D[x][y] >= EnvNAVXYTHETALATCfg.obsthresh)
    {
      return false;
    }
  }

  return true;
}

int LatticeEnvironment::SetStart(double worldX, double worldY, double worldTheta)
{
  int gridX = CONTXY2DISC(worldX, EnvNAVXYTHETALATCfg.cellsize_m);
  int gridY = CONTXY2DISC(worldY, EnvNAVXYTHETALATCfg.cellsize_m);
  int gridTheta = ContTheta2Disc(worldTheta, EnvNAVXYTHETALATCfg.NumThetaDirs);

  if (!IsWithinMapCell(gridX, gridY)) {
    ROS_ERROR("ERROR: trying to set a start cell %d %d that is outside of map", gridX, gridY);
    return -1;
  }

  if (EnvNAVXYTHETALATCfg.Grid2D[gridX][gridY] >= EnvNAVXYTHETALATCfg.cost_inscribed_thresh) {
    ROS_ERROR("ERROR: trying to set a start cell %d %d that is invalid", gridX, gridY);
    return -1;
  }

  if (EnvNAVXYTHETALATCfg.FootprintPolygon.size() > 1 && 
    (int)EnvNAVXYTHETALATCfg.Grid2D[gridX][gridY] >= EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh) 
  {
    if (!IsValidConfiguration(gridX, gridY, gridTheta)) {
      ROS_ERROR("ERROR: start configuration %d %d %d is invalid", gridX, gridY, gridTheta);
      return -1;
    }
  }

  ROS_DEBUG("env: setting start to %.3f %.3f %.3f (%d %d %d)", worldX, worldY, worldTheta, gridX, gridY, gridTheta);

  EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
  if (NULL == (OutHashEntry = GetHashEntry_lookup(gridX, gridY, gridTheta))) {
    // have to create a new entry
    OutHashEntry = CreateNewHashEntry_lookup(gridX, gridY, gridTheta);
  }

  if (EnvNAVXYTHETALAT.startStateID != OutHashEntry->stateID) {
    // because goal heuristics change
    bNeedtoRecomputeGoalHeuristics = true;

    // set start
    EnvNAVXYTHETALAT.startStateID = OutHashEntry->stateID;
    EnvNAVXYTHETALATCfg.startGridX = gridX;
    EnvNAVXYTHETALATCfg.startGridY = gridY;
    EnvNAVXYTHETALATCfg.startGridTheta = gridTheta;
  }

  return EnvNAVXYTHETALAT.startStateID;
}

int LatticeEnvironment::SetGoal(double worldX, double worldY, double worldTheta)
{
  int gridX = CONTXY2DISC(worldX, EnvNAVXYTHETALATCfg.cellsize_m);
  int gridY = CONTXY2DISC(worldY, EnvNAVXYTHETALATCfg.cellsize_m);
  int gridTheta = ContTheta2Disc(worldTheta, EnvNAVXYTHETALATCfg.NumThetaDirs);

  if (!IsWithinMapCell(gridX, gridY)) {
    ROS_ERROR("ERROR: trying to set a goal cell %d %d that is outside of map", gridX, gridY);
    return -1;
  }

  if (EnvNAVXYTHETALATCfg.Grid2D[gridX][gridY] >= EnvNAVXYTHETALATCfg.cost_inscribed_thresh) {
    ROS_ERROR("ERROR: trying to set a goal cell %d %d that is invalid", gridX, gridY);
    return -1;
  }

  if (EnvNAVXYTHETALATCfg.FootprintPolygon.size() > 1 && 
    (int)EnvNAVXYTHETALATCfg.Grid2D[gridX][gridY] >= EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh) 
  {
    if (!IsValidConfiguration(gridX, gridY, gridTheta)) {
      ROS_ERROR("ERROR: goal configuration %d %d %d is invalid", gridX, gridY, gridTheta);
      return -1;
    }
  }  

  ROS_DEBUG("env: setting goal to %.3f %.3f %.3f (%d %d %d)", worldX, worldY, worldTheta, gridX, gridY, gridTheta);

  EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
  if (NULL == (OutHashEntry = GetHashEntry_lookup(gridX, gridY, gridTheta))) {
    // have to create a new entry
    OutHashEntry = CreateNewHashEntry_lookup(gridX, gridY, gridTheta);
  }

  if (EnvNAVXYTHETALAT.goalStateID != OutHashEntry->stateID) {
    // because goal heuristics change
    bNeedtoRecomputeGoalHeuristics = true;

    // set goal
    EnvNAVXYTHETALAT.goalStateID = OutHashEntry->stateID;
    EnvNAVXYTHETALATCfg.goalGridX = gridX;
    EnvNAVXYTHETALATCfg.goalGridY = gridY;
    EnvNAVXYTHETALATCfg.goalGridTheta = gridTheta;
  }

  return EnvNAVXYTHETALAT.goalStateID;
}

bool LatticeEnvironment::SetMap(const unsigned char* mapdata)
{
  int xind = -1, yind = -1;

  for (xind = 0; xind < EnvNAVXYTHETALATCfg.gridMapWidth; xind++) {
    for (yind = 0; yind < EnvNAVXYTHETALATCfg.gridMapHeight; yind++) {
      EnvNAVXYTHETALATCfg.Grid2D[xind][yind] = mapdata[xind + yind * EnvNAVXYTHETALATCfg.gridMapWidth];
    }
  }

  bNeedtoRecomputeGoalHeuristics = true;

  return true;
}

void LatticeEnvironment::EnsureHeuristicsUpdated()
{
  if (bNeedtoRecomputeGoalHeuristics) {
    grid2Dsearchfromgoal->search(EnvNAVXYTHETALATCfg.Grid2D, EnvNAVXYTHETALATCfg.cost_inscribed_thresh,
                                EnvNAVXYTHETALATCfg.goalGridX, EnvNAVXYTHETALATCfg.goalGridY,
                                EnvNAVXYTHETALATCfg.startGridX, EnvNAVXYTHETALATCfg.startGridY,
                                SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH);
  }
  
  bNeedtoRecomputeGoalHeuristics = false;
}

int LatticeEnvironment::GetGoalHeuristic(int stateID) const
{
  if (stateID >= (int)StateID2CoordTable.size()) {
    throw SBPL_Exception("ERROR in GetGoalHeuristic: stateID illegal");
  }

  EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
  // computes time cost from start state that is grid2D, so it is EndX_c EndY_c
  int h2D = grid2Dsearchfromgoal->getlowerboundoncostfromstart_inms(HashEntry->gridX, HashEntry->gridY);

  // define this function if it is used in the planner (heuristic backward search would use it)
  return h2D;
}

void LatticeEnvironment::GetSuccs(
  int SourceStateID, 
  std::vector<int>* SuccIDV, 
  std::vector<int>* CostV,
  std::vector<EnvNAVXYTHETALATAction_t*>* actionV)
{
  // clear the successor array
  SuccIDV->clear();
  CostV->clear();
  if (actionV != NULL) {
    actionV->clear();
  }

  // goal state should be absorbing
  if (SourceStateID == EnvNAVXYTHETALAT.goalStateID) return;

  // get X, Y for the state
  EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];

  // iterate through actions
  for (int aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++) 
  {
    EnvNAVXYTHETALATAction_t* nav3daction = &EnvNAVXYTHETALATCfg.ActionsV[(unsigned int)HashEntry->gridTheta][aind];
    int newX = HashEntry->gridX + nav3daction->dX;
    int newY = HashEntry->gridY + nav3daction->dY;
    int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);

    // get cost
    int cost = GetActionCost(HashEntry->gridX, HashEntry->gridY, HashEntry->gridTheta, nav3daction);
    if (cost >= INFINITECOST) {
      continue;
    }

    EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
    if (NULL == (OutHashEntry = GetHashEntry_lookup(newX, newY, newTheta))) {
      // have to create a new entry
      OutHashEntry = CreateNewHashEntry_lookup(newX, newY, newTheta);
    }

    SuccIDV->push_back(OutHashEntry->stateID);
    CostV->push_back(cost);
    if (actionV != NULL) {
      actionV->push_back(nav3daction);
    }
  }
}

int LatticeEnvironment::GetActionCost(
  int SourceX, int SourceY, int SourceTheta, 
  EnvNAVXYTHETALATAction_t* action)
{
  sbpl_2Dcell_t cell;
  sbpl_2Dcell_t interm2Dcell;
  int i;

  // TODO - go over bounding box (minpt and maxpt) to test validity and skip
  // testing boundaries below, also order intersect cells so that the four
  // farthest pts go first

  if (!IsWithinMapCell(SourceX, SourceY)) {
    throw SBPL_Exception("ERROR is GetActionCost: source pose is outside of map");
    return INFINITECOST;
  }

  if (EnvNAVXYTHETALATCfg.Grid2D[SourceX][SourceY] >= EnvNAVXYTHETALATCfg.cost_inscribed_thresh) {
    throw SBPL_Exception("ERROR in GetActionCost: source pose is invalid");
    return INFINITECOST;
  }

  if (!IsWithinMapCell(SourceX + action->dX, SourceY + action->dY)) {
    return INFINITECOST;
  }

  if (EnvNAVXYTHETALATCfg.Grid2D[SourceX + action->dX][SourceY + action->dY] >=
    EnvNAVXYTHETALATCfg.cost_inscribed_thresh)
  {
    return INFINITECOST;
  }

  // need to iterate over discretized center cells and compute cost based on them
  unsigned char maxcellcost = 0;
  for (i = 0; i < (int)action->interm2DcellsV.size(); i++) {
    interm2Dcell = action->interm2DcellsV.at(i);
    interm2Dcell.x = interm2Dcell.x + SourceX;
    interm2Dcell.y = interm2Dcell.y + SourceY;

    if (!IsWithinMapCell(interm2Dcell.x, interm2Dcell.y)) {
      return INFINITECOST;
    }

    maxcellcost = std::max(maxcellcost, EnvNAVXYTHETALATCfg.Grid2D[interm2Dcell.x][interm2Dcell.y]);

    // check that the robot is NOT in the cell at which there is no valid orientation
    if (maxcellcost >= EnvNAVXYTHETALATCfg.cost_inscribed_thresh) {
      return INFINITECOST;
    }
  }

  // Jian Wen
  // check collisions that for the particular footprint orientation along the action
  if (EnvNAVXYTHETALATCfg.FootprintPolygon.size() > 1 &&
    (int)maxcellcost >= EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh)
  {
    for (i = 0; i < (int)action->intersectingcellsV.size(); i++) {
      // get the cell in the map
      cell = action->intersectingcellsV.at(i);
      cell.x = cell.x + SourceX;
      cell.y = cell.y + SourceY;

      // check validity
      if (!IsWithinMapCell(cell.x, cell.y) || 
          EnvNAVXYTHETALATCfg.Grid2D[cell.x][cell.y] >= EnvNAVXYTHETALATCfg.obsthresh) 
      {
        // if ((int)maxcellcost < EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh) {
        //   printf("source x = %d, y = %d; cell x = %d, y = %d\n", SourceX, SourceY, cell.x, cell.y);
        // }

        return INFINITECOST;
      }
    }
  }

  // to ensure consistency of h2D:
  maxcellcost = std::max(maxcellcost, EnvNAVXYTHETALATCfg.Grid2D[SourceX][SourceY]);
  int currentmaxcost = (int)std::max(
    maxcellcost,
    EnvNAVXYTHETALATCfg.Grid2D[SourceX + action->dX][SourceY + action->dY]);

  // use cell cost as multiplicative factor
  return action->cost * (currentmaxcost + 1);
  // return action->cost;
}

void LatticeEnvironment::ConvertStateIDPathintoXYThetaPath(
  std::vector<int>* stateIDPath,
  std::vector<sbpl_xy_theta_pt_t>* xythetaPath)
{
  std::vector<EnvNAVXYTHETALATAction_t*> actionV;
  std::vector<int> CostV;
  std::vector<int> SuccIDV;
  int targetx_c, targety_c, targettheta_c;
  int sourcex_c, sourcey_c, sourcetheta_c;

  xythetaPath->clear();

  for (int pind = 0; pind < (int)(stateIDPath->size()) - 1; pind++) {
    int sourceID = stateIDPath->at(pind);
    int targetID = stateIDPath->at(pind + 1);

    // get successors and pick the target via the cheapest action
    SuccIDV.clear();
    CostV.clear();
    actionV.clear();
    GetSuccs(sourceID, &SuccIDV, &CostV, &actionV);

    int bestcost = INFINITECOST;
    int bestsind = -1;

    for (int sind = 0; sind < (int)SuccIDV.size(); sind++) {
      if (SuccIDV[sind] == targetID && CostV[sind] <= bestcost) {
        bestcost = CostV[sind];
        bestsind = sind;
      }
    }
    if (bestsind == -1) {
      ROS_ERROR("ERROR: successor not found for transition");
      GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
      GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c);
      ROS_ERROR("%d %d %d -> %d %d %d", sourcex_c, sourcey_c, sourcetheta_c, targetx_c, targety_c, targettheta_c);
      throw SBPL_Exception("ERROR: successor not found for transition");
    }

    // now push in the actual path
    GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
    double sourcex, sourcey;
    sourcex = DISCXY2CONT(sourcex_c, EnvNAVXYTHETALATCfg.cellsize_m);
    sourcey = DISCXY2CONT(sourcey_c, EnvNAVXYTHETALATCfg.cellsize_m);
    // TODO - when there are no motion primitives we should still print source state
    for (int ipind = 0; ipind < ((int)actionV[bestsind]->intermptV.size()) - 1; ipind++) {
      // translate appropriately
      sbpl_xy_theta_pt_t intermpt = actionV[bestsind]->intermptV[ipind];
      intermpt.x += sourcex;
      intermpt.y += sourcey;

      // store
      xythetaPath->push_back(intermpt);
    }
  }
}

void LatticeEnvironment::GetCoordFromState(int stateID, int& gridX, int& gridY, int& gridTheta) const
{
  EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
  gridX = HashEntry->gridX;
  gridY = HashEntry->gridY;
  gridTheta = HashEntry->gridTheta;
}

}  // namespace lattice_path_planner