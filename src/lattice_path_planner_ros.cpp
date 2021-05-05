#include <pluginlib/class_list_macros.hpp>
#include <nav_msgs/Path.h>

#include <costmap_2d/inflation_layer.h>
#include <tf2/LinearMath/Quaternion.h>

#include <string.h>

#include "lattice_path_planner/lattice_path_planner_ros.h"

PLUGINLIB_EXPORT_CLASS(lattice_path_planner::LatticePathPlannerROS, nav_core::BaseGlobalPlanner)

namespace lattice_path_planner
{
LatticePathPlannerROS::LatticePathPlannerROS() : initialized_(false), costmap_ros_(NULL), planner_(NULL), env_(NULL)
{
}

LatticePathPlannerROS::LatticePathPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : initialized_(false), costmap_ros_(NULL), planner_(NULL), env_(NULL)
{
  initialize(name, costmap_ros);
}

LatticePathPlannerROS::~LatticePathPlannerROS()
{
  if (planner_)
  {
    delete planner_;
    planner_ = NULL;
  }

  if (env_)
  {
    delete env_;
    env_ = NULL;
  }
}

void LatticePathPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    ros::NodeHandle private_nh("~/" + name);

    ROS_DEBUG("Name is %s", name.c_str());

    if (!private_nh.getParam("primitive_filename", primitive_filename_))
    {
      ROS_ERROR("Failed to set primitive_filename");
      exit(1);
    }

    private_nh.param("force_scratch_limit", force_scratch_limit_, 500);

    double nominalvel_mpersecs, timetoturn45degsinplace_secs;
    private_nh.param("nominalvel_mpersecs", nominalvel_mpersecs, 0.4);
    private_nh.param("timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 0.6);

    name_ = name;
    costmap_ros_ = costmap_ros;

    footprint_ = costmap_ros_->getRobotFootprint();

    env_ = new LatticeEnvironment();

    circumscribed_cost_ = computeCircumscribedCost();

    if (circumscribed_cost_ == 0)
    {
      // Unfortunately, the inflation_radius is not taken into account by
      // inflation_layer->computeCost(). If inflation_radius is smaller than
      // the circumscribed radius, SBPL will ignore some obstacles, but we
      // cannot detect this problem. If the cost_scaling_factor is too large,
      // SBPL won't run into obstacles, but will always perform an expensive
      // footprint check, no matter how far the nearest obstacle is.
      ROS_WARN("The costmap value at the robot's circumscribed radius (%f m) is 0.",
               costmap_ros_->getLayeredCostmap()->getCircumscribedRadius());
      ROS_WARN("SBPL performance will suffer.");
      ROS_WARN("Please decrease the costmap's cost_scaling_factor.");
    }
    if (!env_->SetEnvParameter("cost_inscribed_thresh", costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
    {
      ROS_ERROR("Failed to set cost_inscribed_thresh parameter");
      exit(1);
    }
    if (!env_->SetEnvParameter("cost_possibly_circumscribed_thresh", circumscribed_cost_))
    {
      ROS_ERROR("Failed to set cost_possibly_circumscribed_thresh parameter");
      exit(1);
    }

    std::vector<sbpl_2Dpt_t> perimeterptsV;
    perimeterptsV.reserve(footprint_.size());
    for (int i = 0; i < (int)footprint_.size(); ++i)
    {
      sbpl_2Dpt_t pt;
      pt.x = footprint_[i].x;
      pt.y = footprint_[i].y;
      perimeterptsV.push_back(pt);
    }

    bool ret;
    try
    {
      ret = env_->InitializeEnv(costmap_ros_->getCostmap()->getSizeInCellsX(),  // width
                                costmap_ros_->getCostmap()->getSizeInCellsY(),  // height
                                0,                                              // mapdata
                                perimeterptsV, costmap_ros_->getCostmap()->getResolution(), nominalvel_mpersecs,
                                timetoturn45degsinplace_secs, costmap_2d::LETHAL_OBSTACLE, primitive_filename_.c_str());
      current_env_width_ = costmap_ros_->getCostmap()->getSizeInCellsX();
      current_env_height_ = costmap_ros_->getCostmap()->getSizeInCellsY();
    }
    catch (SBPL_Exception* e)
    {
      ROS_ERROR("SBPL encountered a fatal exception: %s", e->what());
      ret = false;
    }
    if (!ret)
    {
      ROS_ERROR("SBPL initialization failed!");
      exit(1);
    }

    // update mapdata
    env_->SetMap(costmap_ros_->getCostmap()->getCharMap());

    planner_ = new LatticePathPlanner(env_);

    ROS_INFO("[lattice_path_planner] Initialized successfully");
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);

    initialized_ = true;
  }
}

unsigned char LatticePathPlannerROS::computeCircumscribedCost()
{
  unsigned char result = 0;

  if (!costmap_ros_)
  {
    ROS_ERROR("Costmap is not initialized");
    return 0;
  }

  // check if the costmap has an inflation layer
  for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::const_iterator layer =
           costmap_ros_->getLayeredCostmap()->getPlugins()->begin();
       layer != costmap_ros_->getLayeredCostmap()->getPlugins()->end(); ++layer)
  {
    boost::shared_ptr<costmap_2d::InflationLayer> inflation_layer =
        boost::dynamic_pointer_cast<costmap_2d::InflationLayer>(*layer);
    if (!inflation_layer)
      continue;

    result = inflation_layer->computeCost(costmap_ros_->getLayeredCostmap()->getCircumscribedRadius() /
                                          costmap_ros_->getCostmap()->getResolution());
  }
  return result;
}

bool LatticePathPlannerROS::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                     std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("lattice_path_planner is not initialized");
    return false;
  }

  bool do_init = false;
  if (current_env_width_ != costmap_ros_->getCostmap()->getSizeInCellsX() ||
      current_env_height_ != costmap_ros_->getCostmap()->getSizeInCellsY())
  {
    ROS_INFO("Costmap dimensions have changed from (%d x %d) to (%d x %d), reinitializing lattice_path_planner.",
             current_env_width_, current_env_height_, costmap_ros_->getCostmap()->getSizeInCellsX(),
             costmap_ros_->getCostmap()->getSizeInCellsY());
    do_init = true;
  }
  else if (footprint_ != costmap_ros_->getRobotFootprint())
  {
    ROS_INFO("Robot footprint has changed, reinitializing lattice_path_planner.");
    do_init = true;
  }
  else if (circumscribed_cost_ != computeCircumscribedCost())
  {
    ROS_INFO("Cost at circumscribed radius has changed, reinitializing lattice_path_planner.");
    do_init = true;
  }

  if (do_init)
  {
    initialized_ = false;
    delete planner_;
    planner_ = NULL;
    delete env_;
    env_ = NULL;
    initialize(name_, costmap_ros_);
  }

  plan.clear();

  ROS_INFO("[lattice_path_planner] getting start point (%g,%g) goal point (%g,%g)", start.pose.position.x,
           start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
  double theta_start = 2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
  double theta_goal = 2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);

  try
  {
    int ret = env_->SetStart(start.pose.position.x - costmap_ros_->getCostmap()->getOriginX(),
                             start.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_start);
    if (ret < 0 || planner_->SetSearchStartState(ret) == 0)
    {
      ROS_ERROR("ERROR: failed to set start state");
      return false;
    }
  }
  catch (SBPL_Exception* e)
  {
    ROS_ERROR("SBPL encountered a fatal exception while setting the start state");
    return false;
  }

  try
  {
    int ret = env_->SetGoal(goal.pose.position.x - costmap_ros_->getCostmap()->getOriginX(),
                            goal.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_goal);
    if (ret < 0 || planner_->SetSearchGoalState(ret) == 0)
    {
      ROS_ERROR("ERROR: failed to set goal state");
      return false;
    }
  }
  catch (SBPL_Exception* e)
  {
    ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
    return false;
  }

  // update mapdata
  env_->SetMap(costmap_ros_->getCostmap()->getCharMap());

  ROS_INFO("[lattice_path_planner] run planner");
  std::vector<int> solution_stateIDs;
  int solution_cost;
  try
  {
    int ret = planner_->replan(&solution_stateIDs, &solution_cost);
    if (ret)
      ROS_INFO("Solution is found");
    else
    {
      ROS_INFO("Solution not found");
      return false;
    }
  }
  catch (SBPL_Exception* e)
  {
    ROS_ERROR("SBPL encountered a fatal exception while planning");
    return false;
  }

  ROS_DEBUG("size of solution=%d", (int)solution_stateIDs.size());

  std::vector<sbpl_xy_theta_pt_t> sbpl_path;
  try
  {
    env_->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path);
  }
  catch (SBPL_Exception* e)
  {
    ROS_ERROR("SBPL encountered a fatal exception while reconstructing the path");
    return false;
  }
  // if the plan has zero points, add a single point to make move_base happy
  if (sbpl_path.empty())
  {
    sbpl_xy_theta_pt_t s(start.pose.position.x - costmap_ros_->getCostmap()->getOriginX(),
                         start.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_start);
    sbpl_path.push_back(s);
  }

  ROS_DEBUG("Plan has %d points.", (int)sbpl_path.size());
  ros::Time plan_time = ros::Time::now();

  // create a message for the plan
  nav_msgs::Path gui_path;
  gui_path.poses.resize(sbpl_path.size());
  gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
  gui_path.header.stamp = plan_time;
  for (unsigned int i = 0; i < sbpl_path.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = costmap_ros_->getGlobalFrameID();

    pose.pose.position.x = sbpl_path[i].x + costmap_ros_->getCostmap()->getOriginX();
    pose.pose.position.y = sbpl_path[i].y + costmap_ros_->getCostmap()->getOriginY();
    pose.pose.position.z = start.pose.position.z;

    tf2::Quaternion temp;
    temp.setRPY(0, 0, sbpl_path[i].theta);
    pose.pose.orientation.x = temp.getX();
    pose.pose.orientation.y = temp.getY();
    pose.pose.orientation.z = temp.getZ();
    pose.pose.orientation.w = temp.getW();

    plan.push_back(pose);

    gui_path.poses[i] = plan[i];
  }
  plan_pub_.publish(gui_path);

  return true;
}

}  // namespace lattice_path_planner