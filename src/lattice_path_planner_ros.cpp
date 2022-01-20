#include <pluginlib/class_list_macros.hpp>
#include <nav_msgs/Path.h>

#include <costmap_2d/inflation_layer.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>

#include <string.h>

#include "lattice_path_planner/lattice_path_planner_ros.h"

PLUGINLIB_EXPORT_CLASS(lattice_path_planner::LatticePathPlannerROS, nav_core::BaseGlobalPlanner)

namespace lattice_path_planner
{
LatticePathPlannerROS::LatticePathPlannerROS()
  : initialized_(false), costmap_ros_(NULL), planner_(NULL), env_(NULL), map_data_(NULL)
{
}

LatticePathPlannerROS::LatticePathPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : initialized_(false), costmap_ros_(NULL), planner_(NULL), env_(NULL), map_data_(NULL)
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

  if (map_data_)
  {
    delete[] map_data_;
    map_data_ = NULL;
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
    private_nh.param("sample_stepsize", sample_stepsize_, costmap_ros->getCostmap()->getResolution());
    private_nh.param("allow_unknown", allow_unknown_, true);

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
    for (size_t i = 0; i < footprint_.size(); i++)
    {
      perimeterptsV.emplace_back(footprint_[i].x, footprint_[i].y);
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

      if (map_data_)
      {
        delete[] map_data_;
        map_data_ = NULL;
      }
      map_data_ = new unsigned char[current_env_width_ * current_env_height_];
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

  ROS_DEBUG("[lattice_path_planner] getting start point (%g,%g) goal point (%g,%g)", start.pose.position.x,
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
  memcpy(map_data_, costmap_ros_->getCostmap()->getCharMap(),
         current_env_width_ * current_env_height_ * sizeof(unsigned char));
  if (allow_unknown_)
  {
    for (int i = 0; i < current_env_width_ * current_env_height_; i++)
    {
      if (map_data_[i] == costmap_2d::NO_INFORMATION)
        map_data_[i] = costmap_2d::FREE_SPACE;
    }
  }

  env_->SetMap(map_data_);

  ROS_DEBUG("[lattice_path_planner] run planner");
  std::vector<int> solution_stateIDs;
  int solution_cost;
  try
  {
    int ret = planner_->replan(&solution_stateIDs, &solution_cost);
    if (ret)
      ROS_DEBUG("Solution is found");
    else
    {
      ROS_WARN("Solution not found");
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
    sbpl_path.emplace_back(start.pose.position.x - costmap_ros_->getCostmap()->getOriginX(),
                           start.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_start);
  }

  // sample raw path along sbpl path
  std::vector<sbpl_2Dpt_t> raw_path;
  raw_path.emplace_back(sbpl_path.front().x, sbpl_path.front().y);
  sbpl_xy_theta_pt_t last_pose = sbpl_path.front();

  for (size_t i = 1; i < sbpl_path.size(); i++)
  {
    double dx = sbpl_path[i].x - last_pose.x;
    double dy = sbpl_path[i].y - last_pose.y;
    if (hypot(dx, dy) > sample_stepsize_)
    {
      raw_path.emplace_back(sbpl_path[i].x, sbpl_path[i].y);
      last_pose = sbpl_path[i];
    }
  }
  raw_path.pop_back();
  raw_path.emplace_back(sbpl_path.back().x, sbpl_path.back().y);

  // smooth the raw path
  std::vector<sbpl_2Dpt_t> smooth_path;
  if (raw_path.size() > 2)
  {
    interpolator_.interpolate(raw_path, smooth_path);
  }
  else
  {
    for (size_t i = 0; i < raw_path.size(); i++)
      smooth_path.push_back(raw_path[i]);
  }

  ROS_DEBUG("Plan has %d points.", (int)smooth_path.size());
  ros::Time plan_time = ros::Time::now();

  // return path planning result
  plan.clear();
  geometry_msgs::PoseStamped pose;
  pose.header = start.header;
  double dx, dy, yaw;

  if (smooth_path.size() < 2)
  {
    pose.pose = start.pose;
    plan.push_back(pose);
  }
  else
  {
    for (size_t i = 0; i < smooth_path.size(); i++)
    {
      pose.pose.position.x = smooth_path[i].x + costmap_ros_->getCostmap()->getOriginX();
      pose.pose.position.y = smooth_path[i].y + costmap_ros_->getCostmap()->getOriginY();

      if (i == 0)
      {
        dx = smooth_path[i + 1].x - smooth_path[i].x;
        dy = smooth_path[i + 1].y - smooth_path[i].y;
      }
      else if (i == smooth_path.size() - 1)
      {
        dx = smooth_path[i].x - smooth_path[i - 1].x;
        dy = smooth_path[i].y - smooth_path[i - 1].y;
      }
      else
      {
        dx = 0.5 * (smooth_path[i + 1].x - smooth_path[i - 1].x);
        dy = 0.5 * (smooth_path[i + 1].y - smooth_path[i - 1].y);
      }

      yaw = atan2(dy, dx);
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      plan.push_back(pose);
    }
  }

  publishGlobalPlan(plan);

  return true;
}

void LatticePathPlannerROS::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (plan.empty())
    return;

  nav_msgs::Path gui_path;
  gui_path.header = plan[0].header;
  gui_path.poses = plan;

  plan_pub_.publish(gui_path);
}

}  // namespace lattice_path_planner