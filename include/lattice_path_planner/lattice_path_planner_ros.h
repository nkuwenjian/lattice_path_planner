#ifndef LATTICE_PATH_PLANNER_LATTICE_PATH_PLANNER_ROS_H
#define LATTICE_PATH_PLANNER_LATTICE_PATH_PLANNER_ROS_H

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// global representation
#include <nav_core/base_global_planner.h>

// Costmap used for the map representation
#include <costmap_2d/costmap_2d_ros.h>

#include "lattice_path_planner/sbpl_exception.h"
#include "lattice_path_planner/lattice_environment.h"
#include "lattice_path_planner/lattice_path_planner.h"
#include "lattice_path_planner/spline_interpolation.h"

namespace lattice_path_planner
{
class LatticePathPlannerROS : public nav_core::BaseGlobalPlanner
{
public:
  /**
   * @brief  Default constructor for the LatticePathPlannerROS object
   */
  LatticePathPlannerROS();

  /**
   * @brief  Constructor for the LatticePathPlannerROS object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  LatticePathPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Initialization function for the LatticePathPlannerROS object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  virtual bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan);

  virtual ~LatticePathPlannerROS();

private:
  void publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
  unsigned char computeCircumscribedCost();

private:
  bool initialized_;

  LatticePathPlanner* planner_;
  LatticeEnvironment* env_;
  SplineInterpolation interpolator_;
  double sample_stepsize_;

  std::string primitive_filename_;  //!< where to find the motion primitives for the current robot
  int force_scratch_limit_;  //!< the number of cells that have to be changed in the costmap to force the planner to
                             //!< plan from scratch even if its an incremental planner

  unsigned char circumscribed_cost_;
  bool allow_unknown_;  //!< Specifies whether or not to allow the planner to create plans that traverse unknown space
  unsigned char* map_data_;

  std::string name_;
  costmap_2d::Costmap2DROS* costmap_ros_;  //!< manages the cost map for us
  std::vector<geometry_msgs::Point> footprint_;
  unsigned int current_env_width_;
  unsigned int current_env_height_;

  ros::Publisher plan_pub_;
};

}  // namespace lattice_path_planner

#endif  // LATTICE_PATH_PLANNER_LATTICE_PATH_PLANNER_ROS_H