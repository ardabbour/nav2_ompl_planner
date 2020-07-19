// Copyright 2020 Abdul Rahman Dabbour.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
// associated documentation files (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute,
// sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or
// substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
// NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef NAV2_OMPL_PLANNER__OMPL_PLANNER_HPP_
#define NAV2_OMPL_PLANNER__OMPL_PLANNER_HPP_

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_util/node_utils.hpp"
#include "ompl/base/State.h"
#include "ompl/base/objectives/StateCostIntegralObjective.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/control/SimpleSetup.h"
#include "ompl/control/SpaceInformation.h"
#include "rclcpp/rclcpp.hpp"

namespace nav2_ompl_planner {

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

/**
 * Cost integral objective based on a costmap
 */
class CostMapObjective : public ob::StateCostIntegralObjective {
 public:
  /**
   * @brief  Constructor
   * @param  si The space information pointer
   * @param  costmap The costmap to get the costs from
   */
  CostMapObjective(const ob::SpaceInformationPtr& si, const nav2_costmap_2d::Costmap2D& costmap);

  /**
   * @brief  The cost of the state
   * @param  s The state pointer
   */
  ob::Cost stateCost(const ob::State* s) const override;

 protected:
  const nav2_costmap_2d::Costmap2D& costmap_;
};

/**
 * @class  OMPLPlanner
 * @brief  Constructor
 */
class OMPLPlanner : public nav2_core::GlobalPlanner {
 public:
  /**
   * @brief  Constructor
   */
  OMPLPlanner();

  /**
   * @brief  Destructor
   */
  ~OMPLPlanner() = default;

  /**
   * @brief  Configures the plugin
   * @param  parent the node to which this plugin belongs to
   * @param  name the name of the plugin
   * @param  tf the TF buffer
   * @param  costmap_ros the costmap to be used for planning
   */
  void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief  Activates the plugin
   */
  void activate() override;

  /**
   * @brief  Deactivates the plugin
   */
  void deactivate() override;

  /**
   * @brief  Cleans up the plugin
   */
  void cleanup() override;

  /**
   * @brief  Creates a plan
   * @param  start_pose the start pose of the robot
   * @param  goal_pose the goal pose of the robot
   */
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start,
                                 const geometry_msgs::msg::PoseStamped& goal) override;

 protected:
  /**
   * @brief   Checks if a state is valid, i.e. a point is not in occupied space
   * @param   state the state pointer
   * @return  true if the state is not in occupied space, false otherwise
   */
  bool isStateValid(const ob::State* state);

  /**
   * @brief  Sets the bounds of the state space based on the costmap loaded
   */
  void setBounds();

  /**
   * @brief  Sets the propagator
   */
  void setPropagator();

  /**
   * @brief  Sets the planner
   */
  void setPlanner();

  /**
   * @brief  Converts a ROS pose stamped message to an OMPL state
   * @param  pose_stamped the pose stamped message
   * @param  scoped_state the OMPL state to fill
   */
  void poseStampedToScopedState(const geometry_msgs::msg::PoseStamped& pose_stamped,
                                ob::ScopedState<ob::SE2StateSpace>& scoped_state);

  /**
   * @brief  Converts an OMPL state to a ROS pose stamped message
   * @param  scoped_state the OMPL state
   * @param  pose_stamped the pose stamped message to fill
   */
  void scopedStateToPoseStamped(const ob::ScopedState<ob::SE2StateSpace>& scoped_state,
                                geometry_msgs::msg::PoseStamped& pose_stamped);

  /**
   * @brief  Plugin name
   */
  std::string name_;

  /**
   * @brief  Maximum amount of time given to solve the planning problem
   */
  double timeout_;

  /**
   * @brief  The problem threshold
   */
  double threshold_;

  /**
   * @brief  The weight of the cost objective
   */
  double cost_objective_weight_;

  /**
   * @brief  The weight of the length objective
   */
  double length_objective_weight_;

  /**
   * @brief  Maximum linear velocity (m/s)
   */
  double max_lin_vel_;

  /**
   * @brief  Maximum angular velocity (rad/s)
   */
  double max_ang_vel_;

  /**
   * @brief  Whether or not the robot is allowed to move in reverse
   */
  bool reverse_driving_;

  /**
   * @brief  Whether or not the planner should be allowed to plan through unknown space
   */
  bool allow_unknown_;

  /**
   * @brief  Whether or not to use ODESolver when propagating states
   */
  bool use_ode_solver_;

  /**
   * @brief  Planner simple setup pointer
   */
  oc::SimpleSetupPtr ss_;

  /**
   * @brief  The planner name (est, kpiece, rrt, pdst, sst)
   */
  std::string planner_name_;

  /**
   * @brief  The solver type (basic, adaptive, error)
   */
  std::string solver_type_;

  /**
   * @brief  Costmap global frame
   */
  std::string global_frame_;

  /**
   * @brief  SE2 space pointer representing the robot pose space
   */
  ob::StateSpacePtr pose_space_;

  /**
   * @brief  2D real space pointer representing the control space (lin. vel., ang. vel.)
   */
  oc::ControlSpacePtr control_space_;

  /**
   * The resolution at which to check for collisions
   */
  double collision_checking_resolution_;

  /**
   * @brief  TF buffer
   */
  std::shared_ptr<tf2_ros::Buffer> tf_;

  /**
   * @brief  Node pointer
   */
  nav2_util::LifecycleNode::SharedPtr node_;

  /**
   * @brief  Global costmap
   */
  nav2_costmap_2d::Costmap2D* costmap_;

  /**
   * @brief  ROS costmap
   */
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  /**
   * @brief  ROS costmap subscriber
   */
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
};

}  // namespace nav2_ompl_planner

#endif  // NAV2_OMPL_PLANNER__OMPL_PLANNER_HPP_
