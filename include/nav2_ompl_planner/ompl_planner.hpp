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
#include "ompl/geometric/SimpleSetup.h"
#include "rclcpp/rclcpp.hpp"

namespace nav2_ompl_planner {

namespace ob = ompl::base;
namespace og = ompl::geometric;

/**
 * Cost integral objective based on a costmap
 */
class CostMapObjective : public ob::StateCostIntegralObjective {
 public:
  CostMapObjective(const ob::SpaceInformationPtr& si, const nav2_costmap_2d::Costmap2D& costmap);

  ob::Cost stateCost(const ob::State* s) const override;

  double x;

 protected:
  const nav2_costmap_2d::Costmap2D& costmap_;
};

class OMPLPlanner : public nav2_core::GlobalPlanner {
 public:
  /**
   * Constructor
   */
  OMPLPlanner();

  /**
   * Destructor
   */
  ~OMPLPlanner() = default;

  /**
   * Configures the plugin
   * @param parent the node to which this plugin belongs to
   * @param name the name of the plugin
   * @param tf the TF buffer
   * @param costmap_ros the costmap to be used for planning
   */
  void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * Activates the plugin
   */
  void activate() override;

  /**
   * Deactivates the plugin
   */
  void deactivate() override;

  /**
   * Cleans up the plugin
   */
  void cleanup() override;

  /**
   * Creates a plan
   * @param start_pose the start pose of the robot
   * @param goal_pose the goal pose of the robot
   */
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start,
                                 const geometry_msgs::msg::PoseStamped& goal) override;

 protected:
  /**
   * Checks if a state is valid, i.e. a point is not in occupied space
   * @param state the state pointer
   * @returns true if the state is not in occupied space, false otherwise
   */
  bool isStateValid(const ob::State* state);

  /**
   * Sets the bounds of the state space based on the costmap loaded
   */
  void setBounds();

  /**
   * Sets the planner
   */
  void setPlanner();

  /**
   * Converts a ROS pose stamped message to an OMPL state
   * @param pose_stamped the pose stamped message
   * @param scoped_state the OMPL state to fill
   */
  void poseStampedToScopedState(const geometry_msgs::msg::PoseStamped& pose_stamped,
                                ob::ScopedState<ob::SE2StateSpace>& scoped_state);

  /**
   * Converts an OMPL state to a ROS pose stamped message
   * @param scoped_state the OMPL state
   * @param pose_stamped the pose stamped message to fill
   */
  void scopedStateToPoseStamped(const ob::ScopedState<ob::SE2StateSpace>& scoped_state,
                                geometry_msgs::msg::PoseStamped& pose_stamped);

  /**
   * Plugin name
   */
  std::string name_;

  /**
   * Maximum amount of time given to solve the planning problem
   */
  double timeout_;

  /**
   * The problem threshold
   */
  double threshold_;

  /**
   * The weight of the cost objective
   */
  double cost_objective_weight_;

  /**
   * The weight of the length objective
   */
  double length_objective_weight_;

  /**
   * Whether or not the planner should be allowed to plan through unknown space
   */
  bool allow_unknown_;

  /**
   * Planner simple setup pointer
   */
  og::SimpleSetupPtr ss_;

  /**
   * The planner name (est, kpiece, rrt, pdst, sst)
   */
  std::string planner_name_;

  /**
   * Costmap global frame
   */
  std::string global_frame_;

  /**
   * SE2 space pointer representing the robot pose space
   */
  ob::StateSpacePtr space_;

  /**
   * TF buffer
   */
  std::shared_ptr<tf2_ros::Buffer> tf_;

  /**
   * Node pointer
   */
  nav2_util::LifecycleNode::SharedPtr node_;

  /**
   * Global costmap
   */
  nav2_costmap_2d::Costmap2D* costmap_;

  /**
   * ROS costmap
   */
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  /**
   * ROS costmap subscriber
   */
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
};

}  // namespace nav2_ompl_planner

#endif  // NAV2_OMPL_PLANNER__OMPL_PLANNER_HPP_
