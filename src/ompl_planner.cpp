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

#include "nav2_ompl_planner/ompl_planner.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <chrono>

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/geometric/planners/prm/LazyPRM.h"
#include "ompl/geometric/planners/prm/LazyPRMstar.h"
#include "ompl/geometric/planners/prm/PRM.h"
#include "ompl/geometric/planners/prm/PRMstar.h"
#include "ompl/geometric/planners/rrt/InformedRRTstar.h"
#include "ompl/geometric/planners/rrt/LazyRRT.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/rrt/RRTsharp.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/geometric/planners/kpiece/BKPIECE1.h"
#include "ompl/geometric/planners/kpiece/KPIECE1.h"
#include "ompl/geometric/planners/kpiece/LBKPIECE1.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_ompl_planner::OMPLPlanner, nav2_core::GlobalPlanner)

namespace nav2_ompl_planner
{
  CostMapObjective::CostMapObjective(const ob::SpaceInformationPtr &si,
                                     const nav2_costmap_2d::Costmap2D &costmap)
      : ob::StateCostIntegralObjective(si, true), costmap_(costmap) {}

  // TODO(Abdul Rahman Dabbour): refactor to use OMPLPlanner::isStateValid fn
  ob::Cost CostMapObjective::stateCost(const ob::State *s) const
  {
    const double wx(s->as<ob::SE2StateSpace::StateType>()->getX());
    const double wy(s->as<ob::SE2StateSpace::StateType>()->getY());
    int mx, my;
    costmap_.worldToMapEnforceBounds(wx, wy, mx, my);
    const double cost(static_cast<double>(costmap_.getCost(mx, my)));
    return ob::Cost(cost);
  }

  OMPLPlanner::OMPLPlanner() : tf_(nullptr), node_(nullptr), costmap_(nullptr) {}

  void OMPLPlanner::configure(rclcpp_lifecycle::LifecycleNode::SharedPtr parent, std::string name,
                              std::shared_ptr<tf2_ros::Buffer> tf,
                              std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent;
    tf_ = tf;
    name_ = name;

    RCLCPP_INFO(node_->get_logger(), "Configuring plugin %s of type OMPLPlanner", name_.c_str());

    // Get the ROS costmap pointer
    costmap_ros_ = costmap_ros;

    // Get ROS params

    // TODO(Abdul Rahman Dabbour): Make this footprint-based
    // nav2_util::declare_parameter_if_not_declared(node_, name_ + ".footprint",
    //                                              rclcpp::ParameterValue([[]]));
    // node_->get_parameter(name_ + ".footprint", footprint_);

    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".timeout",
                                                 rclcpp::ParameterValue(1.0));
    node_->get_parameter(name_ + ".timeout", timeout_);

    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".threshold",
                                                 rclcpp::ParameterValue(std::numeric_limits<double>::epsilon()));
    node_->get_parameter(name_ + ".threshold", threshold_);

    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".cost_objective_weight",
                                                 rclcpp::ParameterValue(1.0));
    node_->get_parameter(name_ + ".cost_objective_weight", cost_objective_weight_);

    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".collision_checking_resolution",
                                                 rclcpp::ParameterValue(0.01));
    node_->get_parameter(name_ + ".collision_checking_resolution", collision_checking_resolution_);

    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".turning_radius",
                                                 rclcpp::ParameterValue(0.01));
    node_->get_parameter(name_ + ".turning_radius", turning_radius_);

    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".length_objective_weight",
                                                 rclcpp::ParameterValue(1.0));
    node_->get_parameter(name_ + ".length_objective_weight", length_objective_weight_);

    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".allow_unknown",
                                                 rclcpp::ParameterValue(false));
    node_->get_parameter(name_ + ".allow_unknown", allow_unknown_);

    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".planner_name",
                                                 rclcpp::ParameterValue("lbkpiece"));
    node_->get_parameter(name_ + ".planner_name", planner_name_);
  }

  void OMPLPlanner::cleanup()
  {
    RCLCPP_INFO(node_->get_logger(), "Cleaning up plugin %s of type OMPLPlanner", name_.c_str());
  }

  void OMPLPlanner::activate()
  {
    RCLCPP_INFO(node_->get_logger(), "Activating plugin %s of type OMPLPlanner", name_.c_str());
  }

  void OMPLPlanner::deactivate()
  {
    RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s of type OMPLPlanner", name_.c_str());
  }

  nav_msgs::msg::Path OMPLPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                                              const geometry_msgs::msg::PoseStamped &goal)
  {
    const auto start_time(std::chrono::high_resolution_clock::now());

    // Get the costmap
    costmap_ = costmap_ros_->getCostmap();
    global_frame_ = costmap_ros_->getGlobalFrameID();

    // Assign the pose and control spaces
    space_ = std::make_shared<ob::ReedsSheppStateSpace>(turning_radius_);
    setBounds();

    // Create simple setup
    ss_ = std::make_shared<og::SimpleSetup>(space_);
    ss_->setStateValidityChecker([this](const ob::State *s) { return this->isStateValid(s); });
    ss_->getSpaceInformation()->setStateValidityCheckingResolution(collision_checking_resolution_);
    setPlanner();

    // Define and setup problem
    ob::ScopedState<ob::SE2StateSpace> start_state(space_);
    poseStampedToScopedState(start, start_state);
    ob::ScopedState<ob::SE2StateSpace> goal_state(space_);
    poseStampedToScopedState(goal, goal_state);
    ss_->setStartAndGoalStates(start_state, goal_state, threshold_);
    ob::OptimizationObjectivePtr cost_objective(
        new CostMapObjective(ss_->getSpaceInformation(), *costmap_));
    ob::OptimizationObjectivePtr length_objective(
        new ob::PathLengthOptimizationObjective(ss_->getSpaceInformation()));
    ob::OptimizationObjectivePtr hybrid_objective((cost_objective_weight_ * cost_objective) +
                                                  (length_objective_weight_ * length_objective));
    ss_->setOptimizationObjective(hybrid_objective);
    ss_->setup();
    RCLCPP_INFO(node_->get_logger(), "OMPL setup successful. Starting to create plan.");

    nav_msgs::msg::Path path;

    if (ss_->solve(timeout_))
    {
      RCLCPP_INFO(node_->get_logger(), "A path was found.");

      og::PathGeometric solution_path = ss_->getSolutionPath();
      path.header.stamp = node_->now();
      path.header.frame_id = global_frame_;

      RCLCPP_INFO(node_->get_logger(), "Will now convert the states into a path");
      og::PathSimplifier path_simplifier(ss_->getSpaceInformation(), ss_->getGoal(), ss_->getOptimizationObjective());
      path_simplifier.simplifyMax(solution_path);
      path_simplifier.smoothBSpline(solution_path);

      for (const auto &state : solution_path.getStates())
      {
        ob::ScopedState<ob::SE2StateSpace> scoped_state(ss_->getStateSpace());
        scoped_state = state;
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = path.header;
        scopedStateToPoseStamped(scoped_state, pose_stamped);
        path.poses.push_back(pose_stamped);
      }

      const auto end_time(std::chrono::high_resolution_clock::now());
      const auto duration(std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time));
      RCLCPP_DEBUG(node_->get_logger(), "Plan calculation took %f seconds", duration.count() / 1000000.0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "No path could be found.");
    }
    return path;
  }

  void OMPLPlanner::scopedStateToPoseStamped(const ob::ScopedState<ob::SE2StateSpace> &scoped_state,
                                             geometry_msgs::msg::PoseStamped &pose_stamped)
  {
    pose_stamped.pose.position.x = scoped_state->getX();
    pose_stamped.pose.position.y = scoped_state->getY();
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, scoped_state->getYaw());
    pose_stamped.pose.orientation = tf2::toMsg(quaternion);
  }

  void OMPLPlanner::poseStampedToScopedState(const geometry_msgs::msg::PoseStamped &pose_stamped,
                                             ob::ScopedState<ob::SE2StateSpace> &scoped_state)
  {
    if (global_frame_ != pose_stamped.header.frame_id)
    {
      geometry_msgs::msg::PoseStamped pose_stamped_transformed;
      try
      {
        pose_stamped_transformed = tf_->transform(pose_stamped, global_frame_);
      }
      catch (tf2::TransformException &e)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Could not transform the pose from the %s frame to the %s frame.",
                     pose_stamped.header.frame_id.c_str(), global_frame_.c_str());
        return;
      }
      scoped_state->setX(pose_stamped_transformed.pose.position.x);
      scoped_state->setY(pose_stamped_transformed.pose.position.y);
      scoped_state->setYaw(tf2::getYaw(pose_stamped_transformed.pose.orientation));
    }
    else
    {
      scoped_state->setX(pose_stamped.pose.position.x);
      scoped_state->setY(pose_stamped.pose.position.y);
      scoped_state->setYaw(tf2::getYaw(pose_stamped.pose.orientation));
    }
    scoped_state.enforceBounds();
  }

  void OMPLPlanner::setBounds()
  {
    ob::RealVectorBounds bounds(2);
    // costmap_ros_->
    // const unsigned int max_mx(costmap_->getSizeInCellsX() - 1);
    // const unsigned int max_my(costmap_->getSizeInCellsX() - 1);
    double min_wx, max_wx, min_wy, max_wy;
    costmap_->mapToWorld(0, 0, min_wx, min_wy);
    // costmap_->mapToWorld(max_mx, max_my, max_wx, max_wy);
    max_wx = (costmap_->getResolution() * (costmap_->getSizeInCellsX() - 1)) + costmap_->getOriginX();
    max_wy = (costmap_->getResolution() * (costmap_->getSizeInCellsY() - 1)) + costmap_->getOriginY();

    bounds.setLow(0, min_wx);
    bounds.setHigh(0, max_wx);
    bounds.setLow(1, min_wy);
    bounds.setHigh(1, max_wy);

    space_->as<ob::SE2StateSpace>()->setBounds(bounds);
  }

  void OMPLPlanner::setPlanner()
  {
    std::transform(planner_name_.begin(), planner_name_.end(), planner_name_.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    if (planner_name_ == "auto")
    {
      ss_->setPlanner(ob::PlannerPtr());
    }
    else if (planner_name_ == "lazyprm")
    {
      ss_->setPlanner(std::make_shared<og::LazyPRM>(ss_->getSpaceInformation()));
    }
    else if (planner_name_ == "lazyprmstar")
    {
      ss_->setPlanner(std::make_shared<og::LazyPRMstar>(ss_->getSpaceInformation()));
    }
    else if (planner_name_ == "prm")
    {
      ss_->setPlanner(std::make_shared<og::PRM>(ss_->getSpaceInformation()));
    }
    else if (planner_name_ == "prmstar")
    {
      ss_->setPlanner(std::make_shared<og::PRMstar>(ss_->getSpaceInformation()));
    }
    else if (planner_name_ == "informedrrtstar")
    {
      ss_->setPlanner(std::make_shared<og::InformedRRTstar>(ss_->getSpaceInformation()));
    }
    else if (planner_name_ == "lazyrrt")
    {
      ss_->setPlanner(std::make_shared<og::LazyRRT>(ss_->getSpaceInformation()));
    }
    else if (planner_name_ == "rrt")
    {
      ss_->setPlanner(std::make_shared<og::RRT>(ss_->getSpaceInformation()));
    }
    else if (planner_name_ == "rrtconnect")
    {
      ss_->setPlanner(std::make_shared<og::RRTConnect>(ss_->getSpaceInformation()));
    }
    else if (planner_name_ == "rrtsharp")
    {
      ss_->setPlanner(std::make_shared<og::RRTsharp>(ss_->getSpaceInformation()));
    }
    else if (planner_name_ == "rrtstar")
    {
      ss_->setPlanner(std::make_shared<og::RRTstar>(ss_->getSpaceInformation()));
    }
    else if (planner_name_ == "bkpiece")
    {
      ss_->setPlanner(std::make_shared<og::BKPIECE1>(ss_->getSpaceInformation()));
    }
    else if (planner_name_ == "kpiece")
    {
      ss_->setPlanner(std::make_shared<og::KPIECE1>(ss_->getSpaceInformation()));
    }
    else if (planner_name_ == "lbkpiece")
    {
      ss_->setPlanner(std::make_shared<og::LBKPIECE1>(ss_->getSpaceInformation()));
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Unknown planner chosen; will default to OMPL choice.");
      ss_->setPlanner(ob::PlannerPtr());
    }
  }

  bool OMPLPlanner::isStateValid(const ob::State *state)
  {
    // Check if the state is valid in the OMPL state space bounds
    if (!ss_->getSpaceInformation()->satisfiesBounds(state))
    {
      return false;
    }

    // TODO(Abdul Rahman Dabbour): Make this footprint-based!
    // Check if robot footprint is on top of any illegal map coordinates
    // auto cost = collision_checker.footprintCostAtPose(
    //     state->as<ob::SE2StateSpace::StateType>()->getX(),
    //     state->as<ob::SE2StateSpace::StateType>()->getY(),
    //     start->as<ob::SE2StateSpace::StateType>()->getYaw(), footprint_);

    int mx, my;
    const double wx(state->as<ob::SE2StateSpace::StateType>()->getX());
    const double wy(state->as<ob::SE2StateSpace::StateType>()->getY());
    costmap_->worldToMapEnforceBounds(wx, wy, mx, my);
    const unsigned char cost(costmap_->getCost(mx, my));

    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
        (!allow_unknown_ && cost == nav2_costmap_2d::NO_INFORMATION))
    {
      return false;
    }
    return true;
  }

} // namespace nav2_ompl_planner
