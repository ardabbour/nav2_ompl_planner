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

#include "nav2_ompl_planner/diff_drive.hpp"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/control/ODESolver.h"
#include "ompl/control/planners/est/EST.h"
#include "ompl/control/planners/kpiece/KPIECE1.h"
#include "ompl/control/planners/pdst/PDST.h"
#include "ompl/control/planners/rrt/RRT.h"
#include "ompl/control/planners/sst/SST.h"
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

    RCLCPP_INFO(node_->get_logger(), "Set the costmap_ros pointer!");

    // Get ROS params
    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".timeout",
                                                 rclcpp::ParameterValue(30.0));
    node_->get_parameter(name_ + ".timeout", timeout_);

    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".max_lin_vel",
                                                 rclcpp::ParameterValue(1.5));
    node_->get_parameter(name_ + ".max_lin_vel", max_lin_vel_);

    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".max_ang_vel",
                                                 rclcpp::ParameterValue(0.75));
    node_->get_parameter(name_ + ".max_ang_vel", max_ang_vel_);

    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".threshold",
                                                 rclcpp::ParameterValue(0.2));
    node_->get_parameter(name_ + ".threshold", threshold_);

    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".cost_objective_weight",
                                                 rclcpp::ParameterValue(1.0));
    node_->get_parameter(name_ + ".cost_objective_weight", cost_objective_weight_);

    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".length_objective_weight",
                                                 rclcpp::ParameterValue(1.0));
    node_->get_parameter(name_ + ".length_objective_weight", length_objective_weight_);

    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".reverse_driving",
                                                 rclcpp::ParameterValue(false));
    node_->get_parameter(name_ + ".reverse_driving", reverse_driving_);

    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".allow_unknown",
                                                 rclcpp::ParameterValue(false));
    node_->get_parameter(name_ + ".allow_unknown", allow_unknown_);

    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".use_ode_solver",
                                                 rclcpp::ParameterValue(false));
    node_->get_parameter(name_ + ".use_ode_solver", use_ode_solver_);

    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".solver_type",
                                                 rclcpp::ParameterValue("basic"));
    node_->get_parameter(name_ + ".solver_type", solver_type_);

    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".planner_name",
                                                 rclcpp::ParameterValue("auto"));
    node_->get_parameter(name_ + ".planner_name", planner_name_);

    RCLCPP_INFO(node_->get_logger(), "Set all the planner parameters!");
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
    // Get the costmap
    costmap_ = costmap_ros_->getCostmap();
    global_frame_ = costmap_ros_->getGlobalFrameID();

    // Assign the pose and control spaces
    pose_space_ = std::make_shared<ob::SE2StateSpace>();
    control_space_ = std::make_shared<DiffDriveControlSpace>(pose_space_);
    setBounds();

    // Create simple setup
    ss_ = std::make_shared<oc::SimpleSetup>(control_space_);
    ss_->setStateValidityChecker([this](const ob::State *s) { return this->isStateValid(s); });
    setPropagator();
    setPlanner();

    // Define and setup problem
    ob::ScopedState<ob::SE2StateSpace> start_state(pose_space_);
    poseStampedToScopedState(start, start_state);
    ob::ScopedState<ob::SE2StateSpace> goal_state(pose_space_);
    poseStampedToScopedState(goal, goal_state);
    ss_->setStartAndGoalStates(start_state, goal_state, threshold_);
    ob::OptimizationObjectivePtr cost_objective(
        new CostMapObjective(ss_->getSpaceInformation(), *costmap_));
    ob::OptimizationObjectivePtr length_objective(
        new ob::PathLengthOptimizationObjective(ss_->getSpaceInformation()));
    ss_->setOptimizationObjective((cost_objective_weight_ * cost_objective) +
                                  (length_objective_weight_ * length_objective));
    ss_->setup();

    nav_msgs::msg::Path path;
    if (ss_->solve(timeout_) == ob::PlannerStatus::EXACT_SOLUTION)
    {
      RCLCPP_INFO(node_->get_logger(), "A path was found.");

      og::PathGeometric solution_path = ss_->getSolutionPath().asGeometric();
      path.header.stamp = node_->now();
      path.header.frame_id = global_frame_;

      for (const auto &state : solution_path.getStates())
      {
        ob::ScopedState<ob::SE2StateSpace> scoped_state(ss_->getStateSpace());
        scoped_state = state;
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = path.header;
        scopedStateToPoseStamped(scoped_state, pose_stamped);
        path.poses.push_back(pose_stamped);
      }

      for (const auto &pose_stamped : path.poses)
      {
        RCLCPP_INFO(node_->get_logger(), "x: %f, y: %f, yaw: %f", pose_stamped.pose.position.x,
                    pose_stamped.pose.position.y, tf2::getYaw(pose_stamped.pose.orientation));
      }
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

    double min_wx, max_wx, min_wy, max_wy;
    costmap_->mapToWorld(0, 0, min_wx, min_wy);
    max_wx = (costmap_->getResolution() * (costmap_->getSizeInCellsX() - 1)) + costmap_->getOriginX();
    max_wy = (costmap_->getResolution() * (costmap_->getSizeInCellsY() - 1)) + costmap_->getOriginY();

    ob::RealVectorBounds pose_bounds(2);
    pose_bounds.setLow(0, min_wx);
    pose_bounds.setHigh(0, max_wx);
    pose_bounds.setLow(1, min_wy);
    pose_bounds.setHigh(1, max_wy);
    pose_space_->as<ob::SE2StateSpace>()->setBounds(pose_bounds);

    ob::RealVectorBounds control_bounds(2);
    if (reverse_driving_)
    {
      control_bounds.setLow(0, -max_lin_vel_);
    }
    else
    {
      control_bounds.setLow(0, 0);
    }
    control_bounds.setHigh(0, max_lin_vel_);
    control_bounds.setLow(1, -max_ang_vel_);
    control_bounds.setHigh(1, max_ang_vel_);
    control_space_->as<oc::RealVectorControlSpace>()->setBounds(control_bounds);
  }

  // void OMPLPlanner::setBounds()
  // {

  //   ob::RealVectorBounds pose_bounds(2);
  //   pose_bounds.setLow(0, 0.0);
  //   pose_bounds.setHigh(0, 0.0);
  //   pose_bounds.setLow(1, costmap_->getSizeInCellsX() - 1.0);
  //   pose_bounds.setHigh(1, costmap_->getSizeInCellsY() - 1.0);
  //   pose_space_->as<ob::SE2StateSpace>()->setBounds(pose_bounds);

  //   ob::RealVectorBounds control_bounds(2);
  //   if (reverse_driving_)
  //   {
  //     control_bounds.setLow(0, -max_lin_vel_);
  //   }
  //   else
  //   {
  //     control_bounds.setLow(0, 0);
  //   }
  //   control_bounds.setHigh(0, max_lin_vel_);
  //   control_bounds.setLow(1, -max_ang_vel_);
  //   control_bounds.setHigh(1, max_ang_vel_);
  //   control_space_->as<oc::RealVectorControlSpace>()->setBounds(control_bounds);
  // }

  void OMPLPlanner::setPlanner()
  {
    std::transform(planner_name_.begin(), planner_name_.end(), planner_name_.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    if (planner_name_ == "auto")
    {
      ss_->setPlanner(ob::PlannerPtr());
    }
    else if (planner_name_ == "est")
    {
      ss_->setPlanner(std::make_shared<oc::EST>(ss_->getSpaceInformation()));
    }
    else if (planner_name_ == "kpiece")
    {
      ss_->setPlanner(std::make_shared<oc::KPIECE1>(ss_->getSpaceInformation()));
    }
    else if (planner_name_ == "rrt")
    {
      ss_->setPlanner(std::make_shared<oc::RRT>(ss_->getSpaceInformation()));
    }
    else if (planner_name_ == "pdst")
    {
      ss_->setPlanner(std::make_shared<oc::PDST>(ss_->getSpaceInformation()));
    }
    else if (planner_name_ == "sst")
    {
      ss_->setPlanner(std::make_shared<oc::SST>(ss_->getSpaceInformation()));
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Unknown planner chosen; will default to OMPL choice.");
      ss_->setPlanner(ob::PlannerPtr());
    }
  }

  void OMPLPlanner::setPropagator()
  {
    if (use_ode_solver_)
    {
      std::transform(solver_type_.begin(), solver_type_.end(), solver_type_.begin(),
                     [](unsigned char c) { return std::tolower(c); });

      auto kinematic_ode_lambda = [this](const oc::ODESolver::StateType &q, const oc::Control *ctrl,
                                         oc::ODESolver::StateType &qdot) {
        diffDriveKinematicODE(q, ctrl, qdot);
      };

      oc::ODESolverPtr solver_ptr;
      oc::SpaceInformationPtr si(ss_->getSpaceInformation());
      if (solver_type_ == "basic")
      {
        solver_ptr = std::make_shared<oc::ODEBasicSolver<>>(si, kinematic_ode_lambda);
        // TODO(Abdul Rahman Dabbour): investigate why build fails because of ODEAdaptiveSolver
        // } else if (solver_type_ == "adaptive") {
        //   solver_ptr = std::make_shared<oc::ODEAdaptiveSolver<>>(si, kinematic_ode_lambda);
      }
      else if (solver_type_ == "error")
      {
        solver_ptr = std::make_shared<oc::ODEErrorSolver<>>(si, kinematic_ode_lambda);
      }
      else
      {
        RCLCPP_INFO(node_->get_logger(), "Unknown solver type chosen; will default to basic.");
        solver_ptr = std::make_shared<oc::ODEBasicSolver<>>(si, kinematic_ode_lambda);
      }

      ss_->setStatePropagator(oc::ODESolver::getStatePropagator(
          solver_ptr, [this](const ob::State *s, const oc::Control *c, const double d, ob::State *r) {
            diffDriveKinematicPostIntegration(s, c, d, r);
          }));
    }
    else
    {
      ss_->setStatePropagator([this](const ob::State *s, const oc::Control *c, const double d,
                                     ob::State *r) { return diffDrivePropagation(s, c, d, r); });
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

    // const double mx(state->as<ob::SE2StateSpace::StateType>()->getX());
    // const double my(state->as<ob::SE2StateSpace::StateType>()->getY());
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
