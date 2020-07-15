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

#ifndef NAV2_OMPL_PLANNER__DIFF_DRIVE_HPP_
#define NAV2_OMPL_PLANNER__DIFF_DRIVE_HPP_


#include "ompl/control/ODESolver.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"

namespace nav2_ompl_planner {

namespace ob = ompl::base;
namespace oc = ompl::control;

/**
 * Differential-drive robot control space
 */
class DiffDriveControlSpace : public oc::RealVectorControlSpace {
 public:
  explicit DiffDriveControlSpace(const ob::StateSpacePtr& state_space);
};

/**
 * Uses the ODESolver to propagate the differential drive robot state
 * @param q the initial state
 * @param control the control pointer
 * @param qdot the state after applying the control
 */
void diffDriveKinematicODE(const oc::ODESolver::StateType& q, const oc::Control* control,
                           oc::ODESolver::StateType& qdot);

/**
 * Performs post-integration operations
 * @param state the start state pointer
 * @param control the control pointer
 * @param duration the amount of time for which to apply the control (s)
 * @param result the result state pointer
 */
void diffDriveKinematicPostIntegration(const ob::State* state, const oc::Control* control,
                                       const double duration, ob::State* result);

/**
 * Propagates the state forward in time
 * @param path_geometric the path returned in OMPL format
 * @param path the path in ROS format
 */
void diffDrivePropagation(const ob::State* start, const oc::Control* control, const double duration,
                          ob::State* result);

}  // namespace nav2_ompl_planner

#endif  // NAV2_OMPL_PLANNER__DIFF_DRIVE_HPP_
