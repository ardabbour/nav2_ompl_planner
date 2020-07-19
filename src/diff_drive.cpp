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

#include "nav2_ompl_planner/diff_drive.hpp"

#include "ompl/base/spaces/SE2StateSpace.h"

namespace nav2_ompl_planner {

DiffDriveControlSpace::DiffDriveControlSpace(const ob::StateSpacePtr &state_space)
    : oc::RealVectorControlSpace(state_space, 2) {}

void diffDriveKinematicODE(const oc::ODESolver::StateType &q, const oc::Control *ctrl,
                           oc::ODESolver::StateType &qdot) {
  const double u = ctrl->as<oc::RealVectorControlSpace::ControlType>()->values[0];
  const double v = ctrl->as<oc::RealVectorControlSpace::ControlType>()->values[1];
  qdot.resize(q.size(), 0);
  qdot[0] = cos(q[2]) * u;
  qdot[1] = sin(q[2]) * u;
  qdot[2] = v;
}

void diffDriveKinematicPostIntegration(const ob::State *s __attribute__((unused)),
                                       const oc::Control *c __attribute__((unused)),
                                       const double d __attribute__((unused)), ob::State *result) {
  const ob::SO2StateSpace SO2;
  auto *so2 = result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1);
  SO2.enforceBounds(so2);
}

void diffDrivePropagation(const ob::State *start, const oc::Control *ctrl, const double duration,
                          ob::State *result) {
  const double x = start->as<ob::SE2StateSpace::StateType>()->getX();
  const double y = start->as<ob::SE2StateSpace::StateType>()->getY();
  const double yaw = start->as<ob::SE2StateSpace::StateType>()->getYaw();
  const double u = ctrl->as<oc::RealVectorControlSpace::ControlType>()->values[0];
  const double v = ctrl->as<oc::RealVectorControlSpace::ControlType>()->values[1];

  result->as<ob::SE2StateSpace::StateType>()->setXY(x + ((cos(yaw) * u) * duration),
                                                    y + ((sin(yaw) * u) * duration));
  result->as<ob::SE2StateSpace::StateType>()->setYaw(yaw + (v * duration));

  const ob::SO2StateSpace SO2;
  auto *so2 = result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1);
  SO2.enforceBounds(so2);
}

}  // namespace nav2_ompl_planner
