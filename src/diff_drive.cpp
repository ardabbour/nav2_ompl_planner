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

namespace nav2_ompl_planner
{

  DiffDriveControlSpace::DiffDriveControlSpace(const ob::StateSpacePtr &state_space)
      : oc::RealVectorControlSpace(state_space, 2) {}

  void diffDriveKinematicODE(const oc::ODESolver::StateType &q, const oc::Control *ctrl,
                             oc::ODESolver::StateType &qdot)
  {
    std::cout << "diffDriveKinematicODE here1" << std::endl;
    const double u = ctrl->as<oc::RealVectorControlSpace::ControlType>()->values[0];
    std::cout << "diffDriveKinematicODE here2" << std::endl;
    const double v = ctrl->as<oc::RealVectorControlSpace::ControlType>()->values[1];
    std::cout << "diffDriveKinematicODE here3" << std::endl;
    qdot.resize(q.size(), 0);
    std::cout << "diffDriveKinematicODE here4" << std::endl;
    qdot[0] = cos(q[2]) * u;
    std::cout << "diffDriveKinematicODE here5" << std::endl;
    qdot[1] = sin(q[2]) * u;
    std::cout << "diffDriveKinematicODE here6" << std::endl;
    qdot[2] = v;
    std::cout << "diffDriveKinematicODE here7" << std::endl;
  }

  void diffDriveKinematicPostIntegration(const ob::State *s __attribute__((unused)),
                                         const oc::Control *c __attribute__((unused)),
                                         const double d __attribute__((unused)), ob::State *result)
  {
    std::cout << "diffDriveKinematicPostIntegration here1" << std::endl;
    const ob::SO2StateSpace SO2;
    std::cout << "diffDriveKinematicPostIntegration here2" << std::endl;
    auto *so2 = result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1);
    std::cout << "diffDriveKinematicPostIntegration here3" << std::endl;
    SO2.enforceBounds(so2);
    std::cout << "diffDriveKinematicPostIntegration here4" << std::endl;
  }

  void diffDrivePropagation(const ob::State *start, const oc::Control *ctrl, const double duration,
                            ob::State *result)
  {
    std::cout << "diffDrivePropagation here1" << std::endl;
    const double x = start->as<ob::SE2StateSpace::StateType>()->getX();
    std::cout << "diffDrivePropagation here2" << std::endl;
    const double y = start->as<ob::SE2StateSpace::StateType>()->getY();
    std::cout << "diffDrivePropagation here3" << std::endl;
    const double yaw = start->as<ob::SE2StateSpace::StateType>()->getYaw();
    std::cout << "diffDrivePropagation here4" << std::endl;
    const double u = ctrl->as<oc::RealVectorControlSpace::ControlType>()->values[0];
    std::cout << "diffDrivePropagation here5" << std::endl;
    const double v = ctrl->as<oc::RealVectorControlSpace::ControlType>()->values[1];

    std::cout << "diffDrivePropagation here6" << std::endl;
    result->as<ob::SE2StateSpace::StateType>()->setXY(x + ((cos(yaw) * u) * duration),
                                                      y + ((sin(yaw) * u) * duration));
    std::cout << "diffDrivePropagation here7" << std::endl;
    result->as<ob::SE2StateSpace::StateType>()->setYaw(yaw + (v * duration));

    std::cout << "diffDrivePropagation here8" << std::endl;
    const ob::SO2StateSpace SO2;
    std::cout << "diffDrivePropagation here9" << std::endl;
    auto *so2 = result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1);
    std::cout << "diffDrivePropagation here10" << std::endl;
    SO2.enforceBounds(so2);
    std::cout << "diffDrivePropagation here11" << std::endl;
  }

} // namespace nav2_ompl_planner
