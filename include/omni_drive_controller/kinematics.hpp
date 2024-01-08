#ifndef OMNI_DRIVE_CONTROLLER__KINEMATICS_HPP
#define OMNI_DRIVE_CONTROLLER__KINEMATICS_HPP

#include <vector>

#include "omni_drive_controller/config.hpp"

using namespace robot_params;

namespace omni_drive_controller
{

    class Kinematics
    {
    public:
        Kinematics(RobotConfig config_);

        // Forward kinematics
        RobotVelocity getRobotVelocity(const std::vector<double> &wheels_velocities);

        // Inverse Kinematics
        std::vector<double> getWheelsAngularVelocities(RobotVelocity robotVel);
    };
}

#endif OMNI_DRIVE_CONTROLLER__KINEMATICS_HPP