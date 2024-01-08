// Written by Juan Pablo Gutierrez

#ifndef OMNI_DRIVE_CONTROLLER__CONFIG_HPP_
#define OMNI_DRIVE_CONTROLLER__CONFIG_HPP_

/* Units are in [meters], [meters/seconds], or [radians] */
namespace robot_params
{
    struct RobotConfig
    {
        double wheel_radius;
        double robot_radius;
        double theta = M_PI / 2;
    };

    struct RobotVelocity
    {
        double vx;
        double vy;
        double omega;
    };

    struct RobotPosition
    {
        double x;
        double y;
        double theta;
    };
}

#endif