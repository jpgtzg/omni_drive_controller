/*
    Author: Juan Pablo Gutierrez
*/

#ifndef OMNI_DRIVE_CONTROLLER__OMNI_DRIVE_CONTROLLER_HPP_
#define OMNI_DRIVE_CONTROLLER__OMNI_DRIVE_CONTROLLER_HPP_

#include "omni_drive_controller/visibility_control.h"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"



namespace omni_drive_controller
{
    class OmniDriveController : public controller_interface::ControllerInterface
    {
    public:
        OmniDriveController();

        OMNI_DRIVE_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        OMNI_DRIVE_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        OMNI_DRIVE_CONTROLLER_PUBLIC
        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        OMNI_DRIVE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        OMNI_DRIVE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        OMNI_DRIVE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        OMNI_DRIVE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        OMNI_DRIVE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        OMNI_DRIVE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

    protected:
       /*  struct WheelHandle
        {
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
        };
 */
        double wheel_radius;
        double robot_radius;
        
        std::vector<std::string> wheel_joint_names;
        std::vector<std::string> interface_names_;
    };
}

#endif