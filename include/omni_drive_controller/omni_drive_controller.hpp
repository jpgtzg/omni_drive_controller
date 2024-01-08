/*
    Author: Juan Pablo Gutierrez
*/

#ifndef OMNI_DRIVE_CONTROLLER__OMNI_DRIVE_CONTROLLER_HPP_
#define OMNI_DRIVE_CONTROLLER__OMNI_DRIVE_CONTROLLER_HPP_

#include "omni_drive_controller/visibility_control.h"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "omni_drive_controller/config.hpp"
#include "omni_drive_controller/kinematics.hpp"
using namespace robot_params;
namespace omni_drive_controller
{
    class OmniDriveController : public controller_interface::ControllerInterface
    {

        using Twist = geometry_msgs::msg::TwistStamped;

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
        struct WheelHandle
        {
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command;
        };

        std::vector<WheelHandle> registered_left_wheel_handles_;
        std::vector<WheelHandle> registered_right_wheel_handles_;
        std::vector<WheelHandle> registered_front_wheel_handles_;
        std::vector<WheelHandle> registered_back_wheel_handles_;

        // Parameters

        // Default parameters

        std::vector<std::string> wheel_joint_names;

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

        std::chrono::milliseconds cmd_vel_timeout_{500};

        // Subscriber and publishers

        RobotConfig config_{0.05, 0.26};
        Kinematics kinematics_;

        bool subscriber_is_active_ = false;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
            velocity_command_subscriber_ = nullptr;

        realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};

    private:
        void velocityCommandUnstampedCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);
        geometry_msgs::msg::TwistStamped::SharedPtr cmd_vel_;
    };
}

#endif