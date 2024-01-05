// Written by Juan Pablo Gutierrez

#include "omni_drive_controller.hpp"

namespace
{
    constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
    constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
    constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
    constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
    constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
    constexpr auto WHEELS_QUANTITY = 4;
} // namespace

using namespace std;
namespace omni_drive_controller
{
    OmniDriveController::OmniDriveController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn OmniDriveController::on_init()
    {
        try
        {
            controller_interface::CallbackReturn result = ControllerInterface::on_init();

            auto_declare<vector<string>>("wheel_joint_names", vector<string>());
            auto_declare<vector<string>>("interface_names_", vector<string>());

            auto_declare<double>("wheel_radius", wheel_radius);
            auto_declare<double>("robot_radius", robot_radius);

            if (result != controller_interface::CallbackReturn::SUCCESS)
            {
                throw std::runtime_error("Failed to initialize parent ControllerInterface");
            }
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn OmniDriveController::on_configure(const rclcpp_lifecycle::State &)
    {
        auto logger = get_node()->get_logger();

        // Update parameters if they have changed
        wheel_joint_names = get_node()->get_parameter("wheel_joint_names").as_string_array();
        interface_names_ = get_node()->get_parameter("interface_names_").as_string_array();        

        wheel_radius = get_node()->get_parameter("wheel_radius").as_double();
        robot_radius = get_node()->get_parameter("robot_radius").as_double();

        // Verify parameter status

        if (wheel_joint_names.empty())
        {
            RCLCPP_ERROR(logger, "Wheel joint names parameters are empty!");
            return controller_interface::CallbackReturn::ERROR;
        }

        if (wheel_joint_names.size() != WHEELS_QUANTITY)
        {
            RCLCPP_ERROR(
                logger, "The number of wheels [%zu] and the required [%d] are different",
                wheel_joint_names.size(), WHEELS_QUANTITY);
            return controller_interface::CallbackReturn::ERROR;
        }

        double registered_wheel_radius = get_node()->get_parameter("wheel_radius").as_double();

        if (registered_wheel_radius <= 0)
        {
            RCLCPP_ERROR(
                logger, "The registered wheel radius is: [%f], while it should be greater than 0",
                registered_wheel_radius);
            return controller_interface::CallbackReturn::ERROR;
        }

        double registered_robot_radius = get_node()->get_parameter("robot_radius").as_double();

        if (registered_robot_radius <= 0)
        {
            RCLCPP_ERROR(
                logger, "The registered robot radius is: [%f], while it should be greater than 0",
                registered_robot_radius);
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    controller_interface::InterfaceConfiguration OmniDriveController::command_interface_configuration() const
    {
        // Implement command_interface_configuration
    }

    controller_interface::InterfaceConfiguration OmniDriveController::state_interface_configuration() const
    {
        // Implement state_interface_configuration
    }

    controller_interface::CallbackReturn OmniDriveController::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        // Implement on_activate method
    }

    controller_interface::CallbackReturn OmniDriveController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        // Implement on_deactivate method
    }

    controller_interface::return_type OmniDriveController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // Implement update method
    }

}