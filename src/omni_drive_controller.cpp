// Written by Juan Pablo Gutierrez

#include "omni_drive_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
namespace
{
    constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
    constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
    constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
    constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
    constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
    constexpr auto WHEELS_QUANTITY = 4;
} // namespace

using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using std::placeholders::_1;

using namespace std;
namespace omni_drive_controller
{
    OmniDriveController::OmniDriveController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn OmniDriveController::on_init()
    {

        this->node_ = get_node();
        try
        {
            controller_interface::CallbackReturn result = ControllerInterface::on_init();

            auto_declare<vector<string>>("wheel_names", vector<string>());

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
        auto logger = node_->get_logger();

        // Update parameters if they have changed
        wheel_joint_names = node_->get_parameter("wheel_joint_names").as_string_array();

        wheel_radius = node_->get_parameter("wheel_radius").as_double();
        robot_radius = node_->get_parameter("robot_radius").as_double();

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

        double registered_wheel_radius = node_->get_parameter("wheel_radius").as_double();

        if (registered_wheel_radius <= 0)
        {
            RCLCPP_ERROR(
                logger, "The registered wheel radius is: [%f], while it should be greater than 0",
                registered_wheel_radius);
            return controller_interface::CallbackReturn::ERROR;
        }

        double registered_robot_radius = node_->get_parameter("robot_radius").as_double();

        if (registered_robot_radius <= 0)
        {
            RCLCPP_ERROR(
                logger, "The registered robot radius is: [%f], while it should be greater than 0",
                registered_robot_radius);
            return controller_interface::CallbackReturn::ERROR;
        }

        // Init command subscriber node
        velocity_command_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            DEFAULT_COMMAND_UNSTAMPED_TOPIC,
            rclcpp::SystemDefaultsQoS(),
            std::bind(&OmniDriveController::velocityCommandUnstampedCallback, this, _1));

        // Init publisher node

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration OmniDriveController::command_interface_configuration() const
    {
        std::vector<std::string> conf_names;

        for (const auto &joint_name : wheel_joint_names)
        {
            conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
        }

        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::InterfaceConfiguration OmniDriveController::state_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        for (const auto &joint_name : wheel_joint_names)
        {
            conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
        }
        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::CallbackReturn OmniDriveController::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        // Implement on_activate method
        subscriber_is_active_ = true;
    }

    controller_interface::CallbackReturn OmniDriveController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        // Implement on_deactivate method
        subscriber_is_active_ = false;
    }

    controller_interface::return_type OmniDriveController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // Implement update method
    }

    void OmniDriveController::velocityCommandUnstampedCallback(
        const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
    {
        if (!subscriber_is_active_)
        {
            RCLCPP_WARN(node_->get_logger(), "Can't accept new commands. subscriber is inactive");
            return;
        }

        this->cmd_vel_->twist = *cmd_vel;
        this->cmd_vel_->header.stamp = node_->get_clock()->now();
    }

}