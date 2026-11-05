#include "carmen_ros_bridge/runtime.hpp"
#include LT_MAPPER_RUNTIME
#include "ros_clock_rate.hpp"
#include <future>

#include <chrono>
#include <thread>
#include <cmath>
#include <stdexcept>

struct ClockReceiver
{
    bool received = false;

    void
    receive(rosgraph_msgs::msg::Clock::ConstSharedPtr message)
    {
        received = message->clock.sec == 42;
    }
};

static void
require(bool condition, const std::string &message)
{
    if (!condition)
        throw std::runtime_error(message);
}

int
main(int argc, char **argv)
{
    rclcpp::contexts::get_global_default_context()->init(argc, argv);
    try
    {
        require(!rclcpp::signal_handlers_installed(), "ROS installed the module's signal handlers");
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            rclcpp::Parameter("integer_value", 3),
            rclcpp::Parameter("double_value", 2.5),
            rclcpp::Parameter("invalid_value", "invalid"),
            rclcpp::Parameter("required_zero", 0.0),
            rclcpp::Parameter("required_array", std::vector<double>{1.0, 2.0})});
        bridge_node = std::make_shared<rclcpp::Node>("bridge_parameters_test", options);
        lt_mapper::node = std::make_shared<rclcpp::Node>("mapper_parameters_test", options);

        double value = -1.0;
        require(!carmen_bridge::required_parameter("missing", value), "Missing required parameter accepted");
        require(value == -1.0, "Missing required parameter changed its destination");
        require(carmen_bridge::required_parameter("required_zero", value) && value == 0.0,
            "Required zero rejected");
        std::vector<double> array;
        require(carmen_bridge::required_parameter("required_array", array) &&
            array == std::vector<double>({1.0, 2.0}), "Required array changed");

        lt_mapper::parameter("integer_value", value, 0.0);
        require(value == 3.0, "Integer YAML override rejected for a real parameter");
        lt_mapper::parameter("double_value", value, 0.0);
        require(value == 2.5, "Double override changed");
        lt_mapper::parameter("fallback", value, 4.5);
        require(value == 4.5, "Default changed");
        float float_value = 0.0;
        lt_mapper::parameter("integer_value", float_value, 0.0);
        require(float_value == 3.0, "Float conversion changed");
        bool invalid_rejected = false;
        try
        {
            lt_mapper::parameter("invalid_value", value, 0.0);
        }
        catch (const std::exception &)
        {
            invalid_rejected = true;
        }
        require(invalid_rejected, "Invalid numeric parameter accepted");

        ClockReceiver receiver;
        auto subscription = lt_mapper::node->create_subscription<rosgraph_msgs::msg::Clock>(
            "compatibility_clock", 10, std::bind(&ClockReceiver::receive, &receiver, std::placeholders::_1));
        auto publisher = bridge_node->create_publisher<rosgraph_msgs::msg::Clock>("compatibility_clock", 10);
        rosgraph_msgs::msg::Clock message;
        message.clock.sec = 42;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!receiver.received && std::chrono::steady_clock::now() < deadline)
        {
            publisher->publish(message);
            rclcpp::spin_some(lt_mapper::node);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        require(receiver.received, "ConstSharedPtr callback did not receive its message");

        auto broadcaster = carmen_bridge::static_broadcaster();
        require(broadcaster != nullptr, "Static transform broadcaster initialization failed");
        auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        require(rcl_enable_ros_time_override(clock->get_clock_handle()) == RCL_RET_OK,
            "Cannot enable the simulated clock");
        auto rate = create_ros_rate(10.0, clock);
        auto sleeping = std::async(std::launch::async, [&rate] { return rate->sleep(); });
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        const bool paused = sleeping.wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout;
        rcl_set_ros_time_override(clock->get_clock_handle(), 1000000000LL);
        const bool advanced = sleeping.wait_for(std::chrono::seconds(1)) == std::future_status::ready;
        if (!advanced)
            rclcpp::shutdown();
        sleeping.wait();
        require(paused && advanced, "Rate did not follow the simulated clock");

        auto paused_rate = create_ros_rate(10.0, clock);
        auto stopping = std::async(std::launch::async, [&paused_rate] { return paused_rate->sleep(); });
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        rclcpp::shutdown();
        require(stopping.wait_for(std::chrono::seconds(1)) == std::future_status::ready,
            "Rate blocked shutdown with a paused clock");
        stopping.get();
        lt_mapper::node.reset();
        bridge_node.reset();
        rclcpp::shutdown();
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ros_compatibility"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    return 0;
}
