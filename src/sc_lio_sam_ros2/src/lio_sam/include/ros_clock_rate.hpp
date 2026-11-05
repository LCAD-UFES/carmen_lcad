#pragma once
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <type_traits>
#include <stdexcept>
#include <utility>

// Foxy's Rate cannot use the node clock. Poll it without blocking shutdown.
class ros_clock_rate
{
public:
    ros_clock_rate(double frequency, rclcpp::Clock::SharedPtr clock)
        : clock_(std::move(clock)), period_(rclcpp::Duration::from_seconds(1.0 / frequency)),
          last_interval_(clock_->now())
    {
    }

    bool
    sleep()
    {
        constexpr std::chrono::nanoseconds poll_interval = std::chrono::milliseconds(10);
        auto now = clock_->now();
        auto next_interval = last_interval_ + period_;
        if (now < last_interval_)
            next_interval = now + period_;
        last_interval_ = last_interval_ + period_;
        if (now >= next_interval)
        {
            if (now > next_interval + period_)
                last_interval_ = now + period_;
            return false;
        }
        while (rclcpp::ok())
        {
            const auto previous_time = now;
            now = clock_->now();
            if (now < previous_time)
            {
                last_interval_ = now;
                next_interval = now + period_;
            }
            if (now >= next_interval)
                return true;
            const auto remaining = std::chrono::nanoseconds((next_interval - now).nanoseconds());
            rclcpp::sleep_for(std::min(remaining, poll_interval));
        }
        return false;
    }

private:
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Duration period_;
    rclcpp::Time last_interval_;
};

template<class Rate = rclcpp::Rate>
auto
create_ros_rate(double frequency, const rclcpp::Clock::SharedPtr &clock)
{
    if (!std::isfinite(frequency) || frequency <= 0.0)
        throw std::invalid_argument("ROS rate frequency must be finite and positive");
    if constexpr (std::is_constructible<Rate, double, rclcpp::Clock::SharedPtr>::value)
        return std::make_shared<Rate>(frequency, clock);
    else
        return std::make_shared<ros_clock_rate>(frequency, clock);
}
