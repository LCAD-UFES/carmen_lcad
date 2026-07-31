#pragma once

#include <cstdint>
#include <ros/time.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

namespace carmen_bridge {

// Converte timestamp CARMEN (segundos, epoch) para ros::Time sem offset.
inline ros::Time carmen_to_ros_time(double carmen_ts)
{
    ros::Time t;
    t.fromSec(carmen_ts);
    return t;
}

inline int64_t carmen_to_ros_ns(double carmen_ts)
{
    return static_cast<int64_t>(carmen_ts * 1e9);
}

inline ros::Time carmen_ns_to_ros_time(int64_t ns)
{
    if (ns < 0) ns = 0;
    const uint32_t sec  = static_cast<uint32_t>(ns / 1000000000LL);
    const uint32_t nsec = static_cast<uint32_t>(ns % 1000000000LL);
    return ros::Time(sec, nsec);
}

// Lê /use_sim_time uma vez (cache estático; suficiente para o ciclo de vida do nó).
inline bool use_sim_time()
{
    static bool cached = false;
    static bool value  = false;
    if (!cached) {
        ros::param::get("/use_sim_time", value);
        cached = true;
    }
    return value;
}

// Stamp para publicar TF: tempo do sensor no replay, relógio real caso contrário.
inline ros::Time tf_stamp(const ros::Time &sensor_stamp)
{
    if (use_sim_time())
        return sensor_stamp;
    return ros::Time::now();
}

inline void publish_clock(ros::Publisher &pub, const ros::Time &stamp)
{
    rosgraph_msgs::Clock msg;
    msg.clock = stamp;
    pub.publish(msg);
}

}  // namespace carmen_bridge
