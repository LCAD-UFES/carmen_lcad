#include <iostream>
#include <chrono>
#include <stdio.h>
#include <cmath>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>


#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#include <carmen/carmen.h>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_SPORT_STATE "rt/odommodestate"//high frequency
#define TOPIC_SPORT_LF_STATE "rt/lf/odommodestate"//low frequency
#define NETWORK_INTERFACE "eth0"
#define MAIN_LOOP_SLEEP_INTERVAL 10

//------------------------------------
//          CARMEN PARAMS
//------------------------------------
#define PHI_FILTER_CONSTANT 0.03
#define VEL_FILTER_CONSTANT 0.01
#define WHEEL_AXIS_DISTANCE 0.20
#define VEL_THREASHOLD 0.05


class PrometheusOdomSubscriber
{
public:
    explicit PrometheusOdomSubscriber()
    {}

    ~PrometheusOdomSubscriber()
    {
        std::cout << "Closing CAN Socket" << std::endl;
		if (_can_socket >= 0)
		{
            close(_can_socket);
			std::cout << "CAN Socket Closed" << std::endl;
        }
        std::cout << "Shutting Odom2Can Driver Down" << std::endl;
    }

    void Init();

private:

    /*high frequency message handler function for subscriber*/
    // void HighFreOdomMessageHandler(const void* messages);

    /*low frequency message handler function for subscriber*/
    void LowFreqOdomMessageHandler(const void* messages);

private:

    //unitree_go::msg::dds_::SportModeState_ estimator_state{};
    unitree_go::msg::dds_::SportModeState_ lf_estimator_state{};
    //ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> estimate_state_subscriber;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> lf_estimate_state_subscriber;
    int _can_socket = -1;
    float _global_phi = 0.0;
    float _global_vel = 0.0;
};


void PrometheusOdomSubscriber::Init()
{
    std::cout << "Starting Prometheus Odom2Can Driver" << std::endl;

    // Open CAN socket
    _can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (_can_socket < 0)
    {   
        throw std::runtime_error("Failed to open CAN socket");
    }

    // Specify CAN interface (e.g., can0)
    struct ifreq ifr;
    strcpy(ifr.ifr_name, "vcan0");
    if (ioctl(_can_socket, SIOCGIFINDEX, &ifr) < 0)
    {
        throw std::runtime_error("IOCTL Error. Check if 'vcan0' exists");
    }

    // Bind the socket to the CAN interface
    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(_can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        throw std::runtime_error("Failed to bind CAN socket");
    }

    /*create subscriber high freq*/
    //estimate_state_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_SPORT_STATE));
    //estimate_state_subscriber->InitChannel(std::bind(&Custom::HighFreOdomMessageHandler, this, std::placeholders::_1), 1);

    std::cout << "Subscribing to topic \'rt/lf/odommodestate\'" << std::endl;
    /*create low freq subscriber*/
    lf_estimate_state_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_SPORT_LF_STATE));
    lf_estimate_state_subscriber->InitChannel(std::bind(&PrometheusOdomSubscriber::LowFreqOdomMessageHandler, this, std::placeholders::_1), 1);

    std::cout << "Setup Complete" << std::endl;

}

// void Custom::HighFreOdomMessageHandler(const void* message)
// {
//     estimator_state = *(unitree_go::msg::dds_::SportModeState_*)message;

//     std::cout << "position info: " << std::endl;
//     std::cout << "x: " << estimator_state.position()[0] << std::endl;
//     std::cout << "y: " << estimator_state.position()[1] << std::endl;
//     std::cout << "z: " << estimator_state.position()[2] << std::endl;

//     std::cout << "velocity info: " << std::endl;
//     std::cout << "x: " << estimator_state.velocity()[0] << std::endl;
//     std::cout << "y: " << estimator_state.velocity()[1] << std::endl;
//     std::cout << "z: " << estimator_state.velocity()[2] << std::endl;

//     std::cout << "eular angle info: " << std::endl;
//     std::cout << "x: " << estimator_state.imu_state().rpy()[0] << std::endl;
//     std::cout << "y: " << estimator_state.imu_state().rpy()[1] << std::endl;
//     std::cout << "z: " << estimator_state.imu_state().rpy()[2] << std::endl;

//     std::cout << "yaw speed info: " << std::endl;
//     std::cout << estimator_state.yaw_speed() << std::endl;

//     std::cout << "Quaternion info: " << std::endl;
//     std::cout << "w: " << estimator_state.imu_state().quaternion()[0] << std::endl;
//     std::cout << "x: " << estimator_state.imu_state().quaternion()[1] << std::endl;
//     std::cout << "y: " << estimator_state.imu_state().quaternion()[2] << std::endl;
//     std::cout << "z: " << estimator_state.imu_state().quaternion()[3] << std::endl;
// }

void PrometheusOdomSubscriber::LowFreqOdomMessageHandler(const void* message)
{
    lf_estimator_state = *(unitree_go::msg::dds_::SportModeState_*)message;

    float x_axis_vel = lf_estimator_state.velocity()[0];
    float angular_vel = lf_estimator_state.yaw_speed();

    float phi;
    if (abs(x_axis_vel) < VEL_THREASHOLD)
    {
        phi = 0.0;
    }
    else
    {
        phi = atanf((angular_vel * WHEEL_AXIS_DISTANCE) / x_axis_vel);
    }
    _global_phi += PHI_FILTER_CONSTANT * (phi - _global_phi);
    _global_vel += VEL_FILTER_CONSTANT * (x_axis_vel - _global_vel);

    struct can_frame frame;
    uint8_t* data_ptr = reinterpret_cast<uint8_t*>(&_global_vel);

    frame.can_id = 0x425; // Velocity ID
    frame.can_dlc = 4;  // 4 data bytes
    memset(frame.data, 0, 4);

    frame.data[0] = data_ptr[3];
    frame.data[1] = data_ptr[2];
    frame.data[2] = data_ptr[1];
    frame.data[3] = data_ptr[0];

    if (write(_can_socket, &frame, sizeof(frame)) != sizeof(frame)) {
        throw std::runtime_error("Error when writing Velocity Bytes to CAN");
    }
    printf("Sent %+4.2f via CAN ID 0x425\n", _global_vel);

    frame.can_id = 0x80; // Angle ID
    frame.can_dlc = 4;  // 4 data bytes
    memset(frame.data, 0, 4);  

    data_ptr = reinterpret_cast<uint8_t*>(&_global_phi);
    frame.data[0] = data_ptr[3];
    frame.data[1] = data_ptr[2];
    frame.data[2] = data_ptr[1];
    frame.data[3] = data_ptr[0];

    if (write(_can_socket, &frame, sizeof(frame)) != sizeof(frame)) {
        throw std::runtime_error("Error when writing Phi Bytes to CAN");
    }
    printf("Sent %+4.2f via CAN ID 0x80\n", _global_phi);
    std::cout << "Timestamp =" << (long long) carmen_get_time() << "\n\n" << std::endl;
}

int main(int argc, const char** argv)
{
    #ifndef NETWORK_INTERFACE
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1); 
    }

    ChannelFactory::Instance()->Init(0, argv[1]);
    #else
    ChannelFactory::Instance()->Init(0, NETWORK_INTERFACE);
    #endif



    PrometheusOdomSubscriber sub;
    sub.Init();

    while (1)
    {
        sleep(MAIN_LOOP_SLEEP_INTERVAL);
    }

    return 0;
}