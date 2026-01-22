#include <iostream>
#include <chrono>
#include <stdio.h>
#include <vector>
#include <string>
#include <fstream>
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
#define MAX_ARGS 2
#define DEFAULT_SENSOR_LOG_DATA_FILENAME "log_prometheus_sensor_data.txt"

//------------------------------------
//          CARMEN PARAMS
//------------------------------------
#define PHI_FILTER_CONSTANT 0.09 //0.03
#define VEL_FILTER_CONSTANT 0.08 //0.05 //0.01
#define WHEEL_AXIS_DISTANCE 0.450
#define VEL_THREASHOLD 0.05

enum ErrorCodes
{
    EXCEEDS_MAX_PARAMETER_NUM = -2,
    ERROR_OPENING_SENSOR_DATA_LOG_FILE,
};


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

    void Init(char** parameter_vec, int parameter_len);
    void ArgumentParser(char** parameter_vec, int parameter_len);

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
    bool log_sensor_data = false;
    long long start_carmen_time = 0;
    std::ofstream file;
};


void PrometheusOdomSubscriber::ArgumentParser(char** parameter_vec, int parameter_len)
{
    std::vector<std::string> args(parameter_vec, parameter_vec + parameter_len);
    for (const std::string & s : args)
    {
        if (s == "--log_sensor_data")
        {
            this->log_sensor_data = true;
            this->file.open(DEFAULT_SENSOR_LOG_DATA_FILENAME, std::ios::out | std::ios::app);
            if (!file.is_open())
            {
                throw std::runtime_error("Error Opening Sensor Data Log file!");
                exit(ERROR_OPENING_SENSOR_DATA_LOG_FILE);
            }
            this->start_carmen_time = (long long) carmen_get_time();
        }
    }
}


void PrometheusOdomSubscriber::Init(char** parameter_vec, int parameter_len)
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

    std::cout << "Getting Arguments" << std::endl;

    if (parameter_len > 1)
    {
        this->ArgumentParser(parameter_vec);
    }

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

    //printf("x_axis_vel = %f, _global_vel = %f \n", x_axis_vel, _global_vel);
    //fflush(stdout);

    struct can_frame frame;

    frame.can_id = 0x425; // Velocity ID
    frame.can_dlc = 4;  // 4 data bytes
    memset(frame.data, 0, 4);

    memcpy(&frame.data[0], &_global_vel, sizeof(float));

    if (write(_can_socket, &frame, sizeof(frame)) != sizeof(frame)) {
        throw std::runtime_error("Error when writing Velocity Bytes to CAN");
    }
    //printf("Sent %+4.2f via CAN ID 0x425\n", _global_vel);

    frame.can_id = 0x80; // Angle ID
    frame.can_dlc = 4;  // 4 data bytes
    memset(frame.data, 0, 4);

    memcpy(&frame.data[0], &_global_phi, sizeof(float));

    if (write(_can_socket, &frame, sizeof(frame)) != sizeof(frame)) {
        throw std::runtime_error("Error when writing Phi Bytes to CAN");
    }

    if (this->log_sensor_data)
    {
        if ( this->file.is_open() )
        {
            long long carmen_time = (long long) carmen_get_time();
            file << "VELOCITY (raw,filtered,timestamp): " << x_axis_vel << ", " << _global_vel << ", " << carmen_time - this->start_carmen_time << "\n";
            file << "STEERING (raw,filtered,timestamp): " << phi << ", " << _global_phi << ", " << carmen_time - this->start_carmen_time << "\n";
        }
    }
    //printf("Sent %+4.2f via CAN ID 0x80\n", _global_phi);
    //std::cout << "Timestamp =" << (long long) carmen_get_time() << "\n\n" << std::endl;
}

int main(int argc __attribute__ ((unused)), const char** argv __attribute__ ((unused)))
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
    if (argc > MAX_ARGS)
    {
        std::cout << "Usage: " << argv[0] << " --log_sensor_data" << std::endl;
        exit(EXCEEDS_MAX_PARAMETER_NUM); 
    }
    #endif



    PrometheusOdomSubscriber sub;
    sub.Init();

    while (1)
    {
        sleep(MAIN_LOOP_SLEEP_INTERVAL);
    }

    return 0;
}