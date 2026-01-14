#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <iomanip> 
#include <cmath>
#include <csignal>
#include <atomic>

#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

#include <carmen/carmen.h>

// ----CARMEN PARAM----
#define DEFAULT_FREQUENCY 200.0 //ojTorc CAN messages measured frequency * 4
#define MAX_EFFORT 25600.0
#define WHEEL_AXIS_DISTANCE 0.450

// ----G1 PARAM----
#define NETWORK_INTERFACE "eth0"
#define MAX_VELOCITY 0.7 // m/s
#define MAX_ANGLE 0.60 // rad
#define MAX_ANG_VEL 1.00 // rad/s

int _can_socket = 0;
int _global_phi_effort = 0;

std::vector<float> stringToFloatVector(const std::string &str) {
  std::vector<float> result;
  std::stringstream ss(str);
  float num;
  while (ss >> num) {
    result.push_back(num);
    // ignore any trailing whitespace
    ss.ignore();
  }
  return result;
}


void sleep_for_microseconds(double microseconds) {
    std::chrono::duration<double, std::micro> duration(microseconds);
    std::this_thread::sleep_for(duration);
}

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "Closing CAN Socket" << std::endl;
        if (_can_socket >= 0)
        {
            close(_can_socket);
            std::cout << "CAN Socket Closed" << std::endl;
        }
        std::cout << "Shutting  g1_can2move Driver Down" << std::endl;
    }
    exit(0);
}

int main(int argc, char const *argv[])
{
    std::signal(SIGINT, signal_handler);

    std::map<std::string, std::string> args = {{"network_interface", NETWORK_INTERFACE}};

    //If more params are declared or if you wanna override "network_interface"
    //Use like this -> ./g1_can2move --<param1>=<value1> ... --<param_n>=<value_n>
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.substr(0, 2) == "--")
        {
            size_t pos = arg.find("=");
            std::string key, value;

            if (pos != std::string::npos)
            {
                key = arg.substr(2, pos - 2);
                value = arg.substr(pos + 1);

                if (value.front() == '"' && value.back() == '"')
                {
                    value = value.substr(1, value.length() - 2);
                }
            }
            else
            {
                key = arg.substr(2);
                value = "";
            }

            if (args.find(key) != args.end())
            {
                args[key] = value;
            }
            else
            {
                args.insert({{key, value}});
            }
        }
    }

    unitree::robot::ChannelFactory::Instance()->Init(0,args["network_interface"]);

    unitree::robot::g1::LocoClient client;

    client.Init();
    client.SetTimeout(10.f);

    // Open CAN socket
    _can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (_can_socket < 0)
    {   
        throw std::runtime_error("Failed to open CAN socket");
    }

    // Set filter for CAN ID 0x100
    struct can_filter rfilter;
    rfilter.can_id = 0x100;
    rfilter.can_mask = CAN_SFF_MASK;
    setsockopt(_can_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

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
    std::cout << "CAN Opened!" << std::endl;

    // Configuring CAN to be non-blocking
    int flags = fcntl(_can_socket, F_GETFL, 0);

    if (flags == -1) throw std::runtime_error("fcntl(F_GETFL) failed");

    if (fcntl(_can_socket, F_SETFL, flags | O_NONBLOCK) == -1)  throw std::runtime_error("fcntl(F_SETFL) failed"); 

    struct can_frame frame;
    int can_read = 0;
    while(1)
    {
	    //std::cout << "Waiting CAN Message" << std::endl;    
        can_read = read(_can_socket, &frame, sizeof(struct can_frame));
	    //std::cout << "CAN Read!" << std::endl;
        if (can_read > 0)
        {
            // Communication with Carmen through CAN is little-endian
            int vel_effort = (frame.data[1] << 8) | frame.data[0];
            int steering_effort = (frame.data[3] << 8) | frame.data[2];

            float vel_percentage = (float) vel_effort / MAX_EFFORT;
            if (vel_percentage > 1.0) 
                vel_percentage = 1.0;
            else if (vel_percentage < -1.0)
                vel_percentage= -1.0;

            float vel = vel_percentage * MAX_VELOCITY;
            if (vel > MAX_VELOCITY)
                vel = MAX_VELOCITY;
            else if (vel < -MAX_VELOCITY)
                vel = -MAX_VELOCITY;

            printf("vel_effort = %d, vel_percentage = %f, vel = %f\n", vel_effort, vel_percentage, vel);

            _global_phi_effort += steering_effort;
            if (_global_phi_effort > MAX_EFFORT) _global_phi_effort = MAX_EFFORT;
            else if (_global_phi_effort < -MAX_EFFORT) _global_phi_effort = -MAX_EFFORT;
            //std::cout << "Receveid '_global_phi_effort' =  " << _global_phi_effort << std::endl;

            float phi = ( (float)_global_phi_effort/MAX_EFFORT)*MAX_ANGLE;
            if ( phi > MAX_ANGLE ) phi = MAX_ANGLE;
            else if ( phi < -MAX_ANGLE ) phi = -MAX_ANGLE;
            //std::cout << "Receveid 'phi' =  " << std::fixed << std::setprecision(2) << phi << std::endl;

            float ang_vel = atanf(phi) * vel / WHEEL_AXIS_DISTANCE;
            if (ang_vel > MAX_ANG_VEL) ang_vel = MAX_ANG_VEL;
            else if (ang_vel < -MAX_ANG_VEL) ang_vel = -MAX_ANG_VEL;
            //std::cout << "Receveid 'ang_vel' =  " << std::fixed << std::setprecision(2) << ang_vel << std::endl;


            float vx = vel;
            float vy = 0;
            float yaw  = ang_vel * -1; // Carmen yaw orientation is inverted

            //std::cout << "Published vx = " << vx
            //          << " Published vy = " << vy
            //          << " Published yaw = " << yaw
            //          << " Timestamp =" << (long long) carmen_get_time()
            //          <<"\n\n"<< std::endl;


            if (client.Move(vx, vy, yaw) != 0)
            {
                std::cout << " Error while sending 'Move' command" << std::endl;
                std::cout << " Could not execute 'Move' command" << std::endl;
                //return -1;
            }
        }
        // Disabled for better log visibility
        // else
        // {
        //     //std::cout << "No can messages to read" << std::endl;
        // }
        sleep_for_microseconds( (1.0/DEFAULT_FREQUENCY)*1e6 );
    }

    return 0;
}
