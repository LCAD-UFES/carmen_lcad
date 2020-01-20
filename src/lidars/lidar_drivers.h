#ifndef LIDAR_DRIVERS_H
#define LIDAR_DRIVERS_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string.h>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <signal.h>
#include <carmen/carmen.h>


void initizalize_socket_connection();

bool run_robosense_RSLiDAR16_driver(carmen_velodyne_variable_scan_message &msg);

#endif