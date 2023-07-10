#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include "aruco_interface.h"
#include "aruco_messages.h"
#include <carmen/carmen.h>
#include <localize_ackerman_interface.h>
#include <localize_ackerman_messages.h>
#include <playback_messages.h>
#include <playback_interface.h>
#include <global.h>

//#include "gnuplot_i.hpp"

std::ofstream arq_aruco("dados_aruco.txt");
std::ofstream arq_velodyne("dados_velodyne.txt");

double tsi = 0.0; //timestamp inicial
double tsp = 0.0; //timestamp do playback
bool playback_start = false;

void shutdown_module(int signo)
{
    if (signo == SIGINT)
    {
        carmen_ipc_disconnect();

        printf("Signal %d received, exiting program ...\n", signo);
        exit(0);
    }
}

/*
void handler(carmen_aruco_message *msg)
{
    printf("timestamp: %lf\n", msg->timestamp);
    // std::cout << "n_poses: "  << msg->n_poses << std::endl;
    for (size_t i = 0; i < msg->n_poses; i++)
    {
        // std::cout << "n_markers_detected: " << msg->poses[i].n_markers_detected << std::endl;
        if (msg->poses[i].n_markers_detected == 1)
            std::cout << "id: ";
        else
            std::cout << "ids: ";
        for (size_t j = 0; j < msg->poses[i].n_markers_detected; j++)
            std::cout << msg->poses[i].ids_markers_detected[j] << " ";
        std::cout << std::endl;
        
        std::cout << "R: " << msg->poses[i].rvec[0] << " " << msg->poses[i].rvec[1] << " " << msg->poses[i].rvec[2] << " " << std::endl;
        std::cout << "T: "  << msg->poses[i].tvec[0] << " " << msg->poses[i].tvec[1] << " " << msg->poses[i].tvec[2] << " " << std::endl;
    }
    std::cout << std::endl;
}
*/
/*
void grafico_beta()
{
    std::ifstream arq_aruco1("dados_aruco.txt");
    std::ifstream arq_velodyne1("dados_velodyne.txt");

    if(arq_aruco1.is_open() && arq_velodyne1.is_open(()))
    {
        std::vector<float> beta_aruco;
        std::vector<float> beta_velodyne;
        std::string aux;

        while(std::getline(arq_aruco1, aux))
        {
            beta_aruco.push_back(std::stof(aux));
        }

        while(std::getline(arq_velodyne1, aux))
        {
            beta_velodyne.push_back(std::stof(aux));
        }
    }
}
*/


void handler_playback(carmen_playback_info_message *msg)
{
    if(!playback_start)
    {
        tsi = msg->message_timestamp;
        playback_start = true;
    }
    tsp = msg->message_timestamp;
}


void handler_aruco(carmen_aruco_message *msg)
{
    //for (size_t i = 0; i < msg->n_poses; i++)
    //{   
    std::cout << "Beta - Aruco:" << std::endl;
    std::cout << -msg->poses[0].rvec[1] << std::endl;
    std::cout << std::endl;

    if(arq_aruco.is_open() && playback_start)
    {
        //arq_aruco << msg->poses[i].rvec[0] << " " << msg->poses[i].rvec[1] << " " << msg->poses[i].rvec[2] << std::endl;
        arq_aruco << tsp - tsi << "," << -msg->poses[0].rvec[1] << std::endl;
    }
    //}
}

void handler_velodyne(carmen_localize_ackerman_globalpos_message *msg)
{
    if(msg->num_trailers == 1)
    {
        double beta = convert_theta1_to_beta(msg->globalpos.theta, msg->trailer_theta[0]);

        std::cout << "Beta - Velodyne:" << std::endl;
        std::cout << beta << std::endl;
        std::cout << std::endl;

        if(arq_velodyne.is_open() && playback_start) // && ((msg->timestamp - timestamp_inicial)>=0) )
        {
            arq_velodyne << tsp - tsi << "," << beta << std::endl;
        }
    }
}


int main(int argc, char **argv)
{   
    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);

    signal(SIGINT, shutdown_module);

    carmen_subscribe_playback_info_message(NULL, (carmen_handler_t) handler_playback,  CARMEN_SUBSCRIBE_LATEST);
    aruco_subscribe_message(1, NULL, (carmen_handler_t) handler_aruco, CARMEN_SUBSCRIBE_LATEST);
    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) handler_velodyne, CARMEN_SUBSCRIBE_LATEST);

    carmen_ipc_dispatch();
    
    arq_aruco.close();
    arq_velodyne.close();

    return 0;
}