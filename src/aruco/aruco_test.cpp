#include <carmen/carmen.h>
#include <iostream>
#include "aruco_interface.h"


void 
shutdown_module(int signo)
{
    if (signo == SIGINT)
    {
        carmen_ipc_disconnect();

        printf("Signal %d received, exiting program ...\n", signo);
        exit(0);
    }
}


void 
handler(carmen_aruco_message *msg)
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


void 
subscribe_messages()
{
    aruco_subscribe_message(1, NULL, (carmen_handler_t)handler, CARMEN_SUBSCRIBE_LATEST);
}


int 
main(int argc, char **argv)
{
    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);

    signal(SIGINT, shutdown_module);

    subscribe_messages();
    carmen_ipc_dispatch();

    return 0;
}
