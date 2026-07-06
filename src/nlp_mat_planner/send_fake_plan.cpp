#include <carmen/carmen.h>
#include <carmen/global_planning_interface.h>
#include <algorithm>
#include <car_model.h>
#include <float.h>
#include <math.h>
#include <queue>
#include <list>
#include <vector>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <carmen/command.h>
#include <carmen/task_manager_interface.h>
#include <carmen/offroad_planner_messages.h>
#include <carmen/offroad_planner_interface.h>

carmen_robot_ackerman_config_t robot_config;
carmen_semi_trailers_config_t semi_trailer_config;

carmen_robot_and_trailers_traj_point_t robot_position;
int localize_message_received = 0;
int route_planner_message_received = 0;


typedef struct 
{
    double path_length;
    double phi;
    double velocity;
} ProgramArguments;

ProgramArguments args;

void 
parse_arguments(int argc, char **argv) 
{
    args.path_length = 0.0;
    args.phi = 0.0;
    args.velocity = 0.0;

    printf("Sending path with ");

    int option;
    while ((option = getopt(argc, argv, "s:p:v:")) != -1) 
    {
        printf("[(%c) ", option);
        switch (option) 
        {
            case 's':
                args.path_length = atof(optarg);
                printf("length ");
                break;
            case 'p':
                args.phi = atof(optarg);
                printf("phi ");
                break;
            case 'v':
                args.velocity = atof(optarg);
                printf("velocity ");
                break;
            default:
                printf("wrong_arg ");
                break;
        }
        printf("%s]; ", optarg);
    }
    printf("\n");
}


static void
carmen_localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
    robot_position = {msg->globalpos.x, msg->globalpos.y, msg->globalpos.theta, msg->num_trailers, {0.0}, msg->v, msg->phi};

    for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
    {
        robot_position.trailer_theta[z] = msg->trailer_theta[z];
    }

    localize_message_received = 1;

}


static void
carmen_route_planner_road_network_message_handler(carmen_route_planner_road_network_message *msg)
{
    if (msg->offroad_planner_request == PLAN_FROM_CURRENT_POSE_TO_FINAL_POSE)
    {
        route_planner_message_received = 1;
    }
}


static void
read_parameters(int argc, char **argv)
{
    int num_items;

    carmen_param_t param_list[] =
    {
        {(char *) "robot", 				(char *) "max_steering_angle",CARMEN_PARAM_DOUBLE, &robot_config.max_phi,1, NULL},
        {(char *) "robot",				(char *) "max_velocity", CARMEN_PARAM_DOUBLE, &robot_config.max_v, 1, NULL},
        {(char *) "robot",				(char *) "length", CARMEN_PARAM_DOUBLE, &robot_config.length, 0, NULL},
        {(char *) "robot",				(char *) "width", CARMEN_PARAM_DOUBLE, &robot_config.width, 0, NULL},
        {(char *) "robot",				(char *) "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_and_rear_axles, 1, NULL},
        {(char *) "robot", 				(char *) "understeer_coeficient", CARMEN_PARAM_DOUBLE, &robot_config.understeer_coeficient, 1, NULL},
        {(char *) "semi_trailer",	 	        (char *) "initial_type", CARMEN_PARAM_INT, &(semi_trailer_config.num_semi_trailers), 0, NULL},
        // {(char *) "offroad",			(char *) "planner_max_phi_multiplier", CARMEN_PARAM_DOUBLE, &max_phi_multiplier, 1, NULL},
    };

    num_items = sizeof(param_list)/sizeof(param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);

    if (semi_trailer_config.num_semi_trailers > 0)
    {
        carmen_task_manager_read_semi_trailer_parameters(&semi_trailer_config, argc, argv, semi_trailer_config.num_semi_trailers);
    }

    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) (carmen_localize_ackerman_globalpos_message_handler), CARMEN_SUBSCRIBE_LATEST);
    carmen_route_planner_subscribe_road_network_message(NULL, (carmen_handler_t) carmen_route_planner_road_network_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


void
create_plan(ProgramArguments *args)
{
    std::vector<carmen_robot_and_trailers_traj_point_t> path;
    path.push_back(robot_position);

    // printf("Check args: %lf %lf %lf\n", args->path_length, args->phi, args->velocity); 
    double distance = 0.0;
    while (distance < args->path_length)
    {
        double distance_traveled = 0.0;
        carmen_robot_and_trailers_traj_point_t point = path[path.size() - 1];
        point.v = args->velocity;
        point.phi = args->phi;
        point = carmen_libcarmodel_recalc_pos_ackerman(point, args->velocity, args->phi, 1.0, &distance_traveled, 0.05, robot_config, semi_trailer_config);

        path.push_back(point);
        distance += distance_traveled;
    }

    // printf("Path size: %ld\n", path.size());
    // for (unsigned int i = 0; i < path.size(); i++)
    // {
    //     printf("[%d] %lf %lf %lf %lf\n", i, path[i].x, path[i].y, path[i].theta, path[i].phi);
    // }

    
    carmen_offroad_planner_plan_message plan;
    plan.offroad_planner_feedback = PLAN_OK;
    plan.number_of_poses = path.size();
    plan.poses = &path[0];
    plan.pose_id = -1;
    plan.transition_pose = path[path.size() - 1];
    plan.goal_pose = path[path.size() - 1];

    carmen_offroad_planner_publish_plan(&plan);
    
}


int 
main(int argc, char **argv)
{
    printf("Use this module with route_planner on and offroad_planner OFF\n");
    printf("Run this program and then click in 'Compute Route' in navigator_gui. If it doesn't work, put a final goal, task as Park, and click and Compute Route (after running this program)\n");
    printf("Example usage: ./send_fake_plan -v -1 -p 0.45 -s 50.0\n");
    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);

    read_parameters(argc, argv);
    parse_arguments(argc, argv);

    while (1)
    {
        carmen_ipc_sleep(0.5);
        if (localize_message_received == 1 && (route_planner_message_received == 1))
        {
            break;
        }
    }

    create_plan(&args);


}
