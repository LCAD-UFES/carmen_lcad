/***********************************
 * This file is part of astro system.
 * It is subject to the license terms in the LICENSE file found in the top-level directory
 * of this distribution.
 * Copyright (C) 2023 Lume Robotics
 *************************************/

#ifndef NLP_MAT_PLANNER_H
#define NLP_MAT_PLANNER_H

#include <stdbool.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/collision_detection.h>


#ifdef __cplusplus
extern "C" {
#endif


// A ideia aqui desse .h é apenas colocar métodos praticamente funcionais, na qual não precisa de variavel ou extern. Também structs utilizada por vários arquivos.

typedef struct
{
    carmen_robot_and_trailers_traj_point_t *points;
    int length;
    int capacity;
} offroad_planner_path_t;

typedef struct
{
    carmen_robot_and_trailers_traj_point_t robot;
    carmen_robot_and_trailers_traj_point_t goal;
    offroad_planner_path_t path;
    int goal_set;
} offroad_planner_plan_t;



typedef struct
{
    double goal_achieved_distance;
    double goal_achieved_theta;
    double goal_achieved_trailer_theta;
} goal_constraints_t;


void carmen_navigator_ackerman_goal_triplet(carmen_robot_and_trailers_traj_point_t *point);
void carmen_navigator_ackerman_goal(double x, double y, double theta);
int carmen_navigator_ackerman_goal_place(char *name);
void carmen_navigator_ackerman_set_max_velocity(double vel);
carmen_map_placelist_p carmen_navigator_ackerman_get_places(void);
int carmen_navigator_ackerman_autonomous_status(void);
void carmen_navigator_ackerman_start_autonomous(void);
bool smooth_path_using_conjugate_gradient(carmen_robot_and_trailers_traj_point_t *input_path, int &num_poses,
		carmen_obstacle_distance_mapper_map_message *obstacle_distance_map, double obstacles_safe_distance, double max_phi);
bool smooth_path_using_conjugate_gradient_study(carmen_robot_and_trailers_traj_point_t *input_path, int &num_poses,
		carmen_obstacle_distance_mapper_map_message *obstacle_distance_map, double obstacles_safe_distance, double max_phi, double w1, double w2, double w3, double w4, double w5);
int update_distance_map(double *utility_map, double *cost_map, int x_size, int y_size, int goal_x, int goal_y);
int update_distance_map_new(double *utility_map, double *cost_map, int x_size, int y_size, int goal_x, int goal_y);

bool is_anchor_point(int i, carmen_robot_and_trailers_traj_point_t *input_path, int size);

#ifdef __cplusplus
}
#endif

#endif
