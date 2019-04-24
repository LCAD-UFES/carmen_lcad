/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#ifndef CONVENTIONAL_ASTAR_ACKERMAN_H
#define CONVENTIONAL_ASTAR_ACKERMAN_H
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/collision_detection.h>

	typedef struct {
	    double f_score;
	    double g_score;
	    double h_score;
	    int range;
	    int status;
	    int astar_call_cont;
	    double direction;
	    struct fibheap_el* fh_node;
	    carmen_ackerman_traj_point_t point;
	    carmen_ackerman_traj_point_t prev_point;
	}carmen_astar_node_t, *carmen_astar_node_p;

#define ORIENTATION_LENGHT 3
#define DIRECTION_LENGHT 2

enum NODE_STATUS {OPEN, CLOSE};
/** The data structure **/

#include "FH/fib.hpp"
#include "trajectory_ackerman.h"

class AstarAckerman
{


public:

	AstarAckerman()
	{
		astar_call_cont = 0;
		cont_nos_abertos_novos = 0;
		cont_nos_abertos_alterados = 0;
		cont_nos_podados = 0;
		cont_nos_abertos_alterados_fechados = 0;
		heap = NULL;
	}

	carmen_behavior_selector_state_t current_state;//TODO extern
	carmen_map_t *carmen_planner_map;//TODO extern
	carmen_navigator_ackerman_astar_t astar_config;//TODO extern
	carmen_obstacle_distance_mapper_map_message *distanceMap = NULL;

	int astar_call_cont;
	int cont_nos_abertos_novos;
	int cont_nos_abertos_alterados;
	int cont_nos_podados;
	int cont_nos_abertos_alterados_fechados;

	carmen_robot_ackerman_config_t robot_conf_g;

	carmen_astar_node_p ***astar_map;
	double ***precomputed_cost_map;
	struct fibheap* heap;

	carmen_ackerman_traj_point_t GOAL;

	double ORIENTATION[ORIENTATION_LENGHT];
	double DIRECTION[DIRECTION_LENGHT];

	carmen_ackerman_traj_point_t carmen_conventional_astar_ackerman_kinematic(carmen_ackerman_traj_point_t point, double lenght, double phi, double v);
	void open_node(carmen_astar_node_p tree);
	void carmen_conventional_astar_ackerman_astar(carmen_ackerman_traj_point_t start, carmen_ackerman_traj_point_t goal,carmen_planner_path_p path, carmen_robot_ackerman_config_t *robot_conf);
	void add_list_fh(carmen_astar_node_p new_state);
	carmen_astar_node_p open_node_fh();
	void get_astar_path(carmen_astar_node_p node, carmen_planner_path_p path);
	int hitObstacle(carmen_ackerman_traj_point_t point);
	double h_score(carmen_ackerman_traj_point_t point);
	double calc_delta_theta(double theta1, double theta2);
	void free_astar_map();
	void clean_astar_map();
	void alloc_astar_map();
	void alloc_precomputed_cost_map();
	int open_precomputed_cost_map();
	void smooth_path_astar(carmen_planner_path_p path);

private:
	int get_astar_map_x(double x);
	int get_astar_map_y(double y);
	carmen_ackerman_traj_point_t carmen_conventional_astar_ackerman_kinematic_2(carmen_ackerman_traj_point_t point, float lenght, float phi, float v);
	int rs_get_astar_path(int rs_pathl, carmen_ackerman_traj_point_p points, carmen_planner_path_p path);
	void astar_init_parameters(carmen_ackerman_traj_point_t goal);
	int get_astar_map_theta(double theta);


};





#endif
