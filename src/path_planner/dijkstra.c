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

#include <carmen/carmen.h>
#include <assert.h>
#include "dijkstra.h"
#include "planner_ackerman_interface.h"
#include "conventional_ackerman.h"
//#include "conventional_astar_ackerman.h"
#include "FH/fh.h"
//#include "navigator_ackerman_ipc.h"
//#include "navigator_ackerman.h"
#include "trajectory_ackerman.h"

#define NUM_ACTIONS 8

double **dijkstra_map;
FH* dijkstra_heap = NULL;

extern double *utility;


extern carmen_map_t *carmen_planner_map;
int carmen_planner_x_offset[NUM_ACTIONS] = {0, 1, 1, 1, 0, -1, -1, -1};
int carmen_planner_y_offset[NUM_ACTIONS] = {-1, -1, 0, 1, 1, 1, 0, -1};

void dijkstra(carmen_ackerman_traj_point_t goal){
	alloc_dijkstra_map();
	init_dijkstra_map();
	FH_NODE* node_fh;
	carmen_dijkstra_node_p node = malloc(sizeof(carmen_dijkstra_node_t));
	node->x=round((goal.x - carmen_planner_map->config.x_origin) / carmen_planner_map->config.resolution);
	node->y=round((goal.y - carmen_planner_map->config.y_origin) / carmen_planner_map->config.resolution);
	node->score=0;

	dijkstra_heap = FH_MAKE_HEAP();
    add_point(node);
    int i =0;
	while((node_fh = FH_EXTRACT_MIN(dijkstra_heap))){
		node = node_fh->PTR;
		free(node_fh);
		open_dijkstra_node(node);
		i++;
	}
	dijkstra_map_to_utility();
	//printf("i: %d\n",i);
}

void open_dijkstra_node(carmen_dijkstra_node_p node){
	int new_x, new_y;
	int i;
	carmen_dijkstra_node_p new_node;
	for (i = 0; i < NUM_ACTIONS; i++)
	{
		new_x = node->x + carmen_planner_x_offset[i];
		new_y = node->y + carmen_planner_y_offset[i];

		if (new_x < 0 || new_x >= carmen_planner_map->config.x_size || new_y < 0 || new_y >= carmen_planner_map->config.y_size)
			continue;
		if(carmen_conventional_get_cost(new_x,new_y)>=0.5)
			continue;
		new_node = malloc(sizeof(carmen_ackerman_traj_point_t));
		new_node->x = new_x;
		new_node->y = new_y;
		new_node->score = dijkstra_map[node->x][node->y]+1;
		carmen_test_alloc(new_node);
		add_point(new_node);
	}
}

void add_point(carmen_dijkstra_node_p new_node){
	FH_NODE* fh_node = NULL;
	if(new_node->score < dijkstra_map[new_node->x][new_node->y] || dijkstra_map[new_node->x][new_node->y] <0){
		dijkstra_map[new_node->x][new_node->y] = new_node->score;
		fh_node = FH_MAKE_NODE(round(dijkstra_map[new_node->x][new_node->y]*10000));
		FH_INSERT(dijkstra_heap, fh_node);
		fh_node->PTR=new_node;
	}
}

void alloc_dijkstra_map(){
	int i;
	dijkstra_map = (double **)calloc(carmen_planner_map->config.x_size, sizeof(double*));
	carmen_test_alloc(dijkstra_map);
	for(i=0;i<carmen_planner_map->config.x_size;i++){
		dijkstra_map[i] = (double *)calloc(carmen_planner_map->config.y_size, sizeof(double));
		carmen_test_alloc(dijkstra_map[i]);
	}
}

void init_dijkstra_map(){
	int i,j;
	for(i=0;i<carmen_planner_map->config.x_size;i++){
		for(j=0;j<carmen_planner_map->config.y_size;j++)
			dijkstra_map[i][j]=-1;
	}
}

void dijkstra_map_to_utility(){
	int i,j;
	for(i=0;i<carmen_planner_map->config.x_size;i++){
		for(j=0;j<carmen_planner_map->config.y_size;j++)
			if(dijkstra_map[i][j]>0)
			*(utility + i*carmen_planner_map->config.y_size + j) = 1000-dijkstra_map[i][j]*carmen_planner_map->config.resolution;
	}
}
