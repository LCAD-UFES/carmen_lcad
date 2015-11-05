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
#include "navigator_ackerman.h"
#include "conventional_ackerman.h" 

#define NUM_ACTIONS 8

carmen_robot_ackerman_config_t robot_conf_g;
extern carmen_map_t * carmen_planner_map;
extern int carmen_planner_x_offset[NUM_ACTIONS];
extern int carmen_planner_y_offset[NUM_ACTIONS];

#define MAX_UTILITY 1000.0
/* How much to reduce the cost per meter.
   Kind of arbitrary, but related to MAX_UTILITY */
#define MIN_COST 0.1

struct state_struct {
	int x, y;
	int entropy;
	double cost;
	int due_to_sensing;
};

typedef struct state_struct state;
typedef state *state_ptr;

typedef struct {
	state_ptr *data_array;
	int num_elements;
	int queue_size;
} queue_struct, *queue;

static int x_size, y_size;

static double *costs = NULL;
static double *utility = NULL;

carmen_inline static int 
is_out_of_map(int x, int y)
{
	return (x >= x_size || x < 0 || y >= y_size || y < 0);
}

carmen_inline static 
double *utility_value(int x, int y) 
{
	if (is_out_of_map(x, y))
		return NULL;

	return (utility + x*y_size + y);
}

static carmen_inline double
get_parent_value(queue the_queue, int index)
{
	int parent_index;
	int x, y;

	parent_index = carmen_trunc(index/2)-1;
	x = the_queue->data_array[parent_index]->x;
	y = the_queue->data_array[parent_index]->y;

	return *utility_value(x, y);
}

static carmen_inline queue 
make_queue(void) 
{
	queue new_queue;

	new_queue=(queue)calloc(1, sizeof(queue_struct));
	carmen_test_alloc(new_queue);

	new_queue->data_array=(state_ptr *)calloc(1, sizeof(state_ptr));
	carmen_test_alloc(new_queue->data_array);

	return new_queue;
}

static carmen_inline void
swap_entries(int x1, int x2, queue the_queue)
{
	state_ptr tmp;

	tmp = the_queue->data_array[x1-1];
	the_queue->data_array[x1-1] = the_queue->data_array[x2-1];
	the_queue->data_array[x2-1] = tmp;
}

static carmen_inline void
fix_priority_queue(queue the_queue)
{
	int left, right;
	int index;
	int largest;

	index = 1;

	while (index < the_queue->num_elements)
	{
		left = 2*index;
		right = 2*index+1;

		if (left <= the_queue->num_elements &&
				the_queue->data_array[left-1]->cost > the_queue->data_array[index-1]->cost)
			largest = left;
		else
			largest = index;
		if (right <= the_queue->num_elements &&
				the_queue->data_array[right-1]->cost > the_queue->data_array[largest - 1]->cost)
			largest = right;

		if (largest != index)
		{
			swap_entries(largest, index, the_queue);
			index = largest;
		}
		else
			break;
	}
}

static carmen_inline state_ptr 
pop_queue(queue the_queue) 
{
	state_ptr return_state;

	if (the_queue->num_elements == 0)
		return NULL;

	return_state = the_queue->data_array[0];

	the_queue->data_array[0] = the_queue->data_array[the_queue->num_elements-1];
	the_queue->num_elements--;

	fix_priority_queue(the_queue);

	return(return_state);
}

static carmen_inline void 
delete_queue(queue *queue_pointer) 
{
	queue the_queue;
	state_ptr cur_state;

	the_queue = *queue_pointer;

	if (the_queue == NULL)
		return;

	while (the_queue->num_elements > 0)
	{
		cur_state = pop_queue(the_queue);
		free(cur_state);
	}

	if (the_queue->queue_size > 0)
		free(the_queue->data_array);

	free(the_queue);
	queue_pointer = NULL;
}

static void
resize_queue(queue the_queue)
{
	if (the_queue->queue_size == 0)
	{
		the_queue->data_array=(state_ptr *)calloc(256, sizeof(state_ptr));
		carmen_test_alloc(the_queue->data_array);
		the_queue->queue_size = 256;
		the_queue->num_elements = 0;
		return;
	}

	/* If the queue is full, we had better grow it some. */

	if (the_queue->queue_size < the_queue->num_elements)
		return ;

	/* Realloc twice as much space */

	the_queue->data_array=(state_ptr *)
	realloc(the_queue->data_array, sizeof(state_ptr)*the_queue->queue_size*2);
	carmen_test_alloc(the_queue->data_array);

	the_queue->queue_size *= 2;
	memset(the_queue->data_array+the_queue->num_elements, 0,
			(the_queue->queue_size - the_queue->num_elements)*sizeof(state_ptr));
}

static carmen_inline void 
insert_into_queue(state_ptr new_state, queue the_queue) 
{
	int index;

	if (!the_queue->queue_size ||
			the_queue->queue_size == the_queue->num_elements)
		resize_queue(the_queue);

	the_queue->data_array[the_queue->num_elements] = new_state;
	the_queue->num_elements++;

	/* Fix up priority queue */

	index = the_queue->num_elements;

	while (index > 1 && get_parent_value(the_queue, index) < new_state->cost)
	{
		swap_entries(carmen_trunc(index/2), index, the_queue);
		index = carmen_trunc(index/2);
	}
}

void carmen_conventional_build_costs(carmen_robot_ackerman_config_t *robot_conf,
		carmen_map_point_t *robot_posn,
		carmen_navigator_config_t *navigator_conf)
{
	robot_conf_g = *robot_conf;
	int x_index = 0, y_index = 0;
	double value;
	double *cost_ptr;
	double *map_ptr;
	double resolution;
	int index;
	int x, y;
	int x_start, y_start;
	int x_end, y_end;

	if (!carmen_planner_map)
		return;

	carmen_verbose("Building costs...");

	if (x_size != carmen_planner_map->config.x_size ||
			y_size != carmen_planner_map->config.y_size) {
		free(costs);
		costs = NULL;
		free(utility);
		utility = NULL;
	}

	x_size = carmen_planner_map->config.x_size;
	y_size = carmen_planner_map->config.y_size;

	if (costs == NULL) {
		costs = (double *)calloc(x_size*y_size, sizeof(double));
		carmen_test_alloc(costs);
		for (index = 0; index < x_size*y_size; index++)
			costs[index] = carmen_planner_map->complete_map[index];
	}

	resolution = carmen_planner_map->config.resolution;

	if (robot_posn && navigator_conf) {
		x_start = robot_posn->x - navigator_conf->map_update_radius/
				robot_posn->map->config.resolution;
		x_start = carmen_clamp(0, x_start, x_size);
		y_start = robot_posn->y - navigator_conf->map_update_radius/
				robot_posn->map->config.resolution;
		y_start = carmen_clamp(0, y_start, y_size);
		x_end = robot_posn->x + navigator_conf->map_update_radius/
				robot_posn->map->config.resolution;
		x_end = carmen_clamp(0, x_end, x_size);
		y_end = robot_posn->y + navigator_conf->map_update_radius/
				robot_posn->map->config.resolution;
		y_end = carmen_clamp(0, y_end, y_size);

		cost_ptr = costs+x_start*y_size;
		map_ptr = carmen_planner_map->complete_map+x_start*y_size;
		for (x_index = x_start; x_index < x_end; x_index++) {
			for (y_index = y_start; y_index < y_end; y_index++)
				cost_ptr[y_index] = map_ptr[y_index];
			cost_ptr += y_size;
			map_ptr += y_size;
		}
	} else {
		x_start = 0;
		y_start = 0;
		x_end = x_size;
		y_end = y_size;

		for (index = 0; index < x_size*y_size; index++)
			costs[index] = carmen_planner_map->complete_map[index];
	}


	/* Initialize cost function to match map, where empty cells have
     MIN_COST cost and filled cells have MAX_UTILITY cost*/

  for (x_index = x_start; x_index < x_end; x_index++) {
    cost_ptr = costs+x_index*y_size+y_start;
    for (y_index = y_start; y_index < y_end; y_index++) {
      value = *(cost_ptr);
      if (value < 0.5)
      	value = 1.1 * MIN_COST;
      else if (//value >= 0 && disabled to allow exploration in SLAM mode
      		value < MIN_COST)
	value = MIN_COST;
      else
	value = MAX_UTILITY;
      *(cost_ptr++) = value;
    }
  }
	/* Loop through cost map, starting at top left, updating cost of cell
     as max of itself and most expensive neighbour less the cost 
     downgrade.*/

	for (x_index = x_start; x_index < x_end; x_index++) {
		cost_ptr = costs+x_index*y_size+y_start;
		for (y_index = y_start; y_index < y_end; y_index++, cost_ptr++) {
			if (x_index < 1 || x_index >= x_size-1 || y_index < 1 || y_index >= y_size-1)
				continue;

			for (index = 0; index < NUM_ACTIONS; index++) {
				x = x_index + carmen_planner_x_offset[index];
				y = y_index + carmen_planner_y_offset[index];

				//value = *(costs+x*y_size+y) - (resolution*MAX_UTILITY)/(robot_conf->length/2);
				value = *(costs + x * y_size + y) - (resolution*MAX_UTILITY) / (robot_conf->width / 1.2); // @@@ ler o 1.2 do carmen ini
				if (value > *cost_ptr)
					*cost_ptr = value;
			}
		}
	}

	/* Loop through cost map again, starting at bottom right, updating cost of
     cell as max of itself and most expensive neighbour less the cost 
     downgrade.*/

	for (x_index = x_end-1; x_index >= x_start; x_index--) {
		cost_ptr = costs+x_index*y_size+y_end-1;
		for (y_index = y_end-1; y_index >= y_start; y_index--, cost_ptr--) {
			if (x_index < 1 || x_index >= x_size-1 || y_index < 1 || y_index >= y_size-1)
				continue;

			for (index = 0; index < NUM_ACTIONS; index++) {
				x = x_index + carmen_planner_x_offset[index];
				y = y_index + carmen_planner_y_offset[index];

				//value = *(costs+x*y_size+y) - (resolution*MAX_UTILITY)/(robot_conf->length/2);
				value = *(costs+x*y_size+y) - (resolution*MAX_UTILITY)/(robot_conf->width / 1.2); // @@@ ler o 1.2 do carmen ini
				if (value > *cost_ptr)
					*cost_ptr = value;
			}
		}
	}

	/* Clamp any cell that's closer than the robot width to be
     impassable. Also, rescale cost function so that the max cost
     is 0.5*/
//width
//	int HALF_WIDTH = MAX_UTILITY - round(robot_conf->width/(2*resolution));
//	int HALF_LENGHT = MAX_UTILITY - round(robot_conf->length/(2*resolution));

	for (x_index = x_start; x_index < x_end; x_index++) {
		cost_ptr = costs+x_index*y_size+y_start;
		for (y_index = y_start; y_index < y_end; y_index++) {
			value = *(cost_ptr)/1000;
			//if(value != MIN_COST)
				if(value <= MIN_COST)
					value = MIN_COST;
				*(cost_ptr++) = value;
		}
	}

/*	for (x_index = x_start; x_index < x_end; x_index++) {
		cost_ptr = costs+x_index*y_size+y_start;
		for (y_index = y_start; y_index < y_end; y_index++) {
			value = *(cost_ptr);
			if(value == MAX_UTILITY)
				value =1;
			else
				if (value > HALF_WIDTH)
					value = 0.5;
				else
					if(value > HALF_LENGHT)
						value = 0.15;
					else value = 0.1;
			*(cost_ptr++) = value;
		}
	}*/

	//todo ANTIGO metodo de engordar
	/*
	for (x_index = x_start; x_index < x_end; x_index++) {
		cost_ptr = costs+x_index*y_size+y_start;
		for (y_index = y_start; y_index < y_end; y_index++) {
			value = *(cost_ptr);
			if(value == MAX_UTILITY)
				value =1;
			else
				if (value != MIN_COST)
					value = 0.5;
	 *(cost_ptr++) = value;
		}
	}
	 */
	if (robot_posn != NULL) {
		x_index = robot_posn->x / carmen_planner_map->config.resolution;
		y_index = robot_posn->y / carmen_planner_map->config.resolution;

		if (x_index >= 0 && x_index < x_size && y_index >= 0 && y_index < y_size)
			*(costs+x_index*y_size+y_index) = MIN_COST;
	}
	carmen_verbose("done\n");
}

double 
carmen_conventional_get_cost(int x, int y)  
{
	if (is_out_of_map(x, y))
		return -1;
	return *(costs+x*y_size + y);
}

double 
carmen_conventional_get_utility(int x, int y)  
{
	if (!utility)
		return -1;
	if (is_out_of_map(x, y))
		return -1;
	return *(utility+x*y_size + y);
}

double *
carmen_conventional_get_utility_ptr(void) 
{
	return utility;
}

double *
carmen_conventional_get_costs_ptr(void) 
{
	return costs;
}

static carmen_inline state_ptr 
carmen_conventional_create_state(int x, int y, double util) 
{
	state_ptr new_ptr = NULL;

	if (is_out_of_map(x, y))
		carmen_die("Bad New_State value. %d %d out of (%d %d)\n", x, y, x_size, y_size);

	new_ptr = (state_ptr)calloc(1, sizeof(state));
	carmen_test_alloc(new_ptr);

	new_ptr->x = x;
	new_ptr->y = y;
	new_ptr->cost = util;

	return new_ptr;
}

static carmen_inline void 
push_state(int x, int y, double new_utility, queue state_queue)
{
	state_ptr new_state = carmen_conventional_create_state(x, y, new_utility);
	insert_into_queue(new_state, state_queue);
	*(utility_value(x, y)) = new_utility;
	if (*(costs+x*y_size+y) > 0.9)
		carmen_warn("Bad utility\n");
} 

static carmen_inline void 
add_neighbours_to_queue(int x, int y, queue state_queue)
{
	int index;
	double cur_util, new_util, multiplier;
	int cur_x, cur_y;
	double parent_utility;

	parent_utility = *utility_value(x, y);

	for (index = 0; index < NUM_ACTIONS; index+=1)
	{
		if (index % 2 == 1)
			multiplier = M_SQRT2;
		else
			multiplier = 1;

		cur_x = x + carmen_planner_x_offset[index];
		cur_y = y + carmen_planner_y_offset[index];

		if (is_out_of_map(cur_x, cur_y) || *(costs+cur_x*y_size + cur_y) > 0.5)
			continue;

		cur_util = *(utility_value(cur_x, cur_y));
		//new_util = parent_utility - MIN_COST*multiplier;
		new_util = parent_utility - *(costs+cur_x*y_size + cur_y)*multiplier;

		assert(new_util > 0);
		if (cur_util < 0)
			push_state(cur_x, cur_y, new_util, state_queue);
	} /* End of for (Index = 0...) */

}

void 
carmen_conventional_dynamic_program(int goal_x, int goal_y) 
{
	double *utility_ptr;
	int index;
	double max_val, min_val;
	int num_expanded;

	struct timeval start_time, end_time;
	int delta_sec, delta_usec;
	static double last_time, cur_time, last_print;

	queue state_queue;
	state_ptr current_state;

	if (costs == NULL)
		return;

	gettimeofday(&start_time, NULL);

	cur_time = carmen_get_time();
	if ((int)(cur_time - last_print) > 10) {
		carmen_verbose("Time since last DP: %d secs, %d usecs\n", (int)(cur_time - last_time),
				((int)((cur_time-last_time)*1e6)) % 1000000);
		last_print = cur_time;
	}
	last_time = cur_time;

	if (utility == NULL) {
		utility = (double *)calloc(x_size*y_size, sizeof(double));
		carmen_test_alloc(utility);
	}

	utility_ptr = utility;
	for (index = 0; index < x_size * y_size; index++)
		*(utility_ptr++) = -1;

	if (is_out_of_map(goal_x, goal_y))
		return;

	max_val = -MAXDOUBLE;
	min_val = MAXDOUBLE;

	state_queue = make_queue();

	current_state = carmen_conventional_create_state(goal_x, goal_y, 0);
	max_val = MAX_UTILITY;
	*(utility_value(goal_x, goal_y)) = max_val;
	add_neighbours_to_queue(goal_x, goal_y, state_queue);
	num_expanded = 1;

	while ((current_state = pop_queue(state_queue)) != NULL) {
		num_expanded++;
		if (current_state->cost <= *(utility_value(current_state->x, current_state->y)))
			add_neighbours_to_queue(current_state->x, current_state->y, state_queue);
		if (current_state->cost < min_val)
			min_val = current_state->cost;
		free(current_state);
	}

	delete_queue(&state_queue);

	gettimeofday(&end_time, NULL);

	delta_usec = end_time.tv_usec - start_time.tv_usec;
	delta_sec = end_time.tv_sec - start_time.tv_sec;
	if (delta_usec < 0) {
		delta_sec--;
		delta_usec += 1000000;
	}

	//  carmen_warn("Elasped time for dp: %d secs, %d usecs\n", delta_sec, delta_usec);
}

void 
carmen_conventional_find_best_action(carmen_map_point_p curpoint) 
{
	double best_utility;
	int best_x = curpoint->x;
	int best_y = curpoint->y;
	int new_x, new_y;
	int index;
	double util = 0.0;

	if (utility == NULL)
		return;

	best_utility = *(utility_value(best_x, best_y));
	for (index = 0; index < NUM_ACTIONS; index++)
	{
		new_x = curpoint->x + carmen_planner_x_offset[index];
		new_y = curpoint->y + carmen_planner_y_offset[index];

		if (new_x < 0 || new_x >= carmen_planner_map->config.x_size ||
				new_y < 0 || new_y >= carmen_planner_map->config.y_size)
			continue;
		util = *(utility_value(new_x, new_y));
		if (util >= 0.0 && util > best_utility)
		{
			best_x = new_x;
			best_y = new_y;
			best_utility = util;
		}
	}

	curpoint->x = best_x;
	curpoint->y = best_y;

	return;
}

void 
carmen_conventional_end_planner(void) 
{
	if (costs != NULL)
		free(costs);
	if (utility != NULL)
		free(utility);

}
