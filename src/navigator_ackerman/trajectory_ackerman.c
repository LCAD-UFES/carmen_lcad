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
#include "planner_ackerman_interface.h"
#include "trajectory_ackerman.h"

void
check_path_capacity(carmen_planner_path_p path) 
{	
	carmen_robot_and_trailers_traj_point_t *new_points = NULL;

	if (path->capacity == 0) {
		path->capacity = 20;
		path->points = (carmen_robot_and_trailers_traj_point_t *)
    		  calloc(path->capacity, sizeof(carmen_robot_and_trailers_traj_point_t));
		carmen_test_alloc(path->points);

	} else
	{
		while(path->length >= path->capacity)
		{
			path->capacity *= 2;

			new_points = (carmen_robot_and_trailers_traj_point_t *) realloc
					(path->points, sizeof(carmen_robot_and_trailers_traj_point_t)*path->capacity);
			carmen_test_alloc(new_points);
			path->points = new_points;
		}
	}
}

static void 
update_orientation(int index, carmen_planner_path_p path) 
{
	carmen_robot_and_trailers_traj_point_t *point, *next_point;

	if (index < 0 || index > path->length-2)
		return;

	point = path->points+index;
	next_point = path->points+index+1;

	point->theta = atan2(next_point->y - point->y, next_point->x - point->x);
}

int 
carmen_planner_util_add_path_point(carmen_robot_and_trailers_traj_point_t point,
		carmen_planner_path_p path)
{
	check_path_capacity(path);

	path->points[path->length] = point;

	path->length++;

	update_orientation(path->length-2, path);

	return path->length-1;
}

carmen_robot_and_trailers_traj_point_t *
carmen_planner_util_get_path_point(int index, carmen_planner_path_p path) 
{
	if (index >= path->length || index < 0)
	{
		carmen_warn("Bad path id in %s : requested %d but max id is %d.\n",
				__FUNCTION__, index, path->length-1);
		return NULL;
	}

	return path->points+index;
}

void 
carmen_planner_util_set_path_point(int index, carmen_robot_and_trailers_traj_point_t *path_point,
		carmen_planner_path_p path)
{  
	if (index >= path->length || index < 0)
	{
		carmen_warn("Bad path id in %s : requested %d but max id is %d.\n",
				__FUNCTION__, index, path->length-1);
		return;
	}
	path->points[index] = *path_point;

	update_orientation(index-1, path);

	return;
}

void 
carmen_planner_util_insert_blank(int index, carmen_planner_path_p path)
{
	int num_to_move;

	check_path_capacity(path);

	num_to_move = path->length - index;
	memmove(path->points+index+1, path->points+index,
			num_to_move*sizeof(carmen_robot_and_trailers_traj_point_t));

	path->length++;
}

void 
carmen_planner_util_insert_path_point(int index, 
		carmen_robot_and_trailers_traj_point_t *current_point,
		carmen_planner_path_p path)
{
	carmen_planner_util_insert_blank(index, path);
	carmen_planner_util_set_path_point(index, current_point, path);
}

void 
carmen_planner_util_set_path_velocities(int index, double v,
		double phi, carmen_planner_path_p path)
{
	if (index >= path->length || index < 0)
	{
		carmen_warn("bad path id in %s : requested %d but max id is %d.\n",
				__FUNCTION__, index, path->length-1);
		return;
	}
	path->points[index].v = v;
	path->points[index].phi = phi;

	return;
}

void 
carmen_planner_util_clear_path(carmen_planner_path_p path) 
{
	path->length = 0;
}

void 
carmen_planner_util_clip_path(int length, carmen_planner_path_p path) 
{
	path->length = length;
}

void 
carmen_planner_util_delete_path_point(int index, carmen_planner_path_p path)
{
	int num_to_move;

	if (index >= path->length || index < 0)
	{
		carmen_warn("bad path id in %s : requested %d but max id is %d.\n",
				__FUNCTION__, index, path->length-1);
		return;
	}

	num_to_move = path->length - index - 1;
	memmove(path->points+index,
			path->points+index+1, num_to_move*sizeof(carmen_robot_and_trailers_traj_point_t));

	path->length--;
}

void
carmen_planner_util_test_trajectory(carmen_planner_path_p path)
{
	int index;
	carmen_robot_and_trailers_traj_point_t point = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	carmen_robot_and_trailers_traj_point_t *point_p;

	carmen_planner_util_clear_path(path);
	if (path->length != 0)
		carmen_die("Test failed: clear should set length to 0, but length is %d\n",
				path->length);

	memset(&point, 0, sizeof(carmen_robot_and_trailers_traj_point_t));
	for (index = 0; index < 100; index++)
	{
		point.x = index;
		point.y = index;
		carmen_planner_util_add_path_point(point, path);
	}

	for (index = 0; index < 100; index++)
	{
		point_p = carmen_planner_util_get_path_point(index, path);
		if (point_p->x != index || point_p->y != index)
			carmen_die("Test failed: After 100 set points, get point on %d did not "
					"match: was %.0f %.0f\n", index, point_p->x, point_p->y);
	}

	point.x = 50.5;
	point.y = 50.5;

	carmen_planner_util_insert_blank(50, path);
	carmen_planner_util_set_path_point(50, &point, path);
	point_p = carmen_planner_util_get_path_point(50, path);
	if (fabs(point_p->x - 50.5) > 0.05 || fabs(point_p->y - 50.5) > 0.05)
		carmen_die("Blank then set failed.\n");

	if (path->length != 101)
		carmen_die("Length (%d) not 101 after insert_blank then set.\n",
				path->length);

	point.x = 60.5;
	point.y = 60.5;
	carmen_planner_util_insert_path_point(60, &point, path);

	if (fabs(point_p->x - 50.5) > 0.05 || fabs(point_p->y - 50.5) > 0.05)
		carmen_die("Blank then set failed.\n");

	if (path->length != 102)
		carmen_die("Length (%d) not 102 after insert_blank then set.\n",
				path->length);

	carmen_planner_util_delete_path_point(50, path);
	carmen_planner_util_delete_path_point(60, path);

	if (path->length != 100)
		carmen_die("Length (%d) not 100 after insert_blank then set.\n",
				path->length);

	carmen_planner_util_clip_path(50, path);

	for (index = 0; index < path->length; index++)
		carmen_planner_util_delete_path_point(index, path);

	for (index = 0; index < path->length; index++)
	{
		point_p = carmen_planner_util_get_path_point(index, path);
		if (point_p->x != 2*index+1 || point_p->y != 2*index+1)
			carmen_die("Test failed: After deleting even points, get point on %d "
					"(%d) did not match: was %.0f %.0f %.0f\n", 2*index+1, index,
					point_p->x, point_p->y,
					carmen_radians_to_degrees(point_p->theta));
	}
}

#if MAKING_TEST
int main(int argc __attribute__ ((unused)), 
		char *argv[] __attribute__ ((unused)))
{
	carmen_planner_path_t path = {NULL, 0, 0};

	carmen_planner_util_test_trajectory(&path);

	return 0;
}
#endif
