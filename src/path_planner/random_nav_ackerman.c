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
#include "navigator_ackerman_messages.h"
#include "navigator_ackerman_interface.h"

static carmen_map_t *c_space;
static int up_to_speed = 0;

static void pick_random_place(void)
{
  carmen_map_point_t map_pt;
  carmen_world_point_t world_pt;
  map_pt.map = c_space;

  do {
    map_pt.x = carmen_uniform_random(0, c_space->config.x_size);
    map_pt.y = carmen_uniform_random(0, c_space->config.y_size);

  } while (c_space->map[map_pt.x][map_pt.y] < 0);

  carmen_map_to_world(&map_pt, &world_pt);
  world_pt.pose.theta = carmen_uniform_random(0, 2*M_PI);

  carmen_navigator_ackerman_set_goal(world_pt.pose.x, world_pt.pose.y, world_pt.pose.theta);
  carmen_navigator_ackerman_go();
  up_to_speed = 0;
}

static void autonomous_stopped_handler
(carmen_navigator_ackerman_autonomous_stopped_message *autonomous_stopped
 __attribute__ ((unused)))
{
  pick_random_place();
}

static void odometry_handler(carmen_base_ackerman_odometry_message odometry)
{
  if (!up_to_speed && odometry.v > .25)//todo verify
    up_to_speed = 1;
  else if (up_to_speed && odometry.v < .2)
    pick_random_place();
}

void check_free_space_from_navigator(carmen_map_p c_space)
{
  float *map_ptr;
  float *utility_ptr;
  carmen_map_t utility_map;
  int x, y;
  carmen_map_placelist_t placelist;
  carmen_point_t mean, std_dev;

  if (carmen_map_get_placelist(&placelist) < 0)
    carmen_die("The map must have at least one place in it.\n");

  if (placelist.num_places <= 0)
    carmen_die("The map must have at least one place in it.\n");

  carmen_navigator_ackerman_set_goal_place(placelist.places[0].name);
  mean.x = placelist.places[0].x;
  mean.y = placelist.places[0].y;
  mean.theta = placelist.places[0].theta;
  std_dev.x = 0.2;
  std_dev.y = 0.2;
  std_dev.theta = carmen_degrees_to_radians(4.0);

  carmen_localize_ackerman_initialize_gaussian_command(mean, std_dev);

  carmen_navigator_ackerman_get_map(CARMEN_NAVIGATOR_ACKERMAN_UTILITY_v, &utility_map);

  map_ptr = c_space->complete_map;
  utility_ptr = utility_map.complete_map;
  for (x = 0; x < c_space->config.x_size; x++)
    for (y = 0; y < c_space->config.y_size; y++) {
      if (*utility_ptr < 0)
	*map_ptr = -1.0;
      utility_ptr++;
      map_ptr++;
    }

  free(utility_map.complete_map);
  free(utility_map.map);
  sleep(1);
}

int main(int argc, char *argv[]) 
{
  carmen_ipc_initialize(argc, argv);
  carmen_randomize(&argc, &argv);

  c_space = (carmen_map_t *)calloc(1, sizeof(carmen_map_t));
  carmen_test_alloc(c_space);

  carmen_map_get_gridmap(c_space);

  carmen_navigator_ackerman_subscribe_autonomous_stopped_message
    (NULL, (carmen_handler_t)autonomous_stopped_handler,
     CARMEN_SUBSCRIBE_LATEST);
  
  carmen_base_ackerman_subscribe_odometry_message
    (NULL, (carmen_handler_t)odometry_handler, CARMEN_SUBSCRIBE_LATEST);
  
  pick_random_place();
  carmen_ipc_dispatch();

  return 0;
}
