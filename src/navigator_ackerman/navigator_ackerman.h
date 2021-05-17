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

#ifndef NAVIGATOR_ACKERMAN_H
#define NAVIGATOR_ACKERMAN_H

#include <carmen/behavior_selector_interface.h>


#ifdef __cplusplus
extern "C" {
#endif

  typedef struct {
    int num_lasers_to_use;
    int use_fast_laser;
    double map_update_radius;
    int map_update_obstacles;
    int map_update_freespace;
    double replan_frequency;
    int smooth_path;
    double waypoint_tolerance;
    double goal_size;
    double goal_theta_tolerance;
    int dont_integrate_odometry;
    int plan_to_nearest_free_point;
    char *navigator_planning_method;
  } carmen_navigator_config_t;

  typedef struct {
    double path_interval;
    int state_map_resolution;
    int state_map_theta_resolution;
    int precomputed_cost_size;
    char *precomputed_cost_file_name;
    int use_rs;
    int smooth_path;
    double onroad_max_plan_time;
    double robot_fat_space;

  } carmen_navigator_ackerman_astar_t;


  void carmen_navigator_ackerman_goal_triplet(carmen_robot_and_trailer_traj_point_t *point);
  void carmen_navigator_ackerman_goal(double x, double y, double theta);
  int carmen_navigator_ackerman_goal_place(char *name);
  void carmen_navigator_ackerman_set_max_velocity(double vel);
  carmen_map_placelist_p carmen_navigator_ackerman_get_places(void);
  int carmen_navigator_ackerman_autonomous_status(void);

  void carmen_navigator_ackerman_start_autonomous(void);


  typedef struct {
    double tv;
    double rv;
  } command_t;

#ifdef __cplusplus
}
#endif

#endif
