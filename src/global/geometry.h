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


/** @addtogroup global libgeometry **/
// @{

/** \file geometry.h
 * \brief Library with geometric operations.
 *
 * Library for geometric operations
 **/

#ifndef GLOBAL_GEOMETRY_H
#define GLOBAL_GEOMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  Computes the minimum forward distance an obstacle must be from the robot
 *  to avoid a collision, given its current configuration and velocity.
 *
 *  @param robot_config Contains the robot dimensions and safety parameters.
 *  @param robot Contains the robot pose (not used) and the current robot
 *  velocity (required).
 */

double carmen_geometry_compute_safety_distance(carmen_robot_config_t *robot_config,
		carmen_traj_point_t *robot);

double carmen_geometry_compute_safety_ackerman_distance(carmen_robot_ackerman_config_t *robot_ackerman_config,
		carmen_traj_point_t *robot);

void carmen_geometry_compute_centre_and_curvature(carmen_traj_point_t start_point, double theta, 
		carmen_traj_point_t end_point,
		carmen_traj_point_t *centre, double *radius);

/* 
   Compute the maximum allowable velocity from the current pose of the 
   robot, given by robot, constrained by an obstacle at dest_pt, and the
   configuration of the robot given by robot_config.

   If the point is behind the robot, then the robot is unconstrained and can
   go max_velocity. 

 */

double carmen_geometry_compute_velocity(carmen_traj_point_t robot, carmen_traj_point_t dest_pt, 
		carmen_robot_config_t *robot_config);

double carmen_geometry_compute_ackerman_velocity(carmen_traj_point_t robot,
		carmen_traj_point_t dest_pt,
		carmen_robot_ackerman_config_t *robot_ackerman_config);

double carmen_geometry_compute_radius_and_centre(carmen_traj_point_p prev, carmen_traj_point_p current,
		carmen_traj_point_p next, carmen_traj_point_p centre,
		carmen_traj_point_p end_curve);

void carmen_geometry_move_pt_to_rotating_ref_frame(carmen_traj_point_p obstacle_pt, 
		double tv, double rv);

#ifndef COMPILE_WITHOUT_MAP_SUPPORT 
/* 
   Project a ray from (x, y) in direction theta to the edge of the map defined
   by map_defn. Stores the co-ordinate of the last point in the map in 
   (*x2, *y2).

   For example, if the map is 100x100, then project_point(50, 50, M_PI/2) 
   should give back (50, 100).

 */

void carmen_geometry_project_point(int x, int y, double theta, int *x2, int *y2, 
		carmen_map_config_t map_defn);

void carmen_geometry_generate_laser_data(double *laser_data, carmen_traj_point_p traj_point,
		double start_theta, double end_theta, int num_points,
		carmen_map_p map);

void carmen_geometry_generate_sonar_data(double *sonar_data, carmen_traj_point_p center,
		carmen_point_p sonar_offsets, int num_sonars,
		carmen_map_p map);

void carmen_geometry_fast_generate_laser_data(double *laser_data, carmen_traj_point_p traj_point,
		double start_theta, double end_theta, int num_points,
		carmen_map_p map);

double carmen_geometry_compute_expected_distance(carmen_traj_point_p traj_point, double theta, 
		carmen_map_p map);

void carmen_geometry_cache_stats(int *hits, int *misses);

#define CARMEN_NUM_OFFSETS 8
extern int carmen_geometry_x_offset[];
extern int carmen_geometry_y_offset[];

void carmen_geometry_map_to_cspace(carmen_map_p map, carmen_robot_config_t *robot_conf);
#endif

#ifdef __cplusplus
}
#endif

#endif
// @}

