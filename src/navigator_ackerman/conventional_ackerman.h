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

/** @addtogroup navigator libconventional **/
// @{

/** 
 * \file conventional.h 
 * \brief Library for shortest-path motion planning.
 *
 * ...
 **/


#ifndef CONVENTIONAL_ACKERMAN_H
#define CONVENTIONAL_ACKERMAN_H

#ifdef __cplusplus
extern "C" {
#endif

  /** Computes the utility function using dynamic programming. 
      carmen_conventional_build_costs must have been
      called first. **/ 
  void carmen_conventional_dynamic_program(int goal_x, int goal_y);
  /** Takes in the current position (as a map grid cell) and replaces
      the argument with the best neighbour grid cell to visit
      next. carmen_conventional_dynamic_program must have been
      called first. **/ 
  void carmen_conventional_find_best_action(carmen_map_point_p curpoint);
  /** Frees memory structures. **/ 
  void carmen_conventional_end_planner(void);
  /** Returns the utility function as an array of doubles with the
   same dimensions as the map in row-major order. **/ 
  double *carmen_conventional_get_utility_ptr(void);
  /** Returns the cost map as an array of doubles with the
   same dimensions as the map in row-major order. **/ 
  double *carmen_conventional_get_costs_ptr(void);
  /** Returns the value of the utility function at a specific point,
   that is, the cost-to-goal. **/ 
  double carmen_conventional_get_utility(int x, int y);
  /** Returns the value of the cost map at a specific point. **/ 
  double carmen_conventional_get_cost(int x, int y);
  /** Converts the occupancy grid map into a cost map by incorporating
      the width of the robot, and adding additional penalties for getting
      close to obstacles. **/ 
  void carmen_conventional_build_costs(carmen_robot_ackerman_config_t *robot_conf,
				       carmen_map_point_t *robot_posn,
				       carmen_navigator_config_t *navigator_conf);

#ifdef __cplusplus
}
#endif

#endif
// @}
