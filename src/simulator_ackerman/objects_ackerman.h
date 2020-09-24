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

/****************************************
 * library to create and control random *
 * two-legged people                    *
 ****************************************/

#ifndef OBJECTS_ACKERMAN_H
#define OBJECTS_ACKERMAN_H

#include "simulator_ackerman.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  carmen_simulator_ackerman_object_t type;
  int is_robot;
  double x1, y1;
  double x2, y2;
  double theta;
  double width;
  double tv, rv;
  IPC_CONTEXT_PTR context;
  double time_of_last_update;
} carmen_object_ackerman_t;

/* creates a new objects at the given position */
void carmen_simulator_ackerman_create_object(double x, double y, double theta,
				    carmen_simulator_ackerman_object_t type,
				    double speed);
void carmen_simulator_ackerman_add_robot(char *program_name, char *robot_central);

/* updates all objects */
void 
carmen_simulator_ackerman_update_objects(carmen_simulator_ackerman_config_t *simulator_config);
/* modifies the laser reading to account for objects near the robot */
void carmen_simulator_ackerman_add_objects_to_laser
(carmen_laser_laser_message * laser,
 carmen_simulator_ackerman_config_t *simulator_config, int is_rear);

void carmen_simulator_ackerman_initialize_object_model(int argc, char *argv[]);

/* clears all objects */
void carmen_simulator_objects_clear_objects(void);

/* gets the positions of all objects */
void carmen_simulator_ackerman_get_object_poses(int * num, 
				       carmen_traj_point_t ** coordinates);

void carmen_simulator_ackerman_get_objects(int *num, carmen_simulator_ackerman_objects_t **objects);

int carmen_simulator_object_too_close(double x, double y, int skip);

#ifdef __cplusplus
}
#endif

#endif
