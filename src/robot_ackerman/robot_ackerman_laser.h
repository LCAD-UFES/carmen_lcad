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

#ifndef ROBOT_ACKERMAN_LASER_H
#define ROBOT_ACKERMAN_LASER_H

#ifdef __cplusplus
extern "C" {
#endif

void carmen_robot_ackerman_correct_laser_and_publish(void);
void carmen_robot_ackerman_add_laser_handlers(void);
void carmen_robot_ackerman_add_laser_parameters(int argc, char** argv);
double carmen_robot_ackerman_laser_max_front_velocity(void);
double carmen_robot_ackerman_laser_min_rear_velocity(void);

#ifdef __cplusplus
}
#endif

#endif
