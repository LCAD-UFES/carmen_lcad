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

/*******************************************
 * library with the functions for the guts *
 * of the simulator                        *
 *******************************************/

#ifndef SIMULATOR_ACKERMAN_SIMULATION_H
#define SIMULATOR_ACKERMAN_SIMULATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* recalculates the actual position */

void carmen_simulator_ackerman_recalc_pos(carmen_simulator_ackerman_config_t *simulator_config);

/* produce a laser message based upon the current
   position */

void carmen_simulator_ackerman_calc_laser_msg(carmen_laser_laser_message *flaser,
		carmen_simulator_ackerman_config_t *simulator_config, int is_rear_laser);

#ifdef __cplusplus
}
#endif

#endif
