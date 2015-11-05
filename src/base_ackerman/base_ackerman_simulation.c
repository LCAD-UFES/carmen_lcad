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
#include "base_ackerman.h"
#include "base_ackerman_simulation.h"
#include <carmen/collision_detection.h>



void
carmen_simulator_ackerman_recalc_pos(carmen_base_ackerman_config_t *car_config)
{
	carmen_point_t new_odom;
	double v, phi, t;

	new_odom = car_config->odom_pose;

	v = car_config->v;
	phi = car_config->phi;
	t = car_config->delta_t;

	new_odom.x +=  v * t * cos(new_odom.theta);
	new_odom.y +=  v * t * sin(new_odom.theta);
	new_odom.theta += v * t * (tan(phi) / car_config->distance_between_front_and_rear_axles);
	new_odom.theta = carmen_normalize_theta(new_odom.theta);

	car_config->odom_pose = new_odom;
}
