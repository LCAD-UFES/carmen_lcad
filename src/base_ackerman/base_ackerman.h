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

#ifndef BASE_ACKERMAN_H
#define BASE_ACKERMAN_H

#include "base_ackerman_messages.h"
#include <carmen/localize_ackerman_motion.h>
#include <carmen/fused_odometry_messages.h>


#define NUM_MOTION_COMMANDS_VECTORS	5
#define	NUM_MOTION_COMMANDS_PER_VECTOR	200


#ifdef __cplusplus
extern "C" {
#endif


typedef struct 
{
	carmen_point_t odom_pose;

	double v;
	double phi;
	double delta_t;

	double width;
	double length;
	double distance_between_front_and_rear_axles;

	int publish_odometry;

} carmen_base_ackerman_config_t, *carmen_base_ackerman_config_p;

#ifdef __cplusplus
}
#endif

#endif
