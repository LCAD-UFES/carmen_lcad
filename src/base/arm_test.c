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
 * Public License along with Foobar; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <carmen/carmen.h>
#include <carmen/arm_interface.h>


static int num_joints = 4;
static double joints[10];


void arm_state_handler(carmen_arm_state_message *arm_state)
{
  printf("received arm state message:\n"
	 "  - num_joints:  %d\n"
	 "  - joint_angles:  [ %.2f %.2f %.2f ]\n"
	 "  - timestamp:  %.2f\n\n",
	 arm_state->num_joints, arm_state->joint_angles[0],
	 arm_state->joint_angles[1], arm_state->joint_angles[2],
	 arm_state->timestamp);
}


int main(int argc, char **argv)
{

  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);

  if (argc < 5)
    carmen_die("usage: %s <joint1 position> <joint2 position> <joint3 position> <joint4 position>\n", argv[0]);

  joints[0] = atof(argv[1]);
  joints[1] = atof(argv[2]);
  joints[2] = atof(argv[3]);
  joints[3] = atof(argv[4]);

  carmen_arm_command(num_joints, joints);

  carmen_arm_subscribe_state_message(NULL, (carmen_handler_t) arm_state_handler, CARMEN_SUBSCRIBE_LATEST);

  carmen_ipc_dispatch();

  return 0;
}
