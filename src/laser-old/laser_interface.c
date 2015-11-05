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
#include <carmen/laser_interface.h>

void
carmen_laser_subscribe_frontlaser_message(carmen_laser_laser_message *laser,
					  carmen_handler_t handler,
					  carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_LASER_FRONTLASER_NAME, 
			   CARMEN_LASER_FRONTLASER_FMT,
			   laser, sizeof(carmen_laser_laser_message), handler,
			   subscribe_how);
}

void
carmen_laser_unsubscribe_frontlaser_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_LASER_FRONTLASER_NAME, handler);
}

void
carmen_laser_subscribe_rearlaser_message(carmen_laser_laser_message *laser,
					 carmen_handler_t handler,
					 carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_LASER_REARLASER_NAME, 
			   CARMEN_LASER_REARLASER_FMT,
			   laser, sizeof(carmen_laser_laser_message), handler,
			   subscribe_how);
}

void
carmen_laser_unsubscribe_rearlaser_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_LASER_REARLASER_NAME, handler);
}

void
carmen_laser_subscribe_laser1_message(carmen_laser_laser_message *laser,
				      carmen_handler_t handler,
				      carmen_subscribe_t subscribe_how)
{
  carmen_laser_subscribe_frontlaser_message(laser, handler, subscribe_how);
}

void
carmen_laser_subscribe_laser2_message(carmen_laser_laser_message *laser,
				      carmen_handler_t handler,
				      carmen_subscribe_t subscribe_how)
{
  carmen_laser_subscribe_rearlaser_message(laser, handler, subscribe_how);
}

void
carmen_laser_subscribe_laser3_message(carmen_laser_laser_message *laser,
				      carmen_handler_t handler,
				      carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_LASER_LASER3_NAME, 
			   CARMEN_LASER_LASER3_FMT,
			   laser, sizeof(carmen_laser_laser_message), handler,
			   subscribe_how);
}

void
carmen_laser_unsubscribe_laser3_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_LASER_LASER3_NAME, handler);
}

void
carmen_laser_subscribe_laser4_message(carmen_laser_laser_message *laser,
				      carmen_handler_t handler,
				      carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_LASER_LASER4_NAME, 
			   CARMEN_LASER_LASER4_FMT,
			   laser, sizeof(carmen_laser_laser_message), handler,
			   subscribe_how);
}

void
carmen_laser_subscribe_laser5_message(carmen_laser_laser_message *laser,
				      carmen_handler_t handler,
				      carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_LASER_LASER5_NAME, 
			   CARMEN_LASER_LASER5_FMT,
			   laser, sizeof(carmen_laser_laser_message), handler,
			   subscribe_how);
}

void
carmen_laser_unsubscribe_laser4_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_LASER_LASER4_NAME, handler);
}

void
carmen_laser_unsubscribe_laser5_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_LASER_LASER5_NAME, handler);
}

void
carmen_laser_subscribe_alive_message(carmen_laser_alive_message *alive,
				     carmen_handler_t handler,
				     carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_LASER_ALIVE_NAME, 
			   CARMEN_LASER_ALIVE_FMT,
			   alive, sizeof(carmen_laser_alive_message), handler,
			   subscribe_how);
}

void
carmen_laser_unsubscribe_alive_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_LASER_ALIVE_NAME, handler);
}




void
carmen_laser_define_frontlaser_message() {
  carmen_ipc_define_test_exit(CARMEN_LASER_FRONTLASER_NAME,
			      CARMEN_LASER_FRONTLASER_FMT);
}

void
carmen_laser_define_rearlaser_message() {
  carmen_ipc_define_test_exit(CARMEN_LASER_REARLASER_NAME,
			      CARMEN_LASER_REARLASER_FMT);
}

void
carmen_laser_define_laser3_message() {
  carmen_ipc_define_test_exit(CARMEN_LASER_LASER3_NAME,
			      CARMEN_LASER_LASER3_FMT);
}

void
carmen_laser_define_laser4_message() {
  carmen_ipc_define_test_exit(CARMEN_LASER_LASER4_NAME,
			      CARMEN_LASER_LASER4_FMT);
}

void
carmen_laser_define_laser5_message() {
  carmen_ipc_define_test_exit(CARMEN_LASER_LASER5_NAME,
			      CARMEN_LASER_LASER5_FMT);
}

void
carmen_laser_define_alive_message() {
  carmen_ipc_define_test_exit(CARMEN_LASER_ALIVE_NAME,
			      CARMEN_LASER_ALIVE_FMT);
}

void 
carmen_laser_get_offset(int which, carmen_point_t *laser_offset)
{
  char param_name[2048];
  char module_name[2048];
  int got_module_name = 0;

  char *laser_pose_str;
  double x, y, theta;
  int num_scanned;
  int error;

  if (carmen_param_get_module() != NULL) {
    strcpy(module_name, carmen_param_get_module());
    got_module_name = 1;
    carmen_param_set_module(NULL);
  }

  sprintf(param_name, "laser_laser%d_position", which);
  error = carmen_param_get_string(param_name, &laser_pose_str, NULL);
  if (error == -1) 
    carmen_die(carmen_param_get_error());
  
  num_scanned = sscanf(laser_pose_str, "%*c %lf %lf %lf %*c", &x, &y, 
		       &theta);
  theta = carmen_degrees_to_radians(theta);
  if (num_scanned != 3)
    carmen_die("Bad string for %s: %s\n", param_name, laser_pose_str);

  laser_offset->x = x;
  laser_offset->y = y;
  laser_offset->theta = theta;

  free(laser_pose_str);

  if (got_module_name) 
    carmen_param_set_module(module_name);
}
