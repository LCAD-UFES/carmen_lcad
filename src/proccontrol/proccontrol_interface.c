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
#include <carmen/proccontrol_messages.h>

void
carmen_proccontrol_subscribe_pidtable_message(carmen_proccontrol_pidtable_message
					     *pidtable,
					     carmen_handler_t handler,
					     carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_PROCCONTROL_PIDTABLE_NAME, 
			   CARMEN_PROCCONTROL_PIDTABLE_FMT,
			   pidtable, 
			   sizeof(carmen_proccontrol_pidtable_message), 
			   handler, subscribe_how);
}

void
carmen_proccontrol_unsubscribe_pidtable_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_PROCCONTROL_PIDTABLE_NAME, handler);
}

void
carmen_proccontrol_subscribe_output_message(carmen_proccontrol_output_message
					   *output,
					   carmen_handler_t handler,
					   carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_PROCCONTROL_OUTPUT_NAME,
			   CARMEN_PROCCONTROL_OUTPUT_FMT,
			   output, sizeof(carmen_proccontrol_output_message), 
			   handler, subscribe_how);
}

void
carmen_proccontrol_unsubscribe_output_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_PROCCONTROL_OUTPUT_NAME, handler);
}

void 
carmen_proccontrol_set_module_state(char *module_name, int requested_state)
{
  static carmen_proccontrol_moduleset_message msg;
  static int first = 1;

  if(first) {
    carmen_ipc_define_test_exit(CARMEN_PROCCONTROL_MODULESET_NAME, 
				CARMEN_PROCCONTROL_MODULESET_FMT);
    msg.host = carmen_get_host();
    first = 0;
  }
  msg.module_name = module_name;
  msg.requested_state = requested_state;
  msg.timestamp = carmen_get_time();
  carmen_ipc_publish_exit(CARMEN_PROCCONTROL_MODULESET_NAME, msg);
}

void 
carmen_proccontrol_set_group_state(char *group_name, int requested_state)
{
  static carmen_proccontrol_groupset_message msg;
  static int first = 1;

  if(first) {
    carmen_ipc_define_test_exit(CARMEN_PROCCONTROL_GROUPSET_NAME, 
				CARMEN_PROCCONTROL_GROUPSET_FMT);
    msg.host = carmen_get_host();
    first = 0;
  }
  msg.group_name = group_name;
  msg.requested_state = requested_state;
  msg.timestamp = carmen_get_time();
  carmen_ipc_publish_exit(CARMEN_PROCCONTROL_GROUPSET_NAME, msg);
}
