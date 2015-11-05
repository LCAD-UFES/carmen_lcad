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
//#include <heartbeat_messages.h>
#include "proccontrol_ipc.h"
#include "proccontrol.h"

carmen_proccontrol_process_t plist[MAX_PROCESSES];

void proccontrol_register_ipc_messages(void)
{
  //  carmen_ipc_define_test_exit(CARMEN_HEARTBEAT_NAME, CARMEN_HEARTBEAT_FMT);
  carmen_ipc_define_test_exit(CARMEN_PROCCONTROL_MODULESET_NAME, 
			      CARMEN_PROCCONTROL_MODULESET_FMT);
  carmen_ipc_define_test_exit(CARMEN_PROCCONTROL_GROUPSET_NAME, 
			      CARMEN_PROCCONTROL_GROUPSET_FMT);
  carmen_ipc_define_test_exit(CARMEN_PROCCONTROL_PIDTABLE_NAME, 
			      CARMEN_PROCCONTROL_PIDTABLE_FMT);
  carmen_ipc_define_test_exit(CARMEN_PROCCONTROL_OUTPUT_NAME, 
			      CARMEN_PROCCONTROL_OUTPUT_FMT);
}

void proccontrol_publish_pidtable(int num_processes, process_info_p process)
{
  static carmen_proccontrol_pidtable_message msg;
  static int first = 1;
  int i;

  if(first) {
    msg.host = carmen_get_host();
    first = 0;
  }
  msg.num_processes = num_processes;
  msg.process = plist;
  for(i = 0; i < msg.num_processes; i++) {
    msg.process[i].group_name = process[i].group_name;
    msg.process[i].module_name = process[i].module_name;
    msg.process[i].requested_state = process[i].requested_state;
    msg.process[i].active = process[i].state;
    msg.process[i].pid = process[i].pid;
  }
  msg.timestamp = carmen_get_time();
  carmen_ipc_publish_exit(CARMEN_PROCCONTROL_PIDTABLE_NAME, msg);
}

void proccontrol_publish_output(int pid, char *output)
{
  static carmen_proccontrol_output_message msg;
  static int first = 1;

  if(first) {
    msg.host = carmen_get_host();
    first = 0;
  }
  msg.pid = pid;
  msg.output = output;
  carmen_ipc_publish_exit(CARMEN_PROCCONTROL_OUTPUT_NAME, msg);
}
