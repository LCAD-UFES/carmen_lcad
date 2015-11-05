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
#include <carmen/pantilt_messages.h>


void 
carmen_pantilt_subscribe_status_message(carmen_pantilt_status_message *pantilt,
					carmen_handler_t handler,
					carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_PANTILT_STATUS_MESSAGE_NAME, 
			   CARMEN_PANTILT_STATUS_MESSAGE_FMT,
			   pantilt, sizeof(carmen_pantilt_status_message), 
			   handler,
			   subscribe_how);
}



void
carmen_pantilt_unsubscribe_status_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_PANTILT_STATUS_MESSAGE_NAME, handler);
}


void 
carmen_pantilt_subscribe_scanmark_message(carmen_pantilt_scanmark_message *scanmark,
					  carmen_handler_t handler,
					  carmen_subscribe_t subscribe_how) {

  carmen_subscribe_message(CARMEN_PANTILT_SCANMARK_MESSAGE_NAME, 
			   CARMEN_PANTILT_SCANMARK_MESSAGE_FMT,
			   scanmark, 
			   sizeof(carmen_pantilt_scanmark_message), 
			   handler,
			   subscribe_how);
}

void
carmen_pantilt_unsubscribe_scanmark_message(carmen_handler_t handler) {

  carmen_unsubscribe_message(CARMEN_PANTILT_SCANMARK_MESSAGE_NAME, handler);
}

void 
carmen_pantilt_subscribe_laserpos_message(carmen_pantilt_scanmark_message *laserpos,
					  carmen_handler_t handler,
					  carmen_subscribe_t subscribe_how) {

  carmen_subscribe_message(CARMEN_PANTILT_LASERPOS_MESSAGE_NAME, 
			   CARMEN_PANTILT_LASERPOS_MESSAGE_FMT,
			   laserpos, 
			   sizeof(carmen_pantilt_laserpos_message), 
			   handler,
			   subscribe_how);
}


void
carmen_pantilt_unsubscribe_laserpos_message(carmen_handler_t handler) {

  carmen_unsubscribe_message(CARMEN_PANTILT_LASERPOS_MESSAGE_NAME, handler);
}

////////////////////////////////////////////////////////////////////////////

void
carmen_pantilt_move( double pan, double tilt )
{
   IPC_RETURN_TYPE err;
  static carmen_pantilt_move_message msg;
  static int first = 1;
  
  if(first) {
    msg.host = carmen_get_host();
    err = IPC_defineMsg(CARMEN_PANTILT_MOVE_MESSAGE_NAME,
                        IPC_VARIABLE_LENGTH, 
                        CARMEN_PANTILT_MOVE_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
                         CARMEN_PANTILT_MOVE_MESSAGE_NAME);
    first = 0;
  }
  msg.pan  = pan;
  msg.tilt = tilt;
  msg.timestamp = carmen_get_time();
  
  err = IPC_publishData(CARMEN_PANTILT_MOVE_MESSAGE_NAME, &msg);
  carmen_test_ipc(err, "Could not publish",
                  CARMEN_PANTILT_MOVE_MESSAGE_NAME);


}

void
carmen_pantilt_move_tilt( double tilt )
{

 IPC_RETURN_TYPE err;
  static carmen_pantilt_move_tilt_message msg;
  static int first = 1;
  
  if(first) {
   msg.host = carmen_get_host();
    err = IPC_defineMsg(CARMEN_PANTILT_MOVE_TILT_MESSAGE_NAME,
                        IPC_VARIABLE_LENGTH, 
                        CARMEN_PANTILT_MOVE_TILT_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
                         CARMEN_PANTILT_MOVE_TILT_MESSAGE_NAME);
    first = 0;
  }
  msg.tilt = tilt;
  msg.timestamp = carmen_get_time();
  
  err = IPC_publishData(CARMEN_PANTILT_MOVE_TILT_MESSAGE_NAME, &msg);
  carmen_test_ipc(err, "Could not publish",
                  CARMEN_PANTILT_MOVE_TILT_MESSAGE_NAME);
}

void
carmen_pantilt_move_pan( double pan )
{
  IPC_RETURN_TYPE err;
  static carmen_pantilt_move_pan_message msg;
  static int first = 1;
  
  if(first) {
    msg.host = carmen_get_host();
    err = IPC_defineMsg(CARMEN_PANTILT_MOVE_PAN_MESSAGE_NAME,
                        IPC_VARIABLE_LENGTH, 
                        CARMEN_PANTILT_MOVE_PAN_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
                         CARMEN_PANTILT_MOVE_PAN_MESSAGE_NAME);
    first = 0;
  }
  msg.pan  = pan;
  msg.timestamp = carmen_get_time();
  
  err = IPC_publishData(CARMEN_PANTILT_MOVE_PAN_MESSAGE_NAME, &msg);
  carmen_test_ipc(err, "Could not publish",
                  CARMEN_PANTILT_MOVE_PAN_MESSAGE_NAME);

}

