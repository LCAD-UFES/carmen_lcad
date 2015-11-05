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
#include "pantilt.h"
#include "pantilt_messages.h"

carmen_pantilt_status_message   status;

void
ipc_publish_status( void )
{
  static int firsttime = TRUE;
  IPC_RETURN_TYPE err = IPC_OK;
  if (firsttime) {
    status.host =  carmen_get_host();
    firsttime = FALSE;
  }
  status.timestamp = carmen_get_time();
  err = IPC_publishData (CARMEN_PANTILT_STATUS_MESSAGE_NAME, &status );
  carmen_test_ipc(err, "Could not publish",
		  CARMEN_PANTILT_STATUS_MESSAGE_NAME);
}

void
pantilt_set_pan( MSG_INSTANCE msgRef __attribute__ ((unused)),
		 BYTE_ARRAY callData,
		 void *clientData __attribute__ ((unused)))
{
  FORMATTER_PTR                          formatter;
  IPC_RETURN_TYPE                        err = IPC_OK;
  carmen_pantilt_move_pan_message   query;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &query, 
                           sizeof(carmen_pantilt_move_pan_message));
  IPC_freeByteArray(callData);
  carmen_test_ipc(err, "Could not unmarshall", IPC_msgInstanceName(msgRef));

  fprintf( stderr, "INFO: move \E[31m\033[1mpan\033[0m to %.4f             \n",
	   rad2deg(query.pan) ); 

  if (pLimits.upmin && query.pan < pLimits.pmin) { 
    fprintf( stderr, "INFO: wrong value for pan - change to %.2f\n",
	     rad2deg(pLimits.pmin) );
    query.pan = pLimits.pmin;
  } else if (pLimits.upmax && query.pan > pLimits.pmax) { 
    fprintf( stderr, "INFO: wrong value for pan - change to %.2f\n",
	     rad2deg(pLimits.pmax) );
    query.pan = pLimits.pmax;
  }
  set_pan( query.pan );
    
}

void
pantilt_set_tilt( MSG_INSTANCE msgRef __attribute__ ((unused)),
		  BYTE_ARRAY callData,
		  void *clientData __attribute__ ((unused)))
{
  FORMATTER_PTR                          formatter;
  IPC_RETURN_TYPE                        err = IPC_OK;
  carmen_pantilt_move_tilt_message   query;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &query, 
                           sizeof(carmen_pantilt_move_tilt_message));
  IPC_freeByteArray(callData);
  carmen_test_ipc(err, "Could not unmarshall", IPC_msgInstanceName(msgRef));

  fprintf( stderr, "INFO: move \E[31m\033[1mtilt\033[0m to %.4f             \n",
	   rad2deg(query.tilt) ); 

  if (pLimits.utmin && query.tilt < pLimits.tmin) { 
    fprintf( stderr, "INFO: wrong value for tilt - change to %.2f\n",
	     rad2deg(pLimits.tmin) );
    query.tilt = pLimits.pmin;
  } else if (pLimits.utmax && query.tilt > pLimits.tmax) { 
    fprintf( stderr, "INFO: wrong value for tilt - change to %.2f\n",
	     rad2deg(pLimits.tmax) );
    query.tilt = pLimits.pmax;
  }
  set_tilt( query.tilt );
  
}

void
pantilt_set_pantilt( MSG_INSTANCE msgRef __attribute__ ((unused)),
		     BYTE_ARRAY callData,
		     void *clientData __attribute__ ((unused)))
{
  FORMATTER_PTR                      formatter;
  IPC_RETURN_TYPE                    err = IPC_OK;
  carmen_pantilt_move_message   query;
  
  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &query, 
                           sizeof(carmen_pantilt_move_message));
  IPC_freeByteArray(callData);
  carmen_test_ipc(err, "Could not unmarshall", IPC_msgInstanceName(msgRef));

  fprintf( stderr, "INFO: move \E[31m\033[1mpan/tilt\033[0m to %.4f / %.4f\n",
	   rad2deg(query.pan), rad2deg(query.tilt) ); 
  
  if (pLimits.utmin && query.pan < pLimits.pmin) { 
    fprintf( stderr, "INFO: wrong value for pan - change to %.2f\n",
	     rad2deg(pLimits.pmin) );
    query.pan = pLimits.pmin;
  } else if (pLimits.utmax && query.pan > pLimits.pmax) { 
    fprintf( stderr, "INFO: wrong value for pan - change to %.2f\n",
	     rad2deg(pLimits.pmax) );
    query.pan = pLimits.pmax;
  }
  if (pLimits.utmin && query.tilt < pLimits.tmin) { 
    fprintf( stderr, "INFO: wrong value for tilt - change to %.2f\n",
	     rad2deg(pLimits.tmin) );
    query.tilt = pLimits.pmin;
  } else if (pLimits.utmax && query.tilt > pLimits.tmax) { 
    fprintf( stderr, "INFO: wrong value for tilt - change to %.2f\n",
	     rad2deg(pLimits.tmax) );
    query.tilt = pLimits.pmax;
  }
  set_pantilt( query.pan, query.tilt );
  
}


void
ipc_initialize_messages( void )
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_PANTILT_STATUS_MESSAGE_NAME, IPC_VARIABLE_LENGTH, 
		      CARMEN_PANTILT_STATUS_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define",
		       CARMEN_PANTILT_STATUS_MESSAGE_NAME);

  err = IPC_defineMsg(CARMEN_PANTILT_MOVE_PAN_MESSAGE_NAME, IPC_VARIABLE_LENGTH, 
		      CARMEN_PANTILT_MOVE_PAN_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define",
		       CARMEN_PANTILT_MOVE_PAN_MESSAGE_NAME);

  err = IPC_defineMsg(CARMEN_PANTILT_MOVE_TILT_MESSAGE_NAME, IPC_VARIABLE_LENGTH, 
		      CARMEN_PANTILT_MOVE_TILT_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define",
		       CARMEN_PANTILT_MOVE_TILT_MESSAGE_NAME);
  
  err = IPC_defineMsg(CARMEN_PANTILT_MOVE_MESSAGE_NAME, IPC_VARIABLE_LENGTH, 
		      CARMEN_PANTILT_MOVE_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define",
		       CARMEN_PANTILT_MOVE_MESSAGE_NAME);

  /* subscribe to messages */
  
  err = IPC_subscribe(CARMEN_PANTILT_MOVE_PAN_MESSAGE_NAME, pantilt_set_pan, NULL);
  carmen_test_ipc_exit(err, "Could not subscribe to", 
                       CARMEN_PANTILT_MOVE_PAN_MESSAGE_NAME);
  IPC_setMsgQueueLength(CARMEN_PANTILT_MOVE_PAN_MESSAGE_NAME, 10);

  err = IPC_subscribe(CARMEN_PANTILT_MOVE_TILT_MESSAGE_NAME, pantilt_set_tilt, NULL);
  carmen_test_ipc_exit(err, "Could not subscribe to", 
                       CARMEN_PANTILT_MOVE_TILT_MESSAGE_NAME);
  IPC_setMsgQueueLength(CARMEN_PANTILT_MOVE_TILT_MESSAGE_NAME, 10);

  err = IPC_subscribe(CARMEN_PANTILT_MOVE_MESSAGE_NAME, pantilt_set_pantilt, NULL);
  carmen_test_ipc_exit(err, "Could not subscribe to", 
                       CARMEN_PANTILT_MOVE_MESSAGE_NAME);
  IPC_setMsgQueueLength(CARMEN_PANTILT_MOVE_MESSAGE_NAME, 10);

}

void
PantiltParams( int argc, char **argv )
{
  int num_items = 1;
  carmen_param_t params[] = {
    {"pantilt", "dev", CARMEN_PARAM_STRING, &(pDevice.port), 0, NULL}
  };

  carmen_param_install_params(argc, argv, params, num_items);
}
