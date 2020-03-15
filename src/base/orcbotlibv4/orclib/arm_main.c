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
#include <carmen/arm_messages.h>

#include "arm_low_level.h"

void x_ipcRegisterExitProc(void (*)(void));

static char *model_name;
static char *dev_name;
static char* host;
static int num_joints;
static double reset_time;
static carmen_arm_state_message arm_state;

static int min( int a, int b ) {
  return ( b < a ? b : a );
}

static int initialize_arm(void)
{
  int result;
  //char *host;

  result = carmen_arm_direct_initialize(model_name, dev_name);
  if (result < 0) 
    return -1;

  reset_time = carmen_get_time();

  host = carmen_get_tenchar_host_name();
  //strcpy(arm_state.host, host);     
  
  return 0;
}

static void initialize_arm_message(carmen_arm_state_message * arm)
{
  arm->num_joints = num_joints;
  if( num_joints > 0 ) {
    arm->joint_angles = (double*)calloc( num_joints, sizeof( double ) );
    carmen_test_alloc( arm->joint_angles );
    if( USE_ARM_CURRENT_STATES ) {
      arm->flags |= USING_CURRENTS_MASK;
      arm->num_currents = min( num_joints, 2 ); // ORC board max limit
      arm->joint_currents = (double*)calloc( arm->num_currents, sizeof( double ) );
      carmen_test_alloc( arm->joint_currents );
    }
    if( USE_ARM_ANGULAR_VEL_STATES ) {
      arm->flags |= USING_ANGULAR_VEL_MASK;
      arm->num_vels = min( num_joints, 2 ); // rssII Arm limit
      arm->joint_angular_vels = (double*)calloc( arm->num_vels, sizeof( double ) );
      carmen_test_alloc( arm->joint_angular_vels );
    }
  }
}

static int read_parameters(int argc, char **argv)
{
  int num_items;

  carmen_param_t param_list[] = {
    { "arm", "dev", CARMEN_PARAM_STRING, &dev_name, 0, NULL},
    { "arm", "num_joints", CARMEN_PARAM_INT, &num_joints, 0, NULL },
  };
  
  num_items = sizeof(param_list)/sizeof(param_list[0]);
  carmen_param_install_params(argc, argv, param_list, num_items);

  if (num_joints > 0) {
    initialize_arm_message(&arm_state);
  }

  return 0;
}

static void arm_query_handler(MSG_INSTANCE msgRef, 
			      BYTE_ARRAY callData __attribute__ ((unused)),
			      void *clientData __attribute__ ((unused)))
{
  IPC_RETURN_TYPE err;
  carmen_arm_state_message response;

  // set the number of joints and allocate memory
  response.num_joints = num_joints;
  response.num_currents = arm_state.num_currents;
  response.flags = arm_state.flags;
  if( response.num_joints > 0 ) {
    response.joint_angles = (double*)calloc( response.num_joints, sizeof( double ) );
    carmen_test_alloc( response.joint_angles );
  }
  if( response.num_currents > 0 ) {
    response.joint_currents = (double*)calloc( response.num_currents, sizeof( double ) );
    carmen_test_alloc( response.joint_currents );
  }

  // specific to rssII Arm 11/10/05
  if( response.num_joints == 3 && 
      ( response.flags & USING_ANGULAR_VEL_MASK ) &&  
      response.num_vels == 2 ) {
    
    // set thetas and angular velocities  
    carmen_arm_get_theta_state( &response.joint_angles[ 0 ], &response.joint_angles[ 1 ], &response.joint_angles[ 2 ] );
    carmen_arm_get_state( &response.joint_angular_vels[ 0 ], &response.joint_angular_vels[ 1 ] );
  } else {
  
    // fill with zeros
    if( response.num_joints > 0 )
      memset( response.joint_angles, 0, response.num_joints * sizeof( double ) );
    if( response.num_currents > 0 )
      memset( response.joint_currents, 0, response.num_currents * sizeof( double ) );
    if( response.num_vels > 0 )
      memset( response.joint_angular_vels, 0, response.num_vels * sizeof( double ) );
    
  }

  response.timestamp = carmen_get_time();
  strcpy(response.host, carmen_get_tenchar_host_name());
  err = IPC_respondData(msgRef, CARMEN_ARM_STATE_NAME, &response);

  // free up the allocated memory
  if( response.num_joints > 0 )
    free( response.joint_angles );
  if( response.num_currents > 0 )
    free( response.joint_currents );
  if( response.num_vels > 0 )
    free( response.joint_angular_vels );
}

static void arm_command_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
				void *clientData __attribute__ ((unused)))
{
  IPC_RETURN_TYPE err;
  carmen_arm_command_message msg;
  FORMATTER_PTR formatter;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &msg,
			   sizeof(carmen_arm_command_message));
  IPC_freeByteArray(callData);
  
  // rssII Arm 11/10/05 specific
  if( msg.num_joints == 3 ) {
    carmen_arm_direct_set( msg.joint_angles[ 0 ], msg.joint_angles[ 1 ], msg.joint_angles[ 2 ] );
  } else {
    
    // this is NOT a correct RssII Arm message, just ignore it but warn user
    carmen_warn( "Received Incorrect Arm Message\n" );
  }

  // free data in msg
  if( msg.num_joints > 0 )
    free( msg.joint_angles );
}

static void reset_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
			  void *clientData __attribute__ ((unused)))
{
  IPC_RETURN_TYPE err;
  int base_err;
  carmen_base_reset_message msg;

  FORMATTER_PTR formatter;
  
  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &msg,
			   sizeof(carmen_base_reset_message));
  IPC_freeByteArray(callData);
  carmen_test_ipc_return(err, "Could not unmarshall", 
			 IPC_msgInstanceName(msgRef));

  do 
    {
      base_err = carmen_arm_direct_reset();
      //Edsinger: Reset hack
      if (base_err < 0)
	initialize_arm();
    }
  while (base_err < 0);
}

int 
carmen_arm_initialize_ipc(void)
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_ARM_COMMAND_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_ARM_COMMAND_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_ARM_COMMAND_NAME);

  err = IPC_defineMsg(CARMEN_ARM_QUERY_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_ARM_QUERY_FMT);
  carmen_test_ipc_exit(err, "Could not define", 
		       CARMEN_ARM_QUERY_NAME);

  err = IPC_defineMsg(CARMEN_ARM_STATE_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_ARM_STATE_FMT);
  carmen_test_ipc_exit(err, "Could not define", 
		       CARMEN_ARM_STATE_NAME);

  /* setup incoming message handlers */

  err = IPC_subscribe(CARMEN_ARM_RESET_COMMAND_NAME, reset_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subscribe", 
		       CARMEN_ARM_RESET_COMMAND_NAME);
  IPC_setMsgQueueLength(CARMEN_ARM_RESET_COMMAND_NAME, 1);

  err = IPC_subscribe(CARMEN_ARM_COMMAND_NAME, arm_command_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subscribe", CARMEN_ARM_COMMAND_NAME);
  IPC_setMsgQueueLength(CARMEN_ARM_COMMAND_NAME, 1);

  err = IPC_subscribe(CARMEN_ARM_QUERY_NAME, arm_query_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subscribe", CARMEN_ARM_QUERY_NAME);
  // Yes, unlike the others, this should be 100 so that we don't drop
  // any queries on the floor.
  IPC_setMsgQueueLength(CARMEN_ARM_QUERY_NAME, 100);

  return IPC_No_Error;
}

int carmen_arm_start(int argc, char **argv)
{
  if (read_parameters(argc, argv) < 0)
    return -1;

  if (carmen_arm_initialize_ipc() < 0) {
    carmen_warn("\nError: Could not initialize IPC.\n");
    return -1;
  }
  
  if(initialize_arm() < 0) {
    carmen_warn("\nError: Could not connect to robot on %s. "
		"Did you remember to turn the base on?\n", dev_name);
    return -1;
  }

  strcpy(host, carmen_get_tenchar_host_name());

  // velezj: RssII Arm
  carmen_arm_reset();

  return 0;
}

int carmen_arm_run(void) 
{
  IPC_RETURN_TYPE err;
  int arm_err;
  //static carmen_arm_reset_message reset;  

  do {
    arm_err = carmen_arm_direct_update_status();
    if (arm_err < 0)
      initialize_arm();
  } while (arm_err < 0);      
  

  // velezj: added for rssII arm PID controller loop
  carmen_arm_control();

  // send out arm status message
  carmen_arm_state_message response;

  // set the number of joints and allocate memory
  response.num_joints = num_joints;
  response.num_currents = arm_state.num_currents;
  response.num_vels = arm_state.num_vels;
  response.flags = arm_state.flags;
  if( response.num_joints > 0 ) {
    response.joint_angles = (double*)calloc( response.num_joints, sizeof( double ) );
    carmen_test_alloc( response.joint_angles );
  }
  if( response.num_currents > 0 ) {
    response.joint_currents = (double*)calloc( response.num_currents, sizeof( double ) );
    carmen_test_alloc( response.joint_currents );
  }
  if( response.num_vels > 0 ) {
    response.joint_angular_vels = (double*)calloc( response.num_vels, sizeof( double ) );
    carmen_test_alloc( response.joint_angular_vels );
  }

  // specific to rssII Arm 11/10/05
  if( response.num_joints == 3 && 
      ( response.flags & USING_ANGULAR_VEL_MASK ) &&  
      response.num_vels == 2 ) {
    
    // set thetas and angular velocities  
    carmen_arm_get_theta_state( &response.joint_angles[ 0 ], &response.joint_angles[ 1 ], &response.joint_angles[ 2 ] );
    carmen_arm_get_state( &response.joint_angular_vels[ 0 ], &response.joint_angular_vels[ 1 ] );
  } else {
  
    // fill with zeros
    if( response.num_joints > 0 )
      memset( response.joint_angles, 0, response.num_joints * sizeof( double ) );
    if( response.num_currents > 0 )
      memset( response.joint_currents, 0, response.num_currents * sizeof( double ) );
    if( response.num_vels > 0 )
      memset( response.joint_angular_vels, 0, response.num_vels * sizeof( double ) );
    
  }

  response.timestamp = carmen_get_time();
  strcpy(response.host, carmen_get_tenchar_host_name());
  err = IPC_publishData(CARMEN_ARM_STATE_NAME, &response);
  carmen_test_ipc_exit( err, "Could not publish", CARMEN_ARM_STATE_NAME );

  // free up the allocated memory
  if( response.num_joints > 0 )
    free( response.joint_angles );
  if( response.num_currents > 0 )
    free( response.joint_currents );
  if( response.num_vels > 0 )
    free( response.joint_angular_vels );
 
  carmen_publish_heartbeat("arm_daemon");
 
  return 1;
}

void carmen_arm_shutdown(void)
{
  carmen_verbose("\nShutting down robot...");
  carmen_arm_direct_shutdown();
  exit(0);
}

static void shutdown_arm(int signo __attribute__ ((unused)))
{
  carmen_arm_shutdown();
}

int  main(int argc __attribute__ ((unused)), 
	  char **argv __attribute__ ((unused)))
{
  carmen_initialize_ipc(argv[0]);
  carmen_param_check_version(argv[0]);

  signal(SIGINT, shutdown_arm);
				
  x_ipcRegisterExitProc(carmen_arm_shutdown);

  if (carmen_arm_start(argc, argv) < 0)
    exit(-1);

  while(1) {
    if (carmen_arm_run() == 1)
      fprintf(stderr, "~");
    sleep_ipc(0.1);
  }

  return 0;
}
