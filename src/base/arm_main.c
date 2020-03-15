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

#include <carmen/arm_low_level.h>

void x_ipcRegisterExitProc(void (*)(void));

static carmen_arm_model_t arm_model;

static double *goal_joint_angles = NULL;

static double reset_time;
static carmen_arm_state_message arm_state;

#define USE_ARM_CURRENT_STATES     1
#define USE_ARM_ANGULAR_VEL_STATES 1

#define USING_CURRENTS_MASK    0
#define USING_ANGULAR_VEL_MASK 1


static int initialize_arm(void)
{
  int result;

  result = carmen_arm_direct_initialize(&arm_model);
  if (result < 0) 
    return -1;

  reset_time = carmen_get_time();

  if (goal_joint_angles == NULL) {
    goal_joint_angles = (double *) calloc(arm_model.num_joints, sizeof(double));
    carmen_test_alloc(goal_joint_angles);
  }

  return 0;
}

static void initialize_arm_message(carmen_arm_state_message * arm)
{
  arm->num_joints = arm_model.num_joints;
  if( arm_model.num_joints > 0 ) {
    arm->joint_angles = (double*)calloc( arm_model.num_joints, sizeof( double ) );
    carmen_test_alloc( arm->joint_angles );
    if( USE_ARM_CURRENT_STATES ) {
      arm->flags |= USING_CURRENTS_MASK;
      arm->num_currents = arm_model.num_joints;
      arm->joint_currents = (double*)calloc( arm->num_currents, sizeof( double ) );
      carmen_test_alloc( arm->joint_currents );
    }
    else
      arm->joint_currents = NULL;
    if( USE_ARM_ANGULAR_VEL_STATES ) {
      arm->flags |= USING_ANGULAR_VEL_MASK;
      arm->num_vels = arm_model.num_joints;
      arm->joint_angular_vels = (double*)calloc( arm->num_vels, sizeof( double ) );
      carmen_test_alloc( arm->joint_angular_vels );
    }
    else
      arm->joint_angular_vels = NULL;
  }
  arm->host = carmen_get_host();
}

static int read_parameters(int argc, char **argv)
{
  int num_items;
  char *joint_types_string;

  carmen_param_t param_list[] = {
    { "arm", "dev", CARMEN_PARAM_STRING, &(arm_model.dev), 0, NULL},
    { "arm", "num_joints", CARMEN_PARAM_INT, &(arm_model.num_joints), 0, NULL },
    { "arm", "joint_types", CARMEN_PARAM_STRING, &joint_types_string, 0, NULL }
  };
  
  num_items = sizeof(param_list)/sizeof(param_list[0]);
  carmen_param_install_params(argc, argv, param_list, num_items);

  if (arm_model.num_joints > 0) {
    arm_model.joints = (carmen_arm_joint_t *) calloc(arm_model.num_joints, sizeof(int));  // size_t ??
    carmen_parse_arm_joint_types(joint_types_string, arm_model.joints, arm_model.num_joints);
    initialize_arm_message(&arm_state);
  }

  return 0;
}

static void get_arm_state()
{
  carmen_arm_direct_get_state(arm_state.joint_angles, arm_state.joint_currents, arm_state.joint_angular_vels, NULL);
  arm_state.timestamp = carmen_get_time();
}

static void arm_query_handler(MSG_INSTANCE msgRef, 
			      BYTE_ARRAY callData __attribute__ ((unused)),
			      void *clientData __attribute__ ((unused)))
{
  IPC_RETURN_TYPE err;

  get_arm_state();
  err = IPC_respondData(msgRef, CARMEN_ARM_STATE_NAME, &arm_state);
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
  
  if (msg.num_joints != arm_model.num_joints)
    carmen_warn("Received arm command with incorrect msg.num_joints!\n");

  memcpy(goal_joint_angles, msg.joint_angles, arm_model.num_joints*sizeof(double));

  // free data in msg
  if( msg.num_joints > 0 )
    free( msg.joint_angles );
}

static void reset_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
			  void *clientData __attribute__ ((unused)))
{
  IPC_RETURN_TYPE err;
  int base_err;
  carmen_arm_reset_command_message msg;

  FORMATTER_PTR formatter;
  
  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &msg,
			   sizeof(carmen_arm_reset_command_message));
  IPC_freeByteArray(callData);
  carmen_test_ipc_return(err, "Could not unmarshall", 
			 IPC_msgInstanceName(msgRef));

  do 
    {
      base_err = carmen_arm_direct_reset();
      if (base_err < 0) {
	carmen_arm_direct_shutdown();
	initialize_arm();
      }
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
                      CARMEN_DEFAULT_MESSAGE_FMT);
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
		"Did you remember to turn the base on?\n", arm_model.dev);
    return -1;
  }

  return 0;
}

int carmen_arm_run(void) 
{
  IPC_RETURN_TYPE err;
  //int arm_err;
  //static carmen_arm_reset_message reset;  

  /*
  do {
    arm_err = carmen_arm_direct_update_joints(goal_joint_angles);
    if (arm_err < 0)
      arm_err = carmen_arm_direct_reset();
  } while (arm_err < 0);      
  */
  carmen_arm_direct_update_joints(goal_joint_angles);

  get_arm_state();
  err = IPC_publishData(CARMEN_ARM_STATE_NAME, &arm_state);
  carmen_test_ipc_exit( err, "Could not publish", CARMEN_ARM_STATE_NAME );

  carmen_publish_heartbeat("arm_daemon");  //dbug
 
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

int main(int argc, char **argv)
{
  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);

  signal(SIGINT, shutdown_arm);
				
  x_ipcRegisterExitProc(carmen_arm_shutdown);

  if (carmen_arm_start(argc, argv) < 0)
    exit(-1);

  while(1) {
    if (carmen_arm_run() == 1)
      fprintf(stderr, "~");
    carmen_ipc_sleep(0.1);
  }

  return 0;
}
