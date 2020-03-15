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
#include "base_low_level.h"

#ifdef BASE_HAS_ARM
#include <carmen/arm_messages.h> 
#include "arm_low_level.h"
#endif

void x_ipcRegisterExitProc(void (*)(void));

static int moving = 0;
static double current_acceleration = 0;
static double deceleration;
static carmen_robot_config_t robot_config;
static carmen_base_odometry_message odometry;
static carmen_base_binary_data_message binary_data;
static double reset_time = 0;
static double relative_wheelbase;
static double relative_wheelsize;

static int use_hardware_integrator = 1;
static int use_sonar = 1;
static carmen_base_sonar_message sonar;
static int sonar_state = 0;
static double *ranges = NULL;
static carmen_point_t *positions = NULL;
static int num_sonar_ranges;

static carmen_base_bumper_message bumper;

static double last_motion_command = 0;

static double motion_timeout = 1;
static int odometry_inverted;

static char *model_name;
static char *dev_name;

#ifdef BASE_HAS_ARM
static int num_arm_joints;
static carmen_arm_state_message arm_state;
#endif

static int 
initialize_robot(void)
{
  int result;

  result = carmen_base_direct_initialize_robot
    (model_name, dev_name);
  if (result < 0) 
    return -1;
  

  if (use_sonar) {
    num_sonar_ranges = carmen_base_direct_sonar_on();
    if(num_sonar_ranges < 0) 
      return -1;
    if (ranges == NULL)
      free(ranges);
    ranges = (double *)calloc(num_sonar_ranges, sizeof(double));
    carmen_test_alloc(ranges);
    positions = (carmen_point_t *)
      calloc(num_sonar_ranges, sizeof(carmen_point_t));
    carmen_test_alloc(positions);
    sonar_state = 1;
  } else {
    result = carmen_base_direct_sonar_off();
    if(result < 0) 
      return -1;
  }

  result = carmen_base_direct_reset();
  if (result < 0) 
    return -1;

  reset_time = carmen_get_time();
  odometry.host = carmen_get_host();

  binary_data.host = carmen_get_host();
  binary_data.size = 0;
  
  return 0;
}

#ifdef BASE_HAS_ARM

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
  
  if (msg.num_joints > 0) {
    carmen_arm_direct_set(msg.joint_angles, msg.num_joints);
    free(msg.joint_angles);
  }
}

static void arm_query_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
			      void *clientData __attribute__ ((unused)))
{
  IPC_RETURN_TYPE err;

  IPC_freeByteArray(callData);

  if (num_arm_joints > 0) 
    carmen_arm_direct_get_state(arm_state.joint_angles, 
				arm_state.joint_currents,
				arm_state.joint_angular_vels,
				&arm_state.gripper_closed, num_arm_joints);

  arm_state.timestamp = carmen_get_time();
  arm_state.host = carmen_get_host();
  err = IPC_respondData(msgRef, CARMEN_ARM_STATE_NAME, &arm_state);
}

static void initialize_arm_message(void)
{
  int err;

  carmen_param_allow_unfound_variables(FALSE); 
  err = carmen_param_get_int("arm_num_joints", &num_arm_joints, NULL);
  if (err < 0)
    carmen_die("\nThe base_has_arm parameter is on, but the arm_num_joints "
	       "parameter is\nmissing. Please set the arm_num_joints "
	       "parameter.\n");
  arm_state.num_joints = num_arm_joints;
  if (num_arm_joints > 0) {
    arm_state.joint_angles = (double *)calloc(num_arm_joints, sizeof(double));
    carmen_test_alloc(arm_state.joint_angles);

    if(carmen_arm_direct_num_currents(num_arm_joints)) {
      arm_state.flags |= CARMEN_ARM_HAS_CURRENT_STATES;
      arm_state.num_currents = carmen_arm_direct_num_currents(num_arm_joints);
      arm_state.joint_currents = (double*)calloc
	( arm_state.num_currents, sizeof( double ) );
      carmen_test_alloc( arm_state.joint_currents );
    }
    if(carmen_arm_direct_num_velocities(num_arm_joints)) {
      arm_state.flags |= CARMEN_ARM_HAS_ANGULAR_VEL_STATES;
      arm_state.num_vels = carmen_arm_direct_num_velocities(num_arm_joints);
      arm_state.joint_angular_vels = (double*)calloc
	( arm_state.num_vels, sizeof( double ) );
      carmen_test_alloc( arm_state.joint_angular_vels );
    }
  }

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

  err = IPC_subscribe(CARMEN_ARM_COMMAND_NAME, arm_command_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subscribe", 
		       CARMEN_ARM_COMMAND_NAME);
  IPC_setMsgQueueLength(CARMEN_ARM_COMMAND_NAME, 1);

  err = IPC_subscribe(CARMEN_ARM_QUERY_NAME, arm_query_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subscribe", 
                       CARMEN_ARM_QUERY_NAME);
  IPC_setMsgQueueLength(CARMEN_ARM_QUERY_NAME, 100);
}

#endif

static void
initialize_sonar_message(carmen_base_sonar_message *sonar)
{
  double sensor_angle;

  carmen_param_set_module("robot");
  carmen_param_get_double("sensor_angle", &sensor_angle, NULL);
  
  sonar->cone_angle = sensor_angle;
  sonar->timestamp = 0.0;
  strncpy(sonar->host, odometry.host, 10);
}

static void
handle_sonar_change(char *module __attribute__ ((unused)), 
		    char *variable __attribute__ ((unused)), 
		    char *value __attribute__ ((unused)))
{
  int err;

  if (use_sonar && !sonar_state) {
    initialize_sonar_message(&sonar);
    do {
      num_sonar_ranges = carmen_base_direct_sonar_on();
      if (num_sonar_ranges < 0)
	initialize_robot();
    } 
    while (num_sonar_ranges < 0);

    ranges = (double *)calloc(num_sonar_ranges, sizeof(double));
    carmen_test_alloc(ranges);
    positions = (carmen_point_t *)
      calloc(num_sonar_ranges, sizeof(carmen_point_t));
    carmen_test_alloc(positions);
    sonar_state = 1;
  } else if (!use_sonar && sonar_state) {
    if (ranges) 
      free(ranges);
    if (positions)
      free(positions);
    do {
      err = carmen_base_direct_sonar_off();
      if (err < 0)
	initialize_robot();
    } while (err < 0);
    sonar_state = 0;
  } /* end of if (use_sonar && !sonar_state) ... else if (sonar_state) */
}

static int
read_parameters(int argc, char **argv)
{
  int num_items;

  carmen_param_t param_list[] = {
    {"base", "dev", CARMEN_PARAM_STRING, &dev_name, 0, NULL},
    {"base", "model", CARMEN_PARAM_STRING, &model_name, 0, NULL},
    {"base", "motion_timeout", CARMEN_PARAM_DOUBLE, &motion_timeout, 0, NULL},
    {"base", "use_hardware_integrator", CARMEN_PARAM_ONOFF, 
     &use_hardware_integrator, 0, NULL},
    {"robot", "odometry_inverted", CARMEN_PARAM_ONOFF, &odometry_inverted, 
     0, NULL},
    {"robot", "use_sonar", CARMEN_PARAM_ONOFF, &use_sonar, 1, 
     handle_sonar_change}, 
    {"robot", "acceleration", CARMEN_PARAM_DOUBLE, 
     &(robot_config.acceleration), 1, NULL},
    {"robot", "deceleration", CARMEN_PARAM_DOUBLE, 
     &(deceleration), 1, NULL}};

  carmen_param_t extra_params[] = {
    {"base", "relative_wheelsize", CARMEN_PARAM_DOUBLE, 
     &relative_wheelsize, 0, NULL},
    {"base", "relative_wheelbase", CARMEN_PARAM_DOUBLE, 
     &relative_wheelbase, 0, NULL},
  };

  num_items = sizeof(param_list)/sizeof(param_list[0]);
  carmen_param_install_params(argc, argv, param_list, num_items);

  if (use_sonar)
    initialize_sonar_message(&sonar);
  else
    memset(&sonar, 0, sizeof(carmen_base_sonar_message));

  if (!use_hardware_integrator) {
    num_items = sizeof(extra_params)/sizeof(extra_params[0]);
    carmen_param_install_params(argc, argv, extra_params, num_items);      
  }

#ifdef BASE_HAS_ARM  
  initialize_arm_message();
#endif

  if (robot_config.acceleration > deceleration) 
    carmen_die("ERROR: robot_deceleration must be greater or equal than "
	       "robot_acceleration\n");
  
  return 0;
}

static void 
velocity_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		 void *clientData __attribute__ ((unused)))
{
  IPC_RETURN_TYPE err;
  carmen_base_velocity_message vel;
  int base_err;
  FORMATTER_PTR formatter;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &vel,
                     sizeof(carmen_base_velocity_message));
  IPC_freeByteArray(callData);

  carmen_test_ipc_return(err, "Could not unmarshall", 
			 IPC_msgInstanceName(msgRef));

  if(vel.tv == 0 && vel.rv == 0) 
    {
      if (moving) 
	{
	  do 
	    {
	      base_err = carmen_base_direct_set_deceleration(deceleration);
	      if (base_err < 0)
		initialize_robot();
	    } 
	  while (base_err < 0);
	  moving = 0;
	}
      carmen_warn("S");
    }
  else if (!moving) 
    {
      moving = 1;
      current_acceleration = robot_config.acceleration;
      do 
	{
	  base_err = carmen_base_direct_set_acceleration(current_acceleration);
	  if (base_err < 0)
	    initialize_robot();
	} 
      while (base_err < 0);
      carmen_warn("V");
    }  

  if (odometry_inverted)
    vel.tv *= -1;
  if (!use_hardware_integrator)
    {
      vel.tv /= relative_wheelsize;
      
      vel.rv /= relative_wheelsize;
      vel.rv /= relative_wheelbase;
    }

  do 
    {
      base_err = carmen_base_direct_set_velocity(vel.tv, vel.rv);
      last_motion_command = carmen_get_time();
      if (base_err < 0)
	initialize_robot();
    } 
  while (base_err < 0);
}

static void
binary_command_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
                       void *clientData __attribute__ ((unused)))
{
  IPC_RETURN_TYPE err;
  carmen_base_binary_data_message msg;
  FORMATTER_PTR formatter;
  int base_err;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &msg,
                           sizeof(carmen_base_binary_data_message));
  IPC_freeByteArray(callData);

  carmen_test_ipc_return(err, "Could not unmarshall", 
                         IPC_msgInstanceName(msgRef));

  if (msg.size > 0)
    {
      do 
	{
	  base_err = carmen_base_direct_send_binary_data(msg.data, msg.size);
	  if (base_err < 0)
	    initialize_robot();
	} 
      while (base_err < 0);
      free(msg.data);
    }
}

static void 
reset_odometry()
{
 printf("Odometry Reset...\n");
  odometry.x=0;
  odometry.y=0;
  odometry.theta=0;
}

static void reset_handler(MSG_INSTANCE msgRef __attribute__ ((unused)), 
			  BYTE_ARRAY callData,
			  void *clientData __attribute__ ((unused)))
{
  int base_err;

  IPC_freeByteArray(callData);

  do {
    base_err = carmen_base_direct_reset();
    //Edsinger: Reset hack
    reset_odometry();
    if (base_err < 0)
      initialize_robot();
  } while (base_err < 0);
}

int 
carmen_base_initialize_ipc(void)
{
  IPC_RETURN_TYPE err;

  /* define messages created by base */
  err = IPC_defineMsg(CARMEN_BASE_ODOMETRY_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_BASE_ODOMETRY_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_BASE_ODOMETRY_NAME);

  err = IPC_defineMsg(CARMEN_BASE_SONAR_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_BASE_SONAR_FMT);
  carmen_test_ipc_exit(err, "Could not define IPC message", 
		       CARMEN_BASE_SONAR_NAME);

  err = IPC_defineMsg(CARMEN_BASE_BUMPER_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_BASE_BUMPER_FMT);
  carmen_test_ipc_exit(err, "Could not define IPC message", 
		       CARMEN_BASE_BUMPER_NAME);

  err = IPC_defineMsg(CARMEN_BASE_VELOCITY_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_BASE_VELOCITY_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_BASE_VELOCITY_NAME);

  err = IPC_defineMsg(CARMEN_BASE_RESET_OCCURRED_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_DEFAULT_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_BASE_RESET_OCCURRED_NAME);

  err = IPC_defineMsg(CARMEN_BASE_RESET_COMMAND_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_DEFAULT_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define", 
		       CARMEN_BASE_RESET_COMMAND_NAME);

  err = IPC_defineMsg(CARMEN_BASE_BINARY_COMMAND_NAME, 
                      IPC_VARIABLE_LENGTH,
                      CARMEN_BASE_BINARY_COMMAND_FMT);
  carmen_test_ipc_exit(err, "Could not define", 
                       CARMEN_BASE_BINARY_COMMAND_NAME);

  err = IPC_defineMsg(CARMEN_BASE_BINARY_DATA_NAME, 
                      IPC_VARIABLE_LENGTH,
                      CARMEN_BASE_BINARY_DATA_FMT);
  carmen_test_ipc_exit(err, "Could not define", 
                       CARMEN_BASE_BINARY_DATA_NAME);

  /* setup incoming message handlers */

  err = IPC_subscribe(CARMEN_BASE_VELOCITY_NAME, velocity_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subscribe", CARMEN_BASE_VELOCITY_NAME);
  IPC_setMsgQueueLength(CARMEN_BASE_VELOCITY_NAME, 1);

  err = IPC_subscribe(CARMEN_BASE_RESET_COMMAND_NAME, reset_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subscribe", 
		       CARMEN_BASE_RESET_COMMAND_NAME);
  IPC_setMsgQueueLength(CARMEN_BASE_RESET_COMMAND_NAME, 1);

  err = IPC_subscribe(CARMEN_BASE_BINARY_COMMAND_NAME, 
                      binary_command_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subscribe", 
                       CARMEN_BASE_BINARY_COMMAND_NAME);
  IPC_setMsgQueueLength(CARMEN_BASE_BINARY_COMMAND_NAME, 1);

  return IPC_No_Error;
}

int 
carmen_base_start(int argc, char **argv)
{
  if (read_parameters(argc, argv) < 0)
    return -1;

  if (carmen_base_initialize_ipc() < 0) {
    carmen_warn("\nError: Could not initialize IPC.\n");
    return -1;
  }
  
  if(initialize_robot() < 0) {
    carmen_warn("\nError: Could not connect to robot on %s. "
		"Did you remember to turn the base on?\n", dev_name);
    carmen_warn("     : Are the permissions on %s correct (0666)?\n", dev_name);
    carmen_warn("     : Do you have the line speed set correctly?\n");
    carmen_warn("     : Is the robot cable plugged in securely?\n");
    return -1;
  }

  odometry.host = carmen_get_host();

  return 0;
}

static void
integrate_odometry(double displacement, double rotation, double tv, double rv)
{
  displacement *= relative_wheelsize;
  rotation *= relative_wheelsize;
  rotation *= relative_wheelbase;
      
  odometry.tv = tv * relative_wheelsize;
  odometry.rv = rv * relative_wheelsize / relative_wheelbase;
  
  if (odometry_inverted) {
    odometry.tv *= -1;
    displacement *= -1;
  }

  /*Giorgio 03.03.2004
  added Rung Kutta integration.
  */
  /*
  odometry.x += displacement * cos (odometry.theta);
  odometry.y += displacement * sin (odometry.theta);
  */
  
  odometry.x += displacement * cos (odometry.theta+0.5*rotation);
  odometry.y += displacement * sin (odometry.theta+0.5*rotation);
  odometry.theta = carmen_normalize_theta(odometry.theta+rotation);
}


int 
carmen_base_run(void) 
{
  IPC_RETURN_TYPE err;
  int index;
  static carmen_base_reset_occurred_message reset = {0, 0};  
  int base_err;
  double tv, rv;
  double displacement, rotation;
  carmen_base_binary_data_message binary_data;

  if (reset_time > reset.timestamp) {
    reset.timestamp = reset_time;
    reset.host = carmen_get_host();
    err = IPC_publishData(CARMEN_BASE_RESET_OCCURRED_NAME, &reset);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_BASE_RESET_OCCURRED_NAME);  
    return 1;
  }  

  if (moving && carmen_get_time() - last_motion_command > motion_timeout) {
    moving = 0;
    do {
      base_err = carmen_base_direct_set_deceleration(deceleration);
      if (base_err < 0)
	initialize_robot();
    } while (base_err < 0);
    carmen_warn("T");      
    do {
      base_err = carmen_base_direct_set_velocity(0, 0);
      if (base_err < 0)
	initialize_robot();
    } while (base_err < 0);      
  }

  do {
    double packet_timestamp=0.;
    base_err = carmen_base_direct_update_status(&packet_timestamp);
    if (packet_timestamp==0.)
    	odometry.timestamp = carmen_get_time();
    else
    	odometry.timestamp = packet_timestamp;
    if (base_err < 0)
      initialize_robot();
  } while (base_err < 0); 

  carmen_base_direct_get_binary_data(&(binary_data.data), &(binary_data.size));
  binary_data.host = carmen_get_host();
  /*
  Giorgio 03.03.2006
	The binary data timestamp should be considered when reading the 1st byte of the packet.
  */
  binary_data.timestamp = carmen_get_time();
  IPC_publishData(CARMEN_BASE_BINARY_DATA_NAME, &binary_data);

  if (base_err > 0)
    return 2;

  do {
    if (!use_hardware_integrator) {
      base_err = carmen_base_direct_get_state
	(&displacement, &rotation, &tv, &rv);
      if (base_err < 0)
	initialize_robot();
      else
	integrate_odometry(displacement, rotation, tv, rv);
    } else {
      base_err = carmen_base_direct_get_integrated_state
	(&(odometry.x), &(odometry.y), &(odometry.theta), &(odometry.tv), 
	 &(odometry.rv));
      if (base_err < 0)
	initialize_robot();
      else {
	if (odometry_inverted) {
	  odometry.tv *= -1;
	  odometry.x *= -1;
	  odometry.y *= -1;
	}
      }
    }
  } while (base_err < 0);

  if (use_sonar && sonar_state) {
    carmen_base_direct_get_sonars(ranges, positions, num_sonar_ranges);
    sonar.num_sonars = num_sonar_ranges;
    for (index = 0; index < sonar.num_sonars; index++) {
      sonar.range = ranges;
      sonar.sonar_offsets = positions;
    }
  }
  
  err = IPC_publishData(CARMEN_BASE_ODOMETRY_NAME, &odometry);
  carmen_test_ipc_exit(err, "Could not publish", 
		       CARMEN_BASE_ODOMETRY_NAME);
  
  if (use_sonar) {
    carmen_warn("s");  
    sonar.timestamp = carmen_get_time();
    sonar.host = carmen_get_host();

    err = IPC_publishData(CARMEN_BASE_SONAR_NAME, &sonar);
    carmen_test_ipc_exit(err, "Could not publish", 
			 CARMEN_BASE_SONAR_NAME);
  }

  bumper.num_bumpers = 
    carmen_base_direct_get_bumpers(bumper.state, bumper.num_bumpers);
  if (bumper.state == NULL) {
    bumper.state = (unsigned char *)calloc(bumper.num_bumpers, sizeof(char));
    carmen_test_alloc(bumper.state);
  }

  if (bumper.num_bumpers > 0) {
    bumper.timestamp = carmen_get_time();
    bumper.host = carmen_get_host();
    err = IPC_publishData(CARMEN_BASE_BUMPER_NAME, &bumper);
    carmen_test_ipc_exit(err, "Could not publish", 
			 CARMEN_BASE_BUMPER_NAME);
  }

#if BASE_HAS_ARM
  if (num_arm_joints > 0) 
    carmen_arm_direct_get_state(arm_state.joint_angles, 
				arm_state.joint_currents,
				arm_state.joint_angular_vels, 
				&arm_state.gripper_closed, num_arm_joints);

  arm_state.timestamp = carmen_get_time();
  arm_state.host = carmen_get_host();
  err = IPC_publishData(CARMEN_ARM_STATE_NAME, &arm_state);
#endif

  return 1;
}

void 
carmen_base_shutdown(void)
{
  carmen_verbose("\nShutting down robot...");
  sleep(1);
  if (use_sonar && sonar_state)
    carmen_base_direct_sonar_off();
  carmen_verbose("done.\n");
  carmen_base_direct_shutdown_robot();
  exit(0);
}

void 
carmen_base_emergency_crash(int x __attribute__ ((unused)))
{
  carmen_base_direct_set_velocity(0.0, 0.0);
}

static void
shutdown_base(int signo __attribute__ ((unused)))
{
  carmen_base_shutdown();
}

int 
main(int argc, char **argv)
{
  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);

  signal(SIGINT, shutdown_base);
  signal(SIGSEGV, carmen_base_emergency_crash);

  x_ipcRegisterExitProc(carmen_base_shutdown);

  if (carmen_base_start(argc, argv) < 0)
    exit(-1);

  while(1) {
    if (carmen_base_run() == 1)
      fprintf(stderr, ".");
    carmen_ipc_sleep(0.1);
  }

  return 0;
}
