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
#include "../arm_low_level.h"
#include "orc_arm_constants.h"
#include "orclib_v5/orc.h"
#include <sys/ioctl.h>
#include <limits.h>

#define ORC_PWM_GAIN 1
#define MIN_ANGLE_TO_MOVE 0.005

/*
// velezj: For RssII Arm
#define ORC_SHOULDER_MOTOR 1
#define ORC_SHOULDER_MOTOR_ACTUAL_PWM 17
#define ORC_SHOULDER_MOTOR_DIR 25
#define ORC_SHOULDER_MOTOR_QUAD_PORT 16
#define ORC_SHOULDER_MOTOR_ENCODER 7
#define ORC_SHOULDER_MOTOR_DIRECTION_SIGN 1
#define ORC_ELBOW_MOTOR 3
#define ORC_ELBOW_MOTOR_ACTUAL_PWM 23
#define ORC_ELBOW_MOTOR_DIR 26
#define ORC_ELBOW_MOTOR_QUAD_PORT 18
#define ORC_ELBOW_MOTOR_ENCODER 10
#define ORC_ELBOW_MOTOR_DIRECTION_SIGN -1
#define ORC_ARM_GEAR_REDUCTION ( 65.5 * 5.0 )
#define ORC_ARM_TICKS_PER_RADIAN ( ORC_ENCODER_RESOLUTION * ORC_ARM_GEAR_REDUCTION / ( 2.0 * M_PI ) )
#define ORC_ELBOW_MAX_PWM 40 // 27
#define ORC_ELBOW_MIN_PWM 35 // 22
#define ORC_SHOULDER_MAX_PWM 50 // 37
#define ORC_SHOULDER_MIN_PWM 42 // 30

//#define ORC_ARM_FF_GAIN ((ORC_ARM_MAX_PWM / ORC_ARM_MAX_ANGULAR_VEL) * 0.9)

#define ORC_ARM_GRIPPER_SERVO 0 // 0
#define ORC_ARM_GRIPPER_MIN_PWM 8600
#define ORC_ARM_GRIPPER_PWM_PER_RADIAN 2214.2 

// velezj: rssII Arm -- used by all functions
static double shoulder_desired_theta = 0.0, elbow_desired_theta = 0.0;
static double shoulder_theta_iTerm = 0.0, elbow_theta_iTerm = 0.0;
static double shoulder_theta_error_prev = 0.0, elbow_theta_error_prev = 0.0;
static double shoulder_iTerm = 0.0, elbow_iTerm = 0.0;
static double shoulder_error_prev = 0.0, elbow_error_prev = 0.0;
static double shoulder_desired_angular_velocity = 0.0, elbow_desired_angular_velocity = 0.0;

static int shoulder_pwm, elbow_pwm;
static int shoulder_last_tick = 0, elbow_last_tick = 0;

// copied from arm_low_level.h ... stuff that should get implemented

  // passes in an orc pointer and tell arm how many joints
  int carmen_arm_direct_initialize(orc_t *orc, int num_joints, 
				   carmen_arm_joint_t *joint_types );
  int carmen_arm_direct_shutdown(void);

  // ----- sets ----- //

  // sets error and velocities to zero
  int carmen_arm_direct_reset(void);
  
  // sets safe joint use limits -- input array of limits for each element
  void carmen_arm_direct_set_limits(double min_angle, double max_angle,
				  int min_pwm, int max_pwm);
  


  // ----- gets ----- //


*/

/* 
   currently implemented:
   we assume an arm with three joints 
   all joints are motors -- not using joint types
   we don't allow 360 spins on the base (wire tangle)
*/

// ---- STATIC VARIABLES ---- //
// basic information

static orc_t *s_orc;
static int s_active;
static int s_num_joints;
static carmen_arm_joint_t *s_joint_types;
static double s_time;   
static double s_delta_time;

// current joint data
static double *s_arm_theta;
static int *s_arm_tick;
// emma: just trying
//static double *s_arm_tick;

static double *s_arm_angular_velocity;
static double *s_arm_iTerm;
static double *s_arm_error_prev;
static double *s_arm_current;
 
// limits of safe operation
static double *s_min_angle;
static double *s_max_angle;
static int *s_min_pwm;
static int *s_max_pwm;

// for debugging
//static int s_debug = 1;

// ---- LIST OF HELPER FUNCTIONS ---- //
static void update_internal_data(void);
static double compute_delta_theta( int curr, int prev, double ticks_to_radian );
static int min( int a, int b );
static double d_sign( double v );
static void d_bound_value( double *vPtr, double min, double max );
int i_sign( int v );
static void i_bound_value( int *vPtr, int min, int max );
static carmen_inline double gripper_radians_to_pwm(double theta);

//static void command_angular_velocity( double desired_angular_velocity, 
void command_angular_velocity( double desired_angular_velocity, 
			       double current_angular_velocity, int joint );

// ---- INIT/QUIT/RESET ---- //  
int carmen_arm_direct_initialize(carmen_arm_model_t *arm_model){
 
  // create orc and set scalars
  orc_comms_impl_t *impl = orc_rawprovider_create( arm_model->dev );
  s_orc = orc_create( impl );
  s_active = 1;
  s_time = carmen_get_time();
  s_num_joints = arm_model->num_joints;

  // initialize our static array variables
  s_joint_types = calloc( s_num_joints, sizeof( double ) );
  carmen_test_alloc( s_joint_types );
  s_arm_theta = calloc( s_num_joints, sizeof( double ) );
  carmen_test_alloc( s_arm_theta );
  s_arm_tick = calloc( s_num_joints, sizeof( int ) ); 
  carmen_test_alloc( s_arm_tick );
  s_arm_angular_velocity = calloc( s_num_joints, sizeof( double ) );
  carmen_test_alloc( s_arm_angular_velocity );
  s_arm_iTerm = calloc( s_num_joints, sizeof( double ) );
  carmen_test_alloc( s_arm_iTerm );
  s_arm_error_prev = calloc( s_num_joints, sizeof( double ) );
  carmen_test_alloc( s_arm_error_prev );
  s_arm_current = calloc( s_num_joints, sizeof( double ) );
  carmen_test_alloc( s_arm_current );
  s_min_angle = calloc( s_num_joints, sizeof( double ) );
  carmen_test_alloc( s_min_angle );
  s_max_angle = calloc( s_num_joints, sizeof( double ) );
  carmen_test_alloc( s_max_angle );
  s_min_pwm = calloc( s_num_joints, sizeof( int ) );
  carmen_test_alloc( s_min_pwm );
  s_max_pwm = calloc( s_num_joints, sizeof( int ) );
  carmen_test_alloc( s_max_pwm );
  
  // set arrays to initial values
  memcpy( s_joint_types, arm_model->joints, sizeof( arm_model->joints ) );

  s_joint_types[GRIPPER] = CARMEN_SERVO;  //dbug

  memcpy( s_min_angle, MIN_THETA, sizeof( MIN_THETA ) );
  memcpy( s_max_angle, MAX_THETA, sizeof( MAX_THETA ) );
  memcpy( s_min_pwm, MIN_PWM, sizeof( MIN_PWM ) );
  memcpy( s_max_pwm, MAX_PWM, sizeof( MAX_PWM ) );

  // reset
  carmen_arm_direct_reset();
  return 0;
}

int carmen_arm_direct_shutdown(void){
  carmen_arm_direct_reset();

  // free our static variables
  free( s_joint_types );
  free( s_arm_theta );
  free( s_arm_tick );
  free( s_arm_angular_velocity );
  free( s_arm_iTerm );
  free( s_arm_error_prev );
  free( s_arm_current );
  free( s_min_angle );  
  free( s_max_angle );
  free( s_min_pwm );
  free( s_max_pwm );
  
  // destroy the orc
  orc_destroy( s_orc );
  s_active = 0;

  return 0;
}

int carmen_arm_direct_reset(void){

  int i;

  // stop all motors, zero error and position
  for( i = 0; i < s_num_joints; ++i ){
    if (s_joint_types[i] == CARMEN_MOTOR) {
      orc_motor_set_signed( s_orc, MOTOR_PORTMAP[i], 0 );
      s_arm_iTerm[i] = 0;
      s_arm_error_prev[i] = 0;
      s_arm_theta[i] = 0;
      s_arm_tick[i] = orc_quadphase_read( s_orc, ENCODER_PORTMAP[i] );
      printf("------------------ RESET port[%d]=%d tick=%dq\n",
	     i,ENCODER_PORTMAP[i],s_arm_tick[i]);
    }
  }
  return 0;
}


// ---- SETS ---- //
void carmen_arm_direct_set_limits(double *min_angle, double *max_angle,
				  int *min_pwm, int *max_pwm){

  memcpy( s_min_angle, min_angle, sizeof( double ) );
  memcpy( s_max_angle, max_angle, sizeof( double ) );
  memcpy( s_min_pwm, min_pwm, sizeof( int ) );
  memcpy( s_max_pwm, max_pwm, sizeof( int ) );
}

// needs to be made 360 safe!!!!

// this is OPEN loop, should be part of a larger control loop
// sets desired joint angles and implements control for next time step
// most of this code is adapted from the RSS II arm by velezj
void carmen_arm_direct_update_joints( double *desired_angles ){

  //printf( "orc_arm_lib: desired angles are %f, %f, %f \n " , 
  //	  desired_angles[0], desired_angles[1], desired_angles[2] );

  double velocity_command_set[s_num_joints];
  int pwm_command_set[s_num_joints];

  double pTerm = 0.0, dTerm = 0.0; double *iTermPtr;

  // update to our current position
  update_internal_data();

  // determine the desired angular velocities 
  for( int i = 0; i < s_num_joints; ++i ){

    double theta_desired = carmen_normalize_theta(desired_angles[i]);
    theta_desired = carmen_clamp(s_min_angle[i], theta_desired, s_max_angle[i]);

    if (s_joint_types[i] == CARMEN_SERVO) {

      printf("SERVO(%.0f) ", carmen_radians_to_degrees(theta_desired));
      if (i == GRIPPER)
	orc_pwm_set(s_orc, MOTOR_PORTMAP[i], gripper_radians_to_pwm(theta_desired));
      
    }

    else {  // MOTOR

      printf("MOTOR ");

      // get the desired angular change
      double theta_delta = theta_desired - s_arm_theta[i];
      double desired_angular_velocity = 0;
      int command_pwm = 0;
      
      // compute and make sure velocites are within limits
      iTermPtr = &s_arm_iTerm[i];
      if( fabs( theta_delta ) > MIN_ANGLE_TO_MOVE ) {
	
	s_delta_time = 1.0;
	
	// compute the PID terms
	pTerm = theta_delta * (double)THETA_P_GAIN[i];
	*iTermPtr += ( theta_delta * (double)THETA_I_GAIN[i] ) * s_delta_time ;
	*iTermPtr = carmen_clamp(MIN_I_TERM, *iTermPtr, MAX_I_TERM);
	dTerm = ( theta_delta - s_arm_error_prev[i] ) / s_delta_time * (double)THETA_D_GAIN[i];
	
	// debug on PID terms
	//printf( "Joint %d: delta theta %f, theta_p_gain %f \n", i, theta_delta, (double)THETA_P_GAIN[i]);
	//printf( "Joint %d: p: %.3f, i: %.3f, d: %.3f, dt: %.3f \n", i, pTerm, *iTermPtr, dTerm, s_delta_time );
	
	if (fabs(theta_delta) < FINE_CONTROL_ANGLE)
	  desired_angular_velocity = 2*pTerm;
	else
	  desired_angular_velocity = pTerm + *iTermPtr + dTerm;
	
	// set velocity to be within limits
	d_bound_value( &desired_angular_velocity, -ORC_ARM_MAX_ANGULAR_VEL, ORC_ARM_MAX_ANGULAR_VEL );
	if( fabs( desired_angular_velocity ) < ORC_ARM_MIN_ANGULAR_VEL ) {
	  desired_angular_velocity = d_sign( desired_angular_velocity ) * ORC_ARM_MIN_ANGULAR_VEL;
	}
	
	// set the PWM to be within limits
	command_pwm = (int)( desired_angular_velocity );
	i_bound_value( &command_pwm, -MAX_PWM[i], MAX_PWM[i] );
	if (command_pwm > 0)
	  command_pwm = carmen_clamp(MIN_PWM[i], command_pwm, MAX_PWM[i]);
	else
	  command_pwm = carmen_clamp(-MAX_PWM[i], command_pwm, -MIN_PWM[i]);
	
	printf( "Joint %d: p: %.3f, i: %.3f, d: %.3f, dt: %.3f, pwm: %d \n", i, pTerm, *iTermPtr, dTerm, s_delta_time, command_pwm );
	
	// produce debug outputs
	//if( s_debug == 1 ){
	//	printf( "Joint %d: desired angle %f, actual angle %f, delta angle %f, desired_vel %f \n",
	//	      i, desired_angles[i], s_arm_theta[i], theta_delta, desired_angular_velocity );
	//}
	
	// update the delta error
	s_arm_error_prev[i] = theta_delta;
	
      } else {
	desired_angular_velocity = 0.0;
	command_pwm = 0.0;
	*iTermPtr = 0.0;   
      }
      
      // set the values into our array
      velocity_command_set[i] = desired_angular_velocity;
      pwm_command_set[i] = command_pwm;
      
    }
  }

  // actually command the velocities
  for( int i = 0; i < s_num_joints; ++i ){
    if (s_joint_types[i] == CARMEN_MOTOR) {
      // velocity_command_set( desired_vel[i], s_arm_angular_velocity[i], MOTOR_PORTMAP[i] );
      
      // for now, don't include the command_angular_velocity function and do this directly
      //printf("---------------- actually commanded port[%d]= %d to do pwm=%d\n",
      //	   i,MOTOR_PORTMAP[i],pwm_command_set[i]);
      orc_motor_set_signed(s_orc, MOTOR_PORTMAP[i], pwm_command_set[i]);
    }
  }

  printf( "\n" );


}

// ---- GETS ---- //
void carmen_arm_direct_get_state(double *joint_angles, double *joint_currents,
				 double *joint_angular_vels, int *gripper_closed ){
  int i;

  update_internal_data();

  // put values into the output from what we have in here
   for( i = 0; i < s_num_joints; ++i ){
     joint_angles[i] = s_arm_theta[i]; 
     joint_currents[i] = s_arm_current[i];
     joint_angular_vels[i] = s_arm_angular_velocity[i];
   }
 
  // for now, since the gripper's not being used
   if (gripper_closed)
     *gripper_closed = 0;
}


// velezj: rssII Arm
//static void command_angular_velocity( double desired_angular_velocity, 
void command_angular_velocity( double desired_angular_velocity, 
				      double current_angular_velocity, int joint ) {
  
  double ffTerm = 0.0, pTerm = 0.0, dTerm = 0.0, velError = 0.0;
  double * iTermPtr;
  double * velErrorPrevPtr;;
  int current_pwm;
  int command_pwm;
  int max_pwm, min_pwm;

  // get base parameters for your joint motor
  iTermPtr = &s_arm_iTerm[joint];
  velErrorPrevPtr = &s_arm_error_prev[joint];
  max_pwm = s_max_pwm[joint];
  min_pwm = s_min_pwm[joint];


  //printf("*********************************\n");
  //printf("global max pwm for joint %d: %d\n", joint, max_pwm);
  //printf("global min pwm for joint %d: %d\n", joint, min_pwm);
 
  // if there is non-zero desired velocity
  if (fabs(desired_angular_velocity) > .0005) {

    // compute PID terms
    velError = (desired_angular_velocity - current_angular_velocity);
    ffTerm = desired_angular_velocity * ORC_ARM_FF_GAIN;
    pTerm = velError * ORC_ARM_P_GAIN;
    *iTermPtr += velError * ORC_ARM_I_GAIN;
    dTerm = (velError - *velErrorPrevPtr) * ORC_ARM_D_GAIN;
    *velErrorPrevPtr = velError;

    // set the pwm between the desired bounds
    current_pwm = (int)(ffTerm + pTerm + *iTermPtr + dTerm);
    if (abs(current_pwm) > max_pwm){
      //printf("In max\n");
      //printf("Current_pwm was: %d\n", current_pwm);
      current_pwm = (current_pwm > 0 ? max_pwm : -max_pwm);
      //printf("Current_pwm is now: %d\n", current_pwm);
    }
    if( abs( current_pwm) < min_pwm ){
      //printf("In min\n");
      current_pwm = ( current_pwm > 0 ? min_pwm : -min_pwm );}

    command_pwm = abs( current_pwm );
    //printf("command_pwm is now: %d\n", current_pwm);
  } else {
    command_pwm = 0;
    *iTermPtr = 0.0;
  } // end if motion desired

  
  // debug
  if( command_pwm != 0 ) {
    printf( "[ %d ] p: %f  i: %f  d: %f   ve: %f\n", joint, pTerm, *iTermPtr, dTerm, velError );  
    printf( "[ %d ] sending PWM: %d\n", joint, command_pwm );
  }

  //printf("Current Angular Velocity %f\n", current_angular_velocity);
  //printf("Desired Angular Velocity %f\n", desired_angular_velocity);
  //printf("command_pwm %d\n", command_pwm);

  orc_motor_set_signed(s_orc, MOTOR_PORTMAP[joint], command_pwm);
}


// ---- HELPER FUNCTIONS ---- //

// this reads relevant arm sensors and updates static values
static void update_internal_data(void)
{

  double curr_time = carmen_get_time();
  
  // we'll need this to update elbow state after the loop
  double shoulder_delta_theta = 0.0;

  // updates arm angles, velocities, and currents
  for( int i = 0; i < s_num_joints; ++i ){
    if (s_joint_types[i] == CARMEN_MOTOR) {
      int port = MOTOR_PORTMAP[i];
      int curr_tick_count = orc_quadphase_read( s_orc, ENCODER_PORTMAP[i] );
      int prev_tick_count = s_arm_tick[i];
      
      // compute the change in angle since the last update
      double ticks_to_radian = 1/(double)TICKS_PER_RADIAN[i];
      double delta_theta = compute_delta_theta( curr_tick_count, prev_tick_count,
						ticks_to_radian );
      delta_theta = REVERSE_THETA[i] * delta_theta;
      
      //printf("* Internal Update: port[%d]=%d, old theta was %f, d theta is %f curr_tick=%d prev_tick=%d\n",
      //   i,ENCODER_PORTMAP[i],s_arm_theta[i],delta_theta,curr_tick_count,prev_tick_count);
      
      // set variables
      s_arm_theta[i] = carmen_normalize_theta( delta_theta + s_arm_theta[i] );
      s_arm_angular_velocity[i] = delta_theta / ( curr_time - s_time );
      s_arm_current[i] = (double)orc_analog_read( s_orc, 16 + port );  // motor ports are 16 + reg
      s_arm_tick[i] = curr_tick_count;
      
      // check if this is the shoulder
      if( i == SHOULDER ){
	shoulder_delta_theta = delta_theta;
      }
    }
  }		   
				
  // correct for the fact that when the shoulder moves, the elbow angle also changes
  s_arm_theta[ELBOW] = s_arm_theta[ELBOW] - shoulder_delta_theta;
 
  // update time
  s_delta_time = curr_time - s_time;
  s_time = curr_time;  
}

// we assume the direction is the smallest route around the circle
static double compute_delta_theta( int curr, int prev, double ticks_to_radian )
{

  // compute the number of ticks traversed short and long way around
  // and pick the path with the minimal distance
  int abs_reg = abs( curr - prev );
  int abs_opp = 65536 - abs_reg;
  int actual_delta = min( abs_reg, abs_opp );
  double theta = (double)actual_delta * ticks_to_radian;

  // give the angle the correct sign -- ccw is positive
  if( ( curr > prev && actual_delta == abs_reg ) || 
      ( curr < prev && actual_delta == abs_opp ) )
    {
      theta = -theta;
    }
  
  return theta;
}

static int min( int a, int b ){
  return ( a < b ) ? a : b ;
}

static double d_sign( double v ) {
  if( v < 0.0 )
    return -1.0;
  return 1.0;
}

int i_sign( int v ) {
  if( v < 0 )
    return -1;
  return 1;
}

static void d_bound_value( double *vPtr, double min, double max ) {
  if( *vPtr < min )
    *vPtr = min;
  if( *vPtr > max )
    *vPtr = max;
}

static void i_bound_value( int *vPtr, int min, int max ) {
  if( *vPtr < min )
    *vPtr = min;
  if( *vPtr > max )
    *vPtr = max;
}

static carmen_inline double gripper_radians_to_pwm(double theta) {

  return (theta/M_PI)*0.09 + 0.025;
}


/*

  unsigned char *buffer;
  double start_time = carmen_get_time();

  if (!initialized) {
    start = carmen_get_time();
    initialized = 1;
    // velezj: RssII Arm
    shoulder_theta = 0.0;
    elbow_theta = 0.0;
    shoulder_last_tick = orc_quadphase_read(orc, ORC_SHOULDER_MOTOR_ENCODER);
    elbow_last_tick = orc_quadphase_read(orc, ORC_ELBOW_MOTOR_ENCODER);

    shoulder_pwm = orc_analog_read(orc, ORC_SHOULDER_MOTOR_ACTUAL_PWM);
    elbow_pwm = orc_analog_read(orc, ORC_SHOULDER_MOTOR_ACTUAL_PWM);

    return;
  }

  shoulder_pwm = orc_analog_read(orc, ORC_SHOULDER_MOTOR_ACTUAL_PWM);
  elbow_pwm = orc_analog_read(orc, ORC_ELBOW_MOTOR_ACTUAL_PWM);

  shoulder_tick = orc_quadphase_read(orc, ORC_SHOULDER_MOTOR_ENCODER);
  elbow_tick = orc_quadphase_read(orc, ORC_ELBOW_MOTOR_ENCODER);
  
  // velezj: For RssII Arm

  shoulder_delta_tick = shoulder_tick - shoulder_last_tick;
  if (shoulder_delta_tick > SHRT_MAX/2)
    shoulder_delta_tick = shoulder_delta_tick - 2*SHRT_MAX;
  if (shoulder_delta_tick < -SHRT_MAX/2)
    shoulder_delta_tick = shoulder_delta_tick + 2*SHRT_MAX;

  elbow_delta_tick = elbow_tick - elbow_last_tick;
  if (elbow_delta_tick > SHRT_MAX/2)
    elbow_delta_tick = elbow_delta_tick - 2*SHRT_MAX;
  if (elbow_delta_tick < -SHRT_MAX/2)
    elbow_delta_tick = elbow_delta_tick + 2*SHRT_MAX;

  shoulder_last_tick = shoulder_tick;
  elbow_last_tick = elbow_tick;

  shoulder_angle_change = delta_ticks_to_angle(shoulder_delta_tick);
  elbow_angle_change = delta_ticks_to_angle(elbow_delta_tick);
  if( fabs( shoulder_desired_theta - shoulder_theta ) > 0.01 ) {
    printf( "shoulder_delta_theta: %f  (%d)\n", shoulder_angle_change, shoulder_delta_tick );
  }
  if( fabs( elbow_desired_theta - elbow_theta ) > 0.01 ) {
    printf( "elbow_delta_theta: %f  (%d)\n", elbow_angle_change, elbow_delta_tick );
  }

  // velezj: For RssII Arm
  shoulder_theta = shoulder_theta + shoulder_angle_change;
  elbow_theta = elbow_theta + elbow_angle_change;
  // since elbow relative to shoulder and when the shoulder moves to elbow angle
  // in fact does NOT change with it, so we must update our elbow angle even when
  // only the shoulder moves
  //elbow_theta -= shoulder_angle_change; 
  shoulder_angular_velocity = shoulder_angle_change / delta_slave_time;
  elbow_angular_velocity = elbow_angle_change / delta_slave_time;
  if( fabs( shoulder_desired_theta - shoulder_theta ) > 0.01 || fabs( elbow_desired_theta - elbow_theta ) > 0.01 ) {
    printf( "shoulder_theta: %f  (av = %f)\n", shoulder_theta, shoulder_angular_velocity );
    printf( "elbow_theta:    %f  (av = %f)\n", elbow_theta, elbow_angular_velocity  );
  }

  free(buffer);

  if(0)
  carmen_warn("time to update: %.2f\n", carmen_get_time() -
	      start_time);
  return 0;
}

static double delta_ticks_to_angle( int delta_ticks ) 
{
  double radians = (double)delta_ticks / (double)ORC_ARM_TICKS_PER_RADIAN;
  return radians;
}

static double voltage_to_current(unsigned short voltage)
{
  double current;
  current = voltage*5.0/65536.0;
  // V=IR, R=0.18 ohm
  current = current/0.18;
  return current;
}


// velezj: rssII Arm


// velezj: rssII Arm

*/




