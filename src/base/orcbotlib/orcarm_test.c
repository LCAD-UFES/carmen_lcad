#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <carmen/carmen.h>

#include "../arm_low_level.h"


int main( int argn, char **argv ){

  // so the program does not complain
  argn = argn;
  argv = argv;

  fprintf( stderr,  " started program... \n" );

  // create the model
  carmen_arm_model_t arm_model;
  arm_model.dev = "/dev/ttyUSB0";
  arm_model.num_joints = 3;

  // initialize
  carmen_arm_direct_initialize( &arm_model );

  // set desired angles : order is always end -> base (elbow, shoulder, hip )
  double joint_angles[3] = { 0.0 , 0.0 , 2.0 }; 

  for( int i = 0; i < 200; ++i ){

    fprintf( stderr,  " inside control loop \n" );
    carmen_arm_direct_update_joints( joint_angles );   

    
      double curr_joint_angles[3];
      double curr_joint_currents[3];
      double curr_joint_angular_vels[3];
      int curr_gripper_closed[1];
    
   
      carmen_arm_direct_get_state(curr_joint_angles, curr_joint_currents,
				 curr_joint_angular_vels, 
				 curr_gripper_closed );
      printf("          on iter %d \n", i );
       printf("\nJoint Angles: %f %f %f\n", curr_joint_angles[0],curr_joint_angles[1],curr_joint_angles[2]);
       printf("Joint Currents: %f %f %f\n", curr_joint_currents[0],curr_joint_currents[1],curr_joint_currents[2]);
       printf("Joint Angular Vels: %f %f %f\n", curr_joint_angular_vels[0],curr_joint_angular_vels[1],curr_joint_angular_vels[2]);
    

  }
  
  // clean up
  carmen_arm_direct_shutdown();

}
