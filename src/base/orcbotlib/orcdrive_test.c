#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <carmen/carmen.h>

#include "../drive_low_level.h"


int main( int argn, char **argv ){

  // so the program does not complain
  argn = argn;
  argv = argv;

  fprintf( stderr,  " started program... \n" );

  // initialize
  carmen_base_direct_initialize_robot( "bot", "/dev/ttyUSB0" );
  
  // set a desired velocity
  double tv = -0.0;
  double rv = 0.0;
  
  double x, y, theta, tv_out, rv_out;
  double time;

  // loop 
  for( int i = 0; i <10000; ++i ){

    carmen_base_direct_set_velocity( tv, rv );
    carmen_base_direct_update_status( &time );
    carmen_base_direct_get_integrated_state( &x, &y, &theta, &tv_out, &rv_out );
    
    printf( "  \n  on iter %d \n ", i );

  }

  // clean up
  carmen_base_direct_shutdown_robot();

}
