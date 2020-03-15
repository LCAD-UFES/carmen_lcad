#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include "orc.h"

int main( int argn, char **argv ){


  argn = argn;
  argv = argv;

  if (argn < 2) {
    printf("usage: orc_test <signed pwm>\n");
    exit(1);
  }

  fprintf( stderr,  " started program... " );
  orc_comms_impl_t *impl = orc_rawprovider_create("/dev/ttyUSB0");
  
  orc_t *orc = orc_create( impl  );
  fprintf( stderr, " made orc \n" );

  orc_motor_set_signed( orc, 1, atof(argv[1]) );
  sleep( 1 );

  printf("stopping\n");

  orc_motor_set_signed( orc, 1, 0 );
  sleep( 1 );

  /*
  // motors and encoders
  orc_motor_set_signed( orc, 2, -45 );
  int count = orc_quadphase_read( orc, 2 );
  fprintf( stderr, "encoder count = %d \n", count );

  sleep( 1 );
  int current =  orc_analog_read(orc, 18);
  sleep( 1 );

  orc_motor_set_signed( orc, 2, 0 );
  count = orc_quadphase_read( orc, 2 );
  fprintf( stderr, "current count = %d \n", current );
  fprintf( stderr, "encoder count = %d \n", count );
  */

  /*
  // analog and digital and pwm io
  orc_pwm_set( orc, 3, 0.75 );
  orc_digital_set( orc, 2, 1 );
  sleep( 2 );

  int val = orc_analog_read( orc, 0 );
  fprintf( stderr, "analog 0 = %d \n", val ); 
  val = orc_digital_read( orc, 1 );
  fprintf( stderr, "digital 1 = %d \n", val );
  sleep( 2 );

  orc_digital_set( orc, 2, 0 );
  orc_pwm_set( orc, 3, 0.00 );
  
  val = orc_digital_read( orc, 1 );
  fprintf( stderr, "digital 1 = %d \n", val );
  val = orc_analog_read( orc, 0 );
  fprintf( stderr, "analog 0 = %d \n", val ); 
  */


  fprintf( stderr, "done \n" );
  
  // clean up
  orc_destroy( orc );

}
