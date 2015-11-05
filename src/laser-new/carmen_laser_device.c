#include <string.h>
#include <carmen/carmen.h>
#include "carmen_laser_device.h"
#include <stdio.h>

#ifdef __APPLE__
#include <limits.h>
#include <float.h>
#define MAXDOUBLE DBL_MAX
#else
#include <values.h>
#endif 


int carmen_laser_compare_configuration(carmen_laser_laser_config_t* c1, carmen_laser_laser_config_t* c2){
  if (c1->laser_type!=c2->laser_type){
    return 1;
  }

  if ((c1->angular_resolution!=MAXDOUBLE && c2->angular_resolution!=MAXDOUBLE) && 
      fabs(c1->angular_resolution - c2->angular_resolution) > carmen_degrees_to_radians(0.01))
    return 2;

  if ((c1->start_angle!=MAXDOUBLE && c2->start_angle!=MAXDOUBLE) && c1->start_angle!=c2->start_angle)
    return 4;
  
  if ((c1->fov!=MAXDOUBLE && c2->fov!=MAXDOUBLE) && c1->fov!=c2->fov)
    return 5;

  if ((c1->maximum_range!=MAXDOUBLE && c2->maximum_range!=MAXDOUBLE) && c1->maximum_range!=c2->maximum_range)
    return 6;

  if ((c1->accuracy!=MAXDOUBLE && c2->accuracy!=MAXDOUBLE) && c1->accuracy!=c2->accuracy)
    return 7;

  if (c1->remission_mode!=c2->remission_mode)
    return 8;

  return 0;
}

int carmen_laser_laser_message_check_configuration(carmen_laser_laser_config_t* config){

  char remission_string[256];
  if (config->remission_mode == REMISSION_NONE)
    strcpy(remission_string,"none");
  else if (config->remission_mode == REMISSION_DIRECT)
    strcpy(remission_string,"direct");
  else if (config->remission_mode == REMISSION_NORMALIZED)
    strcpy(remission_string,"normalized");
  else
    strcpy(remission_string,"unkown value");

  char type_string[256];
  if (config->laser_type == SICK_LMS)
    strcpy(type_string,"SICK_LMS");
  else if (config->laser_type == SICK_S300)
    strcpy(type_string,"SICK_S300");
  else if (config->laser_type == SICK_PLS)
    strcpy(type_string,"SICK_PLS");
  else if (config->laser_type == HOKUYO_URG)
    strcpy(type_string,"HOKUYO_URG");
  else if (config->laser_type == SIMULATED_LASER)
    strcpy(type_string,"SIMULATED_LASER");
  else
    strcpy(type_string,"UMKNOWN_PROXIMITY_SENSOR");
  
  int i;
  carmen_warn("Requested laser configuration:\n");
  carmen_warn("  type ............................. %s\n",  type_string);
  carmen_warn("  remission_mode ................... %s\n",  remission_string);

  if (config->angular_resolution>0 && config->angular_resolution != MAXDOUBLE)
    carmen_warn("  resolution ....................... %.2lf deg\n", 
		carmen_radians_to_degrees(config->angular_resolution));


  if (config->maximum_range>0 && config->maximum_range != MAXDOUBLE)
    carmen_warn("  max range ........................ %.3lf m\n", config->maximum_range);
/*   else */
/*     carmen_warn("  max range ........................ not required\n"); */

/*   if (config->accuracy>0) */
/*     carmen_warn("  accuracy ......................... %.4lf\n", config->accuracy); */
/*   else */
/*     carmen_warn("  accuracy ......................... not required\n"); */
  
  if (config->fov>0) {
    carmen_warn("  fov .............................. %.0lf deg\n", carmen_radians_to_degrees(config->fov));
    carmen_warn("  start angle ...................... %.2lf deg\n", carmen_radians_to_degrees(config->start_angle));
  }
/*   else { */
/*     carmen_warn("  fov .............................. not required\n"); */
/*     carmen_warn("  start angle ...................... not required\n"); */
/*   } */
  

  if (config->laser_type == SICK_S300) {
    carmen_warn("  INFO: The current S300 driver requires the laser to be configured\n"); 
    carmen_warn("        in the 500kbps continuous mode (using the SICK Windows tools).\n\n"); 
  }

  
  for (i=0; i<carmen_laser_configurations_num; i++){
	int result=!carmen_laser_compare_configuration(config, carmen_laser_configurations+i);	  
	if (result) {
	  return 1;
	}
	}
  return 0;
}

int carmen_laser_calibrate_timestamp_recover_handler(struct carmen_laser_device_t * device , carmen_laser_laser_static_message* m){
  fprintf(stderr,".");
  if (!device->curr_frames){
    device->expected_period/= device->max_frames;
    fprintf(stderr, "Estimated frequency of laser %d is %.4f Hz\n",  
	    device->laser_id, 1./device->expected_period);
    device->max_frames=0;
    return 1;
  }

  if (device->max_frames==device->curr_frames){
    device->expected_period=0.;
    device->last_packet_time=m->timestamp;
  }
  device->expected_period+=m->timestamp-device->last_packet_time;
  device->curr_frames--;
  device->last_packet_time=m->timestamp;
  return 0;
}

void carmen_laser_calibrate_timestamp(struct carmen_laser_device_t * device, int avg_cycles){
  device->max_frames=avg_cycles;
  device->curr_frames=avg_cycles;
  device->f_onreceive=carmen_laser_calibrate_timestamp_recover_handler;
}


int carmen_laser_configurations_num=0;
carmen_laser_laser_config_t carmen_laser_configurations[MAX_LASER_CONFIGURATIONS];
