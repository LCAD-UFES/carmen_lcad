#ifndef __SICK_LASER_H__
#define __SICK_LASER_H__
#include <sys/select.h>

/* needed for new carmen_inline def for gcc >= 4.3 */
#include <carmen/carmen.h>

typedef struct sick_laser_t{
  unsigned char password[8];
  char filename[1024];
  int fd;
  int baud_rate;
  char parity;
  
  //internally used);
  int baudrate;
  int laser_mode;        //0=measurement, 1=configuration
  int remission_mode;    //0=no remission, 1=remission
  int device_resolution; //0=cm, 1=mm
  int device_range;      //8,16,32 80 meters
  int angular_resolution; //0: interlaced, 25, 50 100;
  unsigned char lms_configuration[100];
  int lms_conf_size;
  struct timeval last_packet_time;
} sick_laser_t;

int sick_connect(sick_laser_t* sick, char* filename, int baudrate);

int sick_configure(sick_laser_t* sick, int unit, int range, int reflectivity, int angular_range, int angular_res);

int sick_start_continuous_mode(sick_laser_t* sick);

int sick_stop_continuous_mode(sick_laser_t* sick);

int sick_wait_packet(sick_laser_t* sick, unsigned char* reply);

int sick_wait_packet_ts(sick_laser_t* sick, unsigned char* reply, struct timeval *tv);

unsigned char sick_parse_measurement(
	sick_laser_t* sick __attribute__((unused)),
	unsigned int* offset,
	unsigned int * n_ranges,
	unsigned int* range, 
	unsigned int* glare, 
	unsigned int* wfv, 
	unsigned int* sfv,
	unsigned int * n_remissions,
	unsigned int* remission,
	const unsigned char* packet);
#endif
