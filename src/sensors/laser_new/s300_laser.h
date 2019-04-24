#ifndef __S300_LASER_H__
#define __S300_LASER_H__
#include <sys/select.h>

typedef struct s300_laser_t{
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
} s300_laser_t;

int s300_connect(s300_laser_t* sick, char* filename, int baudrate);

int s300_wait_packet(s300_laser_t* sick, unsigned char* reply);

int s300_wait_packet_ts(s300_laser_t* sick, unsigned char* reply, struct timeval *tv);

unsigned char s300_get_next_scan( s300_laser_t* sick, unsigned int * n_ranges,
				     			  unsigned int* range, struct timeval *tv );
						 
unsigned char s300_parse_measurement(
	s300_laser_t* sick __attribute__((unused)),
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
