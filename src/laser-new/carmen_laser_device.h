#ifndef CARMEN_LASER_H
#define CARMEN_LASER_H

#ifdef LASER_USE_PTHREAD
#include <pthread.h>
#include <semaphore.h>
#endif

#include "laser_messages.h"
#include "laser_static_messages.h"


struct carmen_laser_device_t;
typedef int (*carmen_laser_fct_onreceive_t)(struct carmen_laser_device_t * , carmen_laser_laser_static_message* );
typedef int (*carmen_laser_fct_init_t)(struct carmen_laser_device_t* );
typedef int (*carmen_laser_fct_connect_t)(struct carmen_laser_device_t *, char* filename, int baudrate);
typedef int (*carmen_laser_fct_configure_t)(struct carmen_laser_device_t *);
typedef int (*carmen_laser_fct_start_t)(struct carmen_laser_device_t*);
typedef int (*carmen_laser_fct_stop_t)(struct carmen_laser_device_t*);
typedef int (*carmen_laser_fct_handle_t)(struct carmen_laser_device_t*);
typedef int (*carmen_laser_fct_close_t)(struct carmen_laser_device_t*);

#define CARMEN_LASER_MAX_DEVICE_NAME_LENGTH 1024

typedef struct carmen_laser_device_t{
	int laser_id;
	char device_name[CARMEN_LASER_MAX_DEVICE_NAME_LENGTH]; 
	carmen_laser_laser_config_t config;

	//virtual functions section
	carmen_laser_fct_init_t f_init;
	carmen_laser_fct_connect_t f_connect;
	carmen_laser_fct_configure_t f_configure;
	carmen_laser_fct_start_t f_start;
	carmen_laser_fct_stop_t f_stop;
	carmen_laser_fct_handle_t f_handle;
	carmen_laser_fct_close_t f_close;
	carmen_laser_fct_onreceive_t f_onreceive;

  
  //timestamp recovering
        int timestamp_recover;
        double expected_timing_error;

        int max_frames;
        int curr_frames;
        double expected_period;
  //
        double last_packet_time;


        void* device_data;
} carmen_laser_device_t;

#define MAX_LASER_DEVICES 1024
extern int carmen_laser_devices_num;
extern carmen_laser_device_t* carmen_laser_devices[MAX_LASER_DEVICES];

int carmen_laser_compare_configuration(carmen_laser_laser_config_t* c1, carmen_laser_laser_config_t* c2);
int carmen_laser_laser_message_check_configuration(carmen_laser_laser_config_t* config);

#define MAX_LASER_CONFIGURATIONS 1024
extern int carmen_laser_configurations_num;
extern carmen_laser_laser_config_t carmen_laser_configurations[MAX_LASER_CONFIGURATIONS];

carmen_laser_device_t* carmen_create_laser_instance(carmen_laser_laser_config_t* config, int laser_id, char* filename);

int carmen_laser_register_devices(void);

void carmen_laser_calibrate_timestamp(struct carmen_laser_device_t * device, int avg_cycles);

#endif
