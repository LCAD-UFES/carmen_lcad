#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <carmen/carmen.h>
#include "carmen_laser_device.h"
#include "s300_laser.h"
#include <sys/time.h>

int carmen_s300_init(carmen_laser_device_t* device){
	s300_laser_t* sick=malloc(sizeof(s300_laser_t));
	carmen_test_alloc(sick);
	device->device_data=sick;
	sick->remission_mode=REMISSION_NONE;
	return 1;
}

int carmen_s300_connect(carmen_laser_device_t * device, char* filename, int baudrate){
	int result;
	
	// the S300 should be configured to 500k
	
	s300_laser_t* sick=(s300_laser_t*)device->device_data;
	result=s300_connect(sick,filename,baudrate);
	if (result<=0){
		fprintf(stderr, "%%Error initializing the device\n");
		return 0;
	}
	return 1;
}

int carmen_s300_configure(carmen_laser_device_t * device ){

	// nothing to configure, configuration is done via the
	// SICK config tool.
	
	if (device->config.laser_type!=SICK_S300)
		return 0;
	
	return 1;	
}


int carmen_s300_handle_sleep(carmen_laser_device_t* device __attribute__ ((unused)) ){
	sleep(1);
	return 1;
}

int carmen_s300_handle(carmen_laser_device_t* device){
	struct timeval timestamp;
	unsigned int irange[1024];
	unsigned int iremission[1024];
	unsigned int n_ranges=0;
	unsigned int n_remissions=0;
	unsigned int reply=0;
	
	s300_laser_t* sick = (s300_laser_t*) device->device_data;
	
	reply = s300_get_next_scan(sick, &n_ranges, irange, &timestamp);
	if (! reply) return 0; // return if no scan found
	
	if (n_ranges){
		unsigned int j;
		carmen_laser_laser_static_message message;
		message.id=device->laser_id;
		message.config=device->config;

		message.config.start_angle= -0.5 * message.config.fov + M_PI/720.;
		
		if (sick->angular_resolution==0)
		  message.config.angular_resolution=M_PI/360;

		message.num_readings=n_ranges;
		message.num_remissions=n_remissions;
		message.timestamp=(double)timestamp.tv_sec+1e-6*timestamp.tv_usec;

		for (j=0; j<n_ranges; j++){
			// the S300 reports cm measurements
			message.range[j]=0.01*irange[j];
		}
		for (j=0; j<n_remissions; j++){
			message.remission[j]=0.001*iremission[j];
		}
		if (device->f_onreceive!=NULL)
			(*device->f_onreceive)(device, &message);
		return 1;
	}
	return 0;
}

int carmen_s300_start(carmen_laser_device_t* device){

	// the S300 should be configured to start in continuous mode
	device->f_handle=carmen_s300_handle;
	return 1;
}

int carmen_s300_stop(carmen_laser_device_t* device){

	device->f_handle=carmen_s300_handle_sleep;
	return 1;
}

int carmen_s300_close(struct carmen_laser_device_t* device){
	s300_laser_t* sick=(s300_laser_t*)device->device_data;
	return ! close(sick->fd);
}

carmen_laser_device_t* carmen_create_s300_instance(carmen_laser_laser_config_t* config, int laser_id){
	carmen_laser_device_t* device=(carmen_laser_device_t*)malloc(sizeof(carmen_laser_device_t));
	carmen_test_alloc(device);
	device->laser_id=laser_id;
	device->config=*config;
	device->f_init=carmen_s300_init;
	device->f_connect=carmen_s300_connect;
	device->f_configure=carmen_s300_configure;
	device->f_start=carmen_s300_start;
	device->f_stop=carmen_s300_stop;
	device->f_handle=carmen_s300_handle_sleep;
	device->f_close=carmen_s300_close;
	device->f_onreceive=NULL;
	return device;
}


carmen_laser_laser_config_t carmen_s300_valid_configs[]={
	{SICK_S300,	(-135./180.)*M_PI ,	(270./180.)*M_PI , M_PI/360,	30.0,	0.01,	0}	
};

int carmen_s300_valid_configs_size=1;

int carmen_init_s300_configs(void){
	int i;
	carmen_laser_laser_config_t* conf=carmen_laser_configurations+carmen_laser_configurations_num;
	for (i=0; i<carmen_s300_valid_configs_size; i++){
		*conf=carmen_s300_valid_configs[i];
		conf++;
	}
	carmen_laser_configurations_num+=carmen_s300_valid_configs_size;
	return carmen_laser_configurations_num;
}
