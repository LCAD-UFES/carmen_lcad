#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <carmen/carmen.h>
#include "carmen_laser_device.h"
#include "sick_laser.h"
#include <sys/time.h>

int carmen_sick_init(carmen_laser_device_t* device){
	sick_laser_t* sick=malloc(sizeof(sick_laser_t));
	carmen_test_alloc(sick);
	device->device_data=sick;
	sick->remission_mode=REMISSION_NONE;
	return 1;
}

int carmen_sick_connect(carmen_laser_device_t * device, char* filename, int baudrate){
	int result;
	sick_laser_t* sick=(sick_laser_t*)device->device_data;
	result=sick_connect(sick,filename,baudrate);
	if (result==-4){
		fprintf(stderr, "\nReconnecting device ................ ");
		result=sick_connect(sick,filename,baudrate);
	}

	if (result<=0){
		fprintf(stderr, "Error initializing the device\n");
		return 0;
	}
	return 1;
}

int carmen_sick_configure(carmen_laser_device_t * device ){
	int unit, range, angular_range, angular_res;
	int result;
	sick_laser_t* sick=(sick_laser_t*)device->device_data;
	//chack if we are in interlaced mode
	if (device->config.laser_type!=SICK_LMS)
		return 0;
	angular_range=(int)(device->config.fov*180./M_PI);
	angular_res=(int)(device->config.angular_resolution*100.*180./M_PI);
	range=(int)(device->config.maximum_range);
	if (range>10){
		range=80;
		unit=0;
	} else {
		range=8;
		unit=1;
	}
	if (device->config.remission_mode!=REMISSION_NONE){
		sick->remission_mode=1;
	}
	result=sick_configure(sick, unit, range, sick->remission_mode, angular_range, angular_res);
	//	fprintf(stderr,"%x, %unit=d, ,range=%d, remission=%d, ang_range=%d, ang_res%d",(unsigned int)sick, unit, range, sick->remission_mode, angular_range, angular_res);
	if (device->config.fov==M_PI && device->config.angular_resolution==M_PI/720){
		sick->angular_resolution=0;
	}
	return result>0;
}

int carmen_sick_handle_sleep(carmen_laser_device_t* device __attribute__ ((unused)) ){
	sleep(1);
	return 1;
}

double expectedPeriod=1./75.;
double timeTolerance=0.5;
int timestampingReconstruction=0;

int carmen_sick_handle(carmen_laser_device_t* device){
	struct timeval timestamp;
	unsigned char buffer[2048];
	unsigned int offset;
	unsigned int irange[1024];
	unsigned int iremission[1024];
	unsigned int n_ranges=0;
	unsigned int n_remissions=0;
	unsigned int reply=0;
	sick_laser_t* sick=(sick_laser_t*)device->device_data;
	struct timeval oldTime=sick->last_packet_time;
	reply=sick_wait_packet_ts(sick, buffer, &timestamp);
	if (! reply)
		return 0;
	int sec=timestamp.tv_sec=oldTime.tv_sec;
	int usec=timestamp.tv_usec=oldTime.tv_usec;
	double dt=sec+1e-6*usec;
	if (timestampingReconstruction){
	  double ett=timeTolerance/expectedPeriod;
	  if (fabs(dt-expectedPeriod)>ett)
	    fprintf(stderr, "D");
	    return 0;
	}
	//fprintf(stderr,"!");
	sick_parse_measurement(sick , &offset, &n_ranges, irange, NULL, NULL, NULL, &n_remissions, iremission, buffer);

	if (n_ranges){
		unsigned int j;
		carmen_laser_laser_static_message message;
		message.id=device->laser_id;
		message.config=device->config;

		/// FIX: explain what  M_PI/720.*offset is ? (the interlaced mode?)
		///		message.config.start_angle=M_PI/720.*offset;

		//this is for adjusting the initial offset when in interlaced mode.
		//offset can range from 0 to 3 and it is the offset of the first beam.
		//the offset in is obtained by multiplying the offset value with 0.25 degrees (M_PI/720 rad).

		message.config.start_angle= -0.5 * message.config.fov + M_PI/720.*offset;
		if (sick->angular_resolution==0)
		  message.config.angular_resolution=M_PI/180;

		message.num_readings=n_ranges;
		message.num_remissions=n_remissions;
		message.timestamp=(double)timestamp.tv_sec+1e-6*timestamp.tv_usec;
		for (j=0; j<n_ranges; j++){
			message.range[j]=0.001*irange[j];
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

int carmen_sick_start(carmen_laser_device_t* device){
	int retries=10, i=0;
	int result=0;
	sick_laser_t* sick=(sick_laser_t*)device->device_data;
	do {
		result=sick_start_continuous_mode(sick);
		i++;
	} while (result<=0 && i<retries);
	device->f_handle=carmen_sick_handle;
	return result;
}

int carmen_sick_stop(carmen_laser_device_t* device){
	int result=0;
	sick_laser_t* sick=(sick_laser_t*)device->device_data;
	do {
		result=sick_stop_continuous_mode(sick);
	} while (result<=0);
	device->f_handle=carmen_sick_handle_sleep;
	return result;
}



//FIXME I do not want  to malloc the ranges!

int carmen_sick_close(struct carmen_laser_device_t* device){
	sick_laser_t* sick=(sick_laser_t*)device->device_data;
	return ! close(sick->fd);
}

carmen_laser_device_t* carmen_create_sick_instance(carmen_laser_laser_config_t* config, int laser_id){
	carmen_laser_device_t* device=(carmen_laser_device_t*)malloc(sizeof(carmen_laser_device_t));
	carmen_test_alloc(device);
	device->laser_id=laser_id;
	device->config=*config;
	device->f_init=carmen_sick_init;
	device->f_connect=carmen_sick_connect;
	device->f_configure=carmen_sick_configure;
	device->f_start=carmen_sick_start;
	device->f_stop=carmen_sick_stop;
	device->f_handle=carmen_sick_handle_sleep;
	device->f_close=carmen_sick_close;
	device->f_onreceive=NULL;
	return device;
}


carmen_laser_laser_config_t carmen_sick_valid_configs[]={
//80 m modes
	{SICK_LMS,	-M_PI/2.,	M_PI,		M_PI/180,	81.9,	0.01,	0},
	{SICK_LMS,	-M_PI/2.,	M_PI,		M_PI/360,	81.9,	0.01,	0},
	{SICK_LMS,	-M_PI/180.*50.,	M_PI/18.*10.,	M_PI/180,	81.9,	0.01,	0},
	{SICK_LMS,	-M_PI/180.*50.,	M_PI/18.*10.,	M_PI/360,	81.9,	0.01,	0},
	{SICK_LMS,	-M_PI/180.*50.,	M_PI/18.*10.,	M_PI/720,	81.9,	0.01,	0},

//remission modes

	{SICK_LMS,	-M_PI/2.,	M_PI,		M_PI/180,	81.9,	0.01,	1},
	{SICK_LMS,	-M_PI/2.,	M_PI,		M_PI/360,	81.9,	0.01,	1},
	{SICK_LMS,	-M_PI/180.*50.,	M_PI/18.*10.,	M_PI/180,	81.9,	0.01,	1},
	{SICK_LMS,	-M_PI/180.*50.,	M_PI/18.*10.,	M_PI/360,	81.9,	0.01,	1},
	{SICK_LMS,	-M_PI/180.*50.,	M_PI/18.*10.,	M_PI/720,	81.9,	0.01,	1},

//8m modes

	{SICK_LMS,	-M_PI/2.,	M_PI,		M_PI/180,	8.19,	0.001,	0},
	{SICK_LMS,	-M_PI/2.,	M_PI,		M_PI/360,	8.19,	0.001,	0},
	{SICK_LMS,	-M_PI/180.*50.,	M_PI/18.*10.,	M_PI/180,	8.19,	0.001,	0},
	{SICK_LMS,	-M_PI/180.*50.,	M_PI/18.*10.,	M_PI/360,	8.19,	0.001,	0},
	{SICK_LMS,	-M_PI/180.*50.,	M_PI/18.*10.,	M_PI/720,	8.19,	0.001,	0},
//interlaced modes
	{SICK_LMS,	-M_PI/2.,	M_PI,		M_PI/720,	8.19,	0.001,	0},
	{SICK_LMS,	-M_PI/2.,	M_PI,		M_PI/720,	81.9,	0.01,	0},
	
};

int carmen_sick_valid_configs_size=17;

int carmen_init_sick_configs(void){
	int i;
	carmen_laser_laser_config_t* conf=carmen_laser_configurations+carmen_laser_configurations_num;
	for (i=0; i<carmen_sick_valid_configs_size; i++){
		*conf=carmen_sick_valid_configs[i];
		conf++;
	}
	carmen_laser_configurations_num+=carmen_sick_valid_configs_size;
	return carmen_laser_configurations_num;
}

