#include "carmen_laser_device.h"
#include "carmen_laser_message_queue.h"
#include "laser_messages.h"
#include <signal.h>
#include <stdio.h>
#include <pthread.h>
#include <math.h>


carmen_laser_message_queue_t queue;

int carmen_laser_enqueue(struct carmen_laser_device_t * device __attribute__((unused)), carmen_laser_laser_static_message* message){
	carmen_laser_message_queue_add(&queue, message);
	device->last_packet_time=message->timestamp;
	return 0;
}


volatile int has_to_stop=0;

void sigquit_handler(int q __attribute__((unused))){
	has_to_stop=1;
}


void* laser_fn (struct carmen_laser_device_t * device){
	int result=0;
	result=(*(device->f_start))(device);
	if (! result){
		has_to_stop=1;
		return 0;
	}
	while (! has_to_stop){
		result=(*(device->f_handle))(device);
	}
	fprintf(stderr, "stopping\n");
	result=(*(device->f_stop))(device);
	if (! result){
		return 0;
	}
	return 0;
}


int main (int argc __attribute__((unused)), char** argv __attribute__((unused))){
	int c=0;
	pthread_t laser_thread;
	void * tresult;


	//CYRILL
	// here you should get the config from the param server
	carmen_laser_laser_config_t config={SICK_LMS, -M_PI/2., M_PI, M_PI/180, 81.9, 0.01, 0};
		/*{SICK_LMS, -M_PI/90., M_PI, M_PI/180, 81.9, 0.01, 0};*/
		/*{SICK_LMS,	-M_PI/180.*50.,	M_PI/18.*10.,	M_PI/180,	81.9,	0.01,	0}*/

	carmen_laser_device_t*	device=0;
	char* filename = "/dev/ttyUSB0";
	int result=0;
	//initialization_stuff

	//this initializes the driver stuctures
	camen_laser_register_devices();
	signal(SIGINT, sigquit_handler);
	
	//check whether the configuration is valid
	int config_valid=carmen_laser_laser_message_check_configuration(&config);
	if (! config_valid){
		fprintf(stderr,"config invalid\n");
		return -1;
	}


	//attempts to create a laser instance
	device=carmen_create_laser_instance(&config, 0, filename);
	if (! device){
		fprintf(stderr, "error in creating the device\n");
		return -2;
	}

	//install the enqueuing handler
	device->f_onreceive=carmen_laser_enqueue;
	device->config=config;

	//attempt initializing the device
	fprintf(stderr, "Device initialization\n");
	result=(*(device->f_init))(device);
	if (! result){
		fprintf(stderr, "Error\n");
		return -3;
	}

	//connect, and set the serial line (inf any) to the desired baudrate
	fprintf(stderr, "Connection\n");
	result=(*(device->f_connect))(device, filename, 38400);
	if (! result){
		fprintf(stderr, "Error\n");
		return -3;
	}

	//configure the device
	fprintf(stderr, "Configuation\n");
	result=(*(device->f_configure))(device);
	if (! result){
		fprintf(stderr, "Error\n");
		return -3;
	}

	//starts the reading thread
	fprintf(stderr, "starting reader\n");
	pthread_create (&laser_thread, NULL, (void*(*)(void*))laser_fn, device);

	//waits in the queue
	double oldTime=0;
	while (! has_to_stop){
		//int n;
		carmen_laser_laser_static_message m;
		carmen_laser_message_queue_wait_get(&queue, &m);
		//n=carmen_laser_message_queue_wait_get(&queue, &m);
		//fprintf(stderr, "%d, %lf, ranges[%d]\n", n, m.timestamp, m.num_readings);
		c++;
		//BEGIN CYRILL
		//here you can senbd the data via IPC
		//END CYRILL
		
		if (!(c%10)){
			double time=m.timestamp;
			fprintf(stderr, "(%d), %lf\n", m.num_readings, 11./(time-oldTime));
			oldTime=time;
		}
	}
	//join teh pending thread
	pthread_join(laser_thread, &tresult);

	//cleanup phase
	fprintf(stderr, "cleanup\n");
	result=(*(device->f_close))(device);
	if (! result){
		fprintf(stderr, "Error\n");
		return -3;
	}
	return 0;
}
