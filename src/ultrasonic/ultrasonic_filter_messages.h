/*********************************************************
	 ---   My Module Specific Messages ---

See IPC documentation for more information:
http://www.cs.cmu.edu/~ipc/

*********************************************************/

#ifndef CARMEN_ULTRASONIC_FILTER_MESSAGES_H
#define CARMEN_ULTRASONIC_FILTER_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Message Struct Example */
typedef struct {
	int number_of_sonars;
	int sonar_beans;
	double fov; //36 degrees
	double angle_step; //0.1 degrees
	double start_angle; //-18 degrees
	double max_range;
	double sensor[4];
	double timestamp; 		/* !!! obrigatory !!! */
	char *host; 			/* !!! obrigatory !!! */
} carmen_ultrasonic_sonar_sensor_message;

/* The message's name, will be used for message registration in IPC Central module */
#define      CARMEN_ULTRASONIC_SONAR_SENSOR_NAME       "carmen_ultrasonic_sonar_sensor"

/* The message's format, will be used for message data marshalling (serialization) for network socket transport. */
#define      CARMEN_ULTRASONIC_SONAR_SENSOR_FMT        "{int,int,double,double,double,double,[double:4],double,string}"

typedef struct {
	double sensor[4];
	double timestamp;
} ipc_ultrasonic_sensor_message_t;

#define      IPC_ULTRASONIC_SENSOR_MESSAGE_NAME       "ipc_ultrasonic_sensor_message"
#define      IPC_ULTRASONIC_SENSOR_MESSAGE_FMT        "{[double:4],double}"

#ifdef __cplusplus
}
#endif

#endif

// @}
