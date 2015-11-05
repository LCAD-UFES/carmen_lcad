
#ifndef XSENS_MTIG_MESSAGES_H
#define XSENS_MTIG_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	carmen_quaternion_t quat;
	carmen_vector_3D_t acc;
	carmen_vector_3D_t gyr;
	carmen_vector_3D_t mag;

	carmen_vector_3D_t velocity;
	
	double latitude;
	double longitude;
	double height;

	int gps_fix;
	int xkf_valid;	

	int sensor_ID;

	double timestamp;
	char *host;
} carmen_xsens_mtig_message;

#define CARMEN_XSENS_MTIG_NAME "carmen_xsens_mtig_message"
#define CARMEN_XSENS_MTIG_FMT "{{double,double,double,double},{double,double,double},{double,double,double},{double,double,double},{double,double,double},double,double,double,int,int,int,double,string}"


typedef struct
{
	carmen_quaternion_t quat;
	carmen_vector_3D_t acc;
	carmen_vector_3D_t gyr;
	carmen_vector_3D_t mag;

	int latitude;
	int longitude;
	int height;
	int vel_north;
	int vel_east;
	int vel_down;
	int horizontal_accuracy;
	int vertical_accuracy;
	int speed_accuracy;	

	int gps_fix;
	int xkf_valid;	

	int sensor_ID;

	double timestamp;
	char *host;
} carmen_xsens_mtig_raw_gps_message;

#define CARMEN_XSENS_MTIG_RAW_GPS_NAME "carmen_xsens_mtig_raw_gps_message"
#define CARMEN_XSENS_MTIG_RAW_GPS_FMT "{{double,double,double,double},{double,double,double},{double,double,double},{double,double,double},int,int,int,int,int,int,int,int,int,int,int,int,double,string}"

#ifdef __cplusplus
}
#endif

#endif
