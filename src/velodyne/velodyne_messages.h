#ifndef CARMEN_VELODYNE_MESSAGES_H
#define CARMEN_VELODYNE_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

#define USE_REAR_BULLBAR

typedef struct
{
	char *model;
	int id;
	char *ip;
	char *port;
	int shot_size;                  // Number of vertical rays
	int min_sensing;                // Minimum sensing value (int), provided by the manual
	int max_sensing;                // Maximum sensing value (int), provided by the manual
	int range_division_factor;      // LiDAR measures are sent as integer values to minimize data transfer, must be divided to get floating point precision
	double max_range;                  // Maximum sensing distance in metters
	double time_between_shots;      // Given by the spinning frequency
	carmen_pose_3D_t pose;          // x, y, z, roll, pitch, yaw
	int *ray_order;                 // Order of the laser beams from the bottom up
	double *vertical_angles;        // Angles beteween laser beams in the same shot
	int sensor_reference;			// If it is in reference in relation to the sensorboard (0), to the front_bullbar (1), or the rear_bullbar (2)
}carmen_lidar_config;

typedef struct
{
	unsigned short  distance[32];	// m * 500
	unsigned char intensity[32];	// 0 to 255
	double angle;					// degrees
}carmen_velodyne_32_laser_shot;

typedef struct {
  int number_of_32_laser_shots;
  carmen_velodyne_32_laser_shot *partial_scan;
  double timestamp;
  char *host;
} carmen_velodyne_partial_scan_message;

#define      CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME       "carmen_velodyne_partial_scan_message"
#define      CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_FMT        "{int, <{[short:32], [byte:32], double}:1>, double, string}"


typedef struct {
		float gyro1, gyro2, gyro3;
		float temp1, temp2, temp3;
		float accel1_x, accel2_x, accel3_x;
		float accel1_y, accel2_y, accel3_y;

		int utc_time;													// hhmmss format
		char status;													// A = valid position, V = NAV receiver warning
		float latitude; 											// ddmm.mmm format
		char latitude_hemisphere;							// N or S
		float longitude;											// ddmm.mm format
		char longitude_hemisphere;						// E or W
		float speed_over_ground;							// 000.0 to 999.9 knots
		float course_over_ground;						// 000.0 to 39.9 degrees
		int utc_date;													// ddmmyy format
		float magnetic_variation_course;		// 000.0 to 180.0 degrees
		float magnetic_variation_direction; // E or W
		char mode_indication;									// A = Autonomous, D = Differential, E = Estimated, N =m Data not valid

		double timestamp;
		char* host;
} carmen_velodyne_gps_message ;

#define      CARMEN_VELODYNE_GPS_MESSAGE_NAME       "carmen_velodyne_gps_message"
#define      CARMEN_VELODYNE_GPS_MESSAGE_FMT        "{float, float, float, float, float, float, float, float, float, float, float, float, int, byte, float, byte, float, byte, float, float, int, float, float, byte, double, string}"

typedef struct
{
	int shot_size;
	unsigned int  *distance;
	unsigned short *intensity;
	double angle;              // In degrees
}carmen_velodyne_shot;

typedef struct
{
	int number_of_shots;
	carmen_velodyne_shot *partial_scan;
	double timestamp;
	char *host;
}carmen_velodyne_variable_scan_message;

#define      CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE_NAME       "carmen_velodyne_variable_scan_message"
#define      CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE_FMT        "{int, <{int, <int:1>, <short:1>, double}:1>, double, string}"

#define      CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE0_NAME       "carmen_velodyne_variable_scan_message0"
#define      CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE1_NAME       "carmen_velodyne_variable_scan_message1"
#define      CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE2_NAME       "carmen_velodyne_variable_scan_message2"
#define      CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE3_NAME       "carmen_velodyne_variable_scan_message3"
#define      CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE4_NAME       "carmen_velodyne_variable_scan_message4"
#define      CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE5_NAME       "carmen_velodyne_variable_scan_message5"
#define      CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE6_NAME       "carmen_velodyne_variable_scan_message6"
#define      CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE7_NAME       "carmen_velodyne_variable_scan_message7"
#define      CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE8_NAME       "carmen_velodyne_variable_scan_message8"
#define      CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE9_NAME       "carmen_velodyne_variable_scan_message9"
#define      CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE10_NAME       "carmen_velodyne_variable_scan_message10"
#define      CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE11_NAME       "carmen_velodyne_variable_scan_message11"
#define      CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE12_NAME       "carmen_velodyne_variable_scan_message12"
#define      CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE13_NAME       "carmen_velodyne_variable_scan_message13"
#define      CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE14_NAME       "carmen_velodyne_variable_scan_message14"
#define      CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE15_NAME       "carmen_velodyne_variable_scan_message15"


#ifdef __cplusplus
}
#endif

#endif
