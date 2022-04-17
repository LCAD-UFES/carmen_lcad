#ifndef VIEWER_3D_H_
#define VIEWER_3D_H_

#include <carmen/carmen.h>

#define WINDOW_WIDTH	1000.0
#define WINDOW_HEIGHT	600.0

//#define USE_REAR_BULLBAR


struct _point_cloud
{
	carmen_vector_3D_t 	*points;
	carmen_vector_3D_t 	*point_color;
	carmen_vector_3D_t 	car_position;
	int 			num_points;
	double 			timestamp;
};

typedef struct _point_cloud point_cloud;

void set_flag_viewer_3D(int flag_num, int value);

#endif
