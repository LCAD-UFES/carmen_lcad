#ifndef VIEWER_3D_H_
#define VIEWER_3D_H_

#include <carmen/carmen.h>

#define WINDOW_WIDTH	1000.0
#define WINDOW_HEIGHT	600.0

struct _point_cloud
{
	carmen_vector_3D_t 	*points;
	carmen_vector_3D_t 	*point_color;
	carmen_vector_3D_t 	car_position;
	int 			num_points;
	double 			timestamp;
};

typedef struct _point_cloud point_cloud;


struct _moving_objects_tracking
{
	carmen_pose_3D_t 	moving_objects_pose;
	double		 		linear_velocity;
	double				length;
	double				width;
	double				height;
	int 				geometric_model; // geometric model of the object - pedestrian: 0, motorcycle / bike/ car: 1, truck: 2.
	int					num_associated;
	double				timestamp;
};

typedef struct _moving_objects_tracking moving_objects_tracking_t;

void set_flag_viewer_3D(int flag_num, int value);

#endif
