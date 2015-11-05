#ifndef CARMEN_SLAM_MONTECARLO_H
#define CARMEN_SLAM_MONTECARLO_H

#include <carmen/global.h>

#ifdef __cplusplus
extern "C" {
#endif

struct _point_cloud
{
	carmen_vector_3D_t 	*points;
	carmen_vector_3D_t 	*point_color;
	carmen_vector_3D_t 	car_position;
	int 			num_points;
	double 			timestamp;
};

typedef struct _point_cloud point_cloud;



#ifdef __cplusplus
}
#endif

#endif
