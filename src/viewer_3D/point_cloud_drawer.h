#ifndef POINT_CLOUD_DRAWER_H_
#define POINT_CLOUD_DRAWER_H_

#include "viewer_3D.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct point_cloud_drawer point_cloud_drawer;

point_cloud_drawer* create_point_cloud_drawer(int max_size);
void add_point_cloud(point_cloud_drawer* drawer, point_cloud pcloud);
void draw_point_cloud(point_cloud_drawer* drawer);
void destroy_point_cloud_drawer(point_cloud_drawer* drawer);

#ifdef __cplusplus
}
#endif

#endif
