#ifndef DRAW_CAR_H_
#define DRAW_CAR_H_

#include <carmen/carmen.h>
#include <carmen/glm.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>

#ifdef __cplusplus
extern "C"
{
#endif

struct CarDrawer
{
	carmen_vector_3D_t car_size;
	carmen_vector_3D_t robot_size;
	carmen_pose_3D_t car_pose;
	double car_axis_distance;
	double car_wheel_radius;

	carmen_pose_3D_t sensor_board_1_pose;

	carmen_vector_3D_t laser_size;
	carmen_pose_3D_t laser_pose;

	carmen_vector_3D_t sensor_box_size;
	carmen_pose_3D_t sensor_box_pose;

	carmen_vector_3D_t xsens_size;
	carmen_pose_3D_t xsens_pose;

	GLMmodel* carModel;
};

typedef struct CarDrawer CarDrawer;

CarDrawer* createCarDrawer(int argc, char** argv);
void destroyCarDrawer(CarDrawer* carDrawer);

void draw_car(CarDrawer* carDrawer);
void draw_car_outline(CarDrawer* carDrawer);
void draw_car_at_pose(CarDrawer* carDrawer, carmen_pose_3D_t position);
void draw_car_outline_at_pose(CarDrawer* carDrawer, carmen_pose_3D_t position);

#ifdef __cplusplus
}
#endif

#endif
