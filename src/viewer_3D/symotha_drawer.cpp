#include <carmen/carmen.h>

#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/glu.h>
#include <carmen/collision_detection.h>

#include "symotha_drawer.h"


symotha_drawer_t *create_symotha_drawer(int argc, char** argv)
{
	symotha_drawer_t *symotha_drawer = (symotha_drawer_t *) malloc(sizeof(symotha_drawer_t));
	symotha_parameters_t symotha_params;

	carmen_param_t param_list[] =
	{
		{ "behavior_selector", "central_lane_obstacles_safe_distance", CARMEN_PARAM_DOUBLE, &(symotha_params.central_lane_obstacles_safe_distance), 0, NULL },
		{ "behavior_selector", "main_central_lane_obstacles_safe_distance", CARMEN_PARAM_DOUBLE, &(symotha_params.main_central_lane_obstacles_safe_distance), 0, NULL },
		{ "behavior_selector", "lateral_lane_obstacles_safe_distance", CARMEN_PARAM_DOUBLE, &(symotha_params.lateral_lane_obstacles_safe_distance), 0, NULL },
		{ "model_predictive_planner", "obstacles_safe_distance", CARMEN_PARAM_DOUBLE, &(symotha_params.obstacles_safe_distance), 0, NULL },
	};

	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	symotha_drawer->symotha_params = symotha_params;

	return (symotha_drawer);
}


void drawHollowCircle(GLfloat x, GLfloat y, GLfloat z, GLfloat radius, double r, double g, double b)
{
	int i;
	int lineAmount = 100; //# of triangles used to draw circle

	//GLfloat radius = 0.8f; //radius
	GLfloat twicePi = 2.0f * M_PI;

	glPushMatrix();

	glColor3f(r, g, b);

	glBegin(GL_LINE_LOOP);
	for (i = 0; i <= lineAmount; i++)
	{
		glVertex3f(x + (radius * cos(i * twicePi / lineAmount)), y + (radius * sin(i * twicePi / lineAmount)), z);
	}
	glEnd();

	glPopMatrix();
}


void
destroy_symotha_drawer(symotha_drawer_t *symotha_drawer)
{
	free(symotha_drawer);
}


void
draw_symotha(symotha_drawer_t *symotha_drawer, carmen_pose_3D_t car_fused_pose)
{
	drawHollowCircle(car_fused_pose.position.x, car_fused_pose.position.y, car_fused_pose.position.z, symotha_drawer->symotha_params.central_lane_obstacles_safe_distance, 1.0, 0.0, 0.0);
	drawHollowCircle(car_fused_pose.position.x, car_fused_pose.position.y, car_fused_pose.position.z, symotha_drawer->symotha_params.main_central_lane_obstacles_safe_distance, 0.0, 1.0, 0.0);


	double theta = car_fused_pose.orientation.yaw;

	carmen_vector_3D_t pose;
	pose.x = car_fused_pose.position.x - (2.0 * symotha_drawer->symotha_params.lateral_lane_obstacles_safe_distance) * sin(theta);
	pose.y = car_fused_pose.position.y + (2.0 * symotha_drawer->symotha_params.lateral_lane_obstacles_safe_distance) * cos(theta);
	pose.z = car_fused_pose.position.z;

	drawHollowCircle(pose.x, pose.y, pose.z, symotha_drawer->symotha_params.main_central_lane_obstacles_safe_distance, 0.0, 0.0, 1.0);

	theta = carmen_normalize_theta(theta + M_PI);

	pose.x = car_fused_pose.position.x - (2.0 * symotha_drawer->symotha_params.lateral_lane_obstacles_safe_distance) * sin(theta);
	pose.y = car_fused_pose.position.y + (2.0 * symotha_drawer->symotha_params.lateral_lane_obstacles_safe_distance) * cos(theta);
	pose.z = car_fused_pose.position.z;

	drawHollowCircle(pose.x, pose.y, pose.z, symotha_drawer->symotha_params.main_central_lane_obstacles_safe_distance, 0.0, 0.0, 1.0);

	// Draw robot collision profile
	carmen_collision_config_t *robot_collision_profile = carmen_get_global_collision_config();
	theta = car_fused_pose.orientation.yaw;
	for (int i = 0; i < robot_collision_profile->n_markers; i++)
	{
		carmen_ackerman_traj_point_t point = {car_fused_pose.position.x, car_fused_pose.position.y, theta, 0.0, 0.0};
		carmen_point_t displaced_point = carmen_collision_detection_displaced_pose_according_to_car_orientation(&point,
				robot_collision_profile->markers[i].x, robot_collision_profile->markers[i].y);

		drawHollowCircle(displaced_point.x, displaced_point.y, pose.z, robot_collision_profile->markers[i].radius, 1.0, 1.0, 0.0);
	}
}

