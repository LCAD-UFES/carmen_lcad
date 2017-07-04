#include <carmen/carmen.h>

#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>

#include "velodyne_360_drawer.h"

#include <carmen/velodyne_interface.h>
#include <carmen/rotation_geometry.h>

struct velodyne_360_drawer
{
	carmen_vector_3D_t* points_current;
	double* angles_current;
	int num_points_current;

	carmen_vector_3D_t* points_last;
	double* angles_last;
	int num_points_last;

	carmen_pose_3D_t velodyne_pose;
	carmen_pose_3D_t sensor_board_pose;
};

int velodyne_360_drawer_get_num_points_current (velodyne_360_drawer* v_drawer)
{
	return v_drawer->num_points_current;
}

double velodyne_360_drawer_get_angle_current (velodyne_360_drawer* v_drawer, int index)
{
	return v_drawer->angles_current[index];
}

velodyne_360_drawer* create_velodyne_360_drawer(carmen_pose_3D_t velodyne_pose, carmen_pose_3D_t sensor_board_pose)
{
	velodyne_360_drawer* v_drawer = (velodyne_360_drawer*)malloc(sizeof(velodyne_360_drawer));

	v_drawer->points_current = NULL;
	v_drawer->angles_current = NULL;
	v_drawer->points_last = NULL;
	v_drawer->angles_last = NULL;
	v_drawer->num_points_current = 0;
	v_drawer->num_points_last = 0;

	v_drawer->velodyne_pose = velodyne_pose;
	v_drawer->sensor_board_pose = sensor_board_pose;

	return v_drawer;
}

static void switch_current_last(velodyne_360_drawer* v_drawer)
{
	carmen_vector_3D_t* temp_vec = v_drawer->points_last;
	double* temp_ang = v_drawer->angles_last;
	int num_points_temp = v_drawer->num_points_last;

	v_drawer->points_last = v_drawer->points_current;
	v_drawer->angles_last = v_drawer->angles_current;
	v_drawer->num_points_last = v_drawer->num_points_current;

	v_drawer->points_current = temp_vec;
	v_drawer->angles_current = temp_ang;
	v_drawer->num_points_current = num_points_temp;
}

static carmen_vector_3D_t
get_velodyne_point_car_reference(double rot_angle, double vert_angle, double range, carmen_pose_3D_t velodyne_pose, carmen_pose_3D_t sensor_board_pose, rotation_matrix* velodyne_to_board_matrix, rotation_matrix* board_to_car_matrix)
{
	double cos_rot_angle = cos(rot_angle);
	double sin_rot_angle = sin(rot_angle);

	double cos_vert_angle = cos(vert_angle);
	double sin_vert_angle = sin(vert_angle);

	double xy_distance = range * cos_vert_angle;

	carmen_vector_3D_t velodyne_reference;

	velodyne_reference.x = (xy_distance * cos_rot_angle);
	velodyne_reference.y = (xy_distance * sin_rot_angle);
	velodyne_reference.z = (range * sin_vert_angle);

	carmen_vector_3D_t board_reference = multiply_matrix_vector(velodyne_to_board_matrix, velodyne_reference);
	board_reference = add_vectors(board_reference, velodyne_pose.position);

	carmen_vector_3D_t car_reference = multiply_matrix_vector(board_to_car_matrix, board_reference);
	car_reference = add_vectors(car_reference, sensor_board_pose.position);
	
	return car_reference;
}


void add_velodyne_message(velodyne_360_drawer* v_drawer, carmen_velodyne_partial_scan_message* velodyne_message)
{
	static double vertical_correction[32] = { -30.67, -9.3299999, -29.33, -8.0, -28.0, -6.6700001, -26.67, -5.3299999, -25.33, -4.0,
	-24.0, -2.6700001, -22.67, -1.33, -21.33, 0.0, -20.0, 1.33, -18.67, 2.6700001, -17.33, 4.0, -16.0, 5.3299999, -14.67, 6.6700001,
	-13.33, 8.0, -12.0, 9.3299999, -10.67, 10.67 };

	switch_current_last(v_drawer);

	int num_points = velodyne_message->number_of_32_laser_shots*32;

	v_drawer->num_points_current = num_points;
	v_drawer->points_current = (carmen_vector_3D_t*)realloc(v_drawer->points_current, num_points*sizeof(carmen_vector_3D_t));
	v_drawer->angles_current = (double*)realloc(v_drawer->angles_current, num_points*sizeof(double));

	rotation_matrix* velodyne_to_board_matrix = create_rotation_matrix(v_drawer->velodyne_pose.orientation);
	rotation_matrix* board_to_car_matrix = create_rotation_matrix(v_drawer->sensor_board_pose.orientation);

	int i;
	for(i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		double rot_angle = carmen_degrees_to_radians(velodyne_message->partial_scan[i].angle);

		int j;
		for(j = 0; j < 32; j++)
		{

			double vert_angle = carmen_degrees_to_radians(vertical_correction[j]);

			carmen_vector_3D_t point_position = get_velodyne_point_car_reference( 	-rot_angle, vert_angle,
												velodyne_message->partial_scan[i].distance[j]/500.0,
												v_drawer->velodyne_pose, v_drawer->sensor_board_pose,
												velodyne_to_board_matrix, board_to_car_matrix);

			v_drawer->points_current[i*32 + j] = point_position;
			v_drawer->angles_current[i*32 + j] = rot_angle;
		}
	}

	destroy_rotation_matrix(velodyne_to_board_matrix);
	destroy_rotation_matrix(board_to_car_matrix);
}

static int get_index_angle_last(velodyne_360_drawer* v_drawer, double angle)
{
	int i;
	for(i = 0; i < v_drawer->num_points_last && v_drawer->angles_last[i] > angle; i++)
	{
	}

	for(; i < v_drawer->num_points_last && v_drawer->angles_last[i] < angle; i++)
	{
	}

	return i - 1;
}

static void
set_laser_point_color(double x, double y, double z)
{
	x = x;
	y = y;

	if(z < -5.0)
	{
		glColor3f(0.0, 0.0, 0.0);
	}
	else
	{
		glColor3f(0.0-z, 0.1 + z/10.0, (z+3.0)/6.0);
	}
}

void draw_velodyne_360(velodyne_360_drawer* v_drawer, carmen_pose_3D_t car_pose)
{
	glPushMatrix();
		glTranslatef(car_pose.position.x, car_pose.position.y, car_pose.position.z);
		glRotatef(carmen_radians_to_degrees(car_pose.orientation.yaw), 0.0f, 0.0f, 1.0f);
		glRotatef(carmen_radians_to_degrees(car_pose.orientation.pitch), 0.0f, 1.0f, 0.0f);
		glRotatef(carmen_radians_to_degrees(car_pose.orientation.roll), 1.0f, 0.0f, 0.0f);

		glBegin(GL_POINTS);

			//glColor3f(0.0,0.0,1.0);

			int num_points_current = v_drawer->num_points_current;

			int i;
			for(i=0; i<num_points_current; i++)
			{
				set_laser_point_color(v_drawer->points_current[i].x, v_drawer->points_current[i].y, v_drawer->points_current[i].z);
				glVertex3f(v_drawer->points_current[i].x, v_drawer->points_current[i].y, v_drawer->points_current[i].z);
			}

			int num_points_last = v_drawer->num_points_last;

			if(num_points_current > 0)
			{
				double last_current_angle = v_drawer->angles_current[num_points_current - 1];
				int last_start_index = get_index_angle_last(v_drawer, last_current_angle);

				for(i=last_start_index; i>=0 && i<num_points_last; i++)
				{
					set_laser_point_color(v_drawer->points_last[i].x, v_drawer->points_last[i].y, v_drawer->points_last[i].z);
					glVertex3f(v_drawer->points_last[i].x, v_drawer->points_last[i].y, v_drawer->points_last[i].z);
				}
			}

		glEnd();

	glPopMatrix();
}


void destroy_velodyne_360_drawer(velodyne_360_drawer* v_drawer)
{
	free(v_drawer->points_current);
	free(v_drawer->angles_current);
	free(v_drawer->points_last);
	free(v_drawer->angles_last);

	free(v_drawer);
}
