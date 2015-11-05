#include <carmen/carmen.h>

#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>
#include <string.h>



#include <carmen/velodyne_interface.h>
#include <carmen/rotation_geometry.h>
#include <carmen/stereo_velodyne.h>

struct variable_velodyne_drawer
{
	carmen_vector_3D_t** points;
	double** angles;
	int* num_points;
	int max_scans_number;
	int next_elem;

	carmen_pose_3D_t velodyne_pose;
	carmen_pose_3D_t car_pose;


	double *vertical_correction;
};

#include "variable_velodyne_drawer.h"

variable_velodyne_drawer*
create_variable_velodyne_drawer(int flipped,carmen_pose_3D_t velodyne_pose, int max_scans_number, int vertical_resolution, int camera,
		int stereo_velodyne_vertical_roi_ini, int stereo_velodyne_vertical_roi_end, int stereo_velodyne_horizontal_roi_ini, int stereo_velodyne_horizontal_roi_end, int bumblebee_basic_width, int bumblebee_basic_height)
{
	variable_velodyne_drawer* v_drawer = (variable_velodyne_drawer*)malloc(sizeof(variable_velodyne_drawer));
	carmen_test_alloc(v_drawer);

	int roi_ini, roi_end;
	v_drawer->max_scans_number = max_scans_number;

	v_drawer->points = (carmen_vector_3D_t **)malloc(max_scans_number * sizeof(carmen_vector_3D_t *));
	v_drawer->angles = (double **)malloc(max_scans_number * sizeof(double *));
	v_drawer->num_points = (int*)malloc(max_scans_number * sizeof(int));

	memset(v_drawer->num_points, 0, max_scans_number * sizeof(int));
	memset(v_drawer->angles, 0, max_scans_number * sizeof(double *));
	memset(v_drawer->points, 0, max_scans_number * sizeof(carmen_vector_3D_t *));

	v_drawer->next_elem = 0;

	if (flipped)
	{
		roi_ini = stereo_velodyne_horizontal_roi_ini;
		roi_end = stereo_velodyne_horizontal_roi_end;
	}
	else
	{
		roi_ini = stereo_velodyne_vertical_roi_ini;
		roi_end = stereo_velodyne_vertical_roi_end;
	}

	v_drawer->vertical_correction = get_stereo_velodyne_correction(flipped, camera, vertical_resolution, roi_ini, roi_end, bumblebee_basic_width, bumblebee_basic_height);
	v_drawer->velodyne_pose = velodyne_pose;

	return v_drawer;
}


//static carmen_vector_3D_t
//get_velodyne_point_car_reference(double rot_angle, double vert_angle, double range, carmen_pose_3D_t velodyne_pose, rotation_matrix* velodyne_to_car_matrix)
//{
//	double cos_rot_angle = cos(rot_angle);
//	double sin_rot_angle = sin(rot_angle);
//
//	double cos_vert_angle = cos(vert_angle);
//	double sin_vert_angle = sin(vert_angle);
//
//	double xy_distance = range * cos_vert_angle;
//
//	carmen_vector_3D_t velodyne_reference;
//
//	velodyne_reference.x = (xy_distance * cos_rot_angle);
//	velodyne_reference.y = (xy_distance * sin_rot_angle);
//	velodyne_reference.z = (range * sin_vert_angle);
//
//	//rotation_matrix* velodyne_to_car_matrix = create_rotation_matrix(velodyne_pose.orientation);
//	carmen_vector_3D_t car_reference = multiply_matrix_vector(velodyne_to_car_matrix, velodyne_reference);
//	carmen_vector_3D_t reading = add_vectors(car_reference, velodyne_pose.position);
//	//destroy_rotation_matrix(velodyne_to_car_matrix);
//
//	return reading;
//}

static carmen_vector_3D_t
get_velodyne_point_car_reference(double rot_angle, double vert_angle, double range, carmen_pose_3D_t velodyne_pose, carmen_pose_3D_t sensor_board_1_pose,rotation_matrix* velodyne_to_board_matrix, rotation_matrix* board_to_car_matrix)
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
	car_reference = add_vectors(car_reference, sensor_board_1_pose.position);

	return car_reference;
}

static carmen_vector_3D_t
get_point_position_global_reference(carmen_vector_3D_t car_position, carmen_vector_3D_t car_reference, rotation_matrix* car_to_global_matrix)
{

	//rotation_matrix* r_matrix = create_rotation_matrix(car_fused_pose.orientation);

	carmen_vector_3D_t global_reference = multiply_matrix_vector(car_to_global_matrix, car_reference);

	carmen_vector_3D_t point = add_vectors(global_reference, car_position);

	//destroy_rotation_matrix(r_matrix);

	return point;
}


void
add_variable_velodyne_message(variable_velodyne_drawer *v_drawer, carmen_velodyne_variable_scan_message *velodyne_message, carmen_pose_3D_t car_pose, carmen_pose_3D_t sensor_board_1_pose)
{
	int num_points = velodyne_message->number_of_shots * velodyne_message->partial_scan[0].shot_size;
	int index = v_drawer->next_elem;

	if (v_drawer->num_points[index] != num_points)
	{
		v_drawer->num_points[index] = num_points;
		v_drawer->points[index] = (carmen_vector_3D_t*)realloc(v_drawer->points[index], num_points * sizeof(carmen_vector_3D_t));
		v_drawer->angles[index] = (double*)realloc(v_drawer->angles[index], num_points * sizeof(double));
	}

	v_drawer->car_pose = car_pose;
	rotation_matrix* velodyne_to_car_matrix = create_rotation_matrix(v_drawer->velodyne_pose.orientation);
	rotation_matrix* r_matrix_car_to_global = create_rotation_matrix(car_pose.orientation);
	rotation_matrix* board_to_car_matrix = create_rotation_matrix(sensor_board_1_pose.orientation);

	int i;
	for(i = 0; i < velodyne_message->number_of_shots; i++)
	{
		double rot_angle = carmen_degrees_to_radians(velodyne_message->partial_scan[i].angle);

		int j;
		for(j = 0; j <  velodyne_message->partial_scan[i].shot_size; j++)
		{

			double vert_angle = carmen_degrees_to_radians(v_drawer->vertical_correction[j]);

			carmen_vector_3D_t point_position = get_velodyne_point_car_reference( 	-rot_angle, vert_angle,
																					velodyne_message->partial_scan[i].distance[j]/500.0,
																					v_drawer->velodyne_pose, sensor_board_1_pose,
																					velodyne_to_car_matrix, board_to_car_matrix);

			carmen_vector_3D_t point_global_position = get_point_position_global_reference(car_pose.position, point_position, r_matrix_car_to_global);


			v_drawer->points[index][i * velodyne_message->partial_scan[i].shot_size + j] = point_global_position;
			v_drawer->angles[index][i * velodyne_message->partial_scan[i].shot_size + j] = rot_angle;
		}
	}

	destroy_rotation_matrix(velodyne_to_car_matrix);
	destroy_rotation_matrix(r_matrix_car_to_global);

	v_drawer->next_elem = (v_drawer->next_elem+1)%v_drawer->max_scans_number;
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

void
draw_variable_velodyne(variable_velodyne_drawer* v_drawer)
{
	glPushMatrix();
		//glTranslatef(car_pose.position.x, car_pose.position.y, car_pose.position.z);
		//glRotatef(carmen_radians_to_degrees(car_pose.orientation.yaw), 0.0f, 0.0f, 1.0f);
		//glRotatef(carmen_radians_to_degrees(car_pose.orientation.pitch), 0.0f, 1.0f, 0.0f);
		//glRotatef(carmen_radians_to_degrees(car_pose.orientation.roll), 1.0f, 0.0f, 0.0f);

		glBegin(GL_POINTS);

			//glColor3f(0.0,0.0,1.0);

			int k;
			for(k=0; k<v_drawer->max_scans_number; k++)
			{
				int num_points_current = v_drawer->num_points[k];

				int i;
				for(i=0; i<num_points_current; i++)
				{

					set_laser_point_color(v_drawer->points[k][i].x, v_drawer->points[k][i].y, v_drawer->points[k][i].z - v_drawer->car_pose.position.z);
					glVertex3f(v_drawer->points[k][i].x, v_drawer->points[k][i].y, v_drawer->points[k][i].z);
				}
			}

		glEnd();

	glPopMatrix();
}


void
destroy_variable_velodyne_drawer(variable_velodyne_drawer* v_drawer)
{
	int k;
	for(k=0; k<v_drawer->max_scans_number; k++)
	{
		free(v_drawer->points[k]);
		free(v_drawer->angles[k]);
	}

	free(v_drawer->points);
	free(v_drawer->angles);
	free(v_drawer->num_points);


	free(v_drawer);
}
