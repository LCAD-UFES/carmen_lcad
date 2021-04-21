#include <carmen/carmen.h>

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>

#include "velodyne_intensity_drawer.h"
#include "point_cloud_drawer.h"

#include <carmen/velodyne_interface.h>
#include <carmen/rotation_geometry.h>

struct velodyne_intensity_drawer
{
	point_cloud_drawer* pcloud_drawer;

	double horizontal_angle;
	double horizontal_range;
	int vertical_position;

	carmen_pose_3D_t velodyne_pose;
	carmen_pose_3D_t sensor_board_pose;
};

velodyne_intensity_drawer* create_velodyne_intensity_drawer(carmen_pose_3D_t velodyne_pose, carmen_pose_3D_t sensor_board_pose)
{
	velodyne_intensity_drawer* v_drawer = (velodyne_intensity_drawer*)malloc(sizeof(velodyne_intensity_drawer));

	v_drawer->pcloud_drawer = create_point_cloud_drawer(10000);
	
	v_drawer->velodyne_pose = velodyne_pose;
	v_drawer->sensor_board_pose = sensor_board_pose;

	v_drawer->horizontal_angle = carmen_degrees_to_radians(0.0);
	v_drawer->horizontal_range = carmen_degrees_to_radians(45.0);
	v_drawer->vertical_position = 18;
	

	return v_drawer;
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

static carmen_vector_3D_t
get_velodyne_point_car_reference(double rot_angle, double vert_angle, double range, rotation_matrix* velodyne_to_board_matrix, rotation_matrix* board_to_car_matrix, carmen_pose_3D_t velodyne_pose, carmen_pose_3D_t sensor_board_pose)
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


static carmen_vector_3D_t
create_point_colors_intensity(double intensity)
{
	//printf("% lf\n", intensity);

	carmen_vector_3D_t colors;

	double intensity_normalized = intensity / 255.0;

	colors.x = intensity_normalized;		
	colors.y = intensity_normalized;
	colors.z = intensity_normalized;
	
	return colors;
}

static int
check_angle_in_range(double angle, double center, double range)
{
	static double pi = M_PI;

	double diff = fabs(angle - center);
	diff -= ((int)(diff/(2*pi)))*2*pi;

	if(diff > pi)
	{
		diff = 2*pi - diff;
	}
	
	return (diff < range);
}

static void
normalize_intensity(point_cloud pcloud)
{
	return;

	double min = 100000.0;
	double max = 0.0;

	int i;
	for (i = 0; i < pcloud.num_points; i++)
	{
		double value = pcloud.point_color[i].x;

		if (value < min)
			min = value;

		if (value > max)
			max = value;
	}

	for (i = 0; i < pcloud.num_points; i++)
	{
		double norm_value = (pcloud.point_color[i].x - min) / (max - min);

		pcloud.point_color[i].x = norm_value;
		pcloud.point_color[i].y = norm_value;
		pcloud.point_color[i].z = norm_value;
	}
}

void
velodyne_intensity_drawer_add_velodyne_message(velodyne_intensity_drawer *v_drawer, carmen_velodyne_partial_scan_message *velodyne_message,
		carmen_pose_3D_t car_fused_pose, carmen_vector_3D_t car_fused_velocity, double car_fused_time)
{
	static double vertical_correction[32] = { -30.67, -9.3299999, -29.33, -8.0, -28.0, -6.6700001, -26.67, -5.3299999, -25.33, -4.0,
	-24.0, -2.6700001, -22.67, -1.33, -21.33, 0.0, -20.0, 1.33, -18.67, 2.6700001, -17.33, 4.0, -16.0, 5.3299999, -14.67, 6.6700001,
	-13.33, 8.0, -12.0, 9.3299999, -10.67, 10.67 };
	
	static point_cloud velodyne_points = {NULL, NULL, {0.0, 0.0, 0.0}, 0, 0.0};
	static double last_timestamp = 0.0;

	if (last_timestamp == 0.0)
	{
		last_timestamp = velodyne_message->timestamp;
		return;
	}
				
	int num_points = velodyne_message->number_of_32_laser_shots*(32);

	if(num_points > velodyne_points.num_points)
	{
		velodyne_points.points = (carmen_vector_3D_t*)realloc(velodyne_points.points, num_points*sizeof(carmen_vector_3D_t));
		velodyne_points.point_color = (carmen_vector_3D_t*)realloc(velodyne_points.point_color, num_points*sizeof(carmen_vector_3D_t));
	}
	velodyne_points.num_points = num_points;
	velodyne_points.car_position = car_fused_pose.position;
	velodyne_points.timestamp = velodyne_message->timestamp;


	rotation_matrix* velodyne_to_board_matrix = create_rotation_matrix(v_drawer->velodyne_pose.orientation);
	rotation_matrix* board_to_car_matrix = create_rotation_matrix(v_drawer->sensor_board_pose.orientation);
	rotation_matrix* r_matrix_car_to_global = create_rotation_matrix(car_fused_pose.orientation);

	int k = 0;
	int i;
	for(i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		double shot_angle = -carmen_degrees_to_radians(velodyne_message->partial_scan[i].angle);
	
		if(check_angle_in_range(shot_angle, v_drawer->horizontal_angle, v_drawer->horizontal_range))		
		{		
			int j;		
			for(j = 0; j < 32; j++)
			{
				if( j == v_drawer->vertical_position )
				{
					carmen_vector_3D_t point_position = get_velodyne_point_car_reference(	shot_angle,
														carmen_degrees_to_radians(vertical_correction[j]),
														velodyne_message->partial_scan[i].distance[j]/500.0,
														velodyne_to_board_matrix, board_to_car_matrix,
														v_drawer->velodyne_pose, v_drawer->sensor_board_pose);

					double shot_time = last_timestamp + ((velodyne_message->timestamp - last_timestamp)*((double)i)/((double)velodyne_message->number_of_32_laser_shots));
					carmen_vector_3D_t car_interpolated_position = carmen_get_interpolated_robot_position_at_time(car_fused_pose, car_fused_velocity, car_fused_time, shot_time, r_matrix_car_to_global);
					carmen_vector_3D_t point_global_position = get_point_position_global_reference(car_interpolated_position, point_position, r_matrix_car_to_global);

					velodyne_points.points[k] = point_global_position;
					velodyne_points.point_color[k] = create_point_colors_intensity(velodyne_message->partial_scan[i].intensity[j]);
			
					k++;
				}				
			}
		}
	}

	velodyne_points.num_points = k;

	normalize_intensity(velodyne_points);

	destroy_rotation_matrix(velodyne_to_board_matrix);
	destroy_rotation_matrix(board_to_car_matrix);
	destroy_rotation_matrix(r_matrix_car_to_global);

	
	add_point_cloud(v_drawer->pcloud_drawer, velodyne_points);
	
	last_timestamp = velodyne_message->timestamp;
}

void
draw_velodyne_intensity(velodyne_intensity_drawer* v_drawer)
{
	glDisable(GL_LIGHTING);
	draw_point_cloud(v_drawer->pcloud_drawer);
	glEnable(GL_LIGHTING);
}


void
destroy_velodyne_intensity_drawer(velodyne_intensity_drawer* v_drawer)
{
	destroy_point_cloud_drawer(v_drawer->pcloud_drawer);

	free(v_drawer);
}
