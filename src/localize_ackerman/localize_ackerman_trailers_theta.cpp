#include <car_model.h>
#include "localize_ackerman_trailers_theta.h"
#include <gsl/gsl_fit.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_errno.h>
#include <sys/stat.h>


#define BETA_ERROR 						100.0
#define PLOT_BETA						1
#define BUFFER_SIZE						5


FILE *gnuplot_pipe = NULL;
int SEMI_TRAILER1 = 0;
double remove_by_mean_dist_between_rays_factor = 4.0; // remove if dist between rays > X*last_mean

extern double triangulation_max_beta;
extern int triangulation_min_cluster_size;


void copy_file(char *src, const char *dst) {
    FILE *fp1, *fp2;
    int a;

    fp1 = fopen(src, "r");
    fp2 = fopen(dst, "w");
    while( (a = fgetc(fp1)) != EOF )
    	fputc(a, fp2);

    fclose(fp1);
    fclose(fp2);
}


void
plot_groups(carmen_vector_3D_t *points_position_with_respect_to_car, double *estimated, 
	int num_filtered_points, int *group, int n_groups, int choosen_group)
{
	if (n_groups <= 0)
	{
		fflush(gnuplot_pipe);
		return;
	}

	FILE *graph_files[n_groups+1];
	char outfile[256];
	static bool first_time = true;
	struct stat st;

	if (first_time)
	{
		first_time = false;
		gnuplot_pipe = popen("taskset -c 0 gnuplot", "w");
		fprintf(gnuplot_pipe, "set size square\n");
		fprintf(gnuplot_pipe, "set xrange [-3:3]\n");
		fprintf(gnuplot_pipe, "set yrange [-3:3]\n");
	}
	
	for (int i = -1; i < n_groups; i++)
	{
		sprintf(outfile, "debug-beta-%d.txt", i);
		remove(outfile);
		graph_files[i+1] = fopen(outfile, "w");
	}
	for (int i = 0; i < num_filtered_points; i++)
		fprintf(graph_files[group[i]+1], "%lf\t%lf\t%lf\n", points_position_with_respect_to_car[i].x, points_position_with_respect_to_car[i].y, estimated[i]);

	sprintf(outfile, "debug-beta-%d.txt", -1);
	fclose(graph_files[0]);
	fprintf(gnuplot_pipe, "plot '%s' u 1:2 t 'not used'", outfile);
	for (int i = 0; i < n_groups; i++)
	{
		sprintf(outfile, "debug-beta-%d.txt", i);
		fclose(graph_files[i+1]);
		stat(outfile, &st);
		if (st.st_size > 0)
		{
			if (i == choosen_group)
				fprintf(gnuplot_pipe, ", '%s' u 1:2 t '%d', '%s' u 1:3 t 'gsl' w l", outfile, i, outfile);
			else
				fprintf(gnuplot_pipe, ", '%s' u 1:2 t '%d'", outfile, i);
		}
	}

	fprintf(gnuplot_pipe, "\n");

	fflush(gnuplot_pipe);

	sprintf(outfile, "debug-beta-%d.txt", choosen_group);
	copy_file(outfile, "debug-choosen.txt");
}


static carmen_vector_3D_t
get_velodyne_point_car_reference(double rot_angle, double vert_angle, double range,
		rotation_matrix *velodyne_to_board_matrix, rotation_matrix *board_to_car_matrix,
		carmen_vector_3D_t velodyne_pose_position, carmen_vector_3D_t sensor_board_1_pose_position)
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
    board_reference = add_vectors(board_reference, velodyne_pose_position);

    carmen_vector_3D_t car_reference = multiply_matrix_vector(board_to_car_matrix, board_reference);
    car_reference = add_vectors(car_reference, sensor_board_1_pose_position);

    return (car_reference);
}


int
compute_points_with_respect_to_king_pin(carmen_vector_3D_t *points_position_with_respect_to_car,
        int ray_of_interest, carmen_semi_trailers_config_t semi_trailer_config, 
		carmen_velodyne_partial_scan_message *velodyne_message, double *vertical_correction,
		rotation_matrix *velodyne_to_board_matrix, rotation_matrix *board_to_car_matrix,
		carmen_vector_3D_t velodyne_pose_position, carmen_vector_3D_t sensor_board_1_pose_position)
{
	int num_valid_points = 0;
	for (int i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		if (velodyne_message->partial_scan[i].distance[ray_of_interest] != 0)
		{
			points_position_with_respect_to_car[num_valid_points] = get_velodyne_point_car_reference(
					-carmen_degrees_to_radians(velodyne_message->partial_scan[i].angle + ((semi_trailer_config.semi_trailers[SEMI_TRAILER1].beta_correct_max_distance < 0.0)? 180.0: 0.0)),
					carmen_degrees_to_radians(vertical_correction[ray_of_interest]), (double) velodyne_message->partial_scan[i].distance[ray_of_interest] / 500.0,
					velodyne_to_board_matrix, board_to_car_matrix, velodyne_pose_position, sensor_board_1_pose_position);

			points_position_with_respect_to_car[num_valid_points].x += semi_trailer_config.semi_trailers[SEMI_TRAILER1].M;
			num_valid_points++;
		}
	}

	return (num_valid_points);
}


int
compute_points_with_respect_to_king_pin(carmen_vector_3D_t *points_position_with_respect_to_car,
        int ray_of_interest, carmen_semi_trailers_config_t semi_trailer_config, 
		carmen_velodyne_variable_scan_message *message, double *vertical_correction,
		rotation_matrix *velodyne_to_board_matrix, rotation_matrix *board_to_car_matrix,
		carmen_vector_3D_t velodyne_pose_position, carmen_vector_3D_t sensor_board_1_pose_position)
{
	int num_valid_points = 0;
	for (int i = 0; i < message->number_of_shots; i++)
	{
		if (message->partial_scan[i].distance[ray_of_interest] != 0)
		{
			points_position_with_respect_to_car[num_valid_points] = get_velodyne_point_car_reference(
					-carmen_degrees_to_radians(message->partial_scan[i].angle + ((semi_trailer_config.semi_trailers[SEMI_TRAILER1].beta_correct_max_distance < 0.0)? 180.0: 0.0)),
					carmen_degrees_to_radians(vertical_correction[ray_of_interest]), (double) message->partial_scan[i].distance[ray_of_interest] / 500.0,
					velodyne_to_board_matrix, board_to_car_matrix, velodyne_pose_position, sensor_board_1_pose_position);

			points_position_with_respect_to_car[num_valid_points].x += semi_trailer_config.semi_trailers[SEMI_TRAILER1].M;
			num_valid_points++;
		}
	}
	return (num_valid_points);
}


int
compare_x(const void *a, const void *b)
{
	carmen_vector_3D_t *arg1 = (carmen_vector_3D_t *) a;
	carmen_vector_3D_t *arg2 = (carmen_vector_3D_t *) b;

	double delta_x = arg1->x - arg2->x;
	if (delta_x < 0.0)
		return (-1);
	if (delta_x > 0.0)
		return (1);

	return (0);
}


int
remove_begin_and_end_points(int num_filtered_points, carmen_vector_3D_t *points_position_with_respect_to_car)
{
	double x_size = points_position_with_respect_to_car[num_filtered_points-1].x - points_position_with_respect_to_car[0].x; //get_points_x_size(num_filtered_points, points_position_with_respect_to_car);
	double fraction_to_remove = x_size * 0.03;
	int index_begin = 0;
	int index_end = 0;
	int new_number_filter_points = 0;
	for(int i = 0; i < num_filtered_points; i++)
	{
		if(fabs(points_position_with_respect_to_car[i].x - points_position_with_respect_to_car[0].x) >= fraction_to_remove)
		{
			index_begin = i;
			break;
		}
	}
	if (num_filtered_points - index_begin > 9)
	{

		for(int i = (num_filtered_points - 1); i > 0; i--)
		{
			if(fabs(points_position_with_respect_to_car[num_filtered_points-1].x - points_position_with_respect_to_car[i].x) >= fraction_to_remove)
			{
				index_end = i;
				break;
			}
		}
	}
	else
	{
		index_end = num_filtered_points-1;
	}

	int j = index_begin;
	for(int i = 0; j <= index_end; i++, j++)
	{
		points_position_with_respect_to_car[i] = points_position_with_respect_to_car[j];
		new_number_filter_points++;
	}

	return (new_number_filter_points);
}


double
compute_new_beta(carmen_vector_3D_t *points, double *estimated, int size)
{
	// https://www.gnu.org/software/gsl/doc/html/lls.html
	const double *x = (double *) points;
	const double *y = (double *) points;
	y = &(y[1]);

	double c0, c1, cov00, cov01, cov11, sumsq;
	gsl_fit_linear(x, 3, y, 3, size, &c0, &c1, &cov00, &cov01, &cov11, &sumsq);

	for (int i = 0; i < size; i++)
		estimated[i] = points[i].x*c1 + c0;

	return (-atan(c1));
}


int
remove_points_by_triangulation_and_compute_beta(carmen_vector_3D_t *points_position_with_respect_to_car, int num_filtered_points, double &beta)
{
	int i, j, angle_flag = 0, n_points_triangularization = fmax(5, triangulation_min_cluster_size/2), 
		actual_group = 0, group_counter = 0, choosen_group = 0, first_time = 1, bigger_group = 0, 
		group[num_filtered_points] = {0};
	double estimated_beta = 0.0, choosen_beta = M_PI, angle, 
		   estimated[num_filtered_points] = {0.0}, 
		   mean_dist_bt_rays = 0.0, actual_mean_dist_bt_rays = 0.0, dist;
	static double last_beta = 0.0, last_mean_dist_bt_rays = 180.0;
	carmen_vector_3D_t *points = (carmen_vector_3D_t*) malloc(sizeof(carmen_vector_3D_t)*num_filtered_points);
	double *points_estimated = (double*) malloc(sizeof(double)*num_filtered_points);

	if (last_beta < 0)
	{
		points[0] = points_position_with_respect_to_car[0];
		group[0] = actual_group;
		points[1] = points_position_with_respect_to_car[1];
		group[1] = actual_group;
		group_counter = 2;

		for (i = 2; i < num_filtered_points; i++)
		{
			points[group_counter] = points_position_with_respect_to_car[i];
			group[i] = actual_group;

			angle_flag = 0;
			for (j = 2; j <= fmin(i, n_points_triangularization); j++)
			{
				angle = (atan2(points_position_with_respect_to_car[i].y - points_position_with_respect_to_car[i-j].y, 
							points_position_with_respect_to_car[i].x - points_position_with_respect_to_car[i-j].x) - 
						atan2(points_position_with_respect_to_car[i-(j/2)].y - points_position_with_respect_to_car[i-j].y, 
							points_position_with_respect_to_car[i-(j/2)].x - points_position_with_respect_to_car[i-j].x));
				if (fabs(angle) > triangulation_max_beta)
				{
					angle_flag = 1;
					break;
				}
			}

			dist = DOT2D(points_position_with_respect_to_car[i-1], points_position_with_respect_to_car[i]);
			mean_dist_bt_rays += dist;

			if (angle_flag || (i == (num_filtered_points-1)) || (dist > remove_by_mean_dist_between_rays_factor*last_mean_dist_bt_rays))
			{
				if (group_counter > triangulation_min_cluster_size)
				{
					// more closest from last beta
					estimated_beta = compute_new_beta(points, points_estimated, group_counter);
					if (first_time && (group_counter > bigger_group))
					{
						choosen_beta = estimated_beta;
						choosen_group = actual_group;
						bigger_group = group_counter;

						actual_mean_dist_bt_rays = mean_dist_bt_rays/(group_counter-1);
						first_time = 0;
					}
					else if (fabs(estimated_beta - last_beta) < fabs(choosen_beta - last_beta))
					{
						choosen_beta = estimated_beta;
						choosen_group = actual_group;
						bigger_group = group_counter;

						actual_mean_dist_bt_rays = mean_dist_bt_rays/(group_counter-1);
					}
					if (PLOT_BETA)
						for (j = group_counter; j > 0; j--)
							estimated[i-j] = points_estimated[group_counter-j];
				}
				else if (PLOT_BETA)
					for (j = group_counter; j > 0; j--)
						group[i-j] = -1;

				group[i] = -1;
				actual_group++;
				group_counter = -1;
				mean_dist_bt_rays = 0.0;
			}
		
			group_counter++;
		}
	}
	else
	{
		points[0] = points_position_with_respect_to_car[num_filtered_points-1];
		group[num_filtered_points-1] = actual_group;
		points[1] = points_position_with_respect_to_car[num_filtered_points-2];
		group[num_filtered_points-2] = actual_group;
		group_counter = 2;

		for (i = num_filtered_points-3; i >= 0; i--)
		{
			points[group_counter] = points_position_with_respect_to_car[i];
			group[i] = actual_group;

			angle_flag = 0;
			for (j = 2; j <= fmin(i, n_points_triangularization); j++)
			{
				angle = (atan2(points_position_with_respect_to_car[i+j].y - points_position_with_respect_to_car[i].y, 
							points_position_with_respect_to_car[i+j].x - points_position_with_respect_to_car[i].x) - 
						atan2(points_position_with_respect_to_car[i+j].y - points_position_with_respect_to_car[i+(j/2)].y, 
							points_position_with_respect_to_car[i+j].x - points_position_with_respect_to_car[i+(j/2)].x));
				if (fabs(angle) > triangulation_max_beta)
				{
					angle_flag = 1;
					break;
				}
			}

			dist = DOT2D(points_position_with_respect_to_car[i+1], points_position_with_respect_to_car[i]);
			mean_dist_bt_rays += dist;

			if (angle_flag || (i == 0) || (dist > remove_by_mean_dist_between_rays_factor*last_mean_dist_bt_rays))
			{
				if (group_counter > triangulation_min_cluster_size)
				{
					// more closest from last beta
					estimated_beta = compute_new_beta(points, points_estimated, group_counter);
					if (first_time && (group_counter > bigger_group))
					{
						choosen_beta = estimated_beta;
						choosen_group = actual_group;
						bigger_group = group_counter;

						actual_mean_dist_bt_rays = mean_dist_bt_rays/(group_counter-1);
						first_time = 0;
					}
					else if (fabs(estimated_beta - last_beta) < fabs(choosen_beta - last_beta))
					{
						choosen_beta = estimated_beta;
						choosen_group = actual_group;
						bigger_group = group_counter;

						actual_mean_dist_bt_rays = mean_dist_bt_rays/(group_counter-1);
					}
					if (PLOT_BETA)
						for (j = group_counter; j > 0; j--)
							estimated[i+j] = points_estimated[group_counter-j];
				}
				else if (PLOT_BETA)
					for (j = group_counter; j > 0; j--)
						group[i+j] = -1;

				group[i] = -1;
				actual_group++;
				group_counter = -1;
				mean_dist_bt_rays = 0.0;
			}
		
			group_counter++;
		}
	}

	if (PLOT_BETA)
		plot_groups(points_position_with_respect_to_car, estimated, num_filtered_points, group, actual_group+1, choosen_group);

	free(points);
	free(points_estimated);

	if (fabs(actual_mean_dist_bt_rays) > 1e-5)
		last_mean_dist_bt_rays = actual_mean_dist_bt_rays;

	if (fabs(choosen_beta - M_PI) > 1e-5)
		last_beta = choosen_beta;

	beta = last_beta;
	return bigger_group;
}


double
compute_points_position_with_respect_to_car(carmen_semi_trailers_config_t semi_trailer_config, 
        sensor_parameters_t *spherical_sensor_params, 
        carmen_velodyne_partial_scan_message *partial_message,
        carmen_velodyne_variable_scan_message *variable_message, int lidar_to_compute_theta)
{
	int num_valid_points = 0, num_filtered_points = 0;
    double angle, distance_to_king_pin, x, y, estimated_beta;
    carmen_vector_3D_t *points_position_with_respect_to_car;

	if (lidar_to_compute_theta < 0)
    {
        points_position_with_respect_to_car = (carmen_vector_3D_t *) malloc(partial_message->number_of_32_laser_shots * sizeof(carmen_vector_3D_t));
        num_valid_points = compute_points_with_respect_to_king_pin(points_position_with_respect_to_car,
            spherical_sensor_params[0].ray_order[semi_trailer_config.semi_trailers[SEMI_TRAILER1].beta_correct_velodyne_ray], semi_trailer_config,
			partial_message, spherical_sensor_params[0].vertical_correction,
			spherical_sensor_params[0].sensor_to_support_matrix, spherical_sensor_params[0].support_to_car_matrix,
			spherical_sensor_params[0].pose.position, spherical_sensor_params[0].sensor_support_pose.position);
    }
	else
    {
        points_position_with_respect_to_car = (carmen_vector_3D_t *) malloc(variable_message->number_of_shots * sizeof(carmen_vector_3D_t));
        num_valid_points = compute_points_with_respect_to_king_pin(points_position_with_respect_to_car,
            spherical_sensor_params[lidar_to_compute_theta].ray_order[semi_trailer_config.semi_trailers[SEMI_TRAILER1].beta_correct_velodyne_ray], semi_trailer_config, 
			variable_message, spherical_sensor_params[lidar_to_compute_theta].vertical_correction,
			spherical_sensor_params[lidar_to_compute_theta].sensor_to_support_matrix, spherical_sensor_params[lidar_to_compute_theta].support_to_car_matrix,
			spherical_sensor_params[lidar_to_compute_theta].pose.position, spherical_sensor_params[lidar_to_compute_theta].sensor_support_pose.position);
    }

	if (num_valid_points == 0)
		return BETA_ERROR;

	for (int i = 0; i < num_valid_points; i++)
	{
		angle = atan2(points_position_with_respect_to_car[i].y, points_position_with_respect_to_car[i].x);
		if ((lidar_to_compute_theta >= 0) && angle >= 0)
			angle -= M_PI_2;
		else if ((lidar_to_compute_theta >= 0) && angle < 0)
			angle += M_PI_2;

        distance_to_king_pin = sqrt(DOT2D(points_position_with_respect_to_car[i], points_position_with_respect_to_car[i]));
		if ((angle > (-semi_trailer_config.semi_trailers[SEMI_TRAILER1].beta_correct_angle_factor * semi_trailer_config.semi_trailers[SEMI_TRAILER1].max_beta)) &&
			(angle < (semi_trailer_config.semi_trailers[SEMI_TRAILER1].beta_correct_angle_factor * semi_trailer_config.semi_trailers[SEMI_TRAILER1].max_beta)) && 
			(distance_to_king_pin < fabs(semi_trailer_config.semi_trailers[SEMI_TRAILER1].beta_correct_max_distance)))
		{
			x = points_position_with_respect_to_car[i].x * cos(M_PI / 2.0) - points_position_with_respect_to_car[i].y * sin(M_PI / 2.0);
			y = points_position_with_respect_to_car[i].x * sin(M_PI / 2.0) + points_position_with_respect_to_car[i].y * cos(M_PI / 2.0);
			points_position_with_respect_to_car[num_filtered_points].x = x;
			points_position_with_respect_to_car[num_filtered_points].y = y;
			num_filtered_points++;
		}
	}

	qsort((void *) (points_position_with_respect_to_car), (size_t) num_filtered_points, sizeof(carmen_vector_3D_t), compare_x);

	num_filtered_points = remove_begin_and_end_points(num_filtered_points, points_position_with_respect_to_car);
	if (num_filtered_points <= 0)
		return BETA_ERROR;

	num_filtered_points = remove_points_by_triangulation_and_compute_beta(points_position_with_respect_to_car, num_filtered_points, estimated_beta);
	if (num_filtered_points <= 0)
		return BETA_ERROR;

    free(points_position_with_respect_to_car);

	return (estimated_beta);
}


double
compute_semi_trailer_theta1(carmen_robot_and_trailers_traj_point_t robot_and_trailer_traj_point, double dt,
		carmen_robot_ackerman_config_t robot_config, carmen_semi_trailers_config_t semi_trailer_config, 
        sensor_parameters_t *spherical_sensor_params, 
        carmen_velodyne_partial_scan_message *partial_message,
        carmen_velodyne_variable_scan_message *variable_message, int lidar_to_compute_theta)
{
	if (semi_trailer_config.num_semi_trailers <= 0)
		return (0.0);

	double predicted_beta = .0, estimated_beta = .0, estimated_theta1 = .0, buffered_theta1;
	static int buffer_pos = -1;
	static double buffer_theta[BUFFER_SIZE] = {0.0};
	
	predicted_beta = compute_semi_trailer_beta(robot_and_trailer_traj_point, dt, robot_config, semi_trailer_config);
	
	if (((lidar_to_compute_theta < 0) && !partial_message) ||
		!spherical_sensor_params[0].sensor_to_support_matrix ||
		!spherical_sensor_params[0].support_to_car_matrix)
		return (predicted_beta);
	else if (((lidar_to_compute_theta >= 0) && !variable_message) ||
		!spherical_sensor_params[lidar_to_compute_theta].sensor_to_support_matrix ||
		!spherical_sensor_params[lidar_to_compute_theta].support_to_car_matrix)
		return (predicted_beta);

	estimated_beta = compute_points_position_with_respect_to_car(semi_trailer_config, 
        spherical_sensor_params, partial_message, variable_message, lidar_to_compute_theta);
	if (fabs(estimated_beta - BETA_ERROR) < 1e-5)
	{
		if (lidar_to_compute_theta < 0)
			printf("empty points during beta estimation in [%lf]s!\n", partial_message->timestamp);
		else
			printf("empty points during beta estimation in [%lf]s!\n", variable_message->timestamp);
		return predicted_beta;
	}

	estimated_theta1 = convert_beta_to_theta1(robot_and_trailer_traj_point.theta, carmen_normalize_theta(estimated_beta - semi_trailer_config.semi_trailers[SEMI_TRAILER1].beta_correct_beta_bias));
	if (buffer_pos < 0)
		for (int i = 0; i < BUFFER_SIZE; i++)
			buffer_theta[i] = estimated_theta1;
	buffer_theta[++buffer_pos % BUFFER_SIZE] = estimated_theta1;

	buffered_theta1 = 0.0;
	for (int i = 0; i < BUFFER_SIZE; i++)
		buffered_theta1 += buffer_theta[i];
	
	return buffered_theta1 / (double) BUFFER_SIZE;
}


double
compute_semi_trailer_theta1(carmen_robot_and_trailers_traj_point_t robot_and_trailer_traj_point, double dt,
		carmen_robot_ackerman_config_t robot_config, carmen_semi_trailers_config_t semi_trailer_config, 
        sensor_parameters_t *spherical_sensor_params, 
        void *message, int lidar_to_compute_theta)
{
	if (semi_trailer_config.num_semi_trailers <= 0)
		return (0.0);

    if (lidar_to_compute_theta < 0)
        return compute_semi_trailer_theta1(robot_and_trailer_traj_point, dt, robot_config, semi_trailer_config, 
            spherical_sensor_params, (carmen_velodyne_partial_scan_message*) message, NULL, -1);
    else
        return compute_semi_trailer_theta1(robot_and_trailer_traj_point, dt, robot_config, semi_trailer_config, 
            spherical_sensor_params, NULL, (carmen_velodyne_variable_scan_message*) message, lidar_to_compute_theta);
}
