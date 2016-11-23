/**
 * @file
 * @author Filipe Mutz
 *
 * @section DESCRIPTION
 * This file implements the polar map and the particle filter in a more efficient way
 */

#include <stdio.h>
#include <stdlib.h>

// TODO: Retirar quando terminar de debugar
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include <carmen/carmen.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/moving_objects3_interface.h>

#include "moving_objects3_particle_filter.h"
#include "tf_util.h"

carmen_fast_polar_slam_measurement_model_message measurement_model_message;
int first_measurement_model_message = 1;

int *old_occupation = NULL;
carmen_polar_point_t *old_map = NULL;

carmen_polar_particle_filter *resampled_particle_filter = NULL;

void
carmen_polar_slam_initialize_particle_filter_configuration(carmen_polar_particle_filter *particle_filter, int num_particles, int num_spheres, int num_sections, int num_points_per_section, double max_range)
{
	particle_filter->config.num_spheres = num_spheres;
	particle_filter->config.num_sections = num_sections;
	particle_filter->config.num_particles = num_particles;
	particle_filter->config.num_points_per_sections = num_points_per_section;
	particle_filter->config.map_size = (num_spheres * num_sections * num_points_per_section);
	particle_filter->config.first_map_has_been_received = 0;
	particle_filter->config.max_range = max_range;
	particle_filter->best_particle = NULL;
}


void
carmen_polar_slam_allocate_particle_filter_data(carmen_polar_particle_filter *particle_filter)
{
	int i, num_particles, map_size;

	num_particles = particle_filter->config.num_particles;
	map_size = particle_filter->config.map_size;

	particle_filter->particles = (carmen_polar_particle *) calloc (num_particles, sizeof(carmen_polar_particle));
	carmen_test_alloc(particle_filter->particles);

	for(i = 0; i < num_particles; i++)
	{
		particle_filter->particles[i].particle_map_occupation = (int *) calloc (map_size, sizeof(int));
		particle_filter->particles[i].particle_map = (carmen_polar_point_t *) calloc (map_size, sizeof(carmen_polar_point_t));

		carmen_test_alloc(particle_filter->particles[i].particle_map_occupation);
		carmen_test_alloc(particle_filter->particles[i].particle_map);
	}
}


void
carmen_polar_slam_create_random_particle_poses(carmen_polar_particle_filter *particle_filter, carmen_pose_3D_t *global_pose)
{
	int i, num_particles;
	carmen_pose_3D_t pose;

	num_particles = particle_filter->config.num_particles;
	pose = (*global_pose);

	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// TODO: Eu estou inicializando todas as poses na
	// global_pose. Se a localizacao ficar muito ruim,
	// passar a criar de fato particles espalhadas!
	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	for(i = 0; i < num_particles; i++)
	{
		particle_filter->particles[i].particle_pose = pose;
		particle_filter->particles[i].pose_before_movement = pose;
	}
}


carmen_polar_particle_filter*
carmen_polar_slam_create_particle_filter(carmen_pose_3D_t *global_pose, int num_particles, int num_spheres, int num_sections, int num_points_per_section, double max_range)
{
	carmen_polar_particle_filter *particle_filter;

	particle_filter = (carmen_polar_particle_filter *) calloc (1, sizeof(carmen_polar_particle_filter));
	carmen_test_alloc(particle_filter);

	carmen_polar_slam_initialize_particle_filter_configuration(particle_filter, num_particles, num_spheres, num_sections, num_points_per_section, max_range);
	carmen_polar_slam_allocate_particle_filter_data(particle_filter);
	carmen_polar_slam_create_random_particle_poses(particle_filter, global_pose);

	return particle_filter;
}


double
carmen_polar_slam_get_sphere_radius_by_index(int i)
{
	// o raio da esfera aumenta em potencias de 2
	return pow(2.0, i + 1.0);
}


int
carmen_polar_slam_get_sphere_index_by_radius(carmen_polar_particle_filter *particle_filter, double radius)
{
	int i, num_spheres;

	num_spheres = particle_filter->config.num_spheres;

	for(i = 0; i < num_spheres; i++)
		if (radius < carmen_polar_slam_get_sphere_radius_by_index(i))
			return i;

	return -1;
}


int
carmen_polar_slam_get_section_index_by_angle(carmen_polar_particle_filter *particle_filter, double angle)
{
	int angle_id;
	double angular_resolution;

	angular_resolution = (2 * M_PI) / (double) particle_filter->config.num_sections;
	angle_id = (int) ((angle + M_PI) / angular_resolution);

	return angle_id;
}


int
carmen_polar_slam_get_point_position(carmen_polar_particle_filter *particle_filter, int sphere_id, int section_id)
{
	int point_list_position;
	int section_size;
	int sphere_size;

	section_size = particle_filter->config.num_points_per_sections;
	sphere_size = particle_filter->config.num_sections * section_size;
	point_list_position = ((sphere_id * sphere_size) + (section_id * section_size));

	return point_list_position;
}


void
carmen_polar_slam_add_point_to_particle_map(carmen_polar_particle *particle, int num_points_per_section, int point_list_position, double radius, double angle)
{
	int i, j;
	int *occupation_list_ptr;
	carmen_polar_point_t *point_list_ptr;

	// get the pointer to the beginning of the point list
	point_list_ptr = (particle->particle_map + point_list_position);
	occupation_list_ptr = (particle->particle_map_occupation + point_list_position);

	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// TODO: O for abaixo percorre a lista de pontos
	// ate encontrar algum ponto com raio maior que
	// o do novo ponto. Nos podemos fazer esse for
	// de tras para frente para que se o ultimo ponto
	// tiver raio maior que o do novo ponto o for saia
	// na primeira iteracao. O codigo vai ficar ate mais
	// simples porque nao vai precisar do segundo for.
	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	// find the position of the first element with radius smaller than the point
	for(i = 0; i < num_points_per_section; i++)
	{
		if ((radius < point_list_ptr[i].radius) || (occupation_list_ptr[i] == 0))
		{
			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// TODO: Pensar em como fazer esse for de forma
			// a parar de percorrer a lista de pontos quando
			// aparecer um ponto nao ocupado
			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

			// shit the array of points to right to open space to the new point
			for(j = (num_points_per_section - 1); j > i; j--)
			{
				point_list_ptr[j] = point_list_ptr[j - 1];
				occupation_list_ptr[j] = occupation_list_ptr[j - 1];
			}

			point_list_ptr[i].angle = angle;
			point_list_ptr[i].radius = radius;
			occupation_list_ptr[i] = 1;

			break;
		}
	}
}


void
copy_map_to_old_map(carmen_polar_point_t *map, int *map_occupation, carmen_polar_point_t *old_map, int *old_map_occupation, int map_size)
{
	memcpy(old_map, map, map_size * sizeof(carmen_polar_point_t));
	memcpy(old_map_occupation, map_occupation, map_size * sizeof(int));
}


void
clear_map(carmen_polar_point_t *map, int *map_occupation, int map_size)
{
	memset(map, 0, map_size * sizeof(carmen_polar_point_t));
	memset(map_occupation, 0, map_size * sizeof(int));
}


void
move_point_to_new_position(double origin_angle, double origin_radius, double *dest_angle, double *dest_radius, TfFrameTransformationWrapper *tf_wrapper, double yaw_correction)
{
	carmen_pose_3D_t point_to_transform;
	memset(&point_to_transform, 0, sizeof(carmen_pose_3D_t));

	transform_polar_coordinates_to_cartesian_coordinates(origin_radius, origin_angle, &(point_to_transform.position.x), &(point_to_transform.position.y));
	carmen_pose_3D_t point_transformed = tf_wrapper->transform(point_to_transform);
	transform_cartesian_coordinates_to_polar_coordinates(point_transformed.position.x, point_transformed.position.y, dest_radius, dest_angle);

	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// TODO: Quando eu faco a transformada usando
	// a TF eu ainda preciso somar uma rotacao ao angulo
	// final para a transformacao funcionar. Eh importante
	// entender o que esta acontecendo e por que.
	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// (*dest_angle) = carmen_normalize_theta((*dest_angle) + yaw_correction);
	(*dest_angle) = carmen_normalize_theta((*dest_angle) - point_transformed.orientation.yaw + yaw_correction);
}


void
add_point_in_new_map(double radius, double angle, carmen_polar_particle *particle, carmen_polar_particle_filter *particle_filter)
{
	int sphere_id = carmen_polar_slam_get_sphere_index_by_radius(particle_filter, radius);
	int section_id = carmen_polar_slam_get_section_index_by_angle(particle_filter, angle);
	int point_list_position = carmen_polar_slam_get_point_position(particle_filter, sphere_id, section_id);

	// if the point is inside the map
	if ((sphere_id >= 0) && (sphere_id < particle_filter->config.num_spheres))
		carmen_polar_slam_add_point_to_particle_map(particle, particle_filter->config.num_points_per_sections, point_list_position, radius, angle);
}


void
move_particle_points(carmen_polar_point_t *old_map, int *old_occupation, carmen_polar_particle *particle, carmen_polar_particle_filter* particle_filter)
{
	int i, j, k;
	int point_is_set;
	double angle = 0.0, radius = 0.0;
	carmen_polar_point_t point;
	int beginning_of_the_section_point_list;
	TfFrameTransformationWrapper tf_wrapper(particle->pose_before_movement, particle->particle_pose);

	for (i = 0; i < particle_filter->config.num_spheres; i++)
	{
		for (j = 0; j < particle_filter->config.num_sections; j++)
		{
			beginning_of_the_section_point_list = carmen_polar_slam_get_point_position(particle_filter, i, j);

			for (k = 0; k < particle_filter->config.num_points_per_sections; k++)
			{
				point_is_set = old_occupation[beginning_of_the_section_point_list + k];
				point = old_map[beginning_of_the_section_point_list + k];

				if (point_is_set)
				{
					move_point_to_new_position(point.angle, point.radius, &angle, &radius, &tf_wrapper, (particle->particle_pose.orientation.yaw - particle->pose_before_movement.orientation.yaw));
					add_point_in_new_map(radius, angle, particle, particle_filter);
				}
			}
		}
	}
}


void
carmen_polar_slam_move_particles_to_new_position(carmen_polar_particle_filter* particle_filter)
{
	int i;
	int num_particles = particle_filter->config.num_particles;

	if (old_occupation == NULL || old_map == NULL)
	{
		// como todos os mapas sao do mesmo tamanho, eu mantenho eu aloco essas variaveis
		// somente uma vez aqui e uso elas durante a copia de todos os mapas
		old_occupation = (int *) calloc (particle_filter->config.map_size, sizeof(int));
		carmen_test_alloc(old_occupation);

		old_map = (carmen_polar_point_t *) calloc (particle_filter->config.map_size, sizeof(carmen_polar_point_t));
		carmen_test_alloc(old_map);
	}

	for(i = 0; i < num_particles; i++)
	{
		// Nos precisamos retirar todos os pontos do mapa antes de comecar a mover para que
		// nenhum ponto corra o risco de ser movido para uma esfera ainda nao tratada. Isso
		// faria com que o ponto fosse movido novamente. Eu trato isso copiando o mapa para
		// old_map e limpando o mapa. Entao eu movo cada ponto do old_map e adiciono o ponto
		// novamente no mapa da particula.

		copy_map_to_old_map(particle_filter->particles[i].particle_map, particle_filter->particles[i].particle_map_occupation, old_map, old_occupation, particle_filter->config.map_size);
		clear_map(particle_filter->particles[i].particle_map, particle_filter->particles[i].particle_map_occupation, particle_filter->config.map_size);
		move_particle_points(old_map, old_occupation, &(particle_filter->particles[i]), particle_filter);
	}
}


double
carmen_polar_slam_perform_ray_cast(carmen_polar_particle_filter* particle_filter, int particle, double angle, double measured_range __attribute__((unused)))
{
	carmen_polar_point_t *point_list_ptr;
	int i, j, point_pos, section_id;
	int *occupation_ptr;

	section_id = carmen_polar_slam_get_section_index_by_angle(particle_filter, angle);

	for(i = 0; i < particle_filter->config.num_spheres; i++)
	{
		point_pos = carmen_polar_slam_get_point_position(particle_filter, i, section_id);
		point_list_ptr = particle_filter->particles[particle].particle_map + point_pos;
		occupation_ptr = particle_filter->particles[particle].particle_map_occupation + point_pos;

		for (j = 0; j < particle_filter->config.num_points_per_sections; j++)
		{
			if (occupation_ptr[j] == 1)
				return point_list_ptr[j].radius;
		}
	}

	return -1;
}


double
carmen_polar_slam_perform_ray_cast_to_closest(carmen_polar_particle_filter* particle_filter, int particle, double angle, double measured_range)
{
	carmen_polar_point_t *point_list_ptr;
	int i, j, point_pos, section_id;
	int *occupation_ptr;
	double diff_of_radius, closest_diff_of_radius, closest_radius;

	closest_diff_of_radius = DBL_MAX;
	closest_radius = -1;

	section_id = carmen_polar_slam_get_section_index_by_angle(particle_filter, angle);

	for(i = 0; i < particle_filter->config.num_spheres; i++)
	{
		point_pos = carmen_polar_slam_get_point_position(particle_filter, i, section_id);
		point_list_ptr = particle_filter->particles[particle].particle_map + point_pos;
		occupation_ptr = particle_filter->particles[particle].particle_map_occupation + point_pos;

		for (j = 0; j < particle_filter->config.num_points_per_sections; j++)
		{
			if (occupation_ptr[j] != 0)
			{
				diff_of_radius = fabs(point_list_ptr[j].radius - measured_range);

				if ((closest_radius == -1) || (diff_of_radius < closest_diff_of_radius))
				{
					closest_diff_of_radius = diff_of_radius;
					closest_radius = point_list_ptr[j].radius;
				}
			}
		}
	}

	return closest_radius;
}


void
allocate_space_to_the_measurement_message(int num_rays)
{
	measurement_model_message.num_rays = num_rays;

	measurement_model_message.angles = (double *) calloc (num_rays, sizeof(double));
	measurement_model_message.measured_ranges = (double *) calloc (num_rays, sizeof(double));
	measurement_model_message.estimated_ranges = (double *) calloc (num_rays, sizeof(double));
	measurement_model_message.ray_probability = (double *) calloc (num_rays, sizeof(double));

	measurement_model_message.host = carmen_get_host();
}


void
add_data_to_the_message(int ray_index, double angle, double estimated_range, double measured_range, double prob)
{
	measurement_model_message.angles[ray_index] = angle;
	measurement_model_message.measured_ranges[ray_index] = measured_range;
	measurement_model_message.estimated_ranges[ray_index] = estimated_range;
	measurement_model_message.ray_probability[ray_index] = prob;
}


double
carmen_polar_slam_calculate_particle_probability(carmen_polar_particle_filter* particle_filter, int particle, double *ranges, double *angles, int num_rays)
{
	double angle, sensor_real_measurement, sensor_measurement_estimated_from_map, particle_weight, ray_weight, particle_total_log_odds;
	double log_odds_of_probability;

	int jmp = 60;
	particle_weight = 0.0;
	particle_total_log_odds = 0.0;

	if (first_measurement_model_message)
	{
		allocate_space_to_the_measurement_message(num_rays / jmp);
		first_measurement_model_message = 0;
	}

//	printf("num rays: %d num rays used: %d\n", num_rays, num_rays / jmp);

	for(int i = 0; i < num_rays; i += jmp)
	{
		angle = angles[i];
		sensor_real_measurement = ranges[i];

		if ((sensor_real_measurement < 2.0) || (sensor_real_measurement > 50.0))
			sensor_real_measurement = particle_filter->config.max_range;

		sensor_measurement_estimated_from_map = carmen_polar_slam_perform_ray_cast_to_closest(particle_filter, particle, angle, sensor_real_measurement);
		// sensor_measurement_estimated_from_map = carmen_polar_slam_perform_ray_cast(particle_filter, particle, angle, sensor_real_measurement);

		if (sensor_measurement_estimated_from_map < 0) // map hasn't information yet
		{
			sensor_measurement_estimated_from_map = particle_filter->config.max_range;
//			num_rays_with_range_max++;

			// *****************************************************
			// TODO: pensar se essa eh a melhor forma de tratar isso
			// *****************************************************
//			ray_weight = 0.5;
		}
//		else
//		{
		ray_weight = beam_range_finder_model_probability(sensor_measurement_estimated_from_map, sensor_real_measurement);

		// *****************************************************
		// TODO: pensar se essa eh a melhor forma de tratar isso
		// *****************************************************
		if (ray_weight <= 0.001)
			ray_weight = 0.001;
		if (ray_weight >= 0.999)
			ray_weight = 0.999;
//		}

		add_data_to_the_message(i / jmp, angle, sensor_measurement_estimated_from_map, sensor_real_measurement, ray_weight);

		log_odds_of_probability = log(ray_weight / (1.0 - ray_weight));
		particle_total_log_odds = particle_total_log_odds + log_odds_of_probability;

//		printf("ray: %05d\testimated: %lf\treal: %lf\tweight: %lf\n", i, sensor_measurement_estimated_from_map, sensor_real_measurement, ray_weight);
//		printf("odd: %lf\ttotal_odd: %lf\n", log_odds_of_probability, particle_total_log_odds);
	}

	particle_weight = 1.0 - (1.0 / (1.0 + exp(particle_total_log_odds)));

//	printf("Particle total odd: %lf weight: %.13lf\n", particle_total_log_odds, particle_weight);

	// TODO: Criar uma forma mais facil de saber qual eh a best particle. Nesse caso, o mapa que eu vou
	// estar mostrando vai ser o da best particle, entao so faz sentido mandar as leituras do laser no
	// mapa dela
	if (particle_filter->particles[particle].particle_map == particle_filter->best_particle->particle_map)
	{
//		publish_measurement_model_message();
	}
	return particle_weight;
}


void
carmen_polar_slam_calculate_sensor_model_probability(carmen_polar_particle_filter* particle_filter, double *range, double *angles, int num_rays)
{
	int i;
	int num_particles = particle_filter->config.num_particles;
	int num_particles_with_weight_equals_to_zero = 0;

	for(i = 0; i < num_particles; i++)
	{
		particle_filter->particles[i].weight = carmen_polar_slam_calculate_particle_probability(particle_filter, i, range, angles, num_rays);

		if (particle_filter->particles[i].weight <= 0.00001)
			num_particles_with_weight_equals_to_zero++;
	}

	if (num_particles_with_weight_equals_to_zero == num_particles)
	{
		// DEBUG:
		printf("=> All particles have weight equals to zero (%d particles)!\n", num_particles);

		for(i = 0; i < num_particles; i++)
			particle_filter->particles[i].weight = (1.0 / ((double) num_particles));
	}
}


void
carmen_polar_slam_normalize_particle_weights(carmen_polar_particle_filter* particle_filter)
{
	int i;
	double total_weight = 0;

	for(i = 0; i < particle_filter->config.num_particles; i++)
		total_weight += particle_filter->particles[i].weight;

	for(i = 0; i < particle_filter->config.num_particles; i++)
		particle_filter->particles[i].weight /= total_weight;
}


void
carmen_polar_slam_calculate_particle_weights(carmen_polar_particle_filter* particle_filter, double *range, double *angles, int num_rays)
{
	carmen_polar_slam_calculate_sensor_model_probability(particle_filter, range, angles, num_rays);
	carmen_polar_slam_normalize_particle_weights(particle_filter);

	// DEBUG:
//	for (int i = 0; i < particle_filter->config.num_particles; i++)
//		printf("\t\tparticle[%05d] = %lf\n", i, particle_filter->particles[i].weight);
}


carmen_polar_particle_filter *
create_particle_filter_to_resample(carmen_polar_particle_filter* particle_filter)
{
	carmen_polar_particle_filter *resampled_particle_filter;

	resampled_particle_filter = (carmen_polar_particle_filter *) calloc (1, sizeof(carmen_polar_particle_filter));
	carmen_test_alloc(resampled_particle_filter);

	carmen_polar_slam_initialize_particle_filter_configuration(resampled_particle_filter, particle_filter->config.num_particles, particle_filter->config.num_spheres, particle_filter->config.num_sections, particle_filter->config.num_points_per_sections, particle_filter->config.max_range);
	carmen_polar_slam_allocate_particle_filter_data(resampled_particle_filter);
	resampled_particle_filter->config.first_map_has_been_received = 1;

	return resampled_particle_filter;
}


void
copy_particle_to_resampled_particles(carmen_polar_particle *resampled_particle, carmen_polar_particle *particle, int map_size)
{
	resampled_particle->pose_before_movement = particle->pose_before_movement;
	resampled_particle->particle_pose = particle->particle_pose;
	resampled_particle->weight = particle->weight;

	memcpy(resampled_particle->particle_map, particle->particle_map, map_size * sizeof(carmen_polar_point_t));
	memcpy(resampled_particle->particle_map_occupation, particle->particle_map_occupation, map_size * sizeof(int));
}


void
switch_data_from_resample_to_particle_filter(carmen_polar_particle_filter *particle_filter, carmen_polar_particle_filter *resampled_particle_filter)
{
	carmen_polar_particle *aux = particle_filter->particles;
	particle_filter->particles = resampled_particle_filter->particles;
	resampled_particle_filter->particles = aux;
}


//void
//carmen_polar_slam_resample_particles(carmen_polar_particle_filter* particle_filter)
//{
//	int i, m;
//	int best_particle_id = 0;
//	int num_particles = particle_filter->config.num_particles;
//
//	double max_weight = 0;
//	double M_inv = 1.0 / ((double) num_particles);
//	double r = M_inv * ((double) rand() / (double) RAND_MAX);
//	double c = particle_filter->particles[0].weight;
//
//
//	carmen_polar_particle_filter *resampled_particle_filter = create_particle_filter_to_resample(particle_filter);
//	// resampled_particle_filter = create_particle_filter_to_resample(particle_filter);
//
//	// DEBUG:
////	 for(m = 0; m < num_particles; m++)
////		printf("particle %d: %lf\n", m, particle_filter->particles[m].weight);
//
//	i = 1;
//	srand(time(NULL));
//
//	for(m = 0; m < num_particles; m++)
//	{
//		double U = r + m * M_inv;
//
//		while (U > c)
//		{
//			c += particle_filter->particles[i].weight;
//			i++;
//		}
//
//		 // DEBUG:
////		 printf("Resample Particle %d To Particle: %d\n", m, i - 1);
//
//		if (i >= num_particles || i <= 0)
//		{
//			// i = rand() % num_particles;
//			i = 1;
//
//			// DEBUG:
//			// printf("Choosen particle index is bigger than particle list length in the resample! Rellocating i to %d\n", i);
//		}
//
//		copy_particle_to_resampled_particles(
//			&(resampled_particle_filter->particles[m]),
//			&(particle_filter->particles[i - 1]), // I used i - 1 because in the thrun's algorithm the index start in 1
//			particle_filter->config.map_size
//		);
//	}
//
//	// o for abaixo verifica quem eh a particula mais pesada.
//	// para aproveitar o loop eu tb dou free nos mapas das particulas antigas
//	for(i = 0; i < num_particles; i++)
//	{
//		if (resampled_particle_filter->particles[i].weight > max_weight)
//		{
//			max_weight = resampled_particle_filter->particles[i].weight;
//			best_particle_id = i;
//		}
//
//		resampled_particle_filter->particles[i].weight = M_inv;
//
//		free(particle_filter->particles[i].particle_map);
//		free(particle_filter->particles[i].particle_map_occupation);
//	}
//
//	free(particle_filter->particles);
//	(*particle_filter) = (*resampled_particle_filter);
//	free(resampled_particle_filter);
//
//	particle_filter->best_particle = &(particle_filter->particles[best_particle_id]);
//}


void
carmen_polar_slam_resample_particles(carmen_polar_particle_filter* particle_filter)
{
	int i, m;
	int best_particle_id = 0;
	int num_particles = particle_filter->config.num_particles;

	double max_weight = 0;
	double M_inv = 1.0 / ((double) num_particles);
	double r = M_inv * ((double) rand() / (double) RAND_MAX);
	double c = particle_filter->particles[0].weight;

	if (resampled_particle_filter == NULL)
		resampled_particle_filter = create_particle_filter_to_resample(particle_filter);

	// DEBUG:
	for(m = 0; m < num_particles; m++)
		printf("particle %d: %lf\n", m, particle_filter->particles[m].weight);

	i = 1;
	srand(time(NULL));

	for(m = 0; m < num_particles; m++)
	{
		double U = r + m * M_inv;

		while (U > c)
		{
			c += particle_filter->particles[i].weight;
			i++;
		}

		if (i <= 0)
			i = 1;

		if (i >= num_particles)
		{
			// i = rand() % num_particles;
			i = num_particles;

			// DEBUG:
			// printf("Choosen particle index is bigger than particle list length in the resample! Rellocating i to %d\n", i);
		}

		// DEBUG:
		printf("Resample Particle %d To Particle: %d\n", m, i - 1);

		copy_particle_to_resampled_particles(
			&(resampled_particle_filter->particles[m]),
			&(particle_filter->particles[i - 1]), // I used i - 1 because in the thrun's algorithm the index start in 1
			particle_filter->config.map_size
		);
	}

	// o for abaixo verifica quem eh a particula mais pesada.
	// para aproveitar o loop eu tb dou free nos mapas das particulas antigas
	for(i = 0; i < num_particles; i++)
	{
		if (resampled_particle_filter->particles[i].weight > max_weight)
		{
			max_weight = resampled_particle_filter->particles[i].weight;
			best_particle_id = i;
		}

		resampled_particle_filter->particles[i].weight = M_inv;

//		free(particle_filter->particles[i].particle_map);
//		free(particle_filter->particles[i].particle_map_occupation);
	}

//	free(particle_filter->particles);
//	(*particle_filter) = (*resampled_particle_filter);
//	free(resampled_particle_filter);
	switch_data_from_resample_to_particle_filter(particle_filter, resampled_particle_filter);

	particle_filter->best_particle = &(particle_filter->particles[best_particle_id]);
}


void
carmen_polar_slam_correct_pose_using_beam_range_finder_model(carmen_polar_particle_filter* particle_filter, double *range, double *angles, int num_rays)
{
	int i;
	int num_particles = particle_filter->config.num_particles;
	double particle_weight_in_abscense_of_map = 1 / ((double) num_particles);

	//
	// if the first map has already been received, the particle weight is calculated in the
	// conventional way, using the beam_range_finder_model. if not, however, all the particles
	// receive the same weight
	//

	if (particle_filter->config.first_map_has_been_received)
		carmen_polar_slam_calculate_particle_weights(particle_filter, range, angles, num_rays);
	else
		for(i = 0; i < num_particles; i++)
			particle_filter->particles[i].weight = particle_weight_in_abscense_of_map;

	carmen_polar_slam_resample_particles(particle_filter);
}


void
remove_points_in_ray_direction(carmen_polar_particle_filter *particle_filter, int particle, int sphere_id, int section_id)
{
	int i, j, point_list_position;
	int point_to_remove;
	int num_points_stored;

	num_points_stored = particle_filter->config.num_points_per_sections;

	for (i = 0; i < sphere_id; i++)
	{
		point_list_position = carmen_polar_slam_get_point_position(particle_filter, sphere_id, section_id);

		// ********************************
		// remove all points in the section
		// ********************************
//		memset(particle_filter->particles[particle].particle_map_occupation + point_list_position, 0, num_points_stored);
//		memset(particle_filter->particles[particle].particle_map + point_list_position, 0, num_points_stored);

		// ************************************************************
		// remove a random point in the sections transversed by the ray
		// ************************************************************
		point_to_remove = rand() % num_points_stored;

		// move the choosen point to the end of the list
		for (j = point_to_remove; j < (num_points_stored - 1); j++)
		{
			particle_filter->particles[particle].particle_map_occupation[point_list_position + j] = particle_filter->particles[particle].particle_map_occupation[point_list_position + j + 1];
			particle_filter->particles[particle].particle_map[point_list_position + j].radius = particle_filter->particles[particle].particle_map[point_list_position + j + 1].radius;
			particle_filter->particles[particle].particle_map[point_list_position + j].angle = particle_filter->particles[particle].particle_map[point_list_position + j + 1].angle;
		}

		// clear the point
		particle_filter->particles[particle].particle_map[point_list_position + num_points_stored - 1].angle = 0.0;
		particle_filter->particles[particle].particle_map[point_list_position + num_points_stored - 1].radius = 0.0;
		particle_filter->particles[particle].particle_map_occupation[point_list_position + num_points_stored - 1] = 0;
	}
}


void
carmen_polar_slam_add_laser_reading_to_particles_map(carmen_polar_particle_filter *particle_filter, double angle, double radius)
{
	int i, num_particles, sphere_id, section_id, point_list_position, num_points_per_section;

	num_particles = particle_filter->config.num_particles;
	num_points_per_section = particle_filter->config.num_points_per_sections;

	sphere_id = carmen_polar_slam_get_sphere_index_by_radius(particle_filter, radius);
	section_id = carmen_polar_slam_get_section_index_by_angle(particle_filter, angle);
	point_list_position = carmen_polar_slam_get_point_position(particle_filter, sphere_id, section_id);

	// if the point is out of the limits of the spheres
	if (sphere_id < 0 || sphere_id >= particle_filter->config.num_spheres || radius >= particle_filter->config.max_range)
		return;
	else
	{
		for(i = 0; i < num_particles; i++)
		{
			remove_points_in_ray_direction(particle_filter, i, sphere_id, section_id);
			carmen_polar_slam_add_point_to_particle_map(&(particle_filter->particles[i]), num_points_per_section, point_list_position, radius, angle);
		}
	}
}


void
carmen_polar_slam_add_obstacles_to_map(carmen_polar_particle_filter* particle_filter, double *ranges, double *angles, int num_rays)
{
	int i;
	double angle, range;

	for(i = 0; i < num_rays; i += 1)
	{
		range = ranges[i];
		angle = angles[i];

		carmen_polar_slam_add_laser_reading_to_particles_map(particle_filter, angle, range);
	}

	particle_filter->config.first_map_has_been_received = 1;
}


void
carmen_polar_slam_predict_pose_using_motion_model(carmen_polar_particle_filter* particle_filter, OdometryMotionCommand *ut, carmen_fused_odometry_message *odometry)
{
	int i, num_particles;
	carmen_point_t odometry_2D;
	carmen_point_t last_particle_pose, predicted_particle_pose;

	odometry_2D.x = odometry->pose.position.x;
	odometry_2D.y = odometry->pose.position.y;
	odometry_2D.theta = odometry->pose.orientation.yaw;

	update_odometry_motion_command(ut, odometry_2D);

	num_particles = particle_filter->config.num_particles;

	for(i = 0; i < num_particles; i++)
	{
		last_particle_pose.x = particle_filter->particles[i].particle_pose.position.x;
		last_particle_pose.y = particle_filter->particles[i].particle_pose.position.y;
		last_particle_pose.theta = particle_filter->particles[i].particle_pose.orientation.yaw;

		// TODO: checar pq a predicted pose esta sempre igual a last particle aqui
		predicted_particle_pose = sample_motion_model_odometry(ut, last_particle_pose);

		particle_filter->particles[i].pose_before_movement = particle_filter->particles[i].particle_pose;

		particle_filter->particles[i].particle_pose.position.x = predicted_particle_pose.x;
		particle_filter->particles[i].particle_pose.position.y = predicted_particle_pose.y;
		particle_filter->particles[i].particle_pose.orientation.yaw = predicted_particle_pose.theta;
	}
}




