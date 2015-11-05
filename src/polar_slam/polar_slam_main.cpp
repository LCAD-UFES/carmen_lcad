
/**
 *
 * Coisas para melhorar:
 *
 * >> TODO: trocar as carmen_base_ackerman_odometry_message por fused odometry ou outro tipo para deixar o modulo preparado para lidar com odometrias em 3D
 * >> TODO: o slam montecarlo inicializa as particulas com a pose que vem do localize e com deviation zero... ver se isso eh aplicavel aqui tambem!
 * >> TODO: fazer o destrutor da classe Particle limpar possiveis coisas alocadas tipo o mapa da particula
 * >> TODO: os pontos criticos do polar slam sao: a movimentacao do mapa que precisa ser feita para cada uma das 50 particulas, a adicao de novos obstaculos que precisa ser feita numero_de_leituras_do_laser * numero_de_particulas * numero_de_pontos_armazenados_por_secao_esferica e a copia das particulas durante e resample.
 * >> TODO: ver quais os parametros do laser que estao sendo usados e remove os outros da lista lida do param_daemon
 * >> TODO: na funcao calculate_particle_probability estou colocando o max_range da laser_message quando o ray_cast retorna max_range, eu deveria colocar o max_range do laser_model_params, mas ele esta nesse arquivo... pensar em como passar essa informacao para o particle filter. talvez seja uma boa colocar fazer as incializacoes desses caras no particle filter...
 *
 */

#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <carmen/carmen.h>
#include <carmen/polar_slam_interface.h>
#include <carmen/laser_interface.h>
#include <prob_motion_model.h>
#include <prob_measurement_model.h>

#include "polar_map.h"
#include "particle_filter.h"
#include "fast_polar_particle_filter.h"

using namespace std;

typedef struct
{
	int num_spheres;
	int num_particles;
	int num_points_to_store;
	int num_angular_sections;

}PolarSlamParams;

carmen_pose_3D_t starting_pose;
carmen_polar_particle_filter *fast_particle_filter;


// parameters of the module
PolarSlamParams polar_slam_params;


// interface de visualizacao
IplImage *polar_map_view = NULL;
int polar_map_view_size = 600; // pixels


// pilha para sincronizacao de mensagens
vector<carmen_base_ackerman_odometry_message> odometry_queue;
vector<double> odometry_timestamps;


// slam
int odometry_is_initialized = 0;
ParticleFilter localize_particle_filter;
OdometryMotionCommand *ut = NULL;
OdometryMotionModelParams odometry_model_params;
BeanRangeFinderMeasurementModelParams laser_model_params;


// debug
struct timeval start, end;


void
start_count_time()
{
	gettimeofday(&start, NULL);
}


void
show_time_ellapsed()
{
	gettimeofday(&end, NULL);
	double ellapsed = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_usec - start.tv_usec);
	printf(" time diff: %.6lf s\n", ellapsed / 1000000.0);
}


void
shutdown_module(int signo)
{
	if(signo == SIGINT)
	{
		carmen_ipc_disconnect();
		cvDestroyAllWindows();

		printf("polar slam module: disconnected.\n");
		exit(0);
	}
}


carmen_base_ackerman_odometry_message
fuse_laser_and_odometry_using_timestamp(carmen_laser_laser_message *laser_message)
{
	int i, first_iteraction;
	double min_timestamp_diff, timestamp_diff;

	carmen_base_ackerman_odometry_message laser_pose;
	memset(&laser_pose, 0, sizeof(carmen_pose_3D_t));

	first_iteraction = 1;

	for (i = 0; i < (int) odometry_queue.size(); i++)
	{
		timestamp_diff = fabs(odometry_timestamps[i] - laser_message->timestamp);

		if ((first_iteraction) || (timestamp_diff < min_timestamp_diff))
		{
			min_timestamp_diff = timestamp_diff;
			laser_pose = odometry_queue[i];

			first_iteraction = 0;
		}
	}

	return laser_pose;
}


void
add_new_odometry_information_to_the_model(carmen_base_ackerman_odometry_message odometry)
{
	carmen_point_t odometry_2D;

	odometry_2D.x = odometry.x;
	odometry_2D.y = odometry.y;
	odometry_2D.theta = odometry.theta;

	update_odometry_motion_command(ut, odometry_2D);
}


void
predict_pose(carmen_base_ackerman_odometry_message odometry)
{
	add_new_odometry_information_to_the_model(odometry);
	localize_particle_filter.predict_pose_using_motion_model(ut);
}


void
correct_pose(carmen_laser_laser_message *laser_message)
{
	localize_particle_filter.correct_pose_using_beam_range_finder_model(laser_message);
}


void
localization(carmen_laser_laser_message *laser_message, carmen_base_ackerman_odometry_message odometry)
{
	predict_pose(odometry);
	correct_pose(laser_message);
}


void
mapping(carmen_laser_laser_message *laser_message)
{
	localize_particle_filter.add_obstacles_to_particles_map(laser_message);
}


void
draw_particles(Particle best_particle)
{
	double pixels_per_meter = ((double) polar_map_view->width) / (2 * pow(2.0, 6));
	vector<Particle> particles = localize_particle_filter.get_particles();

	for(unsigned int i = 0; i < particles.size(); i++)
	{
		double diff_x = particles[i].pose.position.x - best_particle.pose.position.x;
		double diff_y = particles[i].pose.position.y - best_particle.pose.position.y;

		int img_x = (int) (diff_x * pixels_per_meter + 0.5) + (polar_map_view->width / 2);
		int img_y = (int) (diff_y * pixels_per_meter + 0.5) + (polar_map_view->height / 2);

		cvCircle(polar_map_view, cvPoint(img_x, img_y), 2, cvScalar(0, 0, 255, 0), 1, 0, 0);
	}
}


void
view()
{
	Particle best_particle = localize_particle_filter.get_best_particle();
	best_particle.map.draw(polar_map_view);

	double pixels_per_meter = ((double) polar_map_view->width) / (2 * pow(2.0, 6));

	double radius = localize_particle_filter.get_distance_to_farthest_particle(best_particle);
	cvCircle(polar_map_view, cvPoint(polar_map_view->width / 2, polar_map_view->height / 2), radius * pixels_per_meter, cvScalar(0, 255, 0, 0), 1, 1, 0);

	draw_particles(best_particle);

	printf("particle radius: %lf\t", radius);
}


void
slam(carmen_laser_laser_message *laser_message, carmen_base_ackerman_odometry_message odometry)
{
	if (!odometry_is_initialized)
		return;

	localization(laser_message, odometry);
	mapping(laser_message);
}


static void
carmen_laser_laser_message_handler(carmen_laser_laser_message *laser_message)
{
	start_count_time();

	carmen_base_ackerman_odometry_message odometry = fuse_laser_and_odometry_using_timestamp(laser_message);
	slam(laser_message, odometry);
	view();

	show_time_ellapsed();
}


void
create_random_particles(double x, double y, double theta)
{
	carmen_pose_3D_t mean, std;

	memset(&mean, 0, sizeof(mean));
	memset(&std, 0, sizeof(mean));

	mean.position.x = x;
	mean.position.y = y;
	mean.orientation.yaw = theta;

	localize_particle_filter.generate_particles(mean, std);
}


static void
carmen_base_ackerman_odometry_handler(carmen_base_ackerman_odometry_message *odometry_message)
{
	if (!odometry_is_initialized)
	{
		create_random_particles(odometry_message->x, odometry_message->y, odometry_message->theta);

		ut->initial.x = ut->final.x = odometry_message->x;
		ut->initial.y = ut->final.y = odometry_message->y;
		ut->initial.theta = ut->final.theta = odometry_message->theta;

		memset(&starting_pose, 0, sizeof(starting_pose));

		starting_pose.position.x = odometry_message->x;
		starting_pose.position.y = odometry_message->y;
		starting_pose.orientation.yaw = odometry_message->theta;

		odometry_is_initialized = 1;
	}

	odometry_queue.push_back(*odometry_message);
	odometry_timestamps.push_back(odometry_message->timestamp);
}


void
carmen_polar_slam_subscribe_messages()
{
	carmen_laser_subscribe_frontlaser_message(NULL,
			(carmen_handler_t) carmen_laser_laser_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_base_ackerman_subscribe_odometry_message(NULL,
			(carmen_handler_t) carmen_base_ackerman_odometry_handler,
			CARMEN_SUBSCRIBE_LATEST);
}


void
initialize_slam()
{
	localize_particle_filter = ParticleFilter(
		polar_slam_params.num_particles,
		polar_slam_params.num_spheres,
		polar_slam_params.num_angular_sections,
		polar_slam_params.num_points_to_store
	);
}


void
initialize_motion_model()
{
	//
	// inicializa pose corrente e a anterior vindas da odometria com zero
	//
	 ut = (OdometryMotionCommand *) calloc (1, sizeof(OdometryMotionCommand));
	 init_odometry_motion_model(odometry_model_params);
}


void
initialize_beam_range_finder_model()
{
	init_bean_range_finder_measurement_model(laser_model_params);
}


void
carmen_polar_slam_initialize_global_data()
{
	polar_map_view = cvCreateImage(cvSize(polar_map_view_size, polar_map_view_size), IPL_DEPTH_8U, 3);
	memset(polar_map_view->imageData, 255, polar_map_view->imageSize);

	initialize_slam();
	initialize_motion_model();
	initialize_beam_range_finder_model();
}


void
carmen_polar_slam_read_parameters(int argc, char *argv[])
{
	carmen_param_t basic_param_list[] =
	{
		{(char *) "polar_slam", (char *) "odom_a1", CARMEN_PARAM_DOUBLE, &(odometry_model_params.alpha1), 1, NULL},
		{(char *) "polar_slam", (char *) "odom_a2", CARMEN_PARAM_DOUBLE, &(odometry_model_params.alpha2), 1, NULL},
		{(char *) "polar_slam", (char *) "odom_a3", CARMEN_PARAM_DOUBLE, &(odometry_model_params.alpha3), 1, NULL},
		{(char *) "polar_slam", (char *) "odom_a4", CARMEN_PARAM_DOUBLE, &(odometry_model_params.alpha4), 1, NULL},
	};

	carmen_param_t laser_param_list[] =
	{
		{(char *) "polar_slam", (char *) "laser_sampling_step", CARMEN_PARAM_INT, 	&(laser_model_params.sampling_step), 0, NULL},
		{(char *) "polar_slam", (char *) "laser_num_beams", CARMEN_PARAM_INT, 		&(laser_model_params.laser_beams), 0, NULL},
		{(char *) "polar_slam", (char *) "laser_fov_range", CARMEN_PARAM_DOUBLE, 	&(laser_model_params.fov_range), 0, NULL},
		{(char *) "polar_slam", (char *) "laser_max_range", CARMEN_PARAM_DOUBLE, 	&(laser_model_params.max_range), 0, NULL},
		{(char *) "polar_slam", (char *) "laser_lambda_short", CARMEN_PARAM_DOUBLE, &(laser_model_params.lambda_short), 0, NULL},
		{(char *) "polar_slam", (char *) "laser_sigma_zhit", CARMEN_PARAM_DOUBLE, 	&(laser_model_params.sigma_zhit), 0, NULL},
		{(char *) "polar_slam", (char *) "laser_zhit", CARMEN_PARAM_DOUBLE, 		&(laser_model_params.zhit), 0, NULL},
		{(char *) "polar_slam", (char *) "laser_zmax", CARMEN_PARAM_DOUBLE, 		&(laser_model_params.zmax), 0, NULL},
		{(char *) "polar_slam", (char *) "laser_zrand", CARMEN_PARAM_DOUBLE, 		&(laser_model_params.zrand), 0, NULL},
		{(char *) "polar_slam", (char *) "laser_zshort", CARMEN_PARAM_DOUBLE, 		&(laser_model_params.zshort), 0, NULL},
		{(char *) "robot", (char *) "frontlaser_offset", CARMEN_PARAM_DOUBLE, 		&(laser_model_params.front_offset), 0, NULL},
		{(char *) "robot", (char *) "frontlaser_side_offset", CARMEN_PARAM_DOUBLE, 	&(laser_model_params.side_offset), 0, NULL},
		{(char *) "robot", (char *) "frontlaser_angular_offset", CARMEN_PARAM_DOUBLE, &(laser_model_params.angular_offset), 0, NULL},
	};

	carmen_param_t polar_slam_param_list[] =
	{
		{(char *) "polar_slam", (char *) "num_spheres", 		CARMEN_PARAM_INT, &(polar_slam_params.num_spheres), 0, NULL},
		{(char *) "polar_slam", (char *) "num_particles", 			CARMEN_PARAM_INT, &(polar_slam_params.num_particles), 0, NULL},
		{(char *) "polar_slam", (char *) "num_points_to_store", 	CARMEN_PARAM_INT, &(polar_slam_params.num_points_to_store), 0, NULL},
		{(char *) "polar_slam", (char *) "num_angular_sections", 	CARMEN_PARAM_INT, &(polar_slam_params.num_angular_sections), 0, NULL}
	};


	carmen_param_install_params(argc, argv, basic_param_list, sizeof(basic_param_list) / sizeof(basic_param_list[0]));
	carmen_param_install_params(argc, argv, laser_param_list, sizeof(laser_param_list) / sizeof(laser_param_list[0]));
	carmen_param_install_params(argc, argv, polar_slam_param_list, sizeof(polar_slam_param_list) / sizeof(polar_slam_param_list[0]));
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);

	carmen_polar_slam_read_parameters(argc, argv);
	carmen_polar_slam_initialize_global_data();
	carmen_polar_slam_subscribe_messages();

	while(1)
	{
		carmen_ipc_sleep(0.1);

		cvShowImage("polar map view", polar_map_view);

		if ((cvWaitKey(10) & 255) == 27)
			break;
	}

	return 0;
}
