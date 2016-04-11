
#include <opencv/cv.h>
#include <carmen/carmen.h>
#include <opencv/highgui.h>
#include <carmen/grid_mapping_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/robot_ackerman_interface.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_index.h>

using namespace cv;


const double ROBOT_WIDTH = 1.6;
const double ROBOT_LENGTH = 4.437;
const double ROBOT_DISTANCE_BETWEEN_REAR_CAR_AND_REAR_WHEELS = 0.78;
const int GOAL_LIST_STEP = 6; // draw one pose after each 'GOAL_LIST_STEP' number of poses
const int NUM_MOTION_COMMANDS = 10;
const double MAX_SPEED = 10.0;
const double MAX_PHI = carmen_degrees_to_radians(28);

int car_already_accelerated_in_this_experiment = 0;

// The variable rddf_index_already_used_in_the_experiment
// is used to mark the rddf positions that already generated
// positive reinforcements to the car in the current experiment.
// If it is 0, then the associated pose in the rddf_index was
// not used yet. If it is 1, then the pose was already used.
char *rddf_index_already_used_in_the_experiment;

carmen_timestamp_index *rddf_index;
carmen_rddf_road_profile_message rddf_message;
carmen_localize_ackerman_globalpos_message localize_ackerman_message;


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("dqn_emulator: disconnected.\n");

		exit(0);
	}
}


void
rotate(cv::Mat& src, double angle, cv::Mat& dst)
{
	int len = std::max(src.cols, src.rows);

	cv::Point2f pt(len / 2., len / 2.);
	cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);

	cv::warpAffine(src, dst, r, cv::Size(len, len));
}


double
x_coord(double x, double y, double pose_x, double pose_theta)
{
	return x * cos(pose_theta) - y *sin(pose_theta) + pose_x;
}


double
y_coord(double x, double y, double pose_y, double pose_theta)
{
	return x * sin(pose_theta) + y *cos(pose_theta) + pose_y;
}


void
draw_ackerman_shape(Mat *map_image,
		double pose_x, double pose_y, double pose_theta,
		carmen_grid_mapping_message *map_message,
		Scalar color)
{
	static cv::Point *polygons;
	static int first = 1;

	if (first)
	{
		polygons = (cv::Point *) calloc (1, sizeof(cv::Point));
		first = 0;
	}

	double half_width, length, dist_rear_car_rear_wheels;

	dist_rear_car_rear_wheels = ROBOT_DISTANCE_BETWEEN_REAR_CAR_AND_REAR_WHEELS;
	half_width = ROBOT_WIDTH / 2;
	length = ROBOT_LENGTH;

	polygons[0].y = (int) ((x_coord(-dist_rear_car_rear_wheels, half_width, pose_x, pose_theta) - map_message->config.x_origin) / map_message->config.resolution);
	polygons[0].x = (int) ((y_coord(-dist_rear_car_rear_wheels, half_width, pose_y, pose_theta) - map_message->config.y_origin) / map_message->config.resolution);
	polygons[1].y = (int) ((x_coord(-dist_rear_car_rear_wheels, -half_width, pose_x, pose_theta) - map_message->config.x_origin) / map_message->config.resolution);
	polygons[1].x = (int) ((y_coord(-dist_rear_car_rear_wheels, -half_width, pose_y, pose_theta) - map_message->config.y_origin) / map_message->config.resolution);
	polygons[2].y = (int) ((x_coord(length - dist_rear_car_rear_wheels, -half_width, pose_x, pose_theta) - map_message->config.x_origin) / map_message->config.resolution);
	polygons[2].x = (int) ((y_coord(length - dist_rear_car_rear_wheels, -half_width, pose_y, pose_theta) - map_message->config.y_origin) / map_message->config.resolution);
	polygons[3].y = (int) ((x_coord(length - dist_rear_car_rear_wheels, half_width, pose_x, pose_theta) - map_message->config.x_origin) / map_message->config.resolution);
	polygons[3].x = (int) ((y_coord(length - dist_rear_car_rear_wheels, half_width, pose_y, pose_theta) - map_message->config.y_origin) / map_message->config.resolution);

	fillConvexPoly(*map_image, (const cv::Point *) polygons, 4, color);
	fillConvexPoly(*map_image, (const cv::Point *) polygons, 4, color);
}


Mat
map_to_image(carmen_grid_mapping_message *message)
{
	double map_probability;
	Mat map_image = Mat(Size(message->config.x_size, message->config.y_size), CV_8UC3);

	for (int i = 0; i < message->config.y_size; i++)
	{
		for (int j = 0; j < message->config.x_size; j++)
		{
			map_probability = message->complete_map[i * message->config.x_size + j];

			if (map_probability < 0)
			{
				map_image.data[3 * (i * message->config.x_size + j) + 0] = 255;
				map_image.data[3 * (i * message->config.x_size + j) + 1] = 0;
				map_image.data[3 * (i * message->config.x_size + j) + 2] = 0;
			}
			else
			{
				if (map_probability > 1.0) map_probability = 1.0;

				// probs close to 1 mean high probability of occupancy. I inverted it so that
				// when multiplying for 255 to draw the image pixels, the obstacles are
				// drawn dark, and the free areas bright.
				map_probability = 1.0 - map_probability;

				map_image.data[3 * (i * message->config.x_size + j) + 0] = 255 * map_probability;
				map_image.data[3 * (i * message->config.x_size + j) + 1] = 255 * map_probability;
				map_image.data[3 * (i * message->config.x_size + j) + 2] = 255 * map_probability;
			}
		}
	}

	return map_image;
}


void
draw_rddf_in_the_map(Mat *map_image, carmen_rddf_road_profile_message *rddf, carmen_grid_mapping_message *map_message)
{
	for (int i = 0; i < rddf->number_of_poses; i++)
	{
		// ignore the first GOAL_LIST_STEP poses to avoid drawing
		// goals too close to the robot.
		if (i < GOAL_LIST_STEP) continue;

		if (i % GOAL_LIST_STEP == 0)
		{
			draw_ackerman_shape(map_image, rddf->poses[i].x, rddf->poses[i].y, rddf->poses[i].theta,
				map_message, Scalar(0, 255, 0));
		}
	}
}


Mat
map_image_post_processing(Mat *map_image)
{
	double reduction_factor = 2;

	Mat resized_map = Mat(Size(map_image->cols / reduction_factor, map_image->rows / reduction_factor), CV_8UC3);

	resize(*map_image, resized_map, resized_map.size());
	rotate(resized_map, 90, resized_map); // to make it look like in navigator_gui2

	return resized_map;
}


void
draw_globalpose_in_the_map(Mat *map_image,
		carmen_localize_ackerman_globalpos_message *localize_ackerman_message,
		carmen_grid_mapping_message *map_message)
{
	draw_ackerman_shape(map_image, localize_ackerman_message->globalpos.x,
			localize_ackerman_message->globalpos.y,
			localize_ackerman_message->globalpos.theta,
			map_message, Scalar(0, 0, 255));
}


void
publish_test_motion_command()
{
	static int first = 1;
	static carmen_ackerman_motion_command_t *motion_commands;

	if (first)
	{
		motion_commands = (carmen_ackerman_motion_command_t *) calloc (NUM_MOTION_COMMANDS, sizeof(carmen_ackerman_motion_command_t));
		first = 0;
	}

	for (int i = 0; i < NUM_MOTION_COMMANDS; i++)
	{
		motion_commands[i].time = 0.05 * i;
		motion_commands[i].v = MAX_SPEED;
		motion_commands[i].phi = 0.0;
	}

	carmen_robot_ackerman_publish_motion_command(motion_commands, NUM_MOTION_COMMANDS);
}


void
reset_experiment()
{
	carmen_point_t pose;
	carmen_point_t std;

	pose.x = 7757865.0;
	pose.y = -363543.0;
	pose.theta = carmen_degrees_to_radians(120);

	std.x = 0.1;
	std.y = 0.1;
	std.theta = carmen_degrees_to_radians(1);

	carmen_localize_ackerman_initialize_gaussian_command(pose, std);

	/** Reinitialize experiment configuration parameters **/
	car_already_accelerated_in_this_experiment = 0;
	memset(rddf_index_already_used_in_the_experiment, 0, rddf_index->size() * sizeof(char));
}


void
grid_mapping_handler(carmen_grid_mapping_message *message)
{
	// wait for rddf message
	if (rddf_message.number_of_poses == 0) return;

	Mat map_image = map_to_image(message);

	draw_rddf_in_the_map(&map_image, &rddf_message, message);
	draw_globalpose_in_the_map(&map_image, &localize_ackerman_message, message);

	Mat final_map_image = map_image_post_processing(&map_image);

	// ** for debug:
	// imshow("map", final_map_image);
	// waitKey(1);

	publish_test_motion_command();

	if (localize_ackerman_message.v > 0.5)
		car_already_accelerated_in_this_experiment = 1;
	else if (localize_ackerman_message.v == 0 && car_already_accelerated_in_this_experiment)
	{
		// summarize experiment and reward/punish network
		reset_experiment();
	}
}


void
rddf_handler(carmen_rddf_road_profile_message *message __attribute__ ((unused)))
{
}


void
localize_ackerman_handler(carmen_localize_ackerman_globalpos_message *message __attribute__ ((unused)))
{
}


void
initialize_global_structures()
{
	memset(&rddf_message, 0, sizeof(rddf_message));
	rddf_index_already_used_in_the_experiment = (char *) calloc (rddf_index->size(), sizeof(char));
}


void
define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_NAME);
}


void
subscribe_messages()
{
	carmen_grid_mapping_subscribe_message(NULL, (carmen_handler_t) grid_mapping_handler, CARMEN_SUBSCRIBE_LATEST);
    carmen_rddf_subscribe_road_profile_message(&rddf_message, (carmen_handler_t) rddf_handler, CARMEN_SUBSCRIBE_LATEST);
    carmen_localize_ackerman_subscribe_globalpos_message(&localize_ackerman_message, (carmen_handler_t) localize_ackerman_handler, CARMEN_SUBSCRIBE_LATEST);
}


int
main(int argc, char **argv)
{
	char *rddf_filename;

	if (argc < 2)
	{
		printf("Use %s <rddf file>\n", argv[0]);
		return 0;
	}
	else
		rddf_filename = argv[1];

	if (!carmen_rddf_index_exists(rddf_filename))
		exit(printf("Error: rddf file or index files don't exist (this program doesn't generate indices)\n"));

	carmen_rddf_load_index(rddf_filename);
	rddf_index = get_timestamp_index();

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_module);

	initialize_global_structures();
	reset_experiment();
	define_messages();
	subscribe_messages();
	carmen_ipc_dispatch();

	return (0);
}


