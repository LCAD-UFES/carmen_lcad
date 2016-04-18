
#include <opencv/cv.h>
#include <carmen/carmen.h>
#include <opencv/highgui.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/robot_ackerman_interface.h>
#include <carmen/grid_mapping_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/collision_detection.h>
#include <carmen/rddf_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/rddf_index.h>
#include <prob_map.h>

#include <caffe/caffe.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <boost/smart_ptr.hpp>
#include "dqn_caffe.h"

using namespace cv;

/** INTERNAL PARAMETERS - BEGIN **/
const int NUM_MOTION_COMMANDS = 10;
const double MAX_SPEED = 10.0;
const double MAX_PHI = carmen_degrees_to_radians(28);
double SPEED_UPDATE = 10.0;
double STEERING_UPDATE = carmen_degrees_to_radians(28); //carmen_degrees_to_radians(0.5);

const int GOAL_LIST_STEP = 1; //6; // draw one pose after each 'GOAL_LIST_STEP' number of poses

const double PARTIAL_GOAL_REWARD = 5;
const double FINAL_GOAL_REWARD = 100;
const double DISTANCE_PUNISHMENT_PER_METER = -0.1;
const double TIME_PUNISHMENT_PER_SECOND = -0.1;
const double COLLISION_PUNISHMENT = -100;

int NUM_POSES_TO_SEARCH_AROUND_CLOSEST_GOAL = 10;
double MIN_DIST_TO_CONSIDER_ACHIEVEMENT = 0.1;
/** INTERNAL PARAMETERS - END **/

double agent_reward = 0;
long rddf_index_final_goal = 0;
long rddf_index_last_pose = 0;
int achieved_goal_in_this_experiment = 0;
int car_already_accelerated_in_this_experiment = 0;
int collision_detected = 0;
carmen_ackerman_motion_command_t current_command = {0, 0, 0};
DqnAction current_action = DQN_ACTION_NONE;
double epsilon = 0;
InputFrames input_frames;

carmen_robot_ackerman_config_t carmen_robot_ackerman_config;

// The variable rddf_index_already_used_in_the_experiment
// is used to mark the rddf positions that already generated
// positive reinforcements to the car in the current experiment.
// If it is 0, then the associated pose in the rddf_index was
// not used yet. If it is 1, then the pose was already used.
char *rddf_index_already_used_in_the_experiment;

carmen_timestamp_index *rddf_index;
carmen_rddf_road_profile_message rddf_message;
carmen_localize_ackerman_globalpos_message localize_ackerman_message;
carmen_map_server_compact_cost_map_message compact_cost_map_message;
carmen_compact_map_t compact_cost_map;
carmen_map_t cost_map;

DqnCaffe *dqn_caffe;



void
publish_motion_command(double v, double phi)
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
			motion_commands[i].v = v;
			motion_commands[i].phi = phi;
	}

	carmen_robot_ackerman_publish_motion_command(motion_commands, NUM_MOTION_COMMANDS);
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		publish_motion_command(0, 0);

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

	dist_rear_car_rear_wheels = carmen_robot_ackerman_config.distance_between_rear_car_and_rear_wheels;
	half_width = carmen_robot_ackerman_config.width / 2;
	length = carmen_robot_ackerman_config.length;

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


Mat*
cost_map_to_image(carmen_map_t *message)
{
	int i, j;
	static Mat *map_image = NULL;

	if (map_image == NULL)
		map_image = new Mat(Size(message->config.x_size, message->config.y_size), CV_8UC3);

	#pragma omp parallel for default(none) shared(message, map_image) private(i, j)
	for (i = 0; i < message->config.y_size; i++)
	{
		for (j = 0; j < message->config.x_size; j++)
		{
			double map_probability = message->complete_map[i * message->config.x_size + j];

			if (map_probability < 0)
			{
				map_image->data[3 * (i * message->config.x_size + j) + 0] = 255;
				map_image->data[3 * (i * message->config.x_size + j) + 1] = 0;
				map_image->data[3 * (i * message->config.x_size + j) + 2] = 0;
			}
			else
			{
				if (map_probability > 1.0) map_probability = 1.0;

				// probs close to 1 mean high probability of occupancy. I inverted it so that
				// when multiplying for 255 to draw the image pixels, the obstacles are
				// drawn dark, and the free areas bright.
				map_probability = 1.0 - map_probability;

				map_image->data[3 * (i * message->config.x_size + j) + 0] = 255 * map_probability;
				map_image->data[3 * (i * message->config.x_size + j) + 1] = 255 * map_probability;
				map_image->data[3 * (i * message->config.x_size + j) + 2] = 255 * map_probability;
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
draw_final_goal_in_the_map(Mat *map_image, carmen_grid_mapping_message *map_message)
{
	draw_ackerman_shape(map_image, rddf_index->index[rddf_index_final_goal].x,
				rddf_index->index[rddf_index_final_goal].y,
				rddf_index->index[rddf_index_final_goal].yaw,
				map_message, Scalar(255, 0, 0));
}


long
find_closest_pose_in_the_index(carmen_point_t *initial_pose)
{
	return find_timestamp_index_position_with_full_index_search(initial_pose->x, initial_pose->y, initial_pose->theta, 0);
}


void
reinitize_experiment_configuration(carmen_point_t *initial_pose)
{
	// set all fields of the structure to 0. note: I use this in some function to check if some action is necessary.
	// do not erase without checking the other functions.
	memset(rddf_index_already_used_in_the_experiment, 0, rddf_index->size() * sizeof(char));

	// I did it to force the next iteration of the algorithm to wait for a new
	// globalpos message after reinitialization
	memset(&localize_ackerman_message, 0, sizeof(localize_ackerman_message));

	agent_reward = 0;
	collision_detected = 0;
	achieved_goal_in_this_experiment = 0;
	car_already_accelerated_in_this_experiment = 0;
	current_action = DQN_ACTION_NONE;
	// set all fields of the structure to 0
	memset(&current_command, 0, sizeof(current_command));
	input_frames.clear();
	epsilon = 1.0;

	rddf_index_last_pose = find_closest_pose_in_the_index(initial_pose);
	rddf_index_final_goal = rddf_index_last_pose + 70;

// @debug:
//	printf("dx: %lf dy: %lf dist: %lf\n", initial_pose->x - rddf_index->index[rddf_index_last_pose].x,
//			initial_pose->y - rddf_index->index[rddf_index_last_pose].y,
//			sqrt(pow(initial_pose->x - rddf_index->index[rddf_index_last_pose].x, 2) + pow(initial_pose->y - rddf_index->index[rddf_index_last_pose].y, 2)));

}


void
reset_experiment()
{
	carmen_point_t pose;
	carmen_point_t std;

	pose.x = 7757880.0;
	pose.y = -363535.0;
	pose.theta = carmen_degrees_to_radians(140);

	std.x = 0.0001;
	std.y = 0.0001;
	std.theta = carmen_degrees_to_radians(0.01);

	carmen_localize_ackerman_initialize_gaussian_command(pose, std);
	reinitize_experiment_configuration(&pose);
}


Mat*
convert_map_to_image(carmen_grid_mapping_message *message)
{
	//Mat map_image = map_to_image(message);
	Mat *map_image = cost_map_to_image(&cost_map);

	draw_rddf_in_the_map(map_image, &rddf_message, message);
	draw_globalpose_in_the_map(map_image, &localize_ackerman_message, message);
	draw_final_goal_in_the_map(map_image, message);

	// ** for debug:
	Mat resized_map = Mat(Size(map_image->cols / 4, map_image->rows / 4), CV_8UC1);
	resize(*map_image, resized_map, resized_map.size());
	imshow("map", resized_map);
	waitKey(1);

	return map_image;
}


int
agent_achieved_partial_goal()
{
	int i = rddf_index_last_pose - NUM_POSES_TO_SEARCH_AROUND_CLOSEST_GOAL;

	double dist;
	double min_dist = DBL_MAX;
	int min_dist_id = i;

	while ((i < rddf_index_last_pose + NUM_POSES_TO_SEARCH_AROUND_CLOSEST_GOAL) && (i >= 0) && (i < rddf_index->size()))
	{
		dist = sqrt(pow(rddf_index->index[i].x - localize_ackerman_message.globalpos.x, 2) +
				pow(rddf_index->index[i].y - localize_ackerman_message.globalpos.y, 2));

		if (dist < min_dist)
		{
			min_dist = dist;
			min_dist_id = i;
		}

		i++;
	}

	rddf_index_last_pose = min_dist_id;

	// if the agent is close enough to the goal and the closest goal does not generated punctuation yet
	if (min_dist < MIN_DIST_TO_CONSIDER_ACHIEVEMENT && !rddf_index_already_used_in_the_experiment[min_dist_id])
	{
		rddf_index_already_used_in_the_experiment[min_dist_id] = 1;
		return 1;
	}
	else
		return 0;
}


int
agent_achieved_final_goal()
{
	if (rddf_index_last_pose == rddf_index_final_goal && !achieved_goal_in_this_experiment)
	{
		double dist = sqrt(pow(rddf_index->index[rddf_index_last_pose].x - localize_ackerman_message.globalpos.x, 2)
				+ pow(rddf_index->index[rddf_index_last_pose].y - localize_ackerman_message.globalpos.y, 2));

		if (dist < MIN_DIST_TO_CONSIDER_ACHIEVEMENT)
		{
			achieved_goal_in_this_experiment = 1;
			return 1;
		}
	}

	return 0;
}


int
car_hit_obstacle()
{
	return obstacle_avoider_pose_hit_obstacle(localize_ackerman_message.globalpos, &cost_map, &carmen_robot_ackerman_config);
}


void
update_agent_reward()
{
	static carmen_point_t last_pose = {0, 0, 0};
	static double last_time = 0;

	double distance_traveled;
	double time_difference;

	if (agent_achieved_partial_goal())
		agent_reward += PARTIAL_GOAL_REWARD;

	if (agent_achieved_final_goal())
		agent_reward += FINAL_GOAL_REWARD;

	if (last_pose.x != 0)
	{
		distance_traveled = sqrt(pow(localize_ackerman_message.globalpos.x - last_pose.x, 2) + pow(localize_ackerman_message.globalpos.y - last_pose.y, 2));
		agent_reward += (distance_traveled * DISTANCE_PUNISHMENT_PER_METER);

		time_difference = localize_ackerman_message.timestamp - last_time;
		agent_reward += (time_difference * TIME_PUNISHMENT_PER_SECOND);
	}

	if (car_hit_obstacle())
	{
		agent_reward += (COLLISION_PUNISHMENT);
		collision_detected = 1;
	}

	last_pose = localize_ackerman_message.globalpos;
	last_time = localize_ackerman_message.timestamp;
}


void
summarize_experiment(/*FrameDataSp frame, double reward, DqnAction action*/)
{
	if (dqn_caffe->current_iteration() < dqn_caffe->params()->NUM_ITERATIONS)
		epsilon = 1.0 - 0.9 * ((double) dqn_caffe->current_iteration() / (double) dqn_caffe->params()->NUM_ITERATIONS);
	else
		epsilon = 0.1;
}


FrameDataSp
PreprocessScreen(Mat *raw_screen, carmen_grid_mapping_message *message)
{
	// ************************************************
	// CHECK IF IT CAN'T BE OPTIMIZED TO AVOID IMAGE CREATION ALL THE TIME!!!!!!!!!
	// ************************************************

	Mat gray, resized;

	resized = Mat(Size(dqn_caffe->params()->kCroppedFrameSize, dqn_caffe->params()->kCroppedFrameSize), CV_8UC1);

	int robot_x = (int) ((localize_ackerman_message.globalpos.y - message->config.y_origin) / message->config.resolution);
	int robot_y = (int) ((localize_ackerman_message.globalpos.x - message->config.x_origin) / message->config.resolution);

	int top = robot_y - raw_screen->rows / 6;
	int height = raw_screen->rows / 3;

	int left = robot_x - raw_screen->cols / 6;
	int width = raw_screen->cols / 3;

	// ***************************
	// CHECAR SE O -1 EH NECESSARIO
	// ***************************
	if (top < 0) { top = 0; }
	if (top + height >= raw_screen->rows) { top -= (top + height - raw_screen->rows); top -= 1; }

	// ***************************
	// CHECAR SE O -1 EH NECESSARIO
	// ***************************
	if (left < 0) { left = 0; }
	if (left + width >= raw_screen->cols) { left -= (left + width - raw_screen->cols); left -= 1; }

	printf("%d %d -> %d %d %d %d -> %d %d\n", robot_x, robot_y, top, height, left, width, raw_screen->rows, raw_screen->cols);

	Rect roi(top, left, width, height);
	cvtColor((*raw_screen)(roi), gray, CV_BGR2GRAY);
	resize(gray, resized, resized.size());

	rotate(resized, 90, resized); // to make it look like in navigator_gui2

	imshow("netinput", resized);
	waitKey(1);

	FrameDataSp screen = boost::shared_ptr<FrameData>(new FrameData());

	for (int i = 0; i < (resized.rows * resized.cols); i++)
		screen->push_back(gray.data[i]);

	return screen;
}


void
update_current_command_with_dqn_action(DqnAction action)
{
	switch(action)
	{
		case DQN_ACTION_SPEED_DOWN:
		{
			current_command.v -= SPEED_UPDATE;
			break;
		}
		case DQN_ACTION_SPEED_UP:
		{
			current_command.v += SPEED_UPDATE;
			break;
		}
		case DQN_ACTION_STEER_LEFT:
		{
			current_command.phi += STEERING_UPDATE;
			current_command.phi = carmen_normalize_theta(current_command.phi);
			break;
		}
		case DQN_ACTION_STEER_RIGHT:
		{
			current_command.phi -= STEERING_UPDATE;
			current_command.phi = carmen_normalize_theta(current_command.phi);
			break;
		}
		case DQN_ACTION_NONE:
		{
			// keep the same command
			break;
		}
		default:
		{
			printf("** ERROR: INVALID ACTION!! THIS IS SERIOUS!! TRYING ACTION %d\n", (int) action);
			break;
		}
	}

	if (current_command.v > MAX_SPEED) current_command.v = MAX_SPEED;
	if (current_command.v < -MAX_SPEED) current_command.v = -MAX_SPEED;

	if (current_command.phi > MAX_PHI) current_command.phi = MAX_PHI;
	if (current_command.phi < -MAX_PHI) current_command.phi = -MAX_PHI;
}


void
update_command_using_dqn(FrameDataSp frame)
{
	input_frames.push_back(frame);

	if ((int) input_frames.size() >= dqn_caffe->params()->kInputFrameCount)
	{
		// remove the oldest frame if we have more than necessary
		if ((int) input_frames.size() > dqn_caffe->params()->kInputFrameCount)
		{
			for (int i = 0; i < (int) (input_frames.size() - 1); i++)
				input_frames[i] = input_frames[i + 1];

			input_frames.pop_back();
		}

		current_action = dqn_caffe->SelectAction(input_frames, epsilon);
		update_current_command_with_dqn_action(current_action);
	}
	else
		// set all fields of the structure to 0
		memset(&current_command, 0, sizeof(current_command));
}


void
train_dqn(FrameDataSp frame, double reward, DqnAction action, int collision_detected)
{
	// **************************************************
	// O ULTIMO PARAMETRO DA TRANSITION DEVE SER O PROXIMO FRAME DEPOIS DE EXECUTAR A ACAO!!!!!!
	// **************************************************

	static double last_reward = 0;
	Transition transition;

	// Add the current transition to replay memory
	if (!collision_detected)
		transition = Transition(input_frames, action, reward - last_reward, frame);
	else
		transition = Transition(input_frames, action, reward - last_reward, FrameDataSp());

	dqn_caffe->AddTransition(transition);

	// If the size of replay memory is enough, update DQN
	if (dqn_caffe->memory_size() > dqn_caffe->params()->NUM_WARMUP_TRANSITIONS)
		dqn_caffe->Update();

	last_reward = reward;
}


void
grid_mapping_handler(carmen_grid_mapping_message *message)
{
	static int first = 1;

	if (first)
	{
		reset_experiment();
		first = 0;
	}

	// wait for the necessary messages and data
	if (rddf_message.timestamp == 0 || localize_ackerman_message.timestamp == 0 ||
			compact_cost_map_message.timestamp == 0 || cost_map.complete_map == NULL)
		return;

	Mat *final_map_image = convert_map_to_image(message);
	FrameDataSp frame = PreprocessScreen(final_map_image, message);
	update_command_using_dqn(frame);

	// ******************************************************************
	// @filipe: checar handlers da mesma mensagem podem ser chamados em paralelo. Se sim,
	// colocar um watchdog aqui para mandar comandos 0 se colisoes estiverem sido detectadas.
	// ******************************************************************
	publish_motion_command(current_command.v, current_command.phi);
	update_agent_reward();

	printf("***********************************\n");
	printf("****** COLLISION DETECTED: %d\n", collision_detected);
	printf("****** COMMAND: %.2lf %.2lf\n", current_command.v, current_command.phi);
	printf("****** REWARD: %lf\n", agent_reward);
	printf("***********************************\n\n");

	if (collision_detected)
		publish_motion_command(0, 0);

	train_dqn(frame, agent_reward, current_action, collision_detected);

	if (localize_ackerman_message.v > 0.5)
		car_already_accelerated_in_this_experiment = 1;

	// If the car stopped after already accelerating, finish the experiment. Do the same
	// if a collision is detected.
	if ((localize_ackerman_message.v == 0 && car_already_accelerated_in_this_experiment) || collision_detected)
	{
		summarize_experiment();
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
map_server_compact_cost_map_message_handler(carmen_map_server_compact_cost_map_message *message)
{
	if (compact_cost_map.coord_x == NULL || cost_map.complete_map == NULL)
	{
		carmen_grid_mapping_create_new_map(&cost_map, message->config.x_size, message->config.y_size, message->config.resolution);
		memset(cost_map.complete_map, 0, cost_map.config.x_size * cost_map.config.y_size * sizeof(double));
	}
	else
	{
		carmen_prob_models_clear_carmen_map_using_compact_map(&cost_map, &compact_cost_map, 0.0);
		carmen_prob_models_free_compact_map(&compact_cost_map);
	}

	carmen_cpy_compact_cost_message_to_compact_map(&compact_cost_map, message);
	carmen_prob_models_uncompress_compact_map(&cost_map, &compact_cost_map);

	cost_map.config = message->config;
}


void
initialize_global_structures(char **argv)
{
	// set all fields of all structures to 0
	memset(&rddf_message, 0, sizeof(rddf_message));
	memset(&localize_ackerman_message, 0, sizeof(localize_ackerman_message));
	memset(&compact_cost_map_message, 0, sizeof(compact_cost_map_message));
	memset(&compact_cost_map, 0, sizeof(compact_cost_map));
	memset(&cost_map, 0, sizeof(cost_map));

	rddf_index_already_used_in_the_experiment = (char *) calloc (rddf_index->size(), sizeof(char));

	dqn_caffe = new DqnCaffe(DqnParams(), argv[0]);
	dqn_caffe->Initialize();

	namedWindow("map");
	namedWindow("netinput");

	moveWindow("map", 10, 10);
	moveWindow("netinput", 500, 10);
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
	carmen_map_server_subscribe_compact_cost_map(&compact_cost_map_message, (carmen_handler_t) map_server_compact_cost_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


int
read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
			{"robot", "max_velocity", CARMEN_PARAM_DOUBLE,	&carmen_robot_ackerman_config.max_v, 1, NULL},
			{"robot", "max_steering_angle", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.max_phi, 1, NULL},
			{"robot", "min_approach_dist", CARMEN_PARAM_DOUBLE,	&carmen_robot_ackerman_config.approach_dist, 1, NULL},
			{"robot", "min_side_dist", CARMEN_PARAM_DOUBLE,	&carmen_robot_ackerman_config.side_dist, 1, NULL},
			{"robot", "length", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.length, 0, NULL},
			{"robot", "width", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.width, 0, NULL},
			{"robot", "maximum_acceleration_forward", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.maximum_acceleration_forward, 1, NULL},
			{"robot", "maximum_deceleration_forward", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.maximum_deceleration_forward, 1, NULL},
			{"robot", "reaction_time", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.reaction_time, 0, NULL},
			{"robot", "distance_between_rear_wheels", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.distance_between_rear_wheels, 1,NULL},
			{"robot", "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.distance_between_front_and_rear_axles, 1, NULL},
			{"robot", "distance_between_rear_car_and_rear_wheels", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.distance_between_rear_car_and_rear_wheels, 1, NULL},
			{"robot", "allow_rear_motion", CARMEN_PARAM_ONOFF, &carmen_robot_ackerman_config.allow_rear_motion, 1, NULL},
			{"robot", "interpolate_odometry", CARMEN_PARAM_ONOFF, &carmen_robot_ackerman_config.interpolate_odometry, 1, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
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

	// to start the experiment the other modules
	// must have initialized.
	sleep(3);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);

	signal(SIGINT, shutdown_module);

	initialize_global_structures(argv);
	define_messages();
	subscribe_messages();
	carmen_ipc_dispatch();

	return (0);
}


