
#include <cstdio>
#include <cstdlib>
#include <math.h>
#include <opencv/cv.h>
#include <carmen/carmen.h>
#include <opencv/highgui.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/robot_ackerman_interface.h>
#include <carmen/mapper_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/collision_detection.h>
#include <carmen/rddf_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/rddf_index.h>
#include <prob_map.h>

#include <caffe/caffe.hpp>
#include <boost/smart_ptr.hpp>
#include "dqn_caffe.h"

const int VIEWER_ACTIVE = 1;

using namespace cv;

class Event
{
public:
	FrameDataSp frame;
	DqnAction action;
	double v, phi, imediate_reward;
	double q_value_of_selected_action;
	double current_episode_reward;

	Event() { frame = FrameDataSp(); action = DQN_ACTION_NONE; v = phi = 0; imediate_reward = 0; q_value_of_selected_action = 0; }

	Event(FrameDataSp frame_param, DqnAction action_param, double v_param, double phi_param, double imediate_reward_param, double estimated_reward_param, double current_episode_reward_param)
	{
		current_episode_reward = current_episode_reward_param;
		q_value_of_selected_action = estimated_reward_param;
		imediate_reward = imediate_reward_param;
		action = action_param;
		frame = frame_param;
		phi = phi_param;
		v = v_param;
	}
};

/*********************************/
/** INTERNAL PARAMETERS - BEGIN **/
/*********************************/

const int NUM_MOTION_COMMANDS = 10;
const double MAX_SPEED = 10.0;
const double MIN_SPEED = -10.0;
const double MAX_PHI = carmen_degrees_to_radians(28);
const double MIN_PHI = carmen_degrees_to_radians(-28);
double SPEED_UPDATE = 0.2;
double STEERING_UPDATE = carmen_degrees_to_radians(2);

const int GOAL_LIST_STEP = 1; //6; // draw one pose after each 'GOAL_LIST_STEP' number of poses

const double PARTIAL_GOAL_REWARD = 0.05;
const double FINAL_GOAL_REWARD = 0.25;
const double ACHIEVE_FINAL_GOAL_AND_STOP_REWARD = 2.5;
const double DISTANCE_PUNISHMENT_PER_METER = 0;
const double TIME_PUNISHMENT_PER_SECOND = -0.01;
const double COLLISION_PUNISHMENT = -10;

const double MIN_DIST_TO_CONSIDER_ACHIEVEMENT = 1.0;
const double MIN_ORIENTATION_DIFFERENCE_TO_CONSIDER_ACHIEVEMENT = carmen_degrees_to_radians(10);

const double EPSILON_INITIAL = 0.3;
const double EPSILON_MIN = 0.1;
const double EPSILON_DELTA = (EPSILON_INITIAL - EPSILON_MIN) / 1000.0;

const double GAUSSIAN_VARIANCE_X = 0.0;
const double GAUSSIAN_VARIANCE_Y = 0.0;
const double GAUSSIAN_VARIANCE_THETA = carmen_degrees_to_radians(0);

const int NUM_CONSECUTIVE_GOAL_ACHIEVEMENTS_TO_GO_TO_THE_NEXT_LEVEL = 20;
const int JMP_POSES_FROM_BEGINING = 10;

/*********************************/
/** INTERNAL PARAMETERS - END ***/
/*********************************/

double min_delayed_reward = DBL_MAX;
double max_delayed_reward = -DBL_MAX;
double max_reward_so_far = -DBL_MAX;
double episode_total_reward = 0;
long rddf_index_last_pose = 0;
int achieved_goal_in_this_experiment = 0;
int car_already_accelerated_in_this_experiment = 0;
int collision_detected = 0;
int reposed_requested = 1;
carmen_ackerman_motion_command_t current_command = {0, 0, 0};
DqnAction current_action = DQN_ACTION_NONE;
float current_q = 0;
double epsilon = EPSILON_INITIAL;
InputFrames input_frames;
int num_times_car_hit_final_goal = 0;
int use_greedy = 0;

double time_experiment_start = 0;
int new_localization_already_consolidated = 0;
double time_since_new_starting_localization_was_sent = 0;
double instant_car_stoped = 0;
int car_is_stoped = 0;

int num_rddf_poses_from_origin = 5;
int num_consecutive_goal_achievements = 0;
int max_consecutive_goal_achievements = 0;
long num_experiences_in_this_level = 0;

double experiment_starting_pose_x = 0;
double experiment_starting_pose_y = 0;
double experiment_starting_pose_theta = 0;

int rddf_goal_id = 0;
int rddf_starting_pose_id = 0;

carmen_robot_ackerman_config_t carmen_robot_ackerman_config;

// The variable rddf_index_already_used_in_the_experiment
// is used to mark the rddf positions that already generated
// positive reinforcements to the car in the current experiment.
// If it is 0, then the associated pose in the rddf_index was
// not used yet. If it is 1, then the pose was already used.
char *rddf_index_already_used_in_the_experiment;
carmen_timestamp_index *rddf_index;

carmen_localize_ackerman_globalpos_message localize_ackerman_message;
carmen_map_server_compact_cost_map_message compact_cost_map_message;
carmen_compact_map_t compact_cost_map;
carmen_map_t cost_map;

DqnCaffe *dqn_caffe;
vector<Event*> episode;
double start_of_the_program;


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
		static bool stoped = false;

		if (!stoped)
		{
			dqn_caffe->GetSolver()->Snapshot();
			publish_motion_command(0, 0);
			stoped = true;
		}

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
		carmen_mapper_map_message *map_message,
		Scalar color, int filled = 1)
{
	cv::Point polygons[4];

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

	if (filled)
		fillConvexPoly(*map_image, (const cv::Point *) polygons, 4, color);
	else
	{
		fillConvexPoly(*map_image, (const cv::Point *) polygons, 4, Scalar(255,255,255));

		line(*map_image, polygons[0], polygons[1], color, 2);
		line(*map_image, polygons[1], polygons[2], color, 2);
		line(*map_image, polygons[2], polygons[3], color, 2);
		line(*map_image, polygons[3], polygons[0], color, 2);
	}
}


Mat
map_to_image(carmen_mapper_map_message *message)
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

				map_image->data[3 * (i * message->config.x_size + j) + 0] = (unsigned char) (255 * map_probability);
				map_image->data[3 * (i * message->config.x_size + j) + 1] = (unsigned char) (255 * map_probability);
				map_image->data[3 * (i * message->config.x_size + j) + 2] = (unsigned char) (255 * map_probability);
			}
		}
	}

	return map_image;
}


void
draw_rddf_in_the_map(Mat *map_image, carmen_mapper_map_message *map_message)
{
	for (int i = 0; i < rddf_goal_id /*rddf_index->size()*/; i++)
	{
		if (i % GOAL_LIST_STEP == 0)
		{
			Scalar color;
			color = Scalar(0,255, 255);

			draw_ackerman_shape(map_image,
				rddf_index->index[i].x,
				rddf_index->index[i].y,
				rddf_index->index[i].yaw,
				map_message, color);
		}
	}

	for (int i = rddf_starting_pose_id + JMP_POSES_FROM_BEGINING; i < rddf_goal_id; i++)
	{
		if (i % GOAL_LIST_STEP == 0)
		{
			Scalar color;

			color = Scalar(0,255,0);

			draw_ackerman_shape(map_image,
				rddf_index->index[i].x,
				rddf_index->index[i].y,
				rddf_index->index[i].yaw,
				map_message, color);
		}
	}
}


void
draw_globalpose_in_the_map(Mat *map_image,
		carmen_localize_ackerman_globalpos_message *localize_ackerman_message,
		carmen_mapper_map_message *map_message)
{
	draw_ackerman_shape(map_image, localize_ackerman_message->globalpos.x,
			localize_ackerman_message->globalpos.y,
			localize_ackerman_message->globalpos.theta,
			map_message, Scalar(0, 0, 255));
}


void
draw_final_goal_in_the_map(Mat *map_image, carmen_mapper_map_message *map_message)
{
	draw_ackerman_shape(map_image, rddf_index->index[rddf_goal_id].x,
				rddf_index->index[rddf_goal_id].y,
				rddf_index->index[rddf_goal_id].yaw,
				map_message, Scalar(255, 0, 0), 1);
}


long
find_closest_pose_in_the_index(carmen_point_t *initial_pose)
{
	return find_timestamp_index_position_with_full_index_search(initial_pose->x, initial_pose->y, initial_pose->theta, 0);
}


carmen_point_t
publish_starting_pose()
{
	// I did it to force the next iteration of the algorithm to wait for a new
	// globalpos message after reinitialization
	memset(&localize_ackerman_message, 0, sizeof(localize_ackerman_message));

	carmen_point_t pose;
	carmen_point_t std;

	pose.x = experiment_starting_pose_x;
	pose.y = experiment_starting_pose_y;
	pose.theta = experiment_starting_pose_theta;

	std.x = 0.0001;
	std.y = 0.0001;
	std.theta = carmen_degrees_to_radians(0.01);

	carmen_localize_ackerman_initialize_gaussian_command(pose, std);

	time_since_new_starting_localization_was_sent = carmen_get_time();
	new_localization_already_consolidated = 0;

	return pose;
}


void
reinitize_experiment_configuration()
{
	// set all fields of the structure to 0. note: I use this in some function to check if some action is necessary.
	// do not erase without checking the other functions.
	memset(rddf_index_already_used_in_the_experiment, 0, rddf_index->size() * sizeof(char));

	instant_car_stoped = carmen_get_time();
	car_is_stoped = 0;
	num_times_car_hit_final_goal = 0;
	episode_total_reward = 0;
	collision_detected = 0;
	reposed_requested = 1;
	achieved_goal_in_this_experiment = 0;
	car_already_accelerated_in_this_experiment = 0;
	current_action = DQN_ACTION_NONE;
	current_q = 0;

	static int n = 0;

	if (n >= 2)
	{
		use_greedy = 1;
		n = 0;
	}
	else
	{
		use_greedy = 0;
		n++;
	}

	// set all fields of the structure to 0
	memset(&current_command, 0, sizeof(current_command));
	input_frames.clear();

	rddf_index_last_pose = rddf_starting_pose_id;
}


void
reset_experiment()
{
	if (achieved_goal_in_this_experiment)
	{
		num_consecutive_goal_achievements += 2;

		if (num_consecutive_goal_achievements > max_consecutive_goal_achievements)
			max_consecutive_goal_achievements = num_consecutive_goal_achievements;
	}
	else
		num_consecutive_goal_achievements = 0;

	if (num_consecutive_goal_achievements >= NUM_CONSECUTIVE_GOAL_ACHIEVEMENTS_TO_GO_TO_THE_NEXT_LEVEL && num_rddf_poses_from_origin < (rddf_index->size() - 1))
	{
		num_rddf_poses_from_origin += 4;
		max_consecutive_goal_achievements = 0;
		num_consecutive_goal_achievements = 0;
		epsilon = 0.3; // raise epsilon a little to regain exploration power
		num_experiences_in_this_level = 0;
	}

	num_experiences_in_this_level++;

	if (num_rddf_poses_from_origin < 60)
		rddf_goal_id = 20; //(rand() % (rddf_index->size() - num_rddf_poses_from_origin)) + num_rddf_poses_from_origin;
	else
	{
		rddf_goal_id = (rand() % (rddf_index->size() - num_rddf_poses_from_origin)) + num_rddf_poses_from_origin;
		epsilon = 0.3;
	}

	rddf_starting_pose_id = rddf_goal_id - num_rddf_poses_from_origin;

	experiment_starting_pose_x = rddf_index->index[rddf_starting_pose_id].x + carmen_gaussian_random(0, GAUSSIAN_VARIANCE_X);
	experiment_starting_pose_y = rddf_index->index[rddf_starting_pose_id].y + carmen_gaussian_random(0, GAUSSIAN_VARIANCE_Y);
	experiment_starting_pose_theta = carmen_normalize_theta(rddf_index->index[rddf_starting_pose_id].yaw + carmen_gaussian_random(0, GAUSSIAN_VARIANCE_THETA));

	publish_starting_pose();
	reinitize_experiment_configuration();

	// clear the episode
	for (int i = 0; i < episode.size(); i++)
		delete(episode[i]);

	episode.clear();
}


Mat*
draw_map_car_and_rddf(carmen_mapper_map_message *message)
{
	//Mat map_image = map_to_image(message);
	Mat *map_image = cost_map_to_image(&cost_map);

	draw_rddf_in_the_map(map_image, message);
	draw_final_goal_in_the_map(map_image, message);
	draw_globalpose_in_the_map(map_image, &localize_ackerman_message, message);

	// ** for debug:
	if (VIEWER_ACTIVE)
	{
		assert(map_image->cols != 0 && map_image->rows != 0);
		Mat resized_map = Mat(Size(map_image->cols / 3, map_image->rows / 3), CV_8UC1);
		resize(*map_image, resized_map, resized_map.size(), 0, 0, INTER_CUBIC);
		imshow("map", resized_map);
		waitKey(1);
	}

	return map_image;
}


int
agent_achieved_partial_goal()
{
	int i;
	double dist;

	for (i = rddf_starting_pose_id + JMP_POSES_FROM_BEGINING; i < rddf_goal_id; i++)
	{
		dist = sqrt(pow(rddf_index->index[i].x - localize_ackerman_message.globalpos.x, 2) +
				pow(rddf_index->index[i].y - localize_ackerman_message.globalpos.y, 2));

		if (!rddf_index_already_used_in_the_experiment[i] && dist < MIN_DIST_TO_CONSIDER_ACHIEVEMENT &&
				abs(carmen_normalize_theta(localize_ackerman_message.globalpos.theta - rddf_index->index[i].yaw)) < MIN_ORIENTATION_DIFFERENCE_TO_CONSIDER_ACHIEVEMENT)
		{
			rddf_index_already_used_in_the_experiment[i] = 1;
			return 1;
		}
	}

	return 0;
}


double
distance_to_goal()
{
	return sqrt(pow(rddf_index->index[rddf_goal_id].x - localize_ackerman_message.globalpos.x, 2)
			+ pow(rddf_index->index[rddf_goal_id].y - localize_ackerman_message.globalpos.y, 2));
}


int
agent_achieved_final_goal()
{
	if (!achieved_goal_in_this_experiment)
	{
		double dist = distance_to_goal();

		//fprintf(stderr, "DG: %.2lf ", dist);

		if (dist < MIN_DIST_TO_CONSIDER_ACHIEVEMENT &&
				abs(carmen_normalize_theta(localize_ackerman_message.globalpos.theta - rddf_index->index[rddf_goal_id].yaw)) < MIN_ORIENTATION_DIFFERENCE_TO_CONSIDER_ACHIEVEMENT)
		{
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


double
update_agent_reward(double current_time)
{
	double imediate_reward;
	static double last_time = 0;
	double time_difference;

	imediate_reward = 0.0;

	if (agent_achieved_partial_goal())
		imediate_reward += PARTIAL_GOAL_REWARD;

	if (agent_achieved_final_goal())
	{
		if (num_times_car_hit_final_goal < 10)
			imediate_reward += FINAL_GOAL_REWARD;

		if (abs(localize_ackerman_message.v) <= SPEED_UPDATE)
		{
			// se o agente parar no goal ele dobra o premio
			achieved_goal_in_this_experiment = 1;
			imediate_reward += ACHIEVE_FINAL_GOAL_AND_STOP_REWARD;
		}

		num_times_car_hit_final_goal++;
	}

	if (last_time > 0.001)
	{
		time_difference = current_time - last_time;
		imediate_reward += (time_difference * TIME_PUNISHMENT_PER_SECOND);
	}

	if (car_hit_obstacle())
	{
		imediate_reward += (COLLISION_PUNISHMENT);
		collision_detected = 1;
	}

	episode_total_reward += imediate_reward;
	last_time = current_time;

	return imediate_reward;
}


void
add_frame_to_list(InputFrames *frames, FrameDataSp frame)
{
	frames->push_back(frame);

	if ((int) frames->size() > DqnParams::kInputFrameCount)
	{
		for (int i = 0; i < (int) (frames->size() - 1); i++)
			frames->at(i) = frames->at(i + 1);

		frames->pop_back();
	}
}


double
calculate_delayed_reward(vector<Event*> *episode, int t)
{
	double delayed_reward = 0;

	for (int i = t; i < episode->size(); i++)
		delayed_reward += (episode->at(i)->imediate_reward * pow(dqn_caffe->params()->GAMMA, i - t));

	if (delayed_reward < min_delayed_reward) min_delayed_reward = delayed_reward;
	if (delayed_reward > max_delayed_reward) max_delayed_reward = delayed_reward;

	return delayed_reward;
}


void
add_transitions_to_replay_memory()
{
	int i;
	InputFrames frames;
	double total_reward = episode_total_reward;
	double reward;

	if (episode.size() < DqnParams::kInputFrameCount + 1)
		return;

	for (i = 0; i < DqnParams::kInputFrameCount; i++)
		frames.push_back(episode[i]->frame);

	for (i = DqnParams::kInputFrameCount; i < (episode.size() - 1); i++)
	{
		FrameDataSp current_frame = episode[i]->frame;
		FrameDataSp frame_after_action = episode[i + 1]->frame;

		if (DqnParams::TrainingModel == DQN_Q_LEARNING) reward = episode[i]->imediate_reward;
		// @filipe: o modelo de recompensa com desconto nao eh bem assim nao...
		else if (DqnParams::TrainingModel == DQN_DISCOUNTED_TOTAL_REWARD) reward = total_reward; // * pow(dqn_caffe->params()->GAMMA, (double) episode.size() - i);
		else if (DqnParams::TrainingModel == DQN_INFINITY_HORIZON_DISCOUNTED_MODEL) reward = calculate_delayed_reward(&episode, i);
		else exit(printf("ERROR: Unknown training model!\n"));

		add_frame_to_list(&frames, current_frame);
		dqn_caffe->AddTransition(Transition(frames, episode[i]->action, reward, frame_after_action, episode[i]->v, episode[i]->phi, false, episode[i]->q_value_of_selected_action));
	}

	if (DqnParams::TrainingModel == DQN_Q_LEARNING || DqnParams::TrainingModel == DQN_INFINITY_HORIZON_DISCOUNTED_MODEL) reward = episode[i]->imediate_reward;
	// @filipe: o modelo de recompensa com desconto nao eh bem assim nao...
	else if (DqnParams::TrainingModel == DQN_DISCOUNTED_TOTAL_REWARD) reward = total_reward; // * pow(dqn_caffe->params()->GAMMA, (double) episode.size() - i);
//	else if (DqnParams::TrainingModel == DQN_INFINITY_HORIZON_DISCOUNTED_MODEL)
//	{
//		reward = episode[i]->imediate_reward;
//
//		if (reward < min_delayed_reward) min_delayed_reward = reward;
//		if (reward > max_delayed_reward) max_delayed_reward = reward;
//	}
	else exit(printf("ERROR: Unknown training model!\n"));

	// the last transition is a special case
	dqn_caffe->AddTransition(Transition(frames, episode[i]->action, reward, FrameDataSp(), episode[i]->v, episode[i]->phi, true, episode[i]->q_value_of_selected_action));
}


void
summarize_experiment()
{
//	static const double delta_epsilon = (EPSILON_FINAL - EPSILON_INITIAL) / (double) dqn_caffe->params()->NUM_ITERATIONS;
//
//	if (dqn_caffe->current_iteration() < dqn_caffe->params()->NUM_ITERATIONS)
//		epsilon = EPSILON_INITIAL + delta_epsilon * dqn_caffe->current_iteration();
//	else
//		epsilon = EPSILON_FINAL;

	epsilon -= EPSILON_DELTA;

	if (epsilon <= EPSILON_MIN)
		epsilon = EPSILON_MIN;

	// update the max reward
	if (episode_total_reward > max_reward_so_far)
		max_reward_so_far = episode_total_reward;

	add_transitions_to_replay_memory();
}


FrameDataSp
PreprocessScreen(Mat *raw_screen, carmen_mapper_map_message *message)
{
	// ************************************************
	// CHECK IF IT CAN'T BE OPTIMIZED TO AVOID IMAGE CREATION ALL THE TIME!!!!!!!!!
	// ************************************************

	Mat resized;

	resized = Mat(Size(DqnParams::kCroppedFrameSize, DqnParams::kCroppedFrameSize), CV_8UC3);

	int robot_x = (int) ((localize_ackerman_message.globalpos.y - message->config.y_origin) / message->config.resolution);
	int robot_y = (int) ((localize_ackerman_message.globalpos.x - message->config.x_origin) / message->config.resolution);

	int top = robot_x - raw_screen->rows / 6;
	int height = raw_screen->rows / 3;

	int left = robot_y - raw_screen->cols / 6;
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

	Mat *raw_screen_rotated = new Mat(Size(raw_screen->cols, raw_screen->rows), CV_8UC3);

	//rotate image
	cv::Point center = cv::Point(robot_x, robot_y);
	Mat rot_mat = getRotationMatrix2D(center, carmen_radians_to_degrees(-localize_ackerman_message.globalpos.theta), 1.0);
	warpAffine(*raw_screen, *raw_screen_rotated, rot_mat, raw_screen_rotated->size());

	Rect roi(top, left, width, height);
	resize((*raw_screen_rotated)(roi), resized, resized.size(), 0, 0, INTER_CUBIC);

	rotate(resized, 90, resized); // to make it look like in navigator_gui2

	if (VIEWER_ACTIVE)
	{
		Mat net_input_view(Size(200, 200), CV_8UC3);
		resize(resized, net_input_view, net_input_view.size(), 0, 0, INTER_CUBIC);
		imshow("netinput", net_input_view);
		waitKey(1);
	}

	FrameDataSp screen = boost::shared_ptr<FrameData>(new FrameData());

	for (int i = 0; i < (resized.rows * resized.cols * resized.channels()); i++)
		screen->push_back(resized.data[i]);

	delete(raw_screen_rotated);
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
	if (current_command.v < MIN_SPEED) current_command.v = MIN_SPEED;

	if (current_command.phi > MAX_PHI) current_command.phi = MAX_PHI;
	if (current_command.phi < MIN_PHI) current_command.phi = MIN_PHI;
}


void
update_command_using_dqn(double v, double phi)
{
	// DEBUG: faz o carro acelerar para frente apenas
//	current_action = DQN_ACTION_SPEED_UP;
//	update_current_command_with_dqn_action(current_action);
//	return;

	if ((int) input_frames.size() >= DqnParams::kInputFrameCount)
	{
		std::pair<DqnAction, float> action_and_q;

		if (use_greedy)
			action_and_q = dqn_caffe->SelectAction(input_frames, 0, v, phi);
		else
			action_and_q = dqn_caffe->SelectAction(input_frames, epsilon, v, phi);

		current_q = action_and_q.second;
		current_action = action_and_q.first;
		update_current_command_with_dqn_action(current_action);
	}
	else
	{
		current_q = TIME_PUNISHMENT_PER_SECOND; // 0;

		current_action = DQN_ACTION_NONE;
		// set all fields of the structure to 0
		memset(&current_command, 0, sizeof(current_command));
	}
}


void
print_report(double distance_to_goal, int achieved_goal_in_this_experiment)
{
	static int n = 0;
	static int num_goal_achievements = 0;
	static double avg_distance_to_goal = 0;

	num_goal_achievements = achieved_goal_in_this_experiment;
	avg_distance_to_goal = distance_to_goal;
	n++;

	if (n >= 100)
	{
		printf("PERCENTAGE_GOAL_ACHIEVEMENTS: %.2lf\n", 100 * ((double) num_goal_achievements / (double) n));
		printf("AVERAGE_DIST_TO_GOAL: %.2lf\n", (avg_distance_to_goal / (double) n));

		n = 0;
		num_goal_achievements = 0;
		avg_distance_to_goal = 0;
	}

	printf("FINAL DIST TO GOAL: %lf\n", distance_to_goal);
	printf("FINAL REWARD: %lf\n", episode_total_reward);
}


void
train_net_a_little()
{
//	static int n = 0;
//
//	if (++n < 20)
//		return;
//
//	n = 0;

	double init = carmen_get_time();
	if (dqn_caffe->memory_size() > dqn_caffe->params()->NUM_WARMUP_TRANSITIONS)
	{
		for (int i = 0; i < 1000 && i < dqn_caffe->memory_size(); i++)
		{
			if (i % 1 == 0)
				// o cara que completa esse fprintf esta em dqn_caffe.cpp na funcao update
				fprintf(stderr, "TRAINING THE NET %d OF %d SAMPLES | TIME: %.2lf | TIME PER SAMPLE: %.2lf || \t",
					i, (dqn_caffe->memory_size() < 5000) ? (dqn_caffe->memory_size()) : (1000), carmen_get_time() - init, (carmen_get_time() - init) / (double) i);

			dqn_caffe->Update();
		}
	}
}


int
map_is_unstable(carmen_mapper_map_message *message)
{
	static double ox = 0, oy = 0;
	static double cx = 0, cy = 0;

	if (message->config.x_origin == ox && message->config.y_origin == oy && cost_map.config.x_origin == cx && cost_map.config.y_origin == cy)
		return 0;

	ox = message->config.x_origin;
	oy = message->config.y_origin;

	cx = cost_map.config.x_origin;
	cy = cost_map.config.y_origin;

	return 1;
}


void
save_snapshot_if_necessary()
{
	static int n = 0;

	if (n >= 1000)
	{
		n = 0;
		dqn_caffe->GetSolver()->Snapshot();
	}
	else
		n++;
}


void
map_mapping_handler(carmen_mapper_map_message *message)
{
	static int first = 1;

	// cria o primeiro experimento
	if (first)
	{
		reset_experiment();
		first = 0;
	}

	// espera um segundo no inicio de cada novo experimento (necessario para dar tempo das mensagens de inicializacao se consolidarem nos modulos)
	if (abs(carmen_get_time() - time_since_new_starting_localization_was_sent) < 1.0)
	{
		fprintf(stderr, "Waiting for the consolidation of the new experiment\n");
		return;
	}
	else
	{
		if (!new_localization_already_consolidated)
		{
			time_experiment_start = carmen_get_time();
			new_localization_already_consolidated = 1;
		}
	}

	// wait for the necessary messages and data
	if (localize_ackerman_message.timestamp == 0 || compact_cost_map_message.timestamp == 0 || cost_map.complete_map == NULL)
	{
		fprintf(stderr, "Waiting for necessary messages\n");
		return;
	}

	// TODO: pode acontecer da localizacao dizer que o robo ja esta no lugar certo, mas o mapa ainda nao ter desenhado ele no lugar certo.
	if (reposed_requested && sqrt(pow(localize_ackerman_message.globalpos.x - experiment_starting_pose_x, 2) + pow(localize_ackerman_message.globalpos.y - experiment_starting_pose_y, 2)) > 1.5)
	{
		fprintf(stderr, "Waiting for a new localization\n");
		publish_starting_pose();
		return;
	}
	else
		reposed_requested = 0;

	if (map_is_unstable(message))
	{
		fprintf(stderr, "Waiting for a new map\n");
		return;
	}

	if (fabs(carmen_get_time() - localize_ackerman_message.timestamp) > 180.0)
		exit(printf("Error: long time without localizer...\n"));

	double imediate_reward;

	Mat *final_map_image = draw_map_car_and_rddf(message);
	FrameDataSp frame = PreprocessScreen(final_map_image, message);
	add_frame_to_list(&input_frames, frame);

	update_command_using_dqn(localize_ackerman_message.v, localize_ackerman_message.phi);
	publish_motion_command(current_command.v, current_command.phi);
	imediate_reward = update_agent_reward(message->timestamp);

	if (collision_detected)
		publish_motion_command(0, 0);

	episode.push_back(new Event(frame, current_action, localize_ackerman_message.v, localize_ackerman_message.phi, imediate_reward, current_q, episode_total_reward));

	static double time_last_printf = 0;

	static int ncalls = 0;
	ncalls++;

	// printf after a given ammount of  seconds
	if (abs(carmen_get_time() - time_last_printf) > 5.0)
	{
		ncalls = 0;
		time_last_printf = carmen_get_time();
	}

	fprintf(stderr, "%cCMD: (% 2.1lf, % 2.1lf)   Q: % 3.2lf \t RW: % 2.4lf   TOT: % 2.2lf \t LVL: %d of %ld \t EPS: %.2lf \t CONC: %d MCONC: %d \t CALLS_SEC: %.1f \n",
			(use_greedy) ? ('*') : ('-'), current_command.v, current_command.phi, current_q, imediate_reward, episode_total_reward, num_rddf_poses_from_origin, rddf_index->size() - 1, epsilon, num_consecutive_goal_achievements, max_consecutive_goal_achievements, (float) ncalls / (carmen_get_time() - time_last_printf));

	// If the size of replay memory is enough, update DQN
	//if (dqn_caffe->memory_size() > dqn_caffe->params()->NUM_WARMUP_TRANSITIONS)
		//dqn_caffe->Update();

	if (fabs(localize_ackerman_message.v) > 0.5)
	{
		car_already_accelerated_in_this_experiment = 1;
		car_is_stoped = 0;
	}
	else
	{
		if (!car_is_stoped)
		{
			instant_car_stoped = carmen_get_time();
			car_is_stoped = 1;
		}
	}

	// If the car stopped after already accelerating, finish the experiment. Do the same
	// if a collision is detected.
	if (/*(fabs(localize_ackerman_message.v) < 0.01 && car_already_accelerated_in_this_experiment) ||*/
		(collision_detected) ||
		/*((carmen_get_time() - time_experiment_start) > 300.0) ||*/
		(car_is_stoped && fabs(carmen_get_time() - instant_car_stoped) > 10.0) ||
		(achieved_goal_in_this_experiment && (distance_to_goal() < MIN_DIST_TO_CONSIDER_ACHIEVEMENT) && (abs(localize_ackerman_message.v) < SPEED_UPDATE)))
	{
		print_report(distance_to_goal(), achieved_goal_in_this_experiment);

		// DEBUG:
//		printf("COLISION: %d TIME: %d GOAL: %d\n", collision_detected, ((carmen_get_time() - time_experiment_start) > 300.0), (achieved_goal_in_this_experiment && (distance_to_goal() < MIN_DIST_TO_CONSIDER_ACHIEVEMENT) && (abs(localize_ackerman_message.v) < SPEED_UPDATE)));
//		printf("T_C: %lf TS: %lf DELTA: %lf\n", carmen_get_time(), time_experiment_start, carmen_get_time() - time_experiment_start);
//		printf("AG: %d DG: %lf V: %lf\n", achieved_goal_in_this_experiment, distance_to_goal(), abs(localize_ackerman_message.v));
//		printf("\n------------\n");
//		getchar();

		publish_motion_command(0, 0);
		summarize_experiment();
		train_net_a_little();
		save_snapshot_if_necessary();
		reset_experiment();
	}

	//delete(final_map_image);
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
initialize_global_structures(char **argv, char *pre_train)
{
	// set all fields of all structures to 0
	memset(&localize_ackerman_message, 0, sizeof(localize_ackerman_message));
	memset(&compact_cost_map_message, 0, sizeof(compact_cost_map_message));
	memset(&compact_cost_map, 0, sizeof(compact_cost_map));
	memset(&cost_map, 0, sizeof(cost_map));

	rddf_index_already_used_in_the_experiment = (char *) calloc (rddf_index->size(), sizeof(char));

	dqn_caffe = new DqnCaffe(DqnParams(), argv[0]);
	dqn_caffe->Initialize();

	if (pre_train != NULL)
		dqn_caffe->LoadTrainedModel(pre_train);

	if (VIEWER_ACTIVE)
	{
		namedWindow("map");
		namedWindow("netinput");

		moveWindow("map", 10, 10);
		moveWindow("netinput", 500, 10);
	}
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
	carmen_mapper_subscribe_message(NULL, (carmen_handler_t) map_mapping_handler, CARMEN_SUBSCRIBE_LATEST);
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
	char *pre_train = NULL;

	if (argc < 2)
	{
		printf("Use %s <rddf file> [<pre-trained weights>]\n", argv[0]);
		return 0;
	}
	else
		rddf_filename = argv[1];

	if (argc > 2)
		pre_train = argv[2];

	if (!carmen_rddf_index_exists(rddf_filename))
		exit(printf("Error: rddf file or index files don't exist (this program doesn't generate indices)\n"));

	srand(time(NULL));

	start_of_the_program = carmen_get_time();
	carmen_rddf_load_index(rddf_filename);
	rddf_index = get_timestamp_index();
	initialize_global_structures(argv, pre_train);

	// to start the experiment the other modules
	// must have initialized.
	sleep(3);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);

	signal(SIGINT, shutdown_module);

	define_messages();
	subscribe_messages();
	carmen_ipc_dispatch();

	return (0);
}


