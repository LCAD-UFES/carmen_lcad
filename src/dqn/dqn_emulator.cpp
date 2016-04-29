
#include <cstdio>
#include <cstdlib>
#include <math.h>
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
#include <boost/smart_ptr.hpp>
#include "dqn_caffe.h"

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
const double MIN_SPEED = 0.0;
const double MAX_PHI = carmen_degrees_to_radians(28);
const double MIN_PHI = carmen_degrees_to_radians(-28);
double SPEED_UPDATE = 0.5;
double STEERING_UPDATE = carmen_degrees_to_radians(2); //carmen_degrees_to_radians(0.5);

const int GOAL_LIST_STEP = 1; //6; // draw one pose after each 'GOAL_LIST_STEP' number of poses

const double PARTIAL_GOAL_REWARD = 0.2;
const double FINAL_GOAL_REWARD = 2;
const double DISTANCE_PUNISHMENT_PER_METER = 0; // -0.001;
const double TIME_PUNISHMENT_PER_SECOND = 0; // -0.001;
const double COLLISION_PUNISHMENT = -2;

const int NUM_POSES_TO_SEARCH_AROUND_CLOSEST_GOAL = 10;
const double MIN_DIST_TO_CONSIDER_ACHIEVEMENT = 0.5;

const int RDDF_LIMIT_INITIAL_POSE = 138;
const int RDDF_LIMIT_FINAL_POSE  = RDDF_LIMIT_INITIAL_POSE + 20;

const double EPSILON_INITIAL = 0.9;
const double EPSILON_FINAL = 0.01;

/*********************************/
/** INTERNAL PARAMETERS - END ***/
/*********************************/

double min_delayed_reward = DBL_MAX;
double max_delayed_reward = -DBL_MAX;
double max_reward_so_far = -DBL_MAX;
double episode_total_reward = 0;
long rddf_index_final_goal = 0;
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
		carmen_grid_mapping_message *map_message,
		Scalar color)
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

	fillConvexPoly(*map_image, (const cv::Point *) polygons, 4, color);
	//fillConvexPoly(*map_image, (const cv::Point *) polygons, 4, color);
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
draw_rddf_in_the_map(Mat *map_image, carmen_grid_mapping_message *map_message)
{
	for (int i = RDDF_LIMIT_INITIAL_POSE; i < RDDF_LIMIT_FINAL_POSE; i++)
	{
		if (i % GOAL_LIST_STEP == 0)
		{
			draw_ackerman_shape(map_image,
				rddf_index->index[i].x,
				rddf_index->index[i].y,
				rddf_index->index[i].yaw,
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
reinitize_experiment_configuration(carmen_point_t *initial_pose __attribute__((unused)) /* sera usada no futuro */)
{
	// set all fields of the structure to 0. note: I use this in some function to check if some action is necessary.
	// do not erase without checking the other functions.
	memset(rddf_index_already_used_in_the_experiment, 0, rddf_index->size() * sizeof(char));

	// I did it to force the next iteration of the algorithm to wait for a new
	// globalpos message after reinitialization
	memset(&localize_ackerman_message, 0, sizeof(localize_ackerman_message));

	episode_total_reward = 0;
	collision_detected = 0;
	reposed_requested = 1;
	achieved_goal_in_this_experiment = 0;
	car_already_accelerated_in_this_experiment = 0;
	current_action = DQN_ACTION_NONE;
	current_q = 0;
	// set all fields of the structure to 0
	memset(&current_command, 0, sizeof(current_command));
	input_frames.clear();

	rddf_index_last_pose = RDDF_LIMIT_INITIAL_POSE; //find_closest_pose_in_the_index(initial_pose);
	rddf_index_final_goal = RDDF_LIMIT_FINAL_POSE; //rddf_index_last_pose + 70;

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

	// clear the episode
	for (int i = 0; i < episode.size(); i++)
		delete(episode[i]);

	episode.clear();
}


Mat*
draw_map_car_and_rddf(carmen_grid_mapping_message *message)
{
	//Mat map_image = map_to_image(message);
	Mat *map_image = cost_map_to_image(&cost_map);

	draw_rddf_in_the_map(map_image, message);
	draw_globalpose_in_the_map(map_image, &localize_ackerman_message, message);
	draw_final_goal_in_the_map(map_image, message);

	// ** for debug:
	assert(map_image->cols != 0 && map_image->rows != 0);
	Mat resized_map = Mat(Size(map_image->cols / 3, map_image->rows / 3), CV_8UC1);
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
		if (i >= RDDF_LIMIT_INITIAL_POSE && i <= RDDF_LIMIT_INITIAL_POSE)
		{
			dist = sqrt(pow(rddf_index->index[i].x - localize_ackerman_message.globalpos.x, 2) +
					pow(rddf_index->index[i].y - localize_ackerman_message.globalpos.y, 2));

			if (dist < min_dist)
			{
				min_dist = dist;
				min_dist_id = i;
			}
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
	if (!achieved_goal_in_this_experiment)
	{
		double dist = sqrt(pow(rddf_index->index[rddf_index_final_goal].x - localize_ackerman_message.globalpos.x, 2)
				+ pow(rddf_index->index[rddf_index_final_goal].y - localize_ackerman_message.globalpos.y, 2));

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


double
update_agent_reward()
{
	double imediate_reward;
	static carmen_point_t last_pose = {0, 0, 0};
	static double last_time = 0;

	double distance_traveled;
	double time_difference;

	imediate_reward = 0.0;

	if (agent_achieved_partial_goal())
		imediate_reward += PARTIAL_GOAL_REWARD;

	if (agent_achieved_final_goal())
		imediate_reward += FINAL_GOAL_REWARD;

	if (last_pose.x != 0)
	{
		distance_traveled = sqrt(pow(localize_ackerman_message.globalpos.x - last_pose.x, 2) + pow(localize_ackerman_message.globalpos.y - last_pose.y, 2));
		time_difference = localize_ackerman_message.timestamp - last_time;

		imediate_reward += (distance_traveled * DISTANCE_PUNISHMENT_PER_METER) + (time_difference * TIME_PUNISHMENT_PER_SECOND);
	}

	if (car_hit_obstacle())
	{
		imediate_reward += (COLLISION_PUNISHMENT);
		collision_detected = 1;
	}

	episode_total_reward += imediate_reward;

	last_pose = localize_ackerman_message.globalpos;
	last_time = localize_ackerman_message.timestamp;

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
		else if (DqnParams::TrainingModel == DQN_DISCOUNTED_TOTAL_REWARD) reward = total_reward * pow(dqn_caffe->params()->GAMMA, (double) episode.size() - i);
		else if (DqnParams::TrainingModel == DQN_INFINITY_HORIZON_DISCOUNTED_MODEL) reward = calculate_delayed_reward(&episode, i);
		else exit(printf("ERROR: Unknown training model!\n"));

		add_frame_to_list(&frames, current_frame);
		dqn_caffe->AddTransition(Transition(frames, episode[i]->action, reward, frame_after_action, episode[i]->v, episode[i]->phi, false, episode[i]->q_value_of_selected_action));
	}

	if (DqnParams::TrainingModel == DQN_Q_LEARNING || DqnParams::TrainingModel == DQN_INFINITY_HORIZON_DISCOUNTED_MODEL) reward = episode[i]->imediate_reward;
	// @filipe: o modelo de recompensa com desconto nao eh bem assim nao...
	else if (DqnParams::TrainingModel == DQN_DISCOUNTED_TOTAL_REWARD) reward = total_reward * pow(dqn_caffe->params()->GAMMA, (double) episode.size() - i);
	else if (DqnParams::TrainingModel == DQN_INFINITY_HORIZON_DISCOUNTED_MODEL)
	{
		reward = episode[i]->imediate_reward;

		if (reward < min_delayed_reward) min_delayed_reward = reward;
		if (reward > max_delayed_reward) max_delayed_reward = reward;
	}
	else exit(printf("ERROR: Unknown training model!\n"));

	// the last transition is a special case
	dqn_caffe->AddTransition(Transition(frames, episode[i]->action, reward, FrameDataSp(), episode[i]->v, episode[i]->phi, true, episode[i]->q_value_of_selected_action));
}


void
summarize_experiment()
{
	static const double delta_epsilon = (EPSILON_FINAL - EPSILON_INITIAL) / (double) dqn_caffe->params()->NUM_ITERATIONS;

	// update the max reward
	if (episode_total_reward > max_reward_so_far)
		max_reward_so_far = episode_total_reward;

	if (dqn_caffe->current_iteration() < dqn_caffe->params()->NUM_ITERATIONS)
		epsilon = EPSILON_INITIAL + delta_epsilon * dqn_caffe->current_iteration();
	else
		epsilon = EPSILON_FINAL;

	add_transitions_to_replay_memory();
}


FrameDataSp
PreprocessScreen(Mat *raw_screen, carmen_grid_mapping_message *message)
{
	// ************************************************
	// CHECK IF IT CAN'T BE OPTIMIZED TO AVOID IMAGE CREATION ALL THE TIME!!!!!!!!!
	// ************************************************

	Mat gray, resized;

	resized = Mat(Size(DqnParams::kCroppedFrameSize, DqnParams::kCroppedFrameSize), CV_8UC1);

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

//	printf("%d %d -> %d %d %d %d -> %d %d\n", robot_x, robot_y, top, height, left, width, raw_screen->rows, raw_screen->cols);

	Rect roi(top, left, width, height);
	cvtColor((*raw_screen)(roi), gray, CV_BGR2GRAY);
	resize(gray, resized, resized.size());

	rotate(resized, 90, resized); // to make it look like in navigator_gui2

	Mat net_input_view(Size(200, 200), CV_8UC1);
	//cv::rectangle(resized, cv::Point(0, 0), cv::Point(20, 20), Scalar(0,0,0), 1);
	resize(resized, net_input_view, net_input_view.size());
	imshow("netinput", net_input_view);
	waitKey(1);

	FrameDataSp screen = boost::shared_ptr<FrameData>(new FrameData());

	for (int i = 0; i < (resized.rows * resized.cols); i++)
		screen->push_back(resized.data[i]);

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
		std::pair<DqnAction, float> action_and_q = dqn_caffe->SelectAction(input_frames, epsilon, v, phi);

		current_q = action_and_q.second;
		current_action = action_and_q.first;

		update_current_command_with_dqn_action(current_action);
	}
	else
	{
		current_q = 0;
		current_action = DQN_ACTION_NONE;

		// set all fields of the structure to 0
		memset(&current_command, 0, sizeof(current_command));
	}
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
	if (localize_ackerman_message.timestamp == 0 || compact_cost_map_message.timestamp == 0 || cost_map.complete_map == NULL)
	{
//		printf("Waiting necessary messages for the new episode\n");
		return;
	}

	if (reposed_requested && sqrt(pow(localize_ackerman_message.globalpos.x - 7757880.0, 2) + pow(localize_ackerman_message.globalpos.y - (-363535.0), 2)) > 0.5)
	{
//		printf("Waiting reposition...\n");
		return;
	}
	else
	{
//		printf("Reposition succeded...\n");
		reposed_requested = 0;
	}

	if (fabs(carmen_get_time() - localize_ackerman_message.timestamp) > 5.0)
		exit(printf("Error: long time without localizer...\n"));

	double imediate_reward;

	Mat *final_map_image = draw_map_car_and_rddf(message);
	FrameDataSp frame = PreprocessScreen(final_map_image, message);
	add_frame_to_list(&input_frames, frame);

	update_command_using_dqn(localize_ackerman_message.v, localize_ackerman_message.phi);

	// ******************************************************************
	// @filipe: checar handlers da mesma mensagem podem ser chamados em paralelo. Se sim,
	// colocar um watchdog aqui para mandar comandos 0 se colisoes estiverem sido detectadas.
	// ******************************************************************
	publish_motion_command(current_command.v, current_command.phi);
	imediate_reward = update_agent_reward();

	if (collision_detected)
		publish_motion_command(0, 0);

	// pula de tantos em tantos frames
	episode.push_back(new Event(frame, current_action, localize_ackerman_message.v, localize_ackerman_message.phi, imediate_reward, current_q, episode_total_reward));

	printf("***********************************\n");
	printf("****** COLLISION DETECTED: %d EPSILON: %.10lf ITERATION: %d MEM_SIZE: %d\n", collision_detected, epsilon, dqn_caffe->current_iteration(), dqn_caffe->memory_size());
	printf("****** COMMAND: %.2lf %.2lf ODOMETRY: %.2lf %.2lf\n", current_command.v, current_command.phi, localize_ackerman_message.v, localize_ackerman_message.phi);
	printf("****** REWARD: %lf\n", episode_total_reward);
	if (DqnParams::TrainingModel == DQN_INFINITY_HORIZON_DISCOUNTED_MODEL)
	{
		if (max_delayed_reward == -DBL_MAX) printf("****** MAX DELAYED REWARD: <not finished an experiment yet> MIN DELAYED REWARD: <not finished an experiment yet>\n");
		else printf("****** MAX DELAYED REWARD: %lf MIN DELAYED REWARD: %lf\n", max_delayed_reward, min_delayed_reward);
	}
	if (max_reward_so_far == -DBL_MAX) printf("****** MAX REWARD ACHIEVED: <not finished an experiment yet>\n");
	else printf("****** MAX REWARD ACHIEVED: %lf\n", max_reward_so_far);
	printf("***********************************\n\n");

	// If the size of replay memory is enough, update DQN
	if (dqn_caffe->memory_size() > dqn_caffe->params()->NUM_WARMUP_TRANSITIONS)
		dqn_caffe->Update();

	if (fabs(localize_ackerman_message.v) > 0.01)
		car_already_accelerated_in_this_experiment = 1;

	// If the car stopped after already accelerating, finish the experiment. Do the same
	// if a collision is detected.
	if ((fabs(localize_ackerman_message.v) < 0.01 && car_already_accelerated_in_this_experiment) || (collision_detected && car_already_accelerated_in_this_experiment))
	{
		publish_motion_command(0, 0);
		summarize_experiment();
		reset_experiment();
	}
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
	initialize_global_structures(argv);

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


