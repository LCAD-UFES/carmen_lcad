
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <math.h>
#include <opencv/cv.h>
#include <carmen/carmen.h>
#include <opencv/highgui.h>
#include <carmen/navigator_ackerman_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/robot_ackerman_interface.h>
#include <carmen/base_ackerman_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/collision_detection.h>
#include <carmen/mapper_interface.h>
#include <carmen/rddf_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/rddf_index.h>
#include <prob_map.h>

#include "rlcontrol_interface.h"
#include "RLControllerFactory.h"

using namespace std;

const int NUM_TRAINING_EPOCHS = 1;
const int NUM_DESIRED_COMMANDS = 50;
const int NUM_MEASURED_SPEEDS = 50;
const int NUM_COMMANDS = 50;

const double FINAL_GOAL_REWARD = 200.0;
const double COLLISION_PUNISHMENT = -200.0;
const int NUM_MOTION_COMMANDS = 10;

static bool stop_requested = false;

double episode_total_reward = 0;
int achieved_goal_in_this_experiment = 0;
int collision_detected = 0;
int reposed_requested = 1;

double time_experiment_start = 0;
int new_localization_already_consolidated = 0;
double time_since_new_starting_localization_was_sent = 0;

int num_consecutive_goal_achievements = 0;
int max_consecutive_goal_achievements = 0;

double experiment_starting_pose_x = 0;
double experiment_starting_pose_y = 0;
double experiment_starting_pose_theta = 0;

int rddf_goal_id = 0;
int rddf_starting_pose_id = 0;

carmen_robot_ackerman_config_t carmen_robot_ackerman_config;
carmen_timestamp_index *rddf_index;

carmen_localize_ackerman_globalpos_message localize_ackerman_message;
carmen_map_server_compact_cost_map_message compact_cost_map_message;
carmen_compact_map_t compact_cost_map;
carmen_map_t cost_map;

double start_of_the_program;
RLController *rl_controller_velocity;
RLController *rl_controller_phi;

carmen_base_ackerman_motion_command_message motion_command_message;
carmen_mapper_map_message map_message;
carmen_base_ackerman_odometry_message base_ackerman_message;
State state_phi, state_velocity;

double time_last_acceleration;

class TrainEvent
{
public:
	State state;
	State next_state;
	double immediate_reward;
	pair<double, double> command_and_reward;

	TrainEvent() {}

	TrainEvent(State state_p, State next_state_p, double immediate_reward_p, pair<double, double> command_and_reward_p)
	{
		state = state_p;
		next_state = next_state_p;
		immediate_reward = immediate_reward_p;
		command_and_reward = command_and_reward_p;
	}

	TrainEvent operator=(TrainEvent event)
	{
		state = event.state;
		next_state = event.next_state;
		immediate_reward = event.immediate_reward;
		command_and_reward = event.command_and_reward;
		return event;
	}
};

vector<TrainEvent> episode_velocity, episode_phi;


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
publish_control_command(double throttle, double brake, double steering, double timestamp)
{
	carmen_rl_control_message rlmessage;

	rlmessage.timestamp = timestamp;
	rlmessage.host = carmen_get_host();
	rlmessage.throttle = throttle;
	rlmessage.steering = steering;
	rlmessage.brake = brake;

	IPC_publishData(CARMEN_RL_CONTROL_NAME, &rlmessage);
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		publish_control_command(0.0, 100.0, 0.0, carmen_get_time());
		//carmen_navigator_ackerman_stop();
		stop_requested = true;

		carmen_ipc_disconnect();
	}
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
	episode_phi.clear();
	episode_velocity.clear();

	time_last_acceleration = carmen_get_time();

	episode_total_reward = 0;
	collision_detected = 0;
	reposed_requested = 1;
	achieved_goal_in_this_experiment = 0;
}


double
distance_to_starting_pose()
{
	return sqrt(pow(localize_ackerman_message.globalpos.x - experiment_starting_pose_x, 2)
			+ pow(localize_ackerman_message.globalpos.y - experiment_starting_pose_y, 2));
}


void
recalculate_and_publish_first_pose()
{
	if (achieved_goal_in_this_experiment)
	{
		num_consecutive_goal_achievements += 1;

		if (num_consecutive_goal_achievements > max_consecutive_goal_achievements)
			max_consecutive_goal_achievements = num_consecutive_goal_achievements;
	}
	else
		num_consecutive_goal_achievements = 0;

	rddf_goal_id = rddf_index->size() - 1;
	rddf_starting_pose_id = 0;

	experiment_starting_pose_x = rddf_index->index[rddf_starting_pose_id].x;
	experiment_starting_pose_y = rddf_index->index[rddf_starting_pose_id].y;
	experiment_starting_pose_theta = carmen_normalize_theta(rddf_index->index[rddf_starting_pose_id].yaw);

	// I call the function below just to beautify the interface while the nets are trained
	while (distance_to_starting_pose() > 1.0)
	{
		printf("Publishing initial pose...\n");
		publish_starting_pose();
		carmen_ipc_sleep(0.5);
	}
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
	double dist = distance_to_goal();

	if (dist < 1.0 && abs(localize_ackerman_message.v) <= 0.01 &&
			fabs(carmen_normalize_theta(localize_ackerman_message.globalpos.theta - rddf_index->index[rddf_goal_id].yaw)) < carmen_degrees_to_radians(10.0))
	{
		achieved_goal_in_this_experiment = 1;
		return 1;
	}

	return 0;
}


int
car_hit_obstacle()
{
	return obstacle_avoider_pose_hit_obstacle(localize_ackerman_message.globalpos, &cost_map, &carmen_robot_ackerman_config);
}


//double
//update_agent_reward()
//{
//	double imediate_reward;
//
//	imediate_reward = 0.0;
//
//	if (agent_achieved_final_goal())
//		imediate_reward += FINAL_GOAL_REWARD;
//
//	if (car_hit_obstacle())
//	{
//		imediate_reward += (COLLISION_PUNISHMENT);
//		collision_detected = 1;
//	}
//
//	episode_total_reward += imediate_reward;
//
//	return imediate_reward;
//}


int
map_is_instable(carmen_mapper_map_message *message)
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
reinit_states()
{
	state_phi.clear();
	state_velocity.clear();

	for (int i = 0; i < NUM_DESIRED_COMMANDS; i++)
	{
		state_phi.desired.push_back(0);
		state_velocity.desired.push_back(0);
	}

	for (int i = 0; i < NUM_MEASURED_SPEEDS; i++)
	{
		state_phi.measured.push_back(0);
		state_velocity.measured.push_back(0);
	}

	for (int i = 0; i < NUM_COMMANDS; i++)
	{
		state_phi.last_commmands.push_back(0);
		state_velocity.last_commmands.push_back(0);
	}
}


void
print_report(State state, double action, State next_state, double reward, double estimated_reward, FILE *output, char *tag)
{
	fprintf(output, "%s des: %.2lf act: %.2lf meas: %.2lf rew: %.2lf Vfut: %.2lf ** ",
		tag, state.desired[0], action, next_state.measured[next_state.measured.size() - 1],
		reward, estimated_reward);

	fflush(output);
}


void
control_step()
{
	pair<double, double> phi_command_and_reward;
	pair<double, double> vel_command_and_reward;

	//printf("%d\n", motion_command_message.num_motion_commands);
	//return ;

	TrainEvent event_v, event_phi;

	// deveria publicar o comando para fazer o carro parar
	//if (motion_command_message.num_motion_commands < 2 || motion_command_message.num_motion_commands > 100)
		//return;

	if (base_ackerman_message.v > 1.0)
		time_last_acceleration = carmen_get_time();

	event_v.state = state_velocity;
	event_phi.state = state_phi;

	state_velocity.measured.push_back(base_ackerman_message.v);
	state_velocity.measured.pop_front();

	state_phi.measured.push_back(base_ackerman_message.phi);
	state_phi.measured.pop_front();

	state_phi.desired.clear();
	state_velocity.desired.clear();

	for (int i = 0; i < NUM_DESIRED_COMMANDS; i++)
	{
		if (i < motion_command_message.num_motion_commands)
		{
			state_velocity.desired.push_back(motion_command_message.motion_command[i].v);
			state_phi.desired.push_back(motion_command_message.motion_command[i].phi);
		}
		else
		{
			state_phi.desired.push_back(0);
			state_velocity.desired.push_back(0);
		}
	}

	phi_command_and_reward = rl_controller_phi->selectAction(state_phi, stderr, false);
	vel_command_and_reward = rl_controller_velocity->selectAction(state_velocity, stderr, false);

	state_velocity.last_commmands.push_back(vel_command_and_reward.first);
	state_velocity.last_commmands.pop_front();

	state_phi.last_commmands.push_back(phi_command_and_reward.first);
	state_phi.last_commmands.pop_front();

	double reward_velocity = -fabs(base_ackerman_message.v - motion_command_message.motion_command[0].v);
	double reward_phi = -fabs(base_ackerman_message.phi - motion_command_message.motion_command[0].phi);

	if (motion_command_message.num_motion_commands <= 2 || motion_command_message.num_motion_commands > 400)
	{
		reward_velocity -= 10;
		reward_phi -= 10;
	}

	if (car_hit_obstacle())
	{
		reward_velocity -= 200;
		reward_phi -= 200;
	}

//	if (agent_achieved_final_goal())
//	{
//		reward_velocity += 200;
//		reward_phi += 200;
//	}

	event_v.immediate_reward = reward_velocity;
	event_v.command_and_reward = vel_command_and_reward;
	event_v.next_state = state_velocity;

	event_phi.immediate_reward = reward_phi;
	event_phi.command_and_reward = phi_command_and_reward;
	event_phi.next_state = state_phi;

	episode_velocity.push_back(event_v);
	episode_phi.push_back(event_phi);

	// convert nn neron id to control effort (41 is the number of output neurons)
	double throttle = 0, brake = 0, steering = 0;
	// 1. throttle and brake
	double vel_command = 100.0 * ((vel_command_and_reward.first / 41.0) * 2.0 - 1.0);

	// @FILIPE: como tratar throttle para tras??
	if (vel_command < 0.0)
	{
		throttle = 0;
		brake = -vel_command;
	}
	else
	{
		throttle = vel_command;
		brake = 0;
	}

	// 2. steering
	steering = 100.0 * ((phi_command_and_reward.first / 41.0) * 2.0 - 1.0);

	// publish command
	publish_control_command(throttle, brake, steering, motion_command_message.timestamp);
	//publish_control_command(0, 100.0, 0, motion_command_message.timestamp);

	static int n = 0;

	if (n % 3 == 0)
	{
		print_report(event_v.state, vel_command, state_velocity, reward_velocity, vel_command_and_reward.second, stderr, "VEL");
		print_report(event_phi.state, steering, state_phi, reward_phi, phi_command_and_reward.second, stderr, "PHI");
		fprintf(stderr, "\n");
		fflush(stderr);
		n = 0;
	}

	n++;
}


void
mapper_map_handler(carmen_mapper_map_message *message __attribute__((unused)))
{
}


void
localize_ackerman_handler(carmen_localize_ackerman_globalpos_message *message __attribute__((unused)))
{
}


void
base_ackerman_motion_command_message_handler(carmen_base_ackerman_motion_command_message *message __attribute__((unused)))
{
}


void
base_ackerman_odom_handler(carmen_base_ackerman_odometry_message *message __attribute__((unused)))
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

	carmen_cpy_compact_map_message_to_compact_map(&compact_cost_map, message);
	carmen_prob_models_uncompress_compact_map(&cost_map, &compact_cost_map);

	cost_map.config = message->config;
}


void
initialize_global_structures()
{
	// set all fields of all structures to 0
	memset(&localize_ackerman_message, 0, sizeof(localize_ackerman_message));
	memset(&compact_cost_map_message, 0, sizeof(compact_cost_map_message));
	memset(&compact_cost_map, 0, sizeof(compact_cost_map));
	memset(&cost_map, 0, sizeof(cost_map));

	rl_controller_phi = RLControllerFactory::build(Q_LEARNING_CONTROLLER, "solver_qlearning_phi.prototxt");
	rl_controller_velocity = RLControllerFactory::build(Q_LEARNING_CONTROLLER, "solver_qlearning_v.prototxt");
}


void
train_episode()
{
	if (episode_phi.size() < 40 || episode_velocity.size() < 40)
		return;

	fprintf(stderr, "Episodes sizes: phi: %ld vel: %ld\n", episode_phi.size(), episode_velocity.size());
	fflush(stderr);

	for (int n = 0; n < NUM_TRAINING_EPOCHS && !stop_requested; n++)
	{
		fprintf(stderr, "Training epoch: %d of %d\n", n, NUM_TRAINING_EPOCHS);
		fflush(stderr);

		for (int i = 0; i < episode_phi.size() && !stop_requested; i++)
		{
			if (i % 10 == 0)
			{
				fprintf(stderr, "Training PHI SAMPLE: %d of %ld\n", i, episode_phi.size());
				fflush(stderr);
			}

			rl_controller_phi->train(episode_phi[i].state, episode_phi[i].immediate_reward, episode_phi[i].next_state, episode_phi[i].command_and_reward, stderr);
		}

		for (int i = 0; i < episode_velocity.size() && !stop_requested; i++)
		{
			if (i % 10 == 0)
			{
				fprintf(stderr, "Training VELOCITY SAMPLE: %d of %ld\n", i, episode_velocity.size());
				fflush(stderr);
			}

			rl_controller_velocity->train(episode_velocity[i].state, episode_velocity[i].immediate_reward, episode_velocity[i].next_state, episode_velocity[i].command_and_reward, stderr);
		}
	}
}


void
define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_NAME);

	err = IPC_defineMsg(CARMEN_RL_CONTROL_NAME, IPC_VARIABLE_LENGTH, CARMEN_RL_CONTROL_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_RL_CONTROL_NAME);
}


void
subscribe_messages()
{
	carmen_mapper_subscribe_message(&map_message, (carmen_handler_t) mapper_map_handler, CARMEN_SUBSCRIBE_LATEST);
    carmen_localize_ackerman_subscribe_globalpos_message(&localize_ackerman_message, (carmen_handler_t) localize_ackerman_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_map_server_subscribe_compact_cost_map(&compact_cost_map_message, (carmen_handler_t) map_server_compact_cost_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_base_ackerman_subscribe_motion_command(&motion_command_message, (carmen_handler_t) base_ackerman_motion_command_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_base_ackerman_subscribe_odometry_message(&base_ackerman_message, (carmen_handler_t) base_ackerman_odom_handler, CARMEN_SUBSCRIBE_LATEST);
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

	srand(time(NULL));

	start_of_the_program = carmen_get_time();

	carmen_rddf_load_index(rddf_filename);
	rddf_index = get_timestamp_index();

	initialize_global_structures();

	// to start the experiment the other modules
	// must have initialized.
	sleep(3);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);

	signal(SIGINT, shutdown_module);

	define_messages();
	subscribe_messages();
	carmen_navigator_ackerman_go();

	while (!stop_requested)
	{
		publish_control_command(0.0, 100.0, 0.0, carmen_get_time());
		//carmen_navigator_ackerman_stop();

		fprintf(stderr, "-----------------------------\n");
		fflush(stderr);

		reinit_states();
		reinitize_experiment_configuration();
		recalculate_and_publish_first_pose();
		carmen_ipc_sleep(1.5);

		while (!car_hit_obstacle() && !agent_achieved_final_goal() && !stop_requested)
		{
			carmen_ipc_sleep(1.0 / 40.0);
			control_step();

			if (fabs(carmen_get_time() - time_last_acceleration) > 5.0)
			{
				fprintf(stderr, "Stopping experiment due to lack of activity...\n");
				episode_velocity[episode_velocity.size() - 1].immediate_reward -= 200.0;
				break;
			}

			//if (motion_command_message.num_motion_commands <= 2 || motion_command_message.num_motion_commands > 490)
				//break;
		}

		fprintf(stderr, "achieved_goal_in_this_experiment: %s\n", (achieved_goal_in_this_experiment != 0) ? ("true") : ("false"));
		fprintf(stderr, "max_consecutive_goal_achievements: %d\n", max_consecutive_goal_achievements);
		fprintf(stderr, "num_consecutive_goal_achievements: %d\n", num_consecutive_goal_achievements);

		publish_control_command(0.0, 100.0, 0.0, carmen_get_time());
		// I call the function below just to beautify the interface while the nets are trained
		recalculate_and_publish_first_pose();
		train_episode();
	}

	rl_controller_phi->saveTrain();
	rl_controller_velocity->saveTrain();

	return (0);
}


