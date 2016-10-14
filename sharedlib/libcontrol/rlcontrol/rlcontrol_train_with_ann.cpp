
#include <carmen/carmen.h>

#include <fann.h>
#include <fann_train.h>
#include <fann_data.h>
#include <floatfann.h>

#include <cstdio>
#include <cstdlib>
#include <deque>
#include <math.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <car_neural_model.h>
#include "rlcontrol_interface.h"
#include "RLControllerFactory.h"

using namespace std;

const int NUM_TRAINING_EPOCHS = 2;
const int NUM_FUTURE_COMMANDS = 40;
const int NUM_MEASURED_SPEEDS = 40;
const int NUM_COMMANDS = 40;
const double EXPERIMENT_DURATION = 30;
const double EXPERIMENT_DELTA_T = 1.0 / 40.0;

static bool stop_requested = false;

carmen_robot_ackerman_config_t carmen_robot_ackerman_config;
double start_of_the_program;

RLController *rl_controller_phi;
RLController *rl_controller_velocity;

State state_phi, state_velocity;

fann_type steering_ann_input[NUM_STEERING_ANN_INPUTS];
struct fann *steering_ann = NULL;

fann_type velocity_ann_input[NUM_VELOCITY_ANN_INPUTS];
struct fann *velocity_ann = NULL;

double time_last_acceleration;
double current_v, current_phi;

double current_t = 0;

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
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		//publish_control_command(0.0, 100.0, 0.0, carmen_get_time());
		//carmen_navigator_ackerman_stop();
		stop_requested = true;

		carmen_ipc_disconnect();
	}
}


void
reinit_states()
{
	state_phi.clear();
	state_velocity.clear();

	for (int i = 0; i < NUM_FUTURE_COMMANDS; i++)
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
plot_phi(double current, double desired, double steering_command, double current_t, int reset = 0)
{
	//const int PAST_SIZE = 1000;

	static list<double> cphi;
	static list<double> dphi;
	static list<double> phi_cmd;
	static list<double> timestamp;

	static bool first_time = true;
	//static double first_timestamp;
	static FILE *gnuplot_pipe;

	list<double>::iterator itc;
	list<double>::iterator itd;
	list<double>::iterator itt;
	list<double>::iterator itcmd;

	//double t = current_t;

	if (reset)
	{
		cphi.clear();
		dphi.clear();
		phi_cmd.clear();
		timestamp.clear();
		//first_timestamp = t;
	}

	if (first_time)
	{
		//first_timestamp = t;
		first_time = false;

		gnuplot_pipe = popen("gnuplot", "w");
		fprintf(gnuplot_pipe, "set xrange [0:30]\n");
		fprintf(gnuplot_pipe, "set yrange [-0.55:0.55]\n");
	}

	cphi.push_back(current);
	dphi.push_back(desired);
	phi_cmd.push_back(steering_command);
	timestamp.push_back(current_t /*t - first_timestamp*/);

	//while(cphi.size() > PAST_SIZE)
	//{
	//	cphi.pop_back();
	//	dphi.pop_back();
	//	phi_cmd.pop_back();
	//	timestamp.pop_back();
	//}

	static int n = 0;

	if (n >= 60)
	{
		n = 0;

		FILE *gnuplot_data_file = fopen("gnuplot_phi_data.txt", "w");

		for (itc = cphi.begin(), itd = dphi.begin(), itt = timestamp.begin(), itcmd = phi_cmd.begin(); itc != cphi.end(); itc++, itd++, itt++, itcmd++)
			fprintf(gnuplot_data_file, "%lf %lf %lf %lf\n", *itt /*- timestamp.back()*/, *itc, *itd, *itcmd);

		fclose(gnuplot_data_file);

		fprintf(gnuplot_pipe, "plot './gnuplot_phi_data.txt' using 1:2 with lines title 'cphi','./gnuplot_phi_data.txt' using 1:3 with lines title 'dphi'\n");
		//fprintf(gnuplot_pipe, "replot './gnuplot_phi_data.txt' using 1:4 with lines axis x1y2 title 'phi_eff'\n");
		fflush(gnuplot_pipe);
	}
	else
		n++;
}


void
plot_velocity(double current, double desired, double velocity_command, double current_t, int reset = 0)
{
	//const int PAST_SIZE = 1000;

	static list<double> cphi;
	static list<double> dphi;
	static list<double> vcmd;
	static list<double> timestamp;

	static bool first_time = true;
	//static double first_timestamp;
	static FILE *gnuplot_pipe;

	list<double>::iterator itc;
	list<double>::iterator itd;
	list<double>::iterator itt;
	list<double>::iterator itcmd;

	//double t = current_t;

	if (reset)
	{
		cphi.clear();
		dphi.clear();
		vcmd.clear();
		timestamp.clear();
		//first_timestamp = t;
	}

	if (first_time)
	{
		//first_timestamp = t;
		first_time = false;

		gnuplot_pipe = popen("gnuplot", "w");
		fprintf(gnuplot_pipe, "set xrange [0:30]\n");
		fprintf(gnuplot_pipe, "set yrange [-30.0:30.0]\n");
	}

	cphi.push_back(current);
	dphi.push_back(desired);
	vcmd.push_back(velocity_command);
	timestamp.push_back(current_t /*t - first_timestamp*/);

	//printf("** %lf %lf %lf %lf\n", current_t, current, desired, velocity_command);
	//getchar();

	//while(cphi.size() > PAST_SIZE)
	//{
	//	cphi.pop_back();
	//	dphi.pop_back();
	//	vcmd.pop_back();
	//	timestamp.pop_back();
	//}

	static int n = 0;

	if (n >= 60)
	{
		n = 0;

		FILE *gnuplot_data_file = fopen("gnuplot_velocity_data.txt", "w");

		for (itc = cphi.begin(), itd = dphi.begin(), itt = timestamp.begin(), itcmd = vcmd.begin(); itc != cphi.end(); itc++, itd++, itt++, itcmd++)
			fprintf(gnuplot_data_file, "%lf %lf %lf %lf\n", *itt /*- timestamp.back()*/, *itc, *itd, *itcmd);

		fclose(gnuplot_data_file);

		fprintf(gnuplot_pipe, "plot './gnuplot_velocity_data.txt' using 1:2 with lines title 'cvel','./gnuplot_velocity_data.txt' using 1:3 with lines title 'dvel'\n");
		//fprintf(gnuplot_pipe, "replot './gnuplot_velocity_data.txt' using 1:4 with lines axis x1y2 title 'veff'\n");
		fflush(gnuplot_pipe);
	}
	else
		n++;
}


void
reinitize_experiment_configuration()
{
	episode_phi.clear();
	episode_velocity.clear();

	time_last_acceleration = 0.0; //carmen_get_time();

	current_v = 0.0;
	current_phi = 0.0;
	current_t = 0.0;

	plot_velocity(0, 0, 0, 0, 1);
	plot_phi(0, 0, 0, 0, 1);

	carmen_libcarneuralmodel_init_steering_ann_input(steering_ann_input);
	carmen_libcarneuralmodel_init_velocity_ann_input(velocity_ann_input);
}


void
control_step()
{
	//printf("current_t: %lf\n", current_t);
	//getchar();

	double max_phi = carmen_robot_ackerman_config.max_phi;

	pair<double, double> phi_command_and_reward;
	pair<double, double> vel_command_and_reward;

	TrainEvent event_v, event_phi;

	if (current_v > 1.0)
		time_last_acceleration = current_t /*carmen_get_time()*/;

	event_v.state = state_velocity;
	event_phi.state = state_phi;

	state_velocity.measured.push_back(current_v);
	state_velocity.measured.pop_front();

	state_phi.measured.push_back(current_phi);
	state_phi.measured.pop_front();

	state_phi.desired.clear();
	state_velocity.desired.clear();

	for (int i = 0; i < NUM_FUTURE_COMMANDS; i++)
	{
		if (current_t < 2 * M_PI || current_t > EXPERIMENT_DURATION - 2 * M_PI)
		{
			state_velocity.desired.push_back(0.0);
			state_phi.desired.push_back(0.0);
		}
		else
		{
			state_velocity.desired.push_back(sin(current_t + i * EXPERIMENT_DELTA_T) * 30.0);
			state_phi.desired.push_back(sin(current_t + i * EXPERIMENT_DELTA_T) * max_phi * 0.7);
		}
	}

	//printf("\n* PHI: ");
	phi_command_and_reward = rl_controller_phi->selectAction(state_phi, stderr, false);
	//printf("\n* VEL: ");
	vel_command_and_reward = rl_controller_velocity->selectAction(state_velocity, stderr, false);
	//getchar();

	static int n = 0;

	if (n > 200)
	{
		n = 0;

		fprintf(stderr, "VEL CMD: %lf VEL Q: %lf PHI CMD: %lf PHI Q: %lf\n",
				vel_command_and_reward.first, vel_command_and_reward.second,
				phi_command_and_reward.first, phi_command_and_reward.second);
		fflush(stderr);
	}
	else n++;

	state_velocity.last_commmands.push_back(vel_command_and_reward.first);
	state_velocity.last_commmands.pop_front();

	state_phi.last_commmands.push_back(phi_command_and_reward.first);
	state_phi.last_commmands.pop_front();

	static double last_velocity_diff = 0.0;
	static double last_phi_diff = 0.0;

	double velocity_diff = fabs(current_v - state_velocity.desired[0]);
	double phi_diff = fabs(current_phi - state_phi.desired[0]);

	double reward_velocity;
	double reward_phi;

	//reward_velocity = (velocity_diff < last_velocity_diff) ? (1.0) : (0.0);
	//reward_phi = (phi_diff < last_phi_diff) ? (1.0) : (0.0);
	reward_velocity = -velocity_diff / 100.0;
	reward_phi = -phi_diff / 10.0;

	last_velocity_diff = velocity_diff;
	last_phi_diff = phi_diff;

	//static FILE *f = fopen("bla_data.txt", "w");
	//fprintf(f, "%lf %lf\n", current_v, state_velocity.desired[0]);
	//fflush(f);

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

	//static int n = 0;
    //
	//if (n % 3 == 0)
	//{
	//	print_report(event_v.state, vel_command, state_velocity, reward_velocity, vel_command_and_reward.second, stderr, "VEL");
	//	print_report(event_phi.state, steering, state_phi, reward_phi, phi_command_and_reward.second, stderr, "PHI");
	//	fprintf(stderr, "\n");
	//	fflush(stderr);
	//	n = 0;
	//}
	//n++;

	plot_velocity(current_v, state_velocity.desired[0], vel_command, current_t);
	plot_phi(current_phi, state_phi.desired[0], steering, current_t);

	double curvature = carmen_get_curvature_from_phi(
			current_phi, current_v,
			carmen_robot_ackerman_config.understeer_coeficient,
			carmen_robot_ackerman_config.distance_between_front_and_rear_axles);

	current_phi = carmen_libcarneuralmodel_compute_new_phi_from_effort(
			steering, curvature, steering_ann_input, steering_ann,
			current_v, carmen_robot_ackerman_config.understeer_coeficient,
			carmen_robot_ackerman_config.distance_between_front_and_rear_axles,
			carmen_robot_ackerman_config.max_phi);

	current_v = carmen_libcarneuralmodel_compute_new_velocity_from_effort(
			throttle, brake, velocity_ann_input, velocity_ann, current_v);

	current_t += EXPERIMENT_DELTA_T;
}


void
initialize_global_structures()
{
	rl_controller_phi = RLControllerFactory::build(Q_LEARNING_CONTROLLER, "solver_qlearning_phi.prototxt");
	rl_controller_velocity = RLControllerFactory::build(Q_LEARNING_CONTROLLER, "solver_qlearning_v.prototxt");

	steering_ann = fann_create_from_file("steering_ann.net");
	velocity_ann = fann_create_from_file("velocity_ann.net");

	if (steering_ann == NULL)
		exit(printf("Error: Could not create steering_ann\n"));

	if (velocity_ann == NULL)
		exit(printf("Error: Could not create velocity_ann\n"));
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
			//if (i % 200 == 0)
			//{
			//	fprintf(stderr, "Training PHI SAMPLE: %d of %ld\n", i, episode_phi.size());
			//	fflush(stderr);
			//}

			rl_controller_phi->train(episode_phi[i].state, episode_phi[i].immediate_reward, episode_phi[i].next_state, episode_phi[i].command_and_reward, stderr);
		}

		for (int i = 0; i < episode_velocity.size() && !stop_requested; i++)
		{
			//if (i % 200 == 0)
			//{
			//	fprintf(stderr, "Training VELOCITY SAMPLE: %d of %ld\n", i, episode_velocity.size());
			//	fflush(stderr);
			//}

			rl_controller_velocity->train(episode_velocity[i].state, episode_velocity[i].immediate_reward, episode_velocity[i].next_state, episode_velocity[i].command_and_reward, stderr);
		}
	}
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
			{"robot", "interpolate_odometry", CARMEN_PARAM_ONOFF, &carmen_robot_ackerman_config.interpolate_odometry, 1, NULL},
			{"robot", "understeer_coeficient", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.understeer_coeficient, 0, NULL},
			{"robot", "max_steering_angle", CARMEN_PARAM_DOUBLE, &carmen_robot_ackerman_config.max_phi, 1, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}


int
main(int argc, char **argv)
{
	srand(time(NULL));

	start_of_the_program = carmen_get_time();

	initialize_global_structures();

	// to start the experiment the other modules
	// must have initialized.
	sleep(3);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);

	signal(SIGINT, shutdown_module);

	while (!stop_requested)
	{
		fprintf(stderr, "-----------------------------\n");
		fflush(stderr);

		reinit_states();
		reinitize_experiment_configuration();

		while (current_t < EXPERIMENT_DURATION && !stop_requested)
		{
			control_step();

			//if (fabs(current_t /*carmen_get_time()*/ - time_last_acceleration) > 5.0)
			//{
			//	fprintf(stderr, "Stopping experiment due to lack of activity...\n");
			//	episode_velocity[episode_velocity.size() - 1].immediate_reward -= 200.0;
			//	break;
			//}
		}

		//fprintf(stderr, "achieved_goal_in_this_experiment: %s\n", (achieved_goal_in_this_experiment != 0) ? ("true") : ("false"));
		//fprintf(stderr, "max_consecutive_goal_achievements: %d\n", max_consecutive_goal_achievements);
		//fprintf(stderr, "num_consecutive_goal_achievements: %d\n", num_consecutive_goal_achievements);

		train_episode();
	}

	rl_controller_phi->saveTrain();
	rl_controller_velocity->saveTrain();

	return (0);
}


