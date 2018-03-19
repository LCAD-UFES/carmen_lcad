
#include <carmen/carmen.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/robot_ackerman_interface.h>
#include <deque>
#include <vector>

using namespace std;


const int MAX_QUEUE_SIZE = 20;


deque<pair<carmen_vector_2D_t, double>> commands_queue;
deque<pair<carmen_ackerman_traj_point_t, double>> goals_queue;
deque<pair<vector<double>, double>> laser_readings_queue;


void
robot_ackerman_handler(carmen_robot_ackerman_motion_command_message *motion_command)
{
	if (motion_command->num_motion_commands <= 0 || motion_command->timestamp <= 0)
		return;

	pair<carmen_vector_2D_t, double> timestamped_command;

	timestamped_command.first.x = motion_command->motion_command[0].v;
	timestamped_command.first.y = motion_command->motion_command[0].phi;
	timestamped_command.second = motion_command->timestamp;

	commands_queue.push_back(timestamped_command);

	if (commands_queue.size() > MAX_QUEUE_SIZE)
		commands_queue.pop_front();
}


void
behavior_selector_handler(carmen_behavior_selector_goal_list_message *msg)
{
	if (msg->size <= 0 || msg->timestamp <= 0)
		return;

	pair<carmen_ackerman_traj_point_t, double> timestamped_goal;

	timestamped_goal.first = msg->goal_list[0];
	timestamped_goal.second = msg->timestamp;

	goals_queue.push_back(timestamped_goal);

	if (goals_queue.size() > MAX_QUEUE_SIZE)
		goals_queue.pop_front();
}


template<class T> int
find_most_synchronized_message(double t, deque<pair<T, double>> &queue)
{
	int i, id = 0;
	double dti, dt = fabs(queue[id].second - t);

	for (i = 1; i < queue.size(); i++)
	{
		dti = fabs(queue[i].second - t);

		if (dti < dt)
		{
			id = i;
			dt = dti;
		}
	}

	return id;
}


void
localize_ackerman_handler(carmen_localize_ackerman_globalpos_message *globalpos)
{
	if (goals_queue.size() <= 0 || commands_queue.size() <= 0 || laser_readings_queue.size() <= 0)
		return;

	double t = globalpos->timestamp;
	int goal_id = find_most_synchronized_message<carmen_ackerman_traj_point_t>(t, goals_queue);
	int command_id = find_most_synchronized_message<carmen_vector_2D_t>(t, commands_queue);
	int laser_id = find_most_synchronized_message<vector<double>>(t, laser_readings_queue);

	carmen_ackerman_traj_point_t goal = goals_queue[goal_id].first;
	carmen_vector_2D_t command = commands_queue[command_id].first;

	FILE *fptr = fopen("dataset.txt", "a");

	fprintf(fptr, "%lf %lf %lf %lf %lf ", globalpos->globalpos.x, globalpos->globalpos.y, globalpos->globalpos.theta, globalpos->v, globalpos->phi);
	fprintf(fptr, "%lf %lf %lf %lf %lf ", goal.x, goal.y, goal.theta, goal.v, goal.phi);
	fprintf(fptr, "%lf %lf ", command.x, command.y);
	fprintf(fptr, "%lf ", globalpos->timestamp);

	vector<double> readings = laser_readings_queue[laser_id].first;
	for (int i = 0; i < readings.size(); i++)
		fprintf(fptr, "%lf ", readings[i]);

	fprintf(fptr, "\n");
	fclose(fptr);
}


void
frontlaser_handler(carmen_laser_laser_message *frontlaser_message)
{
	if (frontlaser_message->timestamp <= 0. || frontlaser_message->num_readings <= 0)
		return;

	int i;
	vector<double> readings;

	for (i = 0; i < frontlaser_message->num_readings; i++)
		readings.push_back(frontlaser_message->range[i]);

	pair<vector<double>, double> timestamped_readings;

	timestamped_readings.first = readings;
	timestamped_readings.second = frontlaser_message->timestamp;

	laser_readings_queue.push_back(timestamped_readings);
}


int
main()
{
	char *argv[] = {"save_commands_dataset"};
	carmen_ipc_initialize(1, argv);

    carmen_localize_ackerman_subscribe_globalpos_message(NULL,
    	(carmen_handler_t) localize_ackerman_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_behavior_selector_subscribe_goal_list_message(NULL,
    	(carmen_handler_t) behavior_selector_handler,
    	CARMEN_SUBSCRIBE_LATEST);

	carmen_robot_ackerman_subscribe_motion_command(NULL,
		(carmen_handler_t) robot_ackerman_handler,
		CARMEN_SUBSCRIBE_LATEST);

	carmen_laser_subscribe_frontlaser_message(NULL,
		(carmen_handler_t) frontlaser_handler,
		CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();
}
