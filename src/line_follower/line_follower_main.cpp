#include <carmen/carmen.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/robot_ackerman_interface.h>
#include <carmen/base_ackerman_interface.h>
#include <carmen/rddf_interface.h>
#include "g2o/types/slam2d/se2.h"

using namespace g2o;

const int NUM_MOTION_COMMANDS = 8;
const double CONTROL_FREQUENCY = 40.0;

const double DIST_TO_POINT_TO_GOAL = 1000.0;
const double DIST_TO_STOP_STEERING = 3.0;
const double STEERING_STOPPING_DELTA = (0.488677778 / 3.0) / CONTROL_FREQUENCY; // max phi ate 0 em 3 seg
const double STEERING_CHANGING_DELTA = (0.488677778 / 0.5) / CONTROL_FREQUENCY; // max phi ate 0 em 1 seg
const double MIN_SPEED_TO_START_STEERING = 1.0;

const double DIST_TO_REDUCE = 8.0;
const double DIST_TO_STOP = 2.0;
const double MAX_SPEED = 5.5;
const double TIME_TO_ACHIEVE_MAX_SPEED = 3.0;


carmen_localize_ackerman_globalpos_message globalpos;
carmen_behavior_selector_goal_list_message goal_list;
carmen_rddf_road_profile_message path_goals_and_annotations_message;


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("line_follower: disconnected.\n");
		exit(0);
	}
}


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

	carmen_robot_ackerman_publish_motion_command(motion_commands, NUM_MOTION_COMMANDS, carmen_get_time());
}


void
define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_NAME);
}


void
rddf_handler()
{
}


void
behaviour_selector_goal_list_message_handler()
{
}


void
carmen_localize_ackerman_handler()
{
}


void
show_freq()
{
	static int n = 0;
	static double t = carmen_get_time();

	if (carmen_get_time() - t > 1.0)
	{
		printf("n: %d\n", n);
		t = carmen_get_time();
		n = 0;
	}
	else n++;
}


double
dist(double x1, double y1, double x2, double y2)
{
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}


double
compute_v(double current_v, double d, SE2 &goal_pose_in_car_ref)
{
	double v = 0.0;
	double dv = 0.0;

	dv = (MAX_SPEED / TIME_TO_ACHIEVE_MAX_SPEED) / CONTROL_FREQUENCY;

	printf("d: %.2lf || ", d);

	if (d < DIST_TO_STOP + 0.5 && current_v < 0.1)
	{
		v = 0.0;
	}
	else
	{

		if (d < DIST_TO_STOP || goal_pose_in_car_ref[0] <= 0)
		{
			//if (d < DIST_TO_STOP) printf("Stopping because goal was achieved!\n");
			//else printf("Stopping because goal is behind car!\n");
			v = 0.0;
		}
		else if (d < DIST_TO_REDUCE)
		{
			double v_desired = (MAX_SPEED / DIST_TO_REDUCE) * (d - DIST_TO_STOP + 0.5);

			if (current_v < 0.25)
				v = 0.25;
			else if (current_v < v_desired)
//				v = current_v + dv;
				v = current_v + (v_desired - current_v) * 0.1;
			else
				v = v_desired;

			//printf("Approaching the goal! Current v: %lf New v: %lf\n", current_v, v);
		}
		else if (current_v < MAX_SPEED)
		{
			if (current_v < 0.25)
				v = 0.25;
			else
				//v = current_v + dv;
				v = current_v + (MAX_SPEED - current_v) * 0.1;
			//printf("Raising speed! Current v: %lf dv: %lf v: %lf\n", current_v, dv, v);
		}
		else
		{
			//printf("Maintaining max speed!\n");
			v = MAX_SPEED;
		}
	}
	return v;
}


SE2
compute_goal_pose_in_car_reference(carmen_point_t globalpos, carmen_ackerman_traj_point_t goal)
{
	SE2 p(globalpos.x, globalpos.y, globalpos.theta);
	SE2 g(goal.x, goal.y, goal.theta);
	SE2 o = p.inverse() * g;

	return o;
}


double
compute_phi(double current_v, double current_phi, double d, SE2 &goal_pose_in_car_ref)
{
	double phi = 0;

	if (d < DIST_TO_STOP_STEERING || goal_pose_in_car_ref[0] <= 0 /*|| current_v < MIN_SPEED_TO_START_STEERING*/)
	{
		if (current_v < 0.1 || fabs(current_phi) < carmen_degrees_to_radians(2.0))
			phi = 0.0;
		else if (current_phi > 0)
			phi = current_phi - STEERING_STOPPING_DELTA;
		else if (current_phi < 0)
			phi = current_phi + STEERING_STOPPING_DELTA;

		//printf("stopping phi! current: %lf new: %lf\n", current_phi, phi);
	}
	else if (d < DIST_TO_POINT_TO_GOAL)
	{
		double th = atan2(goal_pose_in_car_ref[1], goal_pose_in_car_ref[0]);
		// th = dt * (message->v / 2.625 /* L */) * tan(message->phi);
		// phi = atan((th * L) / (dt * v))
		double desired_phi = atan((th * 2.625) / (MAX_SPEED * 1.0));

		phi = desired_phi;
		//if (desired_phi - current_phi > 0) phi = current_phi + STEERING_CHANGING_DELTA;
		//else phi = current_phi - STEERING_CHANGING_DELTA;

		//printf("t: %lf curr phi: %lf des phi: %lf phi: %lf\n", th, current_phi, desired_phi, phi);
	}

	return phi;
}


void
base_ackerman_odometry_message_handler(carmen_base_ackerman_odometry_message *msg)
{
	double v = 0;
	double phi = 0;

	if (goal_list.size <= 0 || path_goals_and_annotations_message.number_of_poses <= 0)
	{
		printf("Empty goal list (%d) or rddf (%d)! Stopping the car!\n", goal_list.size, path_goals_and_annotations_message.number_of_poses);

		if (msg->v < 1.0)
		{
			v = 0.0;
			phi = 0.0;
		}
		else
		{
			v = msg->v / 2.0;
			phi = msg->phi / 2.0;
		}

		publish_motion_command(v, phi);
		return;
	}

	carmen_point_t predicted_pose;

	if (msg->timestamp > globalpos.timestamp)
	{
		double dt = fabs(msg->timestamp - globalpos.timestamp);

		predicted_pose.x = globalpos.globalpos.x + dt * msg->v * cos(globalpos.globalpos.theta);
		predicted_pose.y = globalpos.globalpos.y + dt * msg->v * sin(globalpos.globalpos.theta);
		predicted_pose.theta = globalpos.globalpos.theta + dt * (msg->v / 2.625 /* L */) * tan(msg->phi);
		predicted_pose.theta = carmen_normalize_theta(predicted_pose.theta);
	}
	else
		predicted_pose = globalpos.globalpos;

	SE2 goal_pose_in_car_ref = compute_goal_pose_in_car_reference(predicted_pose, goal_list.goal_list[goal_list.size - 1]);
	double d = dist(predicted_pose.x, predicted_pose.y, goal_list.goal_list[goal_list.size - 1].x, goal_list.goal_list[goal_list.size - 1].y);
	v = compute_v(msg->v, d, goal_pose_in_car_ref);
	phi = compute_phi(msg->v, msg->phi, d, goal_pose_in_car_ref);
	publish_motion_command(v, phi);

	printf("v: %lf phi: %lf || curr_v: %lf curr_phi: %lf\n", v, phi, msg->v, msg->phi);
}


int
main(int argc, char **argv)
{
	signal(SIGINT, shutdown_module);
	carmen_ipc_initialize(argc, argv);

	printf("3...\n");
	sleep(1);
	printf("2...\n");
	sleep(1);
	printf("1...\n");
	sleep(1);
	printf("GO!\n");

	carmen_localize_ackerman_subscribe_globalpos_message(&globalpos, (carmen_handler_t) carmen_localize_ackerman_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) base_ackerman_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_behavior_selector_subscribe_goal_list_message(&goal_list, (carmen_handler_t) behaviour_selector_goal_list_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_rddf_subscribe_road_profile_message(&path_goals_and_annotations_message, (carmen_handler_t) rddf_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();
	return 0;
}

