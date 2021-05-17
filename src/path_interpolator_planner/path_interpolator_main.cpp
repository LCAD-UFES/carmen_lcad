#include <vector>
#include <carmen/carmen.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/motion_planner_interface.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>

using namespace std;

#define NUM_POSES_BETWEEN_WAYPOINTS 10

vector<carmen_point_t> goal_poses;

void
publish_interface_message(vector<carmen_point_t> &interpolated_path)
{
	carmen_navigator_ackerman_plan_message message;

	message.path_length = interpolated_path.size();
	message.timestamp = carmen_get_time();
	message.host = carmen_get_host();

	message.path = (carmen_robot_and_trailer_traj_point_t *) calloc (interpolated_path.size(), sizeof(carmen_robot_and_trailer_traj_point_t));

	for (uint i = 0; i < interpolated_path.size(); i++)
	{
		message.path[i].x = interpolated_path[i].x;
		message.path[i].y = interpolated_path[i].y;
		message.path[i].theta = interpolated_path[i].theta;

		message.path[i].v = 0.0;
		message.path[i].phi = 0.0;
	}

	static bool	first_time = true;
	IPC_RETURN_TYPE err;

	if (first_time)
	{
		err = IPC_defineMsg(CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME, IPC_VARIABLE_LENGTH, CARMEN_NAVIGATOR_ACKERMAN_PLAN_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME);
		first_time = false;
	}

	err = IPC_publishData(CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME, &message);
	carmen_test_ipc(err, "Could not publish", CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME);

	free(message.path);
}


void
publish_path_planner_message(vector<carmen_point_t> &interpolated_path)
{
	carmen_robot_and_trailer_traj_point_t *path;

	path = (carmen_robot_and_trailer_traj_point_t *) calloc (interpolated_path.size(), sizeof(carmen_robot_and_trailer_traj_point_t));

	for (uint i = 0; i < interpolated_path.size(); i++)
	{
		path[i].x = interpolated_path[i].x;
		path[i].y = interpolated_path[i].y;
		path[i].theta = interpolated_path[i].theta;

		path[i].v = 0.0;
		path[i].phi = 0.0;
	}

	carmen_motion_planner_publish_path_message(path, interpolated_path.size(), CARMEN_BEHAVIOR_SELECTOR_GRADIENT);
}


void
publish_interpolated_path(vector<carmen_point_t> &goal_poses_array)
{
	int i;
	double i_interp_val;
	vector<carmen_point_t> interpolated_path;

	gsl_interp_accel *x_acc = gsl_interp_accel_alloc ();
	gsl_interp_accel *y_acc = gsl_interp_accel_alloc ();
	gsl_interp_accel *theta_acc = gsl_interp_accel_alloc ();

	const gsl_interp_type *x_t = gsl_interp_cspline_periodic;
	const gsl_interp_type *y_t = gsl_interp_cspline_periodic;
	const gsl_interp_type *theta_t = gsl_interp_cspline_periodic;

	gsl_spline *x_spline = gsl_spline_alloc (x_t, goal_poses_array.size());
	gsl_spline *y_spline = gsl_spline_alloc (y_t, goal_poses_array.size());
	gsl_spline *theta_spline = gsl_spline_alloc (theta_t, goal_poses_array.size());

	double time[goal_poses_array.size()];
	double x[goal_poses_array.size()];
	double y[goal_poses_array.size()];
	double theta[goal_poses_array.size()];

	// preenche os vetores para interpolacao
	for (i = 0; i < (int) goal_poses_array.size(); i++)
	{
		time[i] = i;

		x[i] = goal_poses_array[i].x;
		y[i] = goal_poses_array[i].y;
		theta[i] = goal_poses_array[i].theta;
	}

    gsl_spline_init (x_spline, time, x, goal_poses_array.size());
    gsl_spline_init (y_spline, time, y, goal_poses_array.size());
    gsl_spline_init (theta_spline, time, theta, goal_poses_array.size());

    // realiza a interpolacao
	for (i_interp_val = 0.0; i_interp_val <= (double) (goal_poses_array.size() - 1); i_interp_val += (1.0 / NUM_POSES_BETWEEN_WAYPOINTS))
	{
		double x_val = gsl_spline_eval (x_spline, i_interp_val, x_acc);
		double y_val = gsl_spline_eval (y_spline, i_interp_val, y_acc);
		double theta_val = gsl_spline_eval (theta_spline, i_interp_val, theta_acc);

		carmen_point_t interpolated_path_waypoint;

		interpolated_path_waypoint.x = x_val;
		interpolated_path_waypoint.y = y_val;
		interpolated_path_waypoint.theta = theta_val;

		interpolated_path.push_back(interpolated_path_waypoint);
	}

    publish_path_planner_message(interpolated_path);

    printf("Published % 6ld interpolated points\n", interpolated_path.size());

    gsl_spline_free (x_spline);
    gsl_spline_free (y_spline);
    gsl_spline_free (theta_spline);

    gsl_interp_accel_free (x_acc);
    gsl_interp_accel_free (y_acc);
    gsl_interp_accel_free (theta_acc);
}


void
add_goal_handler(carmen_behavior_selector_add_goal_message *msg)
{
	carmen_point_t goal_pose;

	goal_pose.x = msg->goal.x;
	goal_pose.y = msg->goal.y;
	goal_pose.theta = msg->goal.theta;

	goal_poses.push_back(goal_pose);

	if (goal_poses.size() > 2)
		publish_interpolated_path(goal_poses);
}


static void
initialize_carmen(int argc, char** argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	carmen_subscribe_message(
			(char *)CARMEN_BEHAVIOR_SELECTOR_ADD_GOAL_NAME,
			(char *)CARMEN_BEHAVIOR_SELECTOR_ADD_GOAL_FMT,
			NULL, sizeof(carmen_behavior_selector_add_goal_message),
			(carmen_handler_t)add_goal_handler, CARMEN_SUBSCRIBE_LATEST);
}


int
main(int argc, char* argv[])
{
	initialize_carmen(argc, argv);
	carmen_ipc_dispatch();
	return 0;
}

