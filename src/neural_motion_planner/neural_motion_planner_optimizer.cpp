#include <stdio.h>
#include <iostream>
#include <math.h>

#include <carmen/collision_detection.h>
#include <carmen/carmen.h>

#include "robot_state.h"
#include "model/global_state.h"
#include "util.h"
#include "neural_motion_planner_optimizer.h"

#include <fstream>
#include <vector>
#include <cmath>
#include <limits>

#define G_STEP_SIZE	0.001
#define F_STEP_SIZE	G_STEP_SIZE

#define G_TOL		0.01
#define F_TOL		G_TOL

#define G_EPSABS	0.016
#define F_EPSABS	G_EPSABS

bool use_obstacles = true;

double steering_previous = 0.0;

extern int use_unity_simulator;

template<typename T>
std::vector<double> linspace(T start_in, T end_in, int num_in)
{

  std::vector<double> linspaced;

  double start = static_cast<double>(start_in);
  double end = static_cast<double>(end_in);
  double num = static_cast<double>(num_in);

  if (num == 0) { return linspaced; }
  if (num == 1) 
    {
      linspaced.push_back(start);
      return linspaced;
    }

  double delta = (end - start) / (num - 1);

  for(int i=0; i < num-1; ++i)
    {
      linspaced.push_back(start + delta * i);
    }
  linspaced.push_back(end); 
  return linspaced;
}


void print_vector(std::vector<double> vec)
{
  std::cout << "size: " << vec.size() << std::endl;
  for (double d : vec)
    std::cout << d << " ";
  std::cout << std::endl;
}


std::vector<double> get_predicted_vehicle_location(double x, double y, double steering_angle, double yaw, double v, double t) {
	double wheel_heading = yaw + steering_angle;
	double wheel_traveled_dis = v * t; //(timestamp - this->vars.t_previous);
	return {x + wheel_traveled_dis * cos(wheel_heading), y + wheel_traveled_dis * sin(wheel_heading)};
}


double get_distance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}


carmen_robot_and_trailers_path_point_t
convert_to_carmen_robot_and_trailer_path_point_t(const carmen_robot_and_trailers_traj_point_t robot_state, const double time)
{
	carmen_robot_and_trailers_path_point_t path_point;

	path_point.x = robot_state.x;
	path_point.y = robot_state.y;
	path_point.theta = robot_state.theta;
	path_point.v = robot_state.v;
	path_point.phi = robot_state.phi;
	path_point.time = time;

	return (path_point);
}


double
compute_path_via_simulation(carmen_robot_and_trailers_traj_point_t &robot_state, Command &command,
		vector<carmen_robot_and_trailers_path_point_t> &path,
		TrajectoryControlParameters tcp,
		double v0, double delta_t)
{

	robot_state.x = 0.0;
	robot_state.y = 0.0;
	robot_state.theta = 0.0;
	robot_state.v = v0;
	robot_state.phi = 0.0;

	command.v = v0;
	double multiple_delta_t = 3.0 * delta_t;
	int i = 0;
	double t;
	double distance_traveled = 0.0;
	path.push_back(convert_to_carmen_robot_and_trailer_path_point_t(robot_state, delta_t));
	
	std::vector<double> steering_list = linspace(-0.05235988, 0.05235988, 21);
	
	for (t = delta_t; t < tcp.tt; t += delta_t)
	{
		t = tcp.tt;
		command.v = v0 + tcp.a * t;
		command.v = GlobalState::robot_config.max_v;
		if (command.v > GlobalState::param_max_vel)
			command.v = GlobalState::param_max_vel;
		else if (command.v < GlobalState::param_max_vel_reverse)
			command.v = GlobalState::param_max_vel_reverse;
		
		for (auto& steering : steering_list) {
			steering += steering_previous;
		}

		double minimum_d = std::numeric_limits<double>::infinity();
		for (unsigned int i = 0; i < steering_list.size(); i++) {
    		std::vector<double> predicted_vehicle_location = get_predicted_vehicle_location(GlobalState::localizer_pose->x, GlobalState::localizer_pose->y, steering_list[i], GlobalState::localizer_pose->theta, command.v, t);
			double d_to_s1 = get_distance(predicted_vehicle_location[0], predicted_vehicle_location[1], GlobalState::goal_pose->x, GlobalState::goal_pose->y);
			if (d_to_s1 < minimum_d) { 
        		command.phi = steering_list[i]; 
        		minimum_d = d_to_s1;
    		}
		}
		steering_previous = command.phi;

		if ((GlobalState::semi_trailer_config.num_semi_trailers != 0) && (GlobalState::route_planner_state ==  EXECUTING_OFFROAD_PLAN))
			robot_state = carmen_libcarmodel_recalc_pos_ackerman(robot_state, command.v, command.phi, delta_t,
					&distance_traveled, delta_t / 10.0, GlobalState::robot_config, GlobalState::semi_trailer_config);
		else
			robot_state = carmen_libcarmodel_recalc_pos_ackerman(robot_state, command.v, command.phi, t / 2,
					&distance_traveled, delta_t, GlobalState::robot_config, GlobalState::semi_trailer_config);

		path.push_back(convert_to_carmen_robot_and_trailer_path_point_t(robot_state, delta_t));
		
		if (GlobalState::eliminate_path_follower && (i > 70))
			delta_t = multiple_delta_t;
		i++;
	}
	
	return (distance_traveled);
}


double
get_max_distance_in_path(vector<carmen_robot_and_trailers_path_point_t> path, carmen_robot_and_trailers_path_point_t &furthest_point)
{
	double max_dist = 0.0;

	furthest_point = path[0];
	for (unsigned int i = 0; i < path.size(); i++)
	{
		double distance = DIST2D(path[0], path[i]);
		if (distance > max_dist)
		{
			max_dist = distance;
			furthest_point = path[i];
		}
	}

	return (max_dist);
}


vector<carmen_robot_and_trailers_path_point_t>
simulate_car_from_parameters(TrajectoryDimensions &td,
		TrajectoryControlParameters &tcp, double v0, double delta_t)
{
	vector<carmen_robot_and_trailers_path_point_t> path = {};
	if (!tcp.valid)
		return (path);

	Command command;
	carmen_robot_and_trailers_traj_point_t robot_state;
	double distance_traveled = compute_path_via_simulation(robot_state, command, path, tcp, v0, delta_t);

	carmen_robot_and_trailers_path_point_t furthest_point;
	td.dist = get_max_distance_in_path(path, furthest_point);
	td.theta = atan2(furthest_point.y, furthest_point.x);
	td.d_yaw = furthest_point.theta;
	td.phi_i = 0.0;
	td.v_i = v0;
	tcp.vf = command.v;
	tcp.sf = distance_traveled;
	td.control_parameters = tcp;

	return (path);
}


bool
path_has_loop(double dist, double sf)
{
	if (dist < 0.05)
		return (false);

	if (sf > (M_PI * dist * 1.1))
		return (true);
	return (false);
}


void
compute_a_and_t_from_s_reverse(double s, double target_v,
		TrajectoryDimensions target_td,
		TrajectoryControlParameters &tcp_seed,
		ObjectiveFunctionParams *params)
{
	// https://www.wolframalpha.com/input/?i=solve+s%3Dv*x%2B0.5*a*x%5E2
	double a = (target_v * target_v - target_td.v_i * target_td.v_i) / (2.0 * target_td.dist);
	a = (-1.0) * a;
	if (a == 0.0)
	{
		if (target_v != 0.0)
			tcp_seed.tt = fabs(s / target_v);
		else
			tcp_seed.tt = 0.05;
	}
	else if (a < -GlobalState::robot_config.maximum_acceleration_reverse)
	{
		a = GlobalState::robot_config.maximum_acceleration_reverse;
		double v = fabs(target_td.v_i);
		tcp_seed.tt = (sqrt(fabs(2.0 * a * s + v * v) + v)) / a;
		a = (-1.0) * a;

	}
	else if (a > GlobalState::robot_config.maximum_deceleration_reverse)
	{
		a = GlobalState::robot_config.maximum_deceleration_reverse;
		double v = fabs(target_td.v_i);
		tcp_seed.tt = (sqrt(fabs(2.0 * a * s + v * v)) + v) / a;
	}
	else
		tcp_seed.tt = (target_v - target_td.v_i) / a;

	if (tcp_seed.tt > 200.0)
		tcp_seed.tt = 200.0;
	if (tcp_seed.tt < 0.05)
		tcp_seed.tt = 0.05;

	params->suitable_tt = tcp_seed.tt;
	params->suitable_acceleration = tcp_seed.a = a;
}


void
compute_a_and_t_from_s_foward(double s, double target_v,
		TrajectoryDimensions target_td,
		TrajectoryControlParameters &tcp_seed,
		ObjectiveFunctionParams *params)
{

	// https://www.wolframalpha.com/input/?i=solve+s%3Dv*x%2B0.5*a*x%5E2
	double a = (target_v * target_v - target_td.v_i * target_td.v_i) / (2.0 * target_td.dist);
	double v = target_td.v_i;
	if (a == 0.0)
	{
		if (target_v != 0.0) {
			tcp_seed.tt = s / target_v;
		} else {
			tcp_seed.tt = 0.05;
		}
	}
	else if (a > GlobalState::robot_config.maximum_acceleration_forward)
	{
		a = GlobalState::robot_config.maximum_acceleration_forward;
		tcp_seed.tt = (sqrt(2.0 * a * s + v * v) - v) / a;
	}
	else if (a < -GlobalState::robot_config.maximum_deceleration_forward)
	{
		a = -GlobalState::robot_config.maximum_deceleration_forward;
		tcp_seed.tt = (sqrt(2.0 * a * s + v * v) - v) / a;
	}
	else {
		tcp_seed.tt = (target_v - target_td.v_i) / a;
	}
	if (tcp_seed.tt > 200.0)
		tcp_seed.tt = 200.0;
	if (tcp_seed.tt < 0.05)
		tcp_seed.tt = 0.05;

	params->suitable_tt = tcp_seed.tt;
	params->suitable_acceleration = tcp_seed.a = a;
}


void
compute_a_and_t_from_s(double s, double target_v,
		TrajectoryDimensions target_td,
		TrajectoryControlParameters &tcp_seed,
		ObjectiveFunctionParams *params)
{
	if (GlobalState::reverse_planning)
		compute_a_and_t_from_s_reverse(s, target_v, target_td, tcp_seed, params);
	else
		compute_a_and_t_from_s_foward(s, target_v, target_td, tcp_seed, params);
}


void
move_path_to_current_robot_pose(vector<carmen_robot_and_trailers_path_point_t> &path, carmen_robot_and_trailers_pose_t *localizer_pose)
{
	for (std::vector<carmen_robot_and_trailers_path_point_t>::iterator it = path.begin(); it != path.end(); ++it)
	{
		double x = localizer_pose->x + it->x * cos(localizer_pose->theta) - it->y * sin(localizer_pose->theta);
		double y = localizer_pose->y + it->x * sin(localizer_pose->theta) + it->y * cos(localizer_pose->theta);
		it->x = x;
		it->y = y;
		it->theta = carmen_normalize_theta(it->theta + localizer_pose->theta);
	}
}


carmen_robot_and_trailers_path_point_t
move_to_front_axle(carmen_robot_and_trailers_path_point_t pose)
{
	return (pose);

	double L = GlobalState::robot_config.distance_between_front_and_rear_axles;
	carmen_robot_and_trailers_path_point_t pose_moved = pose;
	pose_moved.x += L * cos(pose.theta);
	pose_moved.y += L * sin(pose.theta);

	return (pose_moved);
}


void
compute_suitable_acceleration_and_tt(ObjectiveFunctionParams &params,
		TrajectoryControlParameters &tcp_seed,
		TrajectoryDimensions target_td, double target_v)
{
	// (i) S = Vo*t + 1/2*a*t^2
	// (ii) dS/dt = Vo + a*t
	// dS/dt = 0 => máximo ou mínimo de S => 0 = Vo + a*t; a*t = -Vo; (iii) a = -Vo/t; (iv) t = -Vo/a
	// Se "a" é negativa, dS/dt = 0 é um máximo de S
	// Logo, como S = target_td.dist, "a" e "t" tem que ser tais em (iii) e (iv) que permitam que
	// target_td.dist seja alcançada.
	//
	// O valor de maxS pode ser computado substituindo (iv) em (i):
	// maxS = Vo*-Vo/a + 1/2*a*(-Vo/a)^2 = -Vo^2/a + 1/2*Vo^2/a = -1/2*Vo^2/a
	//
	// Se estou co velocidade vi e quero chagar a vt, sendo que vt < vi, a eh negativo. O tempo, tt, para
	// ir de vi a vt pode ser derivado de dS/dt = Vo + a*t -> vt = vi + a*tt; a*tt = vt - vi; tt = (vt - vi) / a

	if (!GlobalState::reverse_planning && target_v < 0.0)
		target_v = 0.0;

	tcp_seed.s = target_td.dist; 
	compute_a_and_t_from_s(tcp_seed.s, target_v, target_td, tcp_seed, &params);
}


vector<carmen_robot_and_trailers_path_point_t>
move_detailed_lane_to_front_axle(vector<carmen_robot_and_trailers_path_point_t> &detailed_lane)
{
	for (unsigned int i = 0; i < detailed_lane.size(); i++)
		detailed_lane[i] = (move_to_front_axle(detailed_lane[i]));

	return (detailed_lane);
}


TrajectoryControlParameters
get_complete_optimized_trajectory_control_parameters(TrajectoryControlParameters previous_tcp,
		TrajectoryDimensions target_td, double target_v,
		vector<carmen_robot_and_trailers_path_point_t> detailed_lane, bool use_lane)
{
	GlobalState::eliminate_path_follower = 0;

	ObjectiveFunctionParams params;
	params.use_lane = use_lane;

	if (detailed_lane.size() > 1)
	{
		if (GlobalState::reverse_planning)
			params.detailed_lane = detailed_lane;
		else
			params.detailed_lane = move_detailed_lane_to_front_axle(detailed_lane);
	}
	else
		params.use_lane = false;

	TrajectoryControlParameters tcp_seed;
	compute_suitable_acceleration_and_tt(params, tcp_seed, target_td, target_v);
	TrajectoryControlParameters tcp_complete = tcp_seed;
	GlobalState::eliminate_path_follower = 1;

	return (tcp_complete);
}