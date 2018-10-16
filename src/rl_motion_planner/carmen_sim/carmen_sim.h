
#ifndef _RL_MOTION_PLANNER_SIM_SIM_H_
#define _RL_MOTION_PLANNER_SIM_SIM_H_

#include <string>
#include <vector>
#include <carmen/carmen.h>
#include <prob_map.h>
#include <carmen/simulator_ackerman_simulation.h>

#include <opencv/cv.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

//using namespace cv;
using namespace std;


class CarmenSim
{
public:
	CarmenSim(bool fix_initial_position=false, bool use_truepos=true,
			bool allow_negative_commands=true,
			bool enjoy_mode=false,
			bool use_latency=false,
			const char *rddf_name="rddf_ida_guarapari-20170403.txt",
			const char *map_dir="map_ida_guarapari-20170403-2",
			double min_dist_to_initial_goal=5.,
			double max_dist_to_initial_goal=20.);

	void set_seed(int seed);
	void reset();
	void step(double v, double phi, double dt);
	void show(int time=1);

	vector<double> laser();
	vector<double> pose();
	vector<double> goal();

	bool hit_obstacle();

	vector< vector<double> > rddf_forward();
	vector< vector<double> > rddf_backward();

	void draw_pose(double x, double y, double th, int b, int g, int r);
	void draw_occupancy_map();
	void draw_poses(vector< vector<double> > poses, int b, int g, int r);

private:

	string _rddf_name;
	string _map_dir;
	bool _fix_initial_position;
	bool _use_truepos;
	bool _allow_negative_commands;
	bool _enjoy_mode;
	bool _use_latency;
	int _min_pose_skip_to_initial_goal;
	int _max_pose_skip_to_initial_goal;
	int _current_rddf_pose;
	bool _use_velocity_nn;
	bool _use_phi_nn;

	cv::Mat *_view;

	vector<carmen_ackerman_motion_command_t> _rddf;

	carmen_ackerman_motion_command_t _goal;

	bool _robot_is_on_map;
	int _viewer_subsampling;
	double _map_resolution;
	double _map_pixel_by_meter;
	carmen_prob_models_distance_map _obstacle_distance_map;

	carmen_robot_ackerman_config_t _robot_ackerman_config;
	carmen_simulator_ackerman_config_t _simulator_config;
	double _car_length, _car_width;

	carmen_laser_laser_message _front_laser;
	carmen_laser_laser_message _rear_laser;

	void _load_rddf(string path);
	void _load_params();

	// reload map if necessary, and recompute obstacle distance map
	void _update_map();
	void _update_simulator(double v, double phi, double dt);
	int _find_nearest_goal(double x, double y);

	int _my_counter;
};


int pose_is_outside_map(carmen_map_t &map, double x, double y);

#endif
