
#ifndef __CARMEN_COMM__
#define __CARMEN_COMM__

#include <vector>

void publish_stop_command();

void publish_command(std::vector<double> v, std::vector<double> phi, std::vector<double> dt,
		int publish_behavior_selector_state_flag, double x = 0, double y = 0, double th = 0);

void publish_goal_list(std::vector<double> x, std::vector<double> y,
		std::vector<double> th, std::vector<double> v, std::vector<double> phi,
		double timestamp);

void handle_messages(double how_long = 1e-3);

void reset_without_initial_pose();
void reset_initial_pose(double x, double y, double th);

std::vector<double> read_pose();
std::vector<double> read_truepos();
std::vector<double> read_laser();
std::vector<double> read_goal();
std::vector<double> read_rddf();
std::vector<double> read_commands();
int hit_obstacle();

void init();

void simulation_reset(double x, double y, double th, double v, double phi);
void simulation_step(double v, double phi, double delta_t);

std::vector<double> simulation_read_pose();
std::vector<double> simulation_read_goal();
std::vector<double> simulation_read_laser();

int simulation_hit_obstacle();

#endif
