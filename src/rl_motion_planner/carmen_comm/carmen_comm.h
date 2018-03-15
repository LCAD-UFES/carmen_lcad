
#ifndef __CARMEN_COMM__
#define __CARMEN_COMM__

#include <vector>

void env_destroy();
void env_init();

std::vector<double> env_reset(double pos_x, double pos_y, double pos_th,
							double goal_x=0, double goal_y=0, double goal_th=0,
							double goal_v=0, double goal_phi=0);

std::vector<double> env_step(double v, double phi,
							double goal_x=0, double goal_y=0, double goal_th=0,
							double goal_v=0, double goal_phi=0);

bool env_done();

void handle_messages();

void publish_starting_pose(double x, double y, double th);
void publish_command(double v, double phi);
int car_hit_obstacle();

std::vector<double> read_state();
std::vector<double> read_goal();


#endif
