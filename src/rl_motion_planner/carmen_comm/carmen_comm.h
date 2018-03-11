
#ifndef __CARMEN_COMM__
#define __CARMEN_COMM__

#include <vector>

void env_destroy();
void env_init();

std::vector<double> env_reset(double pos_x, double pos_y, double pos_th,
							double goal_x, double goal_y, double goal_th,
							double goal_v, double goal_phi);

std::vector<double> env_step(double v, double phi,
							double goal_x, double goal_y, double goal_th,
							double goal_v, double goal_phi);

bool env_done();

#endif
