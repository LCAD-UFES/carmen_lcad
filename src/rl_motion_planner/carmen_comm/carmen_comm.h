
#ifndef __CARMEN_COMM__
#define __CARMEN_COMM__

#include <vector>

void env_destroy();
void env_init();
std::vector<double> env_reset(double x, double y, double th);
std::vector<double> env_step(double v, double phi);
bool env_done();

#endif
