
#ifndef __CARMEN_COMM__
#define __CARMEN_COMM__

#include <vector>

void env_destroy();
void env_init();
std::vector<float> env_reset(float x, float y, float th);
std::vector<float> env_step(float v, float phi);
bool env_done();

#endif
