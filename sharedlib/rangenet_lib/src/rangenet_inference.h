#ifndef RANGENETINFERENCE_H
#define RANGENETINFERENCE_H

// c++ stuff
#include <string>
#include <vector>

int *
rangenet_process_point_cloud(std::vector<float> values, uint32_t num_points, std::string scan);

void
rangenet_infer(std::vector<float> values, uint32_t num_points, std::string scan);

void
initialize_rangenet();

#endif
