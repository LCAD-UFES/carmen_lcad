
#ifndef VELODYNE_UTIL_H_
#define VELODYNE_UTIL_H_

void load_parameters(int argc, char **argv);
int accumulate_clouds(carmen_velodyne_partial_scan_message *velodyne_message, char *velodyne_storage_dir, double v, double phi);

#endif
