
#include <cstdio>
#include <cstdlib>
#include <string>
#include "segmap_carmen_param_file.h"

using namespace std;


int
main(int argc, char **argv)
{
	if (argc < 2)
		exit(printf("Use %s <path to a carmen.ini file>\n", argv[0]));

	CarmenParamFile params(argv[1]);

	printf("laser_num_laser_devices: %d\n", params.get<int>("laser_num_laser_devices"));
	printf("laser_use_device_locks: %d\n", params.get<int>("laser_use_device_locks"));
	printf("velodyne_mapper: %d\n", params.get<int>("velodyne_mapper"));
	printf("laser_laser1_dev: %s\n", params.get<string>("laser_laser1_dev").c_str());
	printf("laser_laser1_baud: %d\n", params.get<int>("laser_laser1_baud"));

	return 0;
}
