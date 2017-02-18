#include "udatmo_detector.h"

#include "detector.h"
#include "udatmo_interface.h"

using udatmo::getDetector;
using udatmo::Obstacles;

carmen_udatmo_moving_obstacles_message *carmen_udatmo_detector_detect(void)
{
	static carmen_udatmo_moving_obstacles_message *message = NULL;
	if (message == NULL)
		message = carmen_udatmo_new_moving_obstacles_message(NUM_OBSTACLES);

	Obstacles obstacles = getDetector().detect();
	memcpy(message->obstacles, &(obstacles[0]), NUM_OBSTACLES * sizeof(carmen_datmo_moving_obstacle));
	message->timestamp = obstacles.timestamp;

	return message;
}

void carmen_udatmo_detector_setup(int argc, char *argv[])
{
	getDetector().setup(argc, argv);
}

void carmen_udatmo_detector_update_distance_map(carmen_obstacle_distance_mapper_message *message)
{
	getDetector().update(message);
}

void carmen_udatmo_detector_update_globalpos(carmen_localize_ackerman_globalpos_message *message)
{
	getDetector().update(message);
}

void carmen_udatmo_detector_update_rddf(carmen_rddf_road_profile_message *message)
{
	getDetector().update(message);
}
