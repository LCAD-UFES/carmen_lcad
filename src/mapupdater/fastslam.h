#ifndef CARMEN_SLAM_MONTECARLO_H
#define CARMEN_SLAM_MONTECARLO_H

#include <carmen/global.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	carmen_map_set_t *map_set;

	carmen_map_t local_map;
	carmen_compact_map_t local_compacted_map;
	carmen_compact_map_t local_compacted_mean_remission_map;
	carmen_localize_ackerman_map_t localize_map;
} fastslam_map_t;


#ifdef __cplusplus
}
#endif

#endif
