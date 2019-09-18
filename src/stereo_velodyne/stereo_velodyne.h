#ifndef _CARMEN_STEREO_VELODYNE_H_
#define _CARMEN_STEREO_VELODYNE_H_

#include <carmen/carmen.h>
#include "stereo_velodyne_messages.h"
#include <carmen/stereo_util.h>


double *
get_stereo_velodyne_correction(int fliped, int camera, int resolution, int roi_ini, int roi_end, int bumblebee_basic_width, int bumblebee_basic_height);

void
convert_stereo_depth_map_to_velodyne_beams(stereo_util interface, float *disparity, int vertical_resolution,
		int horizontal_resolution, carmen_velodyne_shot *stereo_velodyne_scan,
		double range_max, int vertical_roi_ini, int vertical_roi_end, int horizontal_roi_ini, int horizontal_roi_end, unsigned char *image);

void
convert_stereo_depth_to_velodyne_beams(stereo_util interface, unsigned short *depth, int vertical_resolution,
		int horizontal_resolution, carmen_velodyne_shot *stereo_velodyne_scan,
		unsigned short range_max, int vertical_roi_ini, int vertical_roi_end, int horizontal_roi_ini, int horizontal_roi_end, unsigned char *image);

void
convert_stereo_depth_map_to_velodyne_beams_and_flip(stereo_util interface, float *disparity, int vertical_resolution,
		int horizontal_resolution, carmen_velodyne_shot *stereo_velodyne_scan,
		double range_max, int vertical_roi_ini, int vertical_roi_end, int horizontal_roi_ini, int horizontal_roi_end);

carmen_pose_3D_t
get_stereo_velodyne_pose_3D(int argc, char **argv,int camera);


#endif
