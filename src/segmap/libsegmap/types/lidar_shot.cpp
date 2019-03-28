
#include <cstdlib>
#include <carmen/lidar_shot.h>


LidarShot::LidarShot(int n_rays)
{
	n = n_rays;
	ranges = (double *) calloc(n, sizeof(double));
	v_angles = (double *) calloc(n, sizeof(double));
	intensities = (unsigned char *) calloc(n, sizeof(unsigned char));
	h_angle = 0;
}


LidarShot::~LidarShot()
{
	free(ranges);
	free(v_angles);
	free(intensities);
}
