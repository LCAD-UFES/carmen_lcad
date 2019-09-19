#include <carmen/carmen.h>
#include <carmen/global.h>
#include <carmen/stereo_util.h>
#include <carmen/stereo_velodyne_interface.h>
#include <carmen/stereo_messages.h>
#include <carmen/stereo_interface.h>

//#include <Eigen/Core>
//
//#include <pcl/common/eigen.h>
//#include <pcl/common/transforms.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>
//
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/common/angles.h>
//#include <pcl/filters/filter.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/io/ply_io.h>

void
find_stereo_vertical_correction(stereo_util interface, double **vertical_correction_out, int vertical_resolution,
		int roi_ini, int roi_end, int flipped __attribute__ ((unused)))
{
	int y, i, inc_vertical;
	double *vertical_correction;
	int height = roi_end - roi_ini;

	inc_vertical = (int) (0.5 + ((double) height / (double) vertical_resolution));

	vertical_correction = (double *) calloc(vertical_resolution, sizeof(double));
	carmen_test_alloc(vertical_correction);

	for (y = roi_ini, i = vertical_resolution - 1; y < roi_end; y += inc_vertical, i--)
		vertical_correction[i] = -(carmen_radians_to_degrees(atan((y - (((double)interface.height / 2.0))) / interface.fy)));

	*vertical_correction_out = vertical_correction;
}

void
find_stereo_horizontal_correction(stereo_util interface, double **horizontal_correction_out, int horizontal_resolution,int roi_ini, int roi_end)
{
	int x, j, inc_horizontal;
	double *horizontal_correction;

	int width = roi_end - roi_ini;

	inc_horizontal =  (int) (0.5 + ((double) width / (double) horizontal_resolution) );

	horizontal_correction = (double *) calloc(horizontal_resolution, sizeof(double));
	carmen_test_alloc(horizontal_correction);

	for (x = roi_ini, j = horizontal_resolution - 1; x <  roi_end; x += inc_horizontal, j--)
		horizontal_correction[j] = -carmen_radians_to_degrees(atan((x - (((double)interface.width) / 2.0)) / interface.fx));

	*horizontal_correction_out = horizontal_correction;
}


double *
get_stereo_velodyne_correction(int fliped, int camera, int resolution, int roi_ini, int roi_end, int bumblebee_basic_width, int bumblebee_basic_height)
{
	double *correction;
	stereo_util interface;

	interface = get_stereo_instance(camera, bumblebee_basic_width, bumblebee_basic_height);

	if (!fliped)
		find_stereo_vertical_correction(interface, &correction, resolution, roi_ini, roi_end, fliped);
	else
		find_stereo_horizontal_correction(interface, &correction, resolution, roi_ini, roi_end);

	return correction;
}

//

//void
//statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, int setMeanK, double StddevMulThresh)
//{
//       pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statisticalOutlierRemoval;
//       pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudFiltered (new pcl::PointCloud<pcl::PointXYZRGB>);
//
//       statisticalOutlierRemoval.setInputCloud(pointCloud);
//       statisticalOutlierRemoval.setMeanK(setMeanK);
//       statisticalOutlierRemoval.setStddevMulThresh(StddevMulThresh);
//       statisticalOutlierRemoval.filter(*pointCloudFiltered);
//       *pointCloud = *pointCloudFiltered;
//}


//pcl::PointCloud<pcl::PointXYZRGB>::Ptr
//LeafSize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputPointCloud, double size)
//{
//       pcl::VoxelGrid<pcl::PointXYZRGB> grid;
//       pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputPointCloud  (new pcl::PointCloud<pcl::PointXYZRGB>);
//       grid.setLeafSize(size, size, size);
//       grid.setInputCloud(inputPointCloud);
//       grid.filter(*outputPointCloud);
//       return outputPointCloud;
//}


void
convert_stereo_depth_map_to_velodyne_beams(stereo_util interface, float *disparity, int vertical_resolution,
		int horizontal_resolution, carmen_velodyne_shot *stereo_velodyne_scan,
		double range_max, int vertical_roi_ini, int vertical_roi_end, int horizontal_roi_ini, int horizontal_roi_end, unsigned char *image)
{
	int x, y, inc_vertical, inc_horizontal, i, j;
	carmen_vector_3D_p point;
	int width = horizontal_roi_end - horizontal_roi_ini;
	int height = vertical_roi_end - vertical_roi_ini;

	inc_vertical = (int)(((double) height / (double) vertical_resolution) + 0.5);
	inc_horizontal =  (int)(((double) width / (double) horizontal_resolution) + 0.5);

	for (x = horizontal_roi_ini, j = horizontal_resolution - 1; x <  horizontal_roi_end; x += inc_horizontal, j--)
	{
		stereo_velodyne_scan[j].angle = carmen_radians_to_degrees(atan((x - (((double)interface.width) / 2.0)) / interface.fx));

		for (y = vertical_roi_ini, i = vertical_resolution - 1; y < vertical_roi_end; y += inc_vertical, i--)
		{
			carmen_position_t p;
			p.x = x;
			p.y = y;
			point = reproject_single_point_to_3D(&interface, p, disparity[y * interface.width + x]);

			if (point != NULL)
			{
				double range = sqrt(DOT3D((*point), (*point)));
				range = range > range_max ? 0.0 : range;

				stereo_velodyne_scan[j].distance[i] = (unsigned short) (range * 500.0);
				stereo_velodyne_scan[j].intensity[i] = image[y * interface.width + x];

				free(point);
				point = NULL;
			}
			else
			{
				//stereo_velodyne_scan[j].distance[i] = range_max * 500.0;
				stereo_velodyne_scan[j].distance[i] = 0.0;
			}
		}
	}
}

void
convert_stereo_depth_to_velodyne_beams(stereo_util interface, unsigned short *depth, int vertical_resolution,
		int horizontal_resolution, carmen_velodyne_shot *stereo_velodyne_scan,
		unsigned short range_max, int vertical_roi_ini, int vertical_roi_end, int horizontal_roi_ini, int horizontal_roi_end, unsigned char *image,
		float stride_x=1.0, float stride_y=1.0)
{
	int x, y, inc_vertical, inc_horizontal, i, j;
	carmen_vector_3D_p point;
	int width = horizontal_roi_end - horizontal_roi_ini;
	int height = vertical_roi_end - vertical_roi_ini;

	inc_vertical = (int)(((double) height / (double) vertical_resolution) + 0.5);
	inc_horizontal =  (int)(((double) width / (double) horizontal_resolution) + 0.5);

	for (x = horizontal_roi_ini, j = horizontal_resolution/stride_x - 1; x <  horizontal_roi_end; x += stride_x*inc_horizontal, j--)
	{
		stereo_velodyne_scan[j].angle = carmen_radians_to_degrees(atan((x - (((double)interface.width) / 2.0)) / interface.fx));

		for (y = vertical_roi_ini, i = vertical_resolution/stride_y - 1; y < vertical_roi_end; y += stride_y*inc_vertical, i--)
		{
			carmen_position_t p;
			p.x = x;
			p.y = y;
			point = reproject_single_point_to_3D_with_depth(interface, p, depth[y * interface.width + x]);

			if (point != NULL)
			{
				double range = sqrt(DOT3D((*point), (*point)));
				range = range > range_max ? 0.0 : range;

				stereo_velodyne_scan[j].distance[i] = (unsigned short) (range * 500.0);
				stereo_velodyne_scan[j].intensity[i] = image[y * interface.width + x];

				free(point);
				point = NULL;
			}
			else
			{
				//stereo_velodyne_scan[j].distance[i] = range_max * 500.0;
				stereo_velodyne_scan[j].distance[i] = 0.0;
			}
//			stereo_velodyne_scan[j].distance[i] = depth[y * interface.width + x] > range_max ? 0.0 : depth[y * interface.width + x];
//			stereo_velodyne_scan[j].intensity[i] = image[y * interface.width + x];
		}
	}
}


void
convert_stereo_depth_map_to_velodyne_beams_and_flip(stereo_util interface, float *disparity, int vertical_resolution,
		int horizontal_resolution, carmen_velodyne_shot *stereo_velodyne_scan,
		double range_max, int vertical_roi_ini, int vertical_roi_end, int horizontal_roi_ini, int horizontal_roi_end)
{
	int x, y, inc_vertical, inc_horizontal, i, j;
	carmen_vector_3D_p point;

	int width = horizontal_roi_end - horizontal_roi_ini;
	int height = vertical_roi_end - vertical_roi_ini;

	inc_vertical = (int)(((double) height / (double) vertical_resolution) + 0.5);
	inc_horizontal =  (int)(((double) width / (double) horizontal_resolution) + 0.5);

	for (y = vertical_roi_ini, j = vertical_resolution - 1; y < vertical_roi_end; y += inc_vertical, j--)
	{
		stereo_velodyne_scan[j].angle = -carmen_radians_to_degrees(atan((y - (((double)interface.height / 2.0))) / interface.fy));

		for (x = horizontal_roi_ini, i = horizontal_resolution - 1; x <  horizontal_roi_end; x += inc_horizontal, i--)
		{
			carmen_position_t p;
			p.x = x;
			p.y = y;

			point = reproject_single_point_to_3D(&interface, p, disparity[y * interface.width + x]);

			if(point != NULL)
			{
				double range = sqrt(DOT3D((*point), (*point)));
				range = range > range_max ? 0.0 : range;

				stereo_velodyne_scan[j].distance[i] = (unsigned short)(range * 500.0);

				free(point);
				point = NULL;

			}
			else
			{
				//stereo_velodyne_scan[j].distance[i] = range_max * 500.0;
				stereo_velodyne_scan[j].distance[i] = 0.0;
			}

		}
	}
}


carmen_pose_3D_t
get_stereo_velodyne_pose_3D(int argc, char **argv, int camera)
{
	carmen_pose_3D_t pose;

	char stereo_velodyne_string[256];
	argc = 1;

	sprintf(stereo_velodyne_string, "%s%d", "camera", camera);

	carmen_param_t param_list[] = {
		{(char*)stereo_velodyne_string, (char*)"x", CARMEN_PARAM_DOUBLE, &(pose.position.x), 1, NULL},
		{(char*)stereo_velodyne_string, (char*)"y", CARMEN_PARAM_DOUBLE, &(pose.position.y), 1, NULL},
		{(char*)stereo_velodyne_string, (char*)"z", CARMEN_PARAM_DOUBLE, &(pose.position.z), 1, NULL},
		{(char*)stereo_velodyne_string, (char*)"roll", CARMEN_PARAM_DOUBLE, &(pose.orientation.roll), 1, NULL},
		{(char*)stereo_velodyne_string, (char*)"pitch", CARMEN_PARAM_DOUBLE,&(pose.orientation.pitch), 1, NULL},
		{(char*)stereo_velodyne_string, (char*)"yaw", CARMEN_PARAM_DOUBLE, &(pose.orientation.yaw), 1, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list)/sizeof(param_list[0]));

	return pose;
}
