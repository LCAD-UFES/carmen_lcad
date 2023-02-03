#ifndef LASER_MAPPING_H
#define LASER_MAPPING_H


#ifdef __cplusplus
extern "C"
{
#endif

std::pair<carmen_vector_3D_t, carmen_quaternion_t>
process_lm(std::vector<pcl::PointCloud<PointType>> vector_cloud_in,
		std::pair<carmen_vector_3D_t, carmen_quaternion_t> pair_in, pcl::PointCloud<PointType>::Ptr &cloud_out);

#ifdef __cplusplus
}
#endif

#endif

