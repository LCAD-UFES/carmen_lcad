#ifndef ICP_PROCESSOR_H
#define ICP_PROCESSOR_H


#include <carmen/carmen.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct icp_processor icp_processor;

icp_processor* create_icp_processor(void);
void set_icp_cloud_1(icp_processor* icp, carmen_vector_3D_t* points, int num_points, int downsample);
void set_icp_cloud_2(icp_processor* icp, carmen_vector_3D_t* points, int num_points, int downsample);
void add_to_cloud_1(icp_processor* icp, carmen_vector_3D_t* points, int num_points, int downsample);
void add_to_cloud_2(icp_processor* icp, carmen_vector_3D_t* points, int num_points, int downsample);
void get_cloud_transformation(icp_processor* icp_p, Eigen::Matrix4f transform_guess);
void destroy_icp_processor(icp_processor* icp);

#ifdef __cplusplus
}
#endif

#endif
