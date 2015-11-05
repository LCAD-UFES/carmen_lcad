
#ifndef CARMEN_VELODYNE_ICP_MESSAGES_H
#define CARMEN_VELODYNE_ICP_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {

	carmen_pose_3D_t cloud_transform;

	double timestamp;
	char *host;


} carmen_velodyne_icp_message;
  
#define      CARMEN_VELODYNE_ICP_NAME       "carmen_velodyne_icp_message"
#define      CARMEN_VELODYNE_ICP_FMT        "{{{double,double,double},{double,double,double}},double,string}"

#ifdef __cplusplus
}
#endif

#endif
// @}
