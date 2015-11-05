#ifndef SLAM_ICP_VELODYNE_H
#define SLAM_ICP_VELODYNE_H

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_VELODYNE_POINT_CLOUDS	20


int slam_icp_velodyne_partial_scan(
		carmen_velodyne_partial_scan_message *velodyne_message,
		sensor_parameters_t *velodyne_params, sensor_data_t *velodyne_data,
		carmen_pose_3D_t *robot_pose, carmen_vector_3D_t *robot_velocity,
		double phi, double last_globalpos_timestamp, carmen_pose_3D_t *corrected_pose_out);

void initializeICP();

carmen_pose_3D_t apply_transform(carmen_pose_3D_t *robot_pose);

#ifdef __cplusplus
}
#endif

#endif // slam_icp_VELODYNE_H
