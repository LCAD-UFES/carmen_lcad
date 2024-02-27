/*
 * fused_odomety_kalman.h
 */

#ifndef FUSED_ODOMETY_KALMAN_H_
#define FUSED_ODOMETY_KALMAN_H_

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "fused_odometry.h"
#include "xsens_xyz_handler.h"
#include "car_odometry_handler.h"


carmen_point_t kalmanGPS(sensor_vector_xsens_xyz **xsens_gps, int xsens_gps_index);

int calc_closest_gps(sensor_vector_xsens_xyz **xsens_gps, int xsens_gps_index, double odom_timestamp);

cv::Mat kalman_prediction(sensor_vector_xsens_xyz **xsens_gps, int xsens_gps_index, double odom_timestamp, cv::Mat p_p);

carmen_point_t kalman_filter_correct_pose(sensor_vector_xsens_xyz **xsens_gps, int xsens_gps_index, carmen_base_ackerman_odometry_message *car_odometry, double L);


#endif /* FUSED_ODOMETY_KALMAN_H_ */
