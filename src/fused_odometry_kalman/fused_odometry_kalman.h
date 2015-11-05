/*
 * fused_odomety_kalman.h
 */

#ifndef FUSED_ODOMETY_KALMAN_H_
#define FUSED_ODOMETY_KALMAN_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "fused_odometry.h"
#include "xsens_xyz_handler.h"
#include "car_odometry_handler.h"


carmen_point_t
kalman_filter_correct_pose(sensor_vector_xsens_xyz **gps, sensor_vector_xsens_xyz **xsens, int gps_index, int xsens_index, carmen_base_ackerman_odometry_message *car_odometry, double L);


#endif /* FUSED_ODOMETY_KALMAN_H_ */
