/*
 * post_slam.h
 *
 *  Created on: Apr 12, 2013
 *      Author: filipe
 */

#ifndef _POST_LOCALIZE_H_
#define _POST_LOCALIZE_H_

#include <opencv/cv.h>
#include "landmark.h"
#include "landmark_map.h"

namespace post_slam
{
	/**
	 * TODO: fazer um metodo para gerar a confianca a partir da covariancia
	 */
	class PostLocalize
	{
		double alpha_1;
		double alpha_2;
		double alpha_3;
		double alpha_4;

		cv::Mat mean;
		cv::Mat covariance;
		post_slam::LandmarkMap landmark_map;

		cv::Mat calculate_G_t(double v_t, double w_t, double delta_t);
		cv::Mat calculate_V_t(double v_t, double w_t, double delta_t);
		cv::Mat calculate_M_t(double v_t, double w_t);
		cv::Mat calculate_motion_update(double v_t, double w_t, double delta_t);
		public:

			PostLocalize();
			PostLocalize(char *landmark_map_filename);
			~PostLocalize();

			void initialize(double x, double y, double theta);
			void load_landmark_map(char *filename);
			void integrate_movement_information(double v_t, double w_t, double delta_t);
			void integrate_measurement_information(int sensor_measurement);
			carmen_point_t get_pose();
	};
}

#endif /* _POST_LOCALIZE_H_ */
