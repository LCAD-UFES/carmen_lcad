#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <carmen/carmen.h>
#include <carmen/gps_xyz_interface.h>
#include "fused_odometry.h"
#include "xsens_xyz_handler.h"
#include "car_odometry_handler.h"

#define WINDOW_SIZE 150

cv::Mat
kalmanGPS(sensor_vector_xsens_xyz **xsens_gps, int xsens_gps_index, int w_size)
{
	int i, index;
	int count = 0;

	cv::Mat sigma_corr = cv::Mat::eye(2, 2, CV_64F) * 2.0;
	cv::Mat sigma_p = cv::Mat::zeros(2, 2, CV_64F);
	cv::Mat R = cv::Mat::eye(2, 2, CV_64F) * 5.0;
	cv::Mat I = cv::Mat::eye(2, 2, CV_64F);
	cv::Mat k_kalman = cv::Mat::zeros(2, 2, CV_64F);
	cv::Mat B = cv::Mat::zeros(2, 1, CV_64F);
	cv::Mat x_p = cv::Mat::zeros(2, 1, CV_64F);
	cv::Mat x_corr = cv::Mat::zeros(2, 1, CV_64F);
	cv::Mat GPS = cv::Mat::zeros(2, 3, CV_64F);

	index = xsens_gps_index - w_size;
	index = index < 0 ? index + XSENS_SENSOR_VECTOR_SIZE: index;

	x_corr.at<double>(0, 0) = xsens_gps[index]->position.x;
	x_corr.at<double>(1, 0) = xsens_gps[index]->position.y;


	for (i = 0; i < w_size; i++)
	{
		sigma_p = sigma_corr + 0.1 * I;

		x_p = x_corr;

		k_kalman = sigma_p * (sigma_p + R).inv();

		sigma_corr = (I - k_kalman * I) * sigma_p;

		B.at<double>(0, 0) = xsens_gps[index]->position.x;
		B.at<double>(1, 0) = xsens_gps[index]->position.y;

		x_corr = x_p + k_kalman*(B - x_p);

		if (i >= w_size - 3)
		{
			GPS.at<double>(0, count) = x_corr.at<double>(0, 0);
			GPS.at<double>(1, count) = x_corr.at<double>(1, 0);
			count++;
		}

		index = (index + 1) % XSENS_SENSOR_VECTOR_SIZE;
	}

	return GPS;
}


int
calc_closest_gps(sensor_vector_xsens_xyz **xsens_gps, int xsens_gps_index, double odom_timestamp)
{
	int i, index;

	index = xsens_gps_index;
	for (i = 0; i < WINDOW_SIZE; i++)
	{
		if(xsens_gps[index] == NULL)
			return -1;

		if(xsens_gps[index]->timestamp < odom_timestamp)
			return index;

		index--;
		index = index < 0 ? XSENS_SENSOR_VECTOR_SIZE - 1 : index;
	}

	return -1;
}


cv::Mat
kalman_prediction(sensor_vector_xsens_xyz **xsens_gps, int xsens_gps_index, double odom_timestamp, int w_size)
{
	int nearest_neighbor_index;
	//int index, index2;
	cv::Mat GPS;
	cv::Mat pose_pred = cv::Mat::zeros(3, 1, CV_64F);

	nearest_neighbor_index = calc_closest_gps(xsens_gps, xsens_gps_index, odom_timestamp);

	GPS = kalmanGPS(xsens_gps, nearest_neighbor_index, w_size);

	//index = (nearest_neighbor_index - 1) < 0 ? XSENS_SENSOR_VECTOR_SIZE - 1:  (nearest_neighbor_index - 1);
	//index2 = (index - 1) < 0 ? XSENS_SENSOR_VECTOR_SIZE - 1:  (index - 1);

	pose_pred.at<double>(0, 0) = 1.5 * GPS.at<double>(0, 2) - GPS.at<double>(0, 1) + 0.5 * GPS.at<double>(0, 0);
	pose_pred.at<double>(1, 0) = 1.5 * GPS.at<double>(1, 2) - GPS.at<double>(1, 1) + 0.5 * GPS.at<double>(1, 0);
//	double aux = carmen_normalize_theta(1.5 * p_p.at<double>(2, 0));
//	aux = carmen_normalize_theta(aux - xsens_gps[index]->orientation.yaw);
//	double aux2 = carmen_normalize_theta(0.5 * xsens_gps[index2]->orientation.yaw);
//	pose_pred.at<double>(2, 0) = carmen_normalize_theta(aux + aux2);
	pose_pred.at<double>(2, 0) = /*1.5 **/ xsens_gps[nearest_neighbor_index]->orientation.yaw;// - xsens_gps[index]->orientation.yaw + ;

	return pose_pred;
}


carmen_point_t
kalman_pose(carmen_point_t pose, sensor_vector_xsens_xyz **xsens_gps, int xsens_gps_index, carmen_base_ackerman_odometry_message *car_odometry, double L, double delta_t, int w_size)
{
	carmen_point_t pose_new;

	static cv::Mat R = cv::Mat::eye(3, 3, CV_64F) * 1.0;
	static cv::Mat Q = cv::Mat::eye(2, 2, CV_64F);
	static cv::Mat sigma_corr = cv::Mat::eye(3, 3, CV_64F) * 5.0;
	cv::Mat x_corr = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat p_p = cv::Mat::zeros(3, 1, CV_64F);

	cv::Mat A = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat J = cv::Mat::zeros(3, 2, CV_64F);
	cv::Mat sigma_p = cv::Mat::zeros(3, 3, CV_64F);
	cv::Mat k_kalman = cv::Mat::zeros(3, 3, CV_64F);
	cv::Mat I = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat p_matching;

	x_corr.at<double>(0, 0) = pose.x;
	x_corr.at<double>(1, 0) = pose.y;
	x_corr.at<double>(2, 0) = pose.theta;

	p_p.at<double>(0, 0) = pose.x + delta_t * car_odometry->v * cos(x_corr.at<double>(2, 0));
	p_p.at<double>(1, 0) = pose.y + delta_t * car_odometry->v * sin(x_corr.at<double>(2, 0));
	p_p.at<double>(2, 0) = pose.theta + delta_t * (car_odometry->v / L) * tan(car_odometry->phi);

	p_p.at<double>(2, 0) = carmen_normalize_theta(p_p.at<double>(2, 0));

	A.at<double>(0, 2) = -delta_t * car_odometry->v * sin(x_corr.at<double>(2, 0));
	A.at<double>(1, 2) =  delta_t * car_odometry->v * cos(x_corr.at<double>(2, 0));

	J.at<double>(0, 0) = delta_t * cos(x_corr.at<double>(2, 0));
	J.at<double>(1, 0) = delta_t * sin(x_corr.at<double>(2, 0));
	J.at<double>(2, 1) = 1.0;

	sigma_p = A * (sigma_corr /*+ 0.1 * I*/) * A.t() + J * Q * J.t();

	p_matching = kalman_prediction(xsens_gps, xsens_gps_index, car_odometry->timestamp, w_size);
//	p_matching.at<double>(2, 0) = car_odometry->v > 0.1 ? (atan2(p_p.at<double>(1, 0) - pose.y, p_p.at<double>(0, 0) - pose.x + 0.000001))  : pose.theta;

	double distance_gps_to_robot = sqrt(pow(xsens_gps[xsens_gps_index]->position.y - p_p.at<double>(1, 0), 2.0) + pow(xsens_gps[xsens_gps_index]->position.x - p_p.at<double>(0, 0), 2.0));
	double betha = atan2(xsens_gps[xsens_gps_index]->position.y - p_p.at<double>(1, 0),xsens_gps[xsens_gps_index]->position.x - p_p.at<double>(0, 0) + 0.000001);
	//double a = (p_p.at<double>(1, 0) - pose.y) / (p_p.at<double>(0, 0) - pose.x);
	//double c = pose.y - a * pose.x;
	//double distance_gps_to_trajectory = fabs(a * xsens_gps[xsens_gps_index]->position.x + xsens_gps[xsens_gps_index]->position.y + c) / sqrt(a * a + 1.0) ;

	if(fabs(carmen_normalize_theta((betha - p_p.at<double>(2, 0)))) > ( M_PI / 4.0) && distance_gps_to_robot > 2.0) //colocar no carmen ini
	{
		sigma_corr = sigma_p;

		pose_new.x = p_p.at<double>(0, 0);
		pose_new.y = p_p.at<double>(1, 0);
		pose_new.theta = p_p.at<double>(2, 0);

		return pose_new;
	}

	if (car_odometry->v < 1.0)
	{
		sigma_corr = sigma_p;

		pose_new.x = p_p.at<double>(0, 0);
		pose_new.y = p_p.at<double>(1, 0);
		pose_new.theta = p_p.at<double>(2, 0);

		return pose_new;
	}

	cv::Mat aux = sigma_p + R;

	k_kalman = sigma_p * I * aux.inv();

	sigma_corr = (I - k_kalman * I) * sigma_p;

	cv::Mat aux2 = p_matching - p_p;

	aux2.at<double>(2, 0) = carmen_normalize_theta(aux2.at<double>(2, 0));

	aux2 = k_kalman * aux2;

	aux2.at<double>(2, 0) = carmen_normalize_theta(aux2.at<double>(2, 0));

	x_corr = p_p + aux2;

	x_corr.at<double>(2, 0) = carmen_normalize_theta(x_corr.at<double>(2, 0));

	pose_new.x = x_corr.at<double>(0, 0);
	pose_new.y = x_corr.at<double>(1, 0);
	pose_new.theta = x_corr.at<double>(2, 0);

	return pose_new;
}


carmen_point_t
kalman_filter_correct_pose(sensor_vector_xsens_xyz **xsens_gps, int xsens_gps_index, carmen_base_ackerman_odometry_message *car_odometry, double L)
{
	static int first = 1;
	static double last_timestamp;
	double delta_t;
	static carmen_point_t pose;

	int i = 0;

	if (first)
	{
		last_timestamp = car_odometry->timestamp;

		for (i = 0; i < XSENS_SENSOR_VECTOR_SIZE; i++)
		{
			if (xsens_gps[i] == NULL)
			{
				pose.x = xsens_gps[xsens_gps_index]->position.x;
				pose.y = xsens_gps[xsens_gps_index]->position.y;
				pose.theta = xsens_gps[xsens_gps_index]->orientation.yaw;
				return pose;
			}
		}

		pose.x = xsens_gps[xsens_gps_index]->position.x;
		pose.y = xsens_gps[xsens_gps_index]->position.y;
		pose.theta = xsens_gps[xsens_gps_index]->orientation.yaw;

		first = 0;
		return pose;
	}

	delta_t = car_odometry->timestamp - last_timestamp;

	pose = kalman_pose(pose, xsens_gps, xsens_gps_index, car_odometry, L, delta_t, WINDOW_SIZE);

	last_timestamp = car_odometry->timestamp;

	return pose;
}
