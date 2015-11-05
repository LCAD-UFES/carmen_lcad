/*
 * post_slam.cpp
 *
 *  Created on: Apr 12, 2013
 *      Author: filipe
 */
#include "post_localize.h"
#include <opencv/cv.h>

namespace post_slam
{
	PostLocalize::PostLocalize()
	{
		mean = cv::Mat::zeros(3, 1, CV_64F);
		covariance = cv::Mat::zeros(3, 3, CV_64F);

		// TODO: ler do ini
		alpha_1 = 0.2;
		alpha_2 = 0.01;
		alpha_3 = 0.2;
		alpha_4 = 0.01;
	}


	PostLocalize::PostLocalize(char *landmark_map_filename)
	{
		mean = cv::Mat::zeros(3, 1, CV_64F);
		covariance = cv::Mat::zeros(3, 3, CV_64F);
		landmark_map.load(landmark_map_filename);
	}


	PostLocalize::~PostLocalize()
	{
	}


	void
	PostLocalize::initialize(double x, double y, double theta)
	{
		mean.at<double>(0, 0) = x;
		mean.at<double>(1, 0) = y;
		mean.at<double>(2, 0) = theta;
		// TODO: inicializar a covariancia com zero?
	}


	void
	PostLocalize::load_landmark_map(char *filename)
	{
		landmark_map.load(filename);
	}


//	tf::Matrix3x3
//	operator+(tf::Matrix3x3 a, tf::Matrix3x3 b)
//	{
//		tf::Matrix3x3 sum;
//
//		sum[0][0] = a[0][0] + b[0][0];
//		sum[0][1] = a[0][1] + b[0][1];
//		sum[0][2] = a[0][2] + b[0][2];
//
//		sum[1][0] = a[1][0] + b[1][0];
//		sum[1][1] = a[1][1] + b[1][1];
//		sum[1][2] = a[1][2] + b[1][2];
//
//		sum[2][0] = a[2][0] + b[2][0];
//		sum[2][1] = a[2][1] + b[2][1];
//		sum[2][2] = a[2][2] + b[2][2];
//
//		return sum;
//	}
//
//
//	tf::Matrix3x3
//	operator-(tf::Matrix3x3 a, tf::Matrix3x3 b)
//	{
//		tf::Matrix3x3 sum;
//
//		sum[0][0] = a[0][0] - b[0][0];
//		sum[0][1] = a[0][1] - b[0][1];
//		sum[0][2] = a[0][2] - b[0][2];
//
//		sum[1][0] = a[1][0] - b[1][0];
//		sum[1][1] = a[1][1] - b[1][1];
//		sum[1][2] = a[1][2] - b[1][2];
//
//		sum[2][0] = a[2][0] - b[2][0];
//		sum[2][1] = a[2][1] - b[2][1];
//		sum[2][2] = a[2][2] - b[2][2];
//
//		return sum;
//	}
//
//
//	tf::Matrix3x3
//	operator*(tf::Matrix3x3 a, double scalar)
//	{
//		tf::Matrix3x3 b;
//
//		b[0][0] = a[0][0] * scalar;
//		b[0][1] = a[0][1] * scalar;
//		b[0][2] = a[0][2] * scalar;
//
//		b[1][0] = a[1][0] * scalar;
//		b[1][1] = a[1][1] * scalar;
//		b[1][2] = a[1][2] * scalar;
//
//		b[2][0] = a[2][0] * scalar;
//		b[2][1] = a[2][1] * scalar;
//		b[2][2] = a[2][2] * scalar;
//
//		return b;
//	}
//
//
//	tf::Matrix3x3
//	operator*(double scalar, tf::Matrix3x3 a)
//	{
//		tf::Matrix3x3 b;
//
//		b[0][0] = a[0][0] * scalar;
//		b[0][1] = a[0][1] * scalar;
//		b[0][2] = a[0][2] * scalar;
//
//		b[1][0] = a[1][0] * scalar;
//		b[1][1] = a[1][1] * scalar;
//		b[1][2] = a[1][2] * scalar;
//
//		b[2][0] = a[2][0] * scalar;
//		b[2][1] = a[2][1] * scalar;
//		b[2][2] = a[2][2] * scalar;
//
//		return b;
//	}


	/**
	 * TODO: Criar um nome melhor pra essa funcao
	 */
	cv::Mat
	PostLocalize::calculate_G_t(double v_t, double w_t, double delta_t)
	{
		cv::Mat G_t(3, 3, CV_64F);

		G_t.at<double>(0, 0) = 1.0;
		G_t.at<double>(0, 1) = 0.0;

		// TODO: tratar o w_t = 0 de forma correta aqui!!!
		if (w_t > 0.0001)
			G_t.at<double>(0, 2) = ((v_t / w_t) * sin(mean.at<double>(2, 0))) - ((v_t / w_t) * cos(mean.at<double>(2, 0) + w_t * delta_t));
		else
			G_t.at<double>(0, 2) = 0.0;

		G_t.at<double>(1, 0) = 0.0;
		G_t.at<double>(1, 1) = 1.0;

		// TODO: tratar o w_t = 0 de forma correta aqui!!!
		if (w_t > 0.0001)
			G_t.at<double>(1, 2) = ((v_t / w_t) * sin(mean.at<double>(2, 0))) - ((v_t / w_t) * sin(mean.at<double>(2, 0) + w_t * delta_t));
		else
			G_t.at<double>(1, 2) = 0.0;

		G_t.at<double>(2, 0) = 0.0;
		G_t.at<double>(2, 1) = 0.0;
		G_t.at<double>(2, 2) = 1.0;

		return G_t;
	}


	cv::Mat
	PostLocalize::calculate_V_t(double v_t, double w_t, double delta_t)
	{
		double theta = mean.at<double>(2, 0);

		cv::Mat V_t(3, 2, CV_64F);

		if (w_t > 0.0001)
		{
			V_t.at<double>(0, 0) = (-sin(theta) + sin(theta + w_t * delta_t)) / w_t;
			V_t.at<double>(0, 1) = ((v_t * (sin(theta) - sin(theta + w_t * delta_t))) / pow(w_t, 2)) + ((v_t * cos(theta + w_t * delta_t) * delta_t) / w_t);

			V_t.at<double>(1, 0) = (cos(theta) - cos(theta + w_t * delta_t)) / w_t;
			V_t.at<double>(1, 1) = ((- v_t * (cos(theta) - cos(theta + w_t * delta_t))) / pow(w_t, 2)) + ((v_t * sin(theta + w_t * delta_t) * delta_t) / w_t);
		}
		else
		{
			// TODO: ver como tratar o w_t = 0 de forma correta aqui!!!
			V_t.at<double>(0, 0) = 0.0;
			V_t.at<double>(0, 1) = 0.0;

			V_t.at<double>(1, 0) = 0.0;
			V_t.at<double>(1, 1) = 0.0;
		}

		V_t.at<double>(2, 0) = 0.0;
		V_t.at<double>(2, 1) = delta_t;

		return V_t;
	}


	cv::Mat
	PostLocalize::calculate_M_t(double v_t, double w_t)
	{
		cv::Mat M_t(2, 2, CV_64F);

		M_t.at<double>(0, 0) = alpha_1 * pow(v_t, 2) + alpha_2 * pow(w_t, 2);
		M_t.at<double>(0, 1) = 0.0;
		M_t.at<double>(1, 0) = 0.0;
		M_t.at<double>(1, 1) = alpha_3 * pow(v_t, 2) + alpha_4 * pow(w_t, 2);

		return M_t;
	}


	cv::Mat
	PostLocalize::calculate_motion_update(double v_t, double w_t, double delta_t)
	{
		cv::Mat motion_update(3, 1, CV_64F);

		// TODO: trata as divisoes por zero
		if (w_t > 0.0001)
		{
			motion_update.at<double> (0, 0) = (((-v_t / w_t) * sin(mean.at<double>(2, 0))) + ((v_t / w_t) * sin(mean.at<double>(2, 0) + w_t * delta_t)));
			motion_update.at<double> (1, 0) = (((v_t / w_t) * sin(mean.at<double>(2, 0))) - ((v_t / w_t) * cos(mean.at<double>(2, 0) + w_t * delta_t)));
			motion_update.at<double> (2, 0) = (w_t * delta_t);
		}
		else
		{
			// TODO: checar se estou tratando o w_t = 0 de forma correta aqui!!!
			motion_update.at<double> (0, 0) = (v_t * delta_t) * cos(mean.at<double>(2, 0));
			motion_update.at<double> (1, 0) = (v_t * delta_t) * sin(mean.at<double>(2, 0));
			motion_update.at<double> (2, 0) = 0.0;
		}

		return motion_update;
	}


	void
	PostLocalize::integrate_movement_information(double v_t, double w_t, double delta_t)
	{
		cv::Mat motion_update = calculate_motion_update(v_t, w_t, delta_t);

		cv::Mat G_t = calculate_G_t(v_t, w_t, delta_t);
		cv::Mat G_t_transposed(3, 3, CV_64F);

		cv::Mat V_t = calculate_V_t(v_t, w_t, delta_t);
		cv::Mat V_t_transposed(2, 3, CV_64F);

		cv::Mat M_t = calculate_M_t(v_t, w_t);

		cv::transpose(G_t, G_t_transposed);
		cv::transpose(V_t, V_t_transposed);

		mean += motion_update;
		mean.at<double>(0, 2) = carmen_normalize_theta(mean.at<double>(0, 2));

		covariance = G_t * covariance * G_t_transposed + V_t * M_t * V_t_transposed;

		// TODO: inicializar a matriz Q_t (como atualizar quando eu tiver mais predicoes do que correcoes?)
		// cv::Mat Q_t(3, 3, CV_64F);
	}


	void PostLocalize::integrate_measurement_information(int sensor_measurement __attribute__ ((unused)))
	{
//		int i, number_of_features = 0;
//
//		vector<Landmark*> features;
//		vector<Landmark*> matched_features = landmark_map.match(&features);
//
//		for (i = 0; i < number_of_features; i++)
//		{
//			// TODO: inicializar a feature e a feature correspondente
//			tf::Vector3 feature, matched_feature;
//
//			// TODO: inicializar esses (quem sao esses caras?)
//			double m_j_x = 0.0, m_j_y = 0.0, m_j_s = 0.0;
//
//			double delta_x = m_j_x - mean.getX();
//			double delta_y = m_j_y - mean.getY();
//			double q = delta_x * delta_x + delta_y * delta_y;
//
//			tf::Vector3 z_i_t;
//
//			z_i_t.setX(sqrt(q));
//			z_i_t.setY(atan2(delta_y, delta_x) - mean.getZ());
//			z_i_t.setZ(m_j_s);
//
//			tf::Matrix3x3 H_i_t;
//
//			H_i_t[0][0] = sqrt(q) * delta_x;
//			H_i_t[0][1] = -sqrt(q) * delta_y;
//			H_i_t[0][2] = 0.0;
//
//			H_i_t[1][0] = delta_y;
//			H_i_t[1][1] = delta_x;
//			H_i_t[1][2] = -1.0;
//
//			H_i_t[2][0] = 0.0;
//			H_i_t[2][1] = 0.0;
//			H_i_t[2][2] = 0.0;
//
//			H_i_t = (1 / q) * H_i_t;
//
//			// TODO: checar como acumular o ganho de kalman para as varias features. No livro, o autor
//			// vai colocando os valores em uma variavel i, mas embaixo ele usa essa mesma variavel i...
//			// Acho que deveria rolar uma soma dos ganhos para cada feature.
//			tf::Matrix3x3 K_i_t;
//
//			// TODO: inicializar essa variavel (covariancia do erro da medida)
//			tf::Matrix3x3 Q_t;
//
//			K_i_t = covariance * H_i_t.transpose() * (H_i_t * covariance * H_i_t.transpose() + Q_t).inverse();
//		}
//
//		tf::Matrix3x3 K_i_t;
//		tf::Matrix3x3 H_i_t;
//
//		tf::Matrix3x3 identity;
//		identity.getIdentity();
//
//		// TODO: obter esse valor (de onde ele vem?)
//		tf::Matrix3x3 covariance_i;
//		// TODO: calcular esse vetor
//		tf::Vector3 difference_from_estimated_and_measured_feature(0.0, 0.0, 0.0);
//
//		mean = mean + covariance_i * K_i_t * difference_from_estimated_and_measured_feature;
//		covariance = (identity.getIdentity() - covariance_i * K_i_t * H_i_t) * covariance;
	}


	carmen_point_t
	PostLocalize::get_pose()
	{
		carmen_point_t pose;

		pose.x = mean.at<double>(0, 0);
		pose.y = mean.at<double>(1, 0);
		pose.theta = mean.at<double>(2, 0);

		return pose;
	}
}

