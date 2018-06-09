#include <carmen/carmen.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/velodyne_interface.h>
#include <carmen/matrix.h>
#include <carmen/kalman.h>
#include "virtual_scan.h"

#define MAX_A		2.0		// m/s^2
#define MAX_W 		5.0		// degrees/s

#define SIGMA_S		(1.0)	// m
#define SIGMA_VCA	(1.0)	// m/s
#define SIGMA_VCT	(2.0)	// m/s
#define SIGMA_W		(5.5)	// degrees/s

#define SIGMA_R		1.0		// m
#define SIGMA_THETA	1.0		// degrees

double p[NUM_MODELS][NUM_MODELS] = {
		{0.998, 0.001, 0.001},
		{0.001, 0.998, 0.001},
		{0.001, 0.001, 0.998}};


void
fit_multiple_models_to_track_of_hypotheses(virtual_scan_track_t *track)
{
	for (int j = 1; j < track->size; j++)
	{
		double delta_t = track->box_model_hypothesis[j].hypothesis_points.precise_timestamp - track->box_model_hypothesis[j - 1].hypothesis_points.precise_timestamp;
		double distance_travelled = DIST2D(track->box_model_hypothesis[j].hypothesis, track->box_model_hypothesis[j - 1].hypothesis);
		double angle_in_the_distance_travelled = ANGLE2D(track->box_model_hypothesis[j].hypothesis, track->box_model_hypothesis[j - 1].hypothesis);
		double v = (cos(track->box_model_hypothesis[j].hypothesis.theta - angle_in_the_distance_travelled) * distance_travelled) / delta_t;
		double delta_theta = carmen_normalize_theta(track->box_model_hypothesis[j].hypothesis.theta - track->box_model_hypothesis[j - 1].hypothesis.theta);
		double d_theta = delta_theta / delta_t;
		track->box_model_hypothesis[j].hypothesis_state.v = v;
		track->box_model_hypothesis[j].hypothesis_state.d_theta = d_theta;

		double x = track->box_model_hypothesis[j].hypothesis.x - track->box_model_hypothesis[j].hypothesis_points.sensor_pos.x;
		double y = track->box_model_hypothesis[j].hypothesis.y - track->box_model_hypothesis[j].hypothesis_points.sensor_pos.y;
		double x_1 = track->box_model_hypothesis[j - 1].hypothesis.x - track->box_model_hypothesis[j - 1].hypothesis_points.sensor_pos.x;
		double y_1 = track->box_model_hypothesis[j - 1].hypothesis.y - track->box_model_hypothesis[j - 1].hypothesis_points.sensor_pos.y;
		double radius = sqrt(x * x + y * y);
		double theta = atan2(y, x);
		double yaw = atan2(y - y_1, x - x_1);
		double w = 0.0;

		if (track->box_model_hypothesis[j].hypothesis_state.imm == NULL)
		{
			imm_state_t *imm = new imm_state_t;

			Matrix x_k_k, P_k_k, R_p_k, F_k_1, Q_k_1, H_k;
			CV_system_setup(x, y, yaw, v, x_k_k, P_k_k, F_k_1, Q_k_1, H_k, R_p_k, delta_t, SIGMA_S, SIGMA_R, carmen_degrees_to_radians(SIGMA_THETA));
			imm->x_k_1_k_1.push_back(x_k_k); imm->P_k_1_k_1.push_back(P_k_k); imm->F_k_1_m.push_back(F_k_1); imm->Q_k_1_m.push_back(Q_k_1); imm->H_k_m.push_back(H_k);

			CA_system_setup(x, y, yaw, v, x_k_k, P_k_k, F_k_1, Q_k_1, H_k, R_p_k, delta_t, SIGMA_VCA, SIGMA_R, carmen_degrees_to_radians(SIGMA_THETA));
			imm->x_k_1_k_1.push_back(x_k_k); imm->P_k_1_k_1.push_back(P_k_k); imm->F_k_1_m.push_back(F_k_1); imm->Q_k_1_m.push_back(Q_k_1); imm->H_k_m.push_back(H_k);

			Matrix fx_k_1;
			CT_system_setup(x, y, yaw, v, w, x_k_k, P_k_k, F_k_1, fx_k_1, Q_k_1, H_k, R_p_k, delta_t, carmen_degrees_to_radians(SIGMA_W), SIGMA_VCT, SIGMA_R, carmen_degrees_to_radians(SIGMA_THETA));
			imm->x_k_1_k_1.push_back(x_k_k); imm->P_k_1_k_1.push_back(P_k_k); imm->F_k_1_m.push_back(F_k_1); imm->Q_k_1_m.push_back(Q_k_1); imm->H_k_m.push_back(H_k);

			imm->u_k[0] = 1.0/3.0; imm->u_k[1] = 1.0/3.0; imm->u_k[2] = 1.0/3.0;
			Matrix imm_x_k_k(7, 1), imm_P_k_k(7, 7);
			imm->imm_x_k_k = imm_x_k_k; imm->imm_P_k_k = imm_P_k_k;
			mode_estimate_and_covariance_combination(imm->imm_x_k_k, imm->imm_P_k_k,
					extend_vector_dimensions(imm->x_k_1_k_1), extend_matrix_dimensions(imm->P_k_1_k_1, MAX_A, carmen_degrees_to_radians(MAX_W)), imm->u_k);

			Matrix z_k, R_k;
			position_observation(z_k, R_k, R_p_k, radius, theta, SIGMA_R, carmen_degrees_to_radians(SIGMA_THETA));

			imm_filter(imm->imm_x_k_k, imm->imm_P_k_k, imm->x_k_1_k_1, imm->P_k_1_k_1,
					z_k, R_k,
					imm->F_k_1_m, imm->Q_k_1_m, imm->H_k_m,
					delta_t, carmen_degrees_to_radians(SIGMA_W), SIGMA_VCT, MAX_A, carmen_degrees_to_radians(MAX_W),
					p, imm->u_k);

			track->box_model_hypothesis[j].hypothesis_state.imm = imm;
		}
		else
		{
			imm_state_t *imm = track->box_model_hypothesis[j].hypothesis_state.imm;

			Matrix z_k, R_k, R_p_k;
			set_R_p_k_matriz(R_p_k, SIGMA_R, carmen_degrees_to_radians(SIGMA_THETA));
			position_observation(z_k, R_k, R_p_k, radius, theta, SIGMA_R, carmen_degrees_to_radians(SIGMA_THETA));

			imm_filter(imm->imm_x_k_k, imm->imm_P_k_k, imm->x_k_1_k_1, imm->P_k_1_k_1,
					z_k, R_k,
					imm->F_k_1_m, imm->Q_k_1_m, imm->H_k_m,
					delta_t, carmen_degrees_to_radians(SIGMA_W), SIGMA_VCT, MAX_A, carmen_degrees_to_radians(MAX_W),
					p, imm->u_k);
		}
	}
}


void
update_hypotheses_state(virtual_scan_track_t *track)
{
	if (track == NULL)
		return;

	if (track->size >= 2)
		fit_multiple_models_to_track_of_hypotheses(track);
}
