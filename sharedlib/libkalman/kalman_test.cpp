#include <stdio.h>
#include <carmen/carmen.h>
#include <carmen/matrix.h>
#include <fenv.h>
#include <vector>
#include "kalman.h"

// Linhas de comando para teste com o gnuplot
// set size ratio -1; plot "caco.txt" u 1:2 w l t "true pose", "caco.txt" u 3:4 w l t "xt", "caco.txt" using 3:4:5:6:7 with ellipses t "error", "caco.txt" u 8:9 w l t "zt"
// plot "caco.txt" u 10 w l, "caco.txt" u 11 w l,"caco.txt" u 12 w l

using namespace std;

#define	READ_DATA_FROM_FILE

#define IMM_FILTER

#define CV_MODEL
#define CA_MODEL
#define CT_MODEL

#define	DELTA_T		0.05	// s

#define MAX_A		2.0		// m/s^2
#define MAX_W 		5.0		// degrees/s

#define SIGMA_S		(1.0)	// m
#define SIGMA_VCA	(1.0)	// m/s
#define SIGMA_VCT	(2.0)	// m/s
#define SIGMA_W		(5.5)	// degrees/s

#define SIGMA_R		1.0		// m
#define SIGMA_THETA	1.0		// degrees
#define SIGMA_R_SIMULATION		(SIGMA_R / 2.0)			// m
#define SIGMA_THETA_SIMULATION	(SIGMA_THETA / 2.0)		// degrees

#ifdef IMM_FILTER
double u_k[NUM_MODELS] = {1.0/3.0, 1.0/3.0, 1.0/3.0};
double p[NUM_MODELS][NUM_MODELS] = {
		{0.998, 0.001, 0.001},
		{0.001, 0.998, 0.001},
		{0.001, 0.001, 0.998}};
//static double p[NUM_MODELS][NUM_MODELS] = {
//		{0.980, 0.020, 0.001},
//		{0.030, 0.970, 0.001},
//		{0.300, 0.600, 0.200}};
//static double p[NUM_MODELS][NUM_MODELS] = {
//		{0.950, 0.001, 0.050},
//		{0.050, 0.001, 0.950},
//		{0.030, 0.001, 0.970}};
//static double p[NUM_MODELS][NUM_MODELS] = {
//		{0.998, 0.001, 0.001},
//		{0.998, 0.001, 0.001},
//		{0.998, 0.001, 0.001}};
//static double p[NUM_MODELS][NUM_MODELS] = {
//		{0.001, 0.998, 0.001},
//		{0.001, 0.998, 0.001},
//		{0.001, 0.998, 0.001}};
//static double p[NUM_MODELS][NUM_MODELS] = {
//		{0.001, 0.001, 0.998},
//		{0.001, 0.001, 0.998},
//		{0.001, 0.001, 0.998}};


int
main()
{
//	feenableexcept(FE_UNDERFLOW);

	double angle, major_axis, minor_axis;

	double delta_t = DELTA_T;
	double true_x = -170.0;
	double true_y = -150.0;
	double true_yaw = carmen_degrees_to_radians(45.0);
	double true_v = 15.0;
	double true_w = 0.0;

	double max_a = MAX_A;
	double max_w = carmen_degrees_to_radians(MAX_W);

	double sigma_s = SIGMA_S;
	double sigma_vca = SIGMA_VCA;
	double sigma_vct = SIGMA_VCT;
	double sigma_w = carmen_degrees_to_radians(SIGMA_W);

	double sigma_r = SIGMA_R;
	double sigma_theta = carmen_degrees_to_radians(SIGMA_THETA);

	// True world state + error
	double x = true_x - 5.0;
	double y = true_y + 10.0;
	double v = true_v + 1.0;
	double yaw = true_yaw + carmen_degrees_to_radians(-5.0);
	double w = true_w + carmen_degrees_to_radians(5.0);

#ifdef READ_DATA_FROM_FILE
	double imm_datmo_x, imm_datmo_y;
	FILE *data = fopen("../../bin/imm_data_5_filtered.txt", "r");
	int num_items_read = fscanf(data, "%lf %lf %lf %lf %lf %lf\n", &true_x, &true_y, &imm_datmo_x, &imm_datmo_y, &delta_t, &true_yaw);
	x = true_x;
	y = true_y;
	yaw = true_yaw;
#endif

	Matrix x_k_k, P_k_k, z_k, R_k, R_p_k, fx_k_1;
	vector<Matrix> x_k_1_k_1, P_k_1_k_1, F_k_1_m, Q_k_1_m, H_k_m;
	Matrix F_k_1, Q_k_1, H_k;

	CV_system_setup(x, y, yaw, v, x_k_k, P_k_k, F_k_1, Q_k_1, H_k, R_p_k, delta_t, sigma_s, sigma_r, sigma_theta);
	x_k_1_k_1.push_back(x_k_k); P_k_1_k_1.push_back(P_k_k); F_k_1_m.push_back(F_k_1); Q_k_1_m.push_back(Q_k_1); H_k_m.push_back(H_k);

	CA_system_setup(x, y, yaw, v, x_k_k, P_k_k, F_k_1, Q_k_1, H_k, R_p_k, delta_t, sigma_vca, sigma_r, sigma_theta);
	x_k_1_k_1.push_back(x_k_k); P_k_1_k_1.push_back(P_k_k); F_k_1_m.push_back(F_k_1); Q_k_1_m.push_back(Q_k_1); H_k_m.push_back(H_k);

	CT_system_setup(x, y, yaw, v, w, x_k_k, P_k_k, F_k_1, fx_k_1, Q_k_1, H_k, R_p_k, delta_t, sigma_w, sigma_vct, sigma_r, sigma_theta);
	x_k_1_k_1.push_back(x_k_k); P_k_1_k_1.push_back(P_k_k); F_k_1_m.push_back(F_k_1); Q_k_1_m.push_back(Q_k_1); H_k_m.push_back(H_k);

	Matrix imm_x_k_k(7, 1), imm_P_k_k(7, 7);
	mode_estimate_and_covariance_combination(imm_x_k_k, imm_P_k_k, extend_vector_dimensions(x_k_1_k_1), extend_matrix_dimensions(P_k_1_k_1, max_a, max_w), u_k);

	compute_error_ellipse(angle, major_axis, minor_axis, imm_P_k_k.val[0][0], imm_P_k_k.val[0][1], imm_P_k_k.val[1][1], 2.4477);
	printf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", x, y, imm_x_k_k.val[0][0], imm_x_k_k.val[1][0], major_axis, minor_axis, angle * 180.0 / M_PI,
			x, y,
			u_k[0], u_k[1], u_k[2],
			sqrt(imm_x_k_k.val[2][0] * imm_x_k_k.val[2][0] + imm_x_k_k.val[3][0] * imm_x_k_k.val[3][0]));


	double t = 0.0;
	do
	{
		double v_p_k_[2 * 1] =
		{
			carmen_gaussian_random(0.0, SIGMA_R_SIMULATION),
			carmen_gaussian_random(0.0, carmen_degrees_to_radians(SIGMA_THETA_SIMULATION)),
		};
		Matrix v_p_k(2, 1, v_p_k_);  // [1] eqs. 2.33, 2.34, pg 14

		double radius = sqrt(true_x * true_x + true_y * true_y) + v_p_k.val[0][0]; 	// perfect observation + noise
		double theta = atan2(true_y, true_x) + v_p_k.val[1][0]; 	// perfect observation + noise

		position_observation(z_k, R_k, R_p_k, radius, theta, sigma_r, sigma_theta);

		imm_filter(imm_x_k_k, imm_P_k_k, x_k_1_k_1, P_k_1_k_1,
				z_k, R_k,
				F_k_1_m, Q_k_1_m, H_k_m,
				delta_t, sigma_w, sigma_vct, max_a, max_w,
				p, u_k);

		compute_error_ellipse(angle, major_axis, minor_axis, imm_P_k_k.val[0][0], imm_P_k_k.val[0][1], imm_P_k_k.val[1][1], 2.4477);
		printf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", true_x, true_y, imm_x_k_k.val[0][0], imm_x_k_k.val[1][0], major_axis, minor_axis, angle * 180.0 / M_PI,
				z_k.val[0][0], z_k.val[1][0],
				u_k[0], u_k[1], u_k[2],
				sqrt(imm_x_k_k.val[2][0] * imm_x_k_k.val[2][0] + imm_x_k_k.val[3][0] * imm_x_k_k.val[3][0]));

#ifdef READ_DATA_FROM_FILE
		num_items_read = fscanf(data, "%lf %lf %lf %lf %lf %lf\n", &true_x, &true_y, &imm_datmo_x, &imm_datmo_y, &delta_t, &true_yaw);
#else
		true_x = true_x + true_v * cos(true_yaw) * delta_t;	// true position update
		true_y = true_y + true_v * sin(true_yaw) * delta_t;	// true position update

		if ((t > 55.0) && (t < 75.0))
			true_yaw += 0.3 * delta_t;
#endif

		t += delta_t;
#ifdef READ_DATA_FROM_FILE
		} while (num_items_read == 6);
#else
		} while (t < 100.0);
#endif
}

#else

int
main()
{
	double angle, major_axis, minor_axis;
	Matrix x_k_k, P_k_k, F_k_1, Q_k_1, H_k, R_p_k;

	double delta_t = DELTA_T;
	double true_x = -170.0;
	double true_y = -150.0;
	double true_yaw = carmen_degrees_to_radians(45.0);
	double true_v = 5.0;

	double sigma_r = SIGMA_R;
	double sigma_theta = carmen_degrees_to_radians(SIGMA_THETA);

	// True world state + error
	double x = true_x - 5.0;
	double y = true_y + 10.0;
	double v = true_v + 1.0;
	double yaw = true_yaw + carmen_degrees_to_radians(-5.0);

#ifdef CV_MODEL
	double sigma_s = SIGMA_S;
	CV_system_setup(x, y, yaw, v, x_k_k, P_k_k, F_k_1, Q_k_1, H_k, R_p_k, delta_t, sigma_s, sigma_r, sigma_theta);
#endif

#ifdef CA_MODEL
	double sigma_vca = SIGMA_VCA;
	CA_system_setup(x, y, yaw, v, x_k_k, P_k_k, F_k_1, Q_k_1, H_k, R_p_k, delta_t, sigma_vca, sigma_r, sigma_theta);
#endif

#ifdef CT_MODEL
	double w = 0.0;
	double sigma_vct = SIGMA_VCT;
	double sigma_w = carmen_degrees_to_radians(SIGMA_W);

	Matrix fx_k_1;
	CT_system_setup(x, y, yaw, v, w, x_k_k, P_k_k, F_k_1, fx_k_1, Q_k_1, H_k, R_p_k, delta_t, sigma_w, sigma_vct, sigma_r, sigma_theta);
#endif

	compute_error_ellipse(angle, major_axis, minor_axis, P_k_k.val[0][0], P_k_k.val[0][1], P_k_k.val[1][1], 2.4477);
	printf("%lf %lf %lf %lf %lf %lf %lf %lf %lf\n", true_x, true_y, x_k_k.val[0][0], x_k_k.val[1][0], major_axis, minor_axis, angle * 180.0 / M_PI,
			true_x, true_y);

	Matrix z_k, R_k, delta_zk, S_k;
	double t = 0.0;
	do
	{
		double v_p_k_[2 * 1] =
		{
			carmen_gaussian_random(0.0, SIGMA_R_SIMULATION),
			carmen_gaussian_random(0.0, carmen_degrees_to_radians(SIGMA_THETA_SIMULATION)),
		};
		Matrix v_p_k(2, 1, v_p_k_);  // [1] eqs. 2.33, 2.34, pg 14

		double radius = sqrt(true_x * true_x + true_y * true_y) + v_p_k.val[0][0]; 	// perfect observation + noise
		double theta = atan2(true_y, true_x) + v_p_k.val[1][0]; 	// perfect observation + noise

		position_observation(z_k, R_k, R_p_k, radius, theta, sigma_r, sigma_theta);

#if defined(CV_MODEL) || defined(CA_MODEL)
		kalman_filter(x_k_k, P_k_k, delta_zk, S_k, z_k, F_k_1, Q_k_1, H_k, R_k);
#endif

#ifdef CT_MODEL
		set_CT_system_matrixes(F_k_1, fx_k_1, Q_k_1, delta_t, x_k_k.val[4][0], x_k_k.val[2][0], x_k_k.val[3][0], sigma_w, sigma_vct);
		extended_kalman_filter(x_k_k, P_k_k, delta_zk, S_k, z_k, F_k_1, fx_k_1, Q_k_1, H_k, R_k);
#endif

		compute_error_ellipse(angle, major_axis, minor_axis, P_k_k.val[0][0], P_k_k.val[0][1], P_k_k.val[1][1], 2.4477);
		printf("%lf %lf %lf %lf %lf %lf %lf %lf %lf\n", true_x, true_y, x_k_k.val[0][0], x_k_k.val[1][0], major_axis, minor_axis, angle * 180.0 / M_PI,
				z_k.val[0][0], z_k.val[1][0]);

		true_x = true_x + true_v * cos(true_yaw) * delta_t;	// true position update
		true_y = true_y + true_v * sin(true_yaw) * delta_t;	// true position update

		if ((t > 55.0) && (t < 75.0))
			true_yaw += 0.3 * delta_t;

		t += delta_t;
	} while (t < 100.0);
}
#endif
