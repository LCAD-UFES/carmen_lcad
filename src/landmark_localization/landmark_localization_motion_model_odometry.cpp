#include "landmark_localization_motion_model_odometry.h"

OdometryMotionModel::OdometryMotionModel() {
	alpha1 = 0.005; // related to initial/final rotation
	alpha2 = 0.05; // related to initial/final rotation
	alpha3 = 0.05; // related to translation
	alpha4 = 0.05; // related to translation
}

void
OdometryMotionModel::setCoefs(double* coefs)
{
	alpha1 = coefs[0];
	alpha2 = coefs[1];
	alpha3 = coefs[2];
	alpha4 = coefs[3];
}

Matrix
OdometryMotionModel::MotionModelOdometry(OdometryMotionCommand ut, Matrix xt_1)
{
	Matrix _xt = Matrix(3, 1);

	double delta_rot1, delta_rot2, delta_trans;

	ComputeDeltaOdometry(&delta_rot1, &delta_trans, &delta_rot2, ut);

	_xt.val[0][0] = xt_1.val[0][0] + delta_trans * cos(xt_1.val[2][0] + delta_rot1);
	_xt.val[1][0] = xt_1.val[1][0] + delta_trans * sin(xt_1.val[2][0] + delta_rot1);
	_xt.val[2][0] = xt_1.val[2][0] + delta_rot1 + delta_rot2;

	return _xt;
}

Matrix
OdometryMotionModel::NoiseMatrixInControlSpace(OdometryMotionCommand ut)
{
	double delta_rot1, delta_rot2, delta_trans;
	double sqr_delta_rot1, sqr_delta_rot2, sqr_delta_trans;

	ComputeDeltaOdometry(&delta_rot1, &delta_trans, &delta_rot2, ut);
	sqr_delta_rot1 = delta_rot1 * delta_rot1;
	sqr_delta_rot2 = delta_rot2 * delta_rot2;
	sqr_delta_trans = delta_trans * delta_trans;
	double erro_matrix_in_control_space[9] =  {alpha1 * sqr_delta_rot1 + alpha2 * sqr_delta_trans, 0, 0,
											   0, alpha3 * sqr_delta_trans + alpha4 * sqr_delta_rot1 + alpha4 * sqr_delta_rot2, 0,
											   0, 0, alpha1 * sqr_delta_rot2 + alpha2 * sqr_delta_trans};
	Matrix Mt = Matrix(3, 3, erro_matrix_in_control_space);

	return Mt;
}


void
OdometryMotionModel::ComputeDeltaOdometry(double *delta_rot1, double *delta_trans, double *delta_rot2, OdometryMotionCommand ut)
{
	double o_x_1, o_y_1, o_theta_1; // related to o_xt_1
	double o_x, o_y, o_theta;       // related to o_xt

	o_x_1 = ut.initial.x;
	o_y_1 = ut.initial.y;
	o_theta_1 = ut.initial.theta;
	o_x = ut.final.x;
	o_y = ut.final.y;
	o_theta = ut.final.theta;

	*delta_trans = sqrt(pow(o_x - o_x_1, 2.0) + pow(o_y - o_y_1, 2.0));
	if (*delta_trans < SMALL_TRANS)
	{
		*delta_trans = 0.0;
		*delta_rot1 = o_theta - o_theta_1;
		*delta_rot2 = 0.0;
	}
	else
	{
		*delta_rot1 = atan2(o_y - o_y_1, o_x - o_x_1) - o_theta_1;
		*delta_rot2 = o_theta - o_theta_1 - (*delta_rot1);
	}
}

OdometryMotionModel::~OdometryMotionModel() {
	// TODO Auto-generated destructor stub
}

