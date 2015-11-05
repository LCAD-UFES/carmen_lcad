#include "landmark_localization_motion_model_odometry_ekf.h"

EkfMotionModelOdometry::EkfMotionModelOdometry() : OdometryMotionModel() {

}

EkfMotionModelOdometry::~EkfMotionModelOdometry() {
	// TODO Auto-generated destructor stub
}

Matrix
EkfMotionModelOdometry::ComputeJacobianOfFunctiong(OdometryMotionCommand ut, Matrix mut_1)
{
	double delta_rot1, delta_rot2, delta_trans;
	double theta;

	theta = mut_1.val[2][0];
	ComputeDeltaOdometry(&delta_rot1, &delta_trans, &delta_rot2, ut);
	double jacobian[9] = {1, 0, -delta_trans * sin(theta + delta_rot1),
						  0, 1,  delta_trans * cos(theta + delta_rot1),
						  0, 0, 1 };

	Matrix Gt = Matrix(3, 3, jacobian);

	return Gt;
}

Matrix
EkfMotionModelOdometry::ControlSpaceToStateSpaceMatrix(OdometryMotionCommand ut, Matrix _mut)
{
	double delta_rot1, delta_rot2, delta_trans, theta;

	ComputeDeltaOdometry(&delta_rot1, &delta_trans, &delta_rot2, ut);
	theta = _mut.val[2][0];
	double state_space_to_control_space_matrix[9] = {-delta_trans * sin(theta + delta_rot1), cos(theta + delta_rot1), 0,
													 delta_trans * cos(theta + delta_rot1), sin(theta + delta_rot1), 0,
													 1, 0, 1 };

	Matrix Vt = Matrix(3, 3, state_space_to_control_space_matrix);
	return Vt;
}
