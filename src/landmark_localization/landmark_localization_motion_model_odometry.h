#ifndef LANDMARK_LOCALIZATION_MOTION_MODEL_ODOMETRY_H_
#define LANDMARK_LOCALIZATION_MOTION_MODEL_ODOMETRY_H_

#include <matrix.h>
#include <prob_motion_model.h>

class OdometryMotionModel {
public:
	static const double SMALL_TRANS = 0.001;
	static const double SMALL_ROT = 0.001;

	double alpha1; // related to initial/final rotation
	double alpha2; // related to initial/final rotation
	double alpha3; // related to translation
	double alpha4; // related to translation

	OdometryMotionModel();
	Matrix MotionModelOdometry(OdometryMotionCommand ut, Matrix xt_1);
	Matrix NoiseMatrixInControlSpace(OdometryMotionCommand ut);
	void ComputeDeltaOdometry(double *delta_rot1, double *delta_trans, double *delta_rot2, OdometryMotionCommand ut);
	void setCoefs(double* coefs);
	virtual ~OdometryMotionModel();
};

#endif /* LANDMARK_LOCALIZATION_MOTION_MODEL_ODOMETRY_H_ */
