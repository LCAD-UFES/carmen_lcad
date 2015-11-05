#ifndef LANDMARK_LOCALIZATION_MOTION_MODEL_ODOMETRY_EKF_H_
#define LANDMARK_LOCALIZATION_MOTION_MODEL_ODOMETRY_EKF_H_

#include "landmark_localization_motion_model_odometry.h"

class EkfMotionModelOdometry : public OdometryMotionModel {
public:
	EkfMotionModelOdometry();
	Matrix ComputeJacobianOfFunctiong(OdometryMotionCommand ut, Matrix mut_1);
	Matrix ControlSpaceToStateSpaceMatrix(OdometryMotionCommand ut, Matrix _mut);
	virtual ~EkfMotionModelOdometry();
};

#endif /* LANDMARK_LOCALIZATION_MOTION_MODEL_ODOMETRY_EKF_H_ */
