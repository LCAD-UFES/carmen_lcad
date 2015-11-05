#ifndef LANDMARK_LOCALIZATION_EKF_H_
#define LANDMARK_LOCALIZATION_EKF_H_

#include <tf.h>
#include <matrix.h>
#include <prob_motion_model.h>

#include "landmark_localization_motion_model_odometry_ekf.h"
#include "landmark_localization_measurement_model_ekf.h"

//class EkfMotionModelOdometry;
//class EkfMeasurementModel;

class EkfLocalization
{

public:
	Matrix _mut;              // Average of the state at prediction step (Greek letter mu. See http://www.ibiblio.org/koine/greek/lessons/alphabet.html)
	Matrix mut;               // Average of the state at correction step.
	Matrix _St;               // Variable sigma, or covariance matrix, of the predition step of the algorithm in Table 3.3 of the book Probabilistic Robotics
	Matrix St;                // Variable sigma, or covariance matrix, of the correction step of the algorithm in Table 3.3 of the book Probabilistic Robotics
	Matrix Gt;                // See Table 3.3 of the book Probabilistic Robotics for the remaining variables below
	Matrix VtMtVt;
	Matrix GtSt_1Gt;
	Matrix Vt;
	Matrix Mt;
	Matrix Rt;
	Matrix Sti;
	Matrix Kti;
	Matrix Hti;
	Matrix _zti;
	Matrix Qt;
	Matrix I;

	OdometryMotionCommand ut;

	EkfMotionModelOdometry *motionModel;
    EkfMeasurementModel    *measurementModel;

	EkfLocalization();
	void InitializeEkfLocalization(carmen_point_t _ut);
	carmen_point_t RunLocalization(carmen_point_t _odom/*, int[] zt, ProbabilisticMap map*/);
	Matrix ExtendedKalmanFilterLocalization(Matrix mut_1, Matrix St_1, OdometryMotionCommand ut /*, int[] zt, ProbabilisticMap map*/);

	Matrix g(OdometryMotionCommand ut, Matrix mut_1);
	Matrix ComputeGt(OdometryMotionCommand ut, Matrix mut_1);
	Matrix ComputeVt(OdometryMotionCommand ut, Matrix mut_1);
	Matrix ComputeMt(OdometryMotionCommand ut);

	Matrix PointToMatrix(carmen_point_t point);
	carmen_point_t MatrixToPoint(Matrix matrix);

	virtual ~EkfLocalization();
};

#endif /* LANDMARK_LOCALIZATION_EKF_H_ */
