#include "landmark_localization_ekf.h"

EkfLocalization::EkfLocalization()
{
	if (this->motionModel == NULL)
		this->motionModel = new EkfMotionModelOdometry();

//	if (this->measurementModel ==  NULL)
//		this->measurementModel = new EkfMeasurementModel(map._range_max);
}

void
EkfLocalization::InitializeEkfLocalization(carmen_point_t _odom)
{
	this->ut.initial = _odom;
	this->ut.final = _odom;

	double initial_mean_covariance[9] = {0.05, 0.0, 0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 0.05}; //3x3

	St = Matrix(3, 3, initial_mean_covariance);

	_mut = mut = PointToMatrix(_odom);

	Gt = ComputeGt(ut, mut);
	Mt = ComputeMt(ut);
	Vt = ComputeVt(ut, mut);
	Rt = Vt * Mt * (~Vt);
	_St = Gt * St * (~Gt) + Rt;

	double qt_data[9] = {0.005, 0.0, 0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.005}; //3x3

	Qt = Matrix(3, 3, qt_data);
	Kti = Matrix(3, 1);
	_zti = Matrix(3, 1);
	Hti = Matrix(3, 3);
	Sti = Matrix(3, 3);
	I = Matrix(3, 3);
	I.eye();
}

carmen_point_t EkfLocalization::RunLocalization(carmen_point_t _odom/*, int[] zt, ProbabilisticMap map*/)
{
	OdometryMotionCommand previous_ut;
	previous_ut = ut;

	ut.initial = ut.final;
	ut.final = _odom;

	mut = this->ExtendedKalmanFilterLocalization(mut, St, ut/*, zt, map*/);

	carmen_point_t xt = this->MatrixToPoint(mut);
	return xt;
}

Matrix
EkfLocalization::ExtendedKalmanFilterLocalization(Matrix mut_1, Matrix St_1, OdometryMotionCommand ut /*, int[] zt, ProbabilisticMap map*/)
{

	//prediction
	_mut = g(ut, mut_1);

	Gt = ComputeGt(ut, mut_1);
	Vt = ComputeVt(ut, mut_1);
	Mt = ComputeMt(ut);
	VtMtVt = Vt * Mt * (~Vt);
	Rt = VtMtVt;
	GtSt_1Gt = Gt * St_1 * (~Gt);
	_St = GtSt_1Gt + Rt;

	//correction
//	Ht = ComputeHt(_mut, zt, map);
//	Kt = _St * Ht.Transpose() * (Ht * _St * Ht.Transpose() + Qt).Inverse;
//
//	mut = _mut + Kt * (MatrixFromZt(zt) - h(_mut, zt, map));
//
//	St = (I - Kt * Ht) * _St;

	mut = _mut;
	St = _St;

	return mut;
}

Matrix
EkfLocalization::g(OdometryMotionCommand ut, Matrix mut_1)
{
	return motionModel->MotionModelOdometry(ut, mut_1);
}

Matrix
EkfLocalization::ComputeGt(OdometryMotionCommand ut, Matrix mut_1)
{
	return motionModel->ComputeJacobianOfFunctiong(ut, mut_1);
}

Matrix
EkfLocalization::ComputeVt(OdometryMotionCommand ut, Matrix mut_1)
{
	return motionModel->ControlSpaceToStateSpaceMatrix(ut, mut_1);
}

Matrix
EkfLocalization::ComputeMt(OdometryMotionCommand ut)
{
	return motionModel->NoiseMatrixInControlSpace(ut);
}


//Matrix EkfLocalization::ComputeHt(Matrix _mut, int[] zt, LandmarkMap map)
//{
//	return measurementModel.JacobianOfFunctionh(_mut, zt, map);
//}

Matrix
EkfLocalization::PointToMatrix(carmen_point_t point)
{
	Matrix matrix;

	double _mut_data[3];
	_mut_data[0] = point.x;
	_mut_data[1] = point.y;
	_mut_data[2] = point.theta;

	matrix = Matrix(3, 1, _mut_data);

	return matrix;
}

carmen_point_t
EkfLocalization::MatrixToPoint(Matrix matrix)
{
	carmen_point_t point;

	point.x = matrix.val[0][0];
	point.y = matrix.val[1][0];
	point.theta = matrix.val[2][0];

	return point;
}

EkfLocalization::~EkfLocalization() {
	// TODO Auto-generated destructor stub
}

