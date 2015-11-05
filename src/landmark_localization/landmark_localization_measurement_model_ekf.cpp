#include "landmark_localization_measurement_model_ekf.h"

EkfMeasurementModel::EkfMeasurementModel()
{
}

EkfMeasurementModel::EkfMeasurementModel(double range_max, MeasurementModelsTypes measurement_model_type)
{
	if (measurement_model_type == BeanRangeFinderModel)
	{
		weight_zhit = 0.3;
		weight_zshort = 0.1;
		weight_zmax = 0.5;
		weight_zrand = 0.1;
	}else if (measurement_model_type == LikehoodFieldRangeFinderModel)
	{
		weight_zhit = 0.9;
		weight_zmax = 0.3;
		weight_zrand = 0.1;
	}

	stereocamera_weight_zhit = 0.6;
	stereocamera_weight_zshort = 0.1;
	stereocamera_weight_zrand = 0.1;
	stereocamera_weight_zmax = 0.2;
	stereocamera_zmax = 16.0;

	//weight_zhit = 0.8;
	//weight_zshort = 0.1;
	//weight_zmax = 0.1;
	//weight_zrand = 0.1;

	sigma_zhit = 0.1;
	lambda_short = 1.0;
	zmax = range_max;
	sampling_step = 36;
	stereocamera_sampling_step = (640 * 480) / 157;

	current_measurement_model = measurement_model_type;
}

Matrix JacobianOfFunctionh(Matrix _mut, int* zt, int zt_length, carmen_map_t map)
{
	double delta_x = 0.001;
	double delta_y = 0.001;
	double delta_theta = M_PI / 1800.0;

	Matrix Ht = new Matrix(zt_length / sampling_step + 1, _mut.Rows);

	Matrix h = ComputehFunction(_mut, zt, map);

	Matrix mut_plus_delta = new Matrix(_mut.Rows, _mut.Columns);

	mut_plus_delta = _mut;
	mut_plus_delta[0, 0] += delta_x;
	Matrix h_delta = ComputehFunction(mut_plus_delta, zt, map);
	for (int i = 0; i < Ht.Rows; i++)
		Ht[i, 0] = (h_delta[i, 0] - h[i, 0]) / delta_x;

	mut_plus_delta = _mut;
	mut_plus_delta[1, 0] += delta_y;
	h_delta = ComputehFunction(mut_plus_delta, zt, map);
	for (int i = 0; i < Ht.Rows; i++)
		Ht[i, 1] = (h_delta[i, 0] - h[i, 0]) / delta_y;

	mut_plus_delta = _mut;
	mut_plus_delta[2, 0] += delta_theta;
	h_delta = ComputehFunction(mut_plus_delta, zt, map);
	for (int i = 0; i < Ht.Rows; i++)
		Ht[i, 2] = (h_delta[i, 0] - h[i, 0]) / delta_theta;

	return Ht;
}

//double EkfMeasurementModel::MeasurementModel(int* zt, Pose xt, ProbabilisticMap map)
//{
//	return MeasurementModel(zt, xt, map, ProbabilisticMap.MapColor.WHITE);
//}
//
//
//double EkfMeasurementModel::MeasurementModel(int* zt, Pose xt, ProbabilisticMap map, ProbabilisticMap.MapColor color)
//{
//	switch (current_measurement_model)
//	{
//		case MeasurementModelsTypes.BeanRangeFinderModel:
//			return BeamRangeFinderModel(zt, xt, map, color);
//
//		case MeasurementModelsTypes.LikehoodFieldRangeFinderModel:
//			return LikelihoodFieldRangeFinderModel(zt, xt, map);
//
//		case MeasurementModelsTypes.ScanMatchingModel:
//			return ScanMatchingModel(zt, xt, map);
//
//		default:
//			return BeamRangeFinderModel(zt, xt, map, color);
//
//	}
//}

EkfMeasurementModel::~EkfMeasurementModel() {
	// TODO Auto-generated destructor stub
}

