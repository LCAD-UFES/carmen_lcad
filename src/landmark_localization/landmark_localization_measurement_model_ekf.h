#ifndef LANDMARK_LOCALIZATION_MEASUREMENT_MODEL_EKF_H_
#define LANDMARK_LOCALIZATION_MEASUREMENT_MODEL_EKF_H_

#include <matrix.h>

class EkfMeasurementModel {

public:

	typedef enum
	{
		BeanRangeFinderModel,
		LikehoodFieldRangeFinderModel,
		ScanMatchingModel
	}MeasurementModelsTypes;

	 // weights of each distribution
	double weight_zhit;
	double weight_zshort;
	double weight_zmax;
	double weight_zrand;

	// weights of each distribution for stereo camera
	double stereocamera_weight_zhit;
	double stereocamera_weight_zshort;
	double stereocamera_weight_zrand;
	double stereocamera_weight_zmax;

	// A value to work like zmax
	double stereocamera_zmax;

	// distribution parameters
	double sigma_zhit;
	double lambda_short;
	double zmax;

	// Sampling step (affects the number of laser rays used)
	int sampling_step;
	int stereocamera_sampling_step;

	EkfMeasurementModel();
	EkfMeasurementModel(double range_max, MeasurementModelsTypes measurement_model_type);

	virtual ~EkfMeasurementModel();


private:
	MeasurementModelsTypes current_measurement_model;
};

#endif /* LANDMARK_LOCALIZATION_MEASUREMENT_MODEL_EKF_H_ */
