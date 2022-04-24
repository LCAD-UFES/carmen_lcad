#ifndef CARMEN_PROB_MEASUREMENT_MODEL_H
#define CARMEN_PROB_MEASUREMENT_MODEL_H

#ifdef __cplusplus
extern "C" 
{
#endif

typedef enum 
_MeasurementModelTypes
{
	BeanRangeFinderModel = 1,
	LikehoodFieldRangeFinderModel,
	ScanMatchingModel
} MeasurementModelTypes;

typedef enum 
_RangeSensorTypes
{
	SickLMS200 = 1,
	VelodyneHDL32E,
} RangeSensorTypes;

typedef struct 
_BeanRangeFinderMeasurementModelParams
{
	double zhit, zshort, zmax, zrand;
	double sigma_zhit;
	double lambda_short;
	double max_range;
	double fov_range;
	double angle_step;
	double start_angle;
	int sampling_step;
	int laser_beams;
	double front_offset;
	double side_offset;
	double angular_offset;
	double l0, lfree, locc;
	double lambda_short_min, lambda_short_max;
} BeanRangeFinderMeasurementModelParams;

void init_bean_range_finder_measurement_model(BeanRangeFinderMeasurementModelParams params);
double carmen_beam_range_finder_measurement_model(double *zt, carmen_point_t *xt, carmen_map_t *map);
double beam_range_finder_model_probability(double ztk_star, double ztk);

#ifdef __cplusplus
}
#endif

#endif
