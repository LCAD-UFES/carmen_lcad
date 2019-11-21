#ifndef SRC_UTILITIES_MOUNT_RDDF_PREDICT_PYTHON_RDDF_PREDICT_OPTIMIZER_H_
#define SRC_UTILITIES_MOUNT_RDDF_PREDICT_PYTHON_RDDF_PREDICT_OPTIMIZER_H_

#include <stdio.h>
#include <iostream>
#include <math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_multimin.h>
#include <carmen/carmen.h>


struct SplineControlParams
{
	bool valid;
	double xtotal;
	double k2;
	double k3;
	double k1;
};

struct ObjectiveFunctionParams
{
	SplineControlParams *spline_control_params;
	unsigned int n_poses;
	carmen_ackerman_traj_point_t * rddf_poses;
	double plan_cost;
	int xtotal;
};

SplineControlParams
optimize_spline_knots(carmen_behavior_selector_road_profile_message *last_rddf_poses);

#endif /* SRC_UTILITIES_MOUNT_RDDF_PREDICT_PYTHON_RDDF_PREDICT_OPTIMIZER_H_ */
