#include "stereo_mapping_kalman_filter.h"

void kalman_update_state(kalman_filter *filter, kalman_filter_params *state[], double controls[], double measurements[])
{
  int n_dof = sizeof(state) / sizeof(state[0]);
  for (int i = 0; i < n_dof; i ++)
  {
    state[i]->outlier_mean_value = state[i]->value;
    state[i]->outlier_max_distance_from_mean = state[i]->value_variance_factor * state[i]->value_variance;

    if (state[i]->filter_outliers && (fabs(measurements[i] - state[i]->outlier_mean_value) > state[i]->outlier_max_distance_from_mean))
      measurements[i] = state[i]->value;

    if (state[i]->filter_outliers && fabs(controls[i]) > state[i]->outlier_max_distance_from_mean)
      controls[i] = 0.0;
  }

  // Initialize Kalman filter error matrixes
  CV_MAT_ELEM(*filter->kalman_filter->process_noise_cov, float, 0, 0) = state[0]->value_variance;
  CV_MAT_ELEM(*filter->kalman_filter->process_noise_cov, float, 1, 0) = state[1]->value_variance;

  CV_MAT_ELEM(*filter->kalman_filter->measurement_noise_cov, float, 0, 0) = state[0]->observation_variance;
  CV_MAT_ELEM(*filter->kalman_filter->measurement_noise_cov, float, 1, 0) = state[1]->observation_variance;

  CV_MAT_ELEM(*filter->kalman_filter->error_cov_post, float, 0, 0) = 1.0;
  CV_MAT_ELEM(*filter->kalman_filter->error_cov_post, float, 1, 0) = 1.0;

  // Initial state
  CV_MAT_ELEM(*filter->kalman_filter->state_post, float, 0, 0) = state[0]->value;
  CV_MAT_ELEM(*filter->kalman_filter->state_post, float, 1, 0) = state[1]->value;

  CV_MAT_ELEM(*filter->control, float, 0, 0) = controls[0];
  CV_MAT_ELEM(*filter->control, float, 1, 0) = controls[1];
  const CvMat *y_k = cvKalmanPredict(filter->kalman_filter, filter->control);
  cvKalmanPredict(filter->kalman_filter, filter->control);

  CV_MAT_ELEM(*filter->z_k, float, 0, 0) = measurements[0];
  CV_MAT_ELEM(*filter->z_k, float, 1, 0) = measurements[1];

  const CvMat *x_k = cvKalmanCorrect(filter->kalman_filter, filter->z_k);

  fprintf(stdout, "Measurement = [%f°, %f m]\n", carmen_radians_to_degrees(measurements[0]), measurements[1]);
  fprintf(stdout, "Predicted   = [%f°, %f m]\n", carmen_radians_to_degrees(CV_MAT_ELEM(*y_k, float, 0, 0)), CV_MAT_ELEM(*y_k, float, 1, 0));
  fprintf(stdout, "Corrected   = [%f°, %f m]\n", carmen_radians_to_degrees(CV_MAT_ELEM(*x_k, float, 0, 0)), CV_MAT_ELEM(*x_k, float, 1, 0));

//  state[0]->value = CV_MAT_ELEM(*x_k, float, 0, 0);
//  state[1]->value = CV_MAT_ELEM(*x_k, float, 1, 0);
}

void init_kalman_filter_params(kalman_filter_params *state, double value_variance, double value_variance_factor, double observation_variance)
{
  state->value_variance = value_variance;
  state->value_variance_factor = value_variance_factor;
  state->observation_variance = observation_variance;
  state->filter_outliers = true;
}
