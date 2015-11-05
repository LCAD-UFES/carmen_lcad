#ifndef __EKF_ODOMETRY_CORE__
#define __EKF_ODOMETRY_CORE__

#include <carmen/carmen.h>
#include "ekf_odometry_interface.h"

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>
#include "nonlinearanalyticconditionalgaussianodo.h"
#include "nonlinearanalyticconditionalgaussianmeasurement.h"

// TF
#include <tf.h>

// log files
#include <fstream>

namespace estimation
{

class OdomEstimation
{
public:
  /// constructor
  OdomEstimation();

  /// destructor
  virtual ~OdomEstimation();

  /** update the extended Kalman filter
   * \param odom_active specifies if the odometry sensor is active or not
   * \param vo_active specifies if the vo sensor is active or not
   * \param filter_time update the ekf up to this time
   * \param diagnostics_res returns false if the diagnostics found that the sensor measurements are inconsistent
   * returns true on successfull update
   */
  bool update(bool neural_globalpos_active, bool vo_active, int use_landmark_correction, const tf::Time&  filter_time);

  /** initialize the extended Kalman filter
   * \param prior the prior robot pose
   * \param time the initial time of the ekf
   */
  void initialize(const tf::Transform& prior, const tf::Time& time);

  /** check if the filter is initialized
   * returns true if the ekf has been initialized already
   */
  bool isInitialized() {return filter_initialized_;};

  /** get the filter posterior
   * \param estimate the filter posterior as a columnvector
   */
  void getEstimate(MatrixWrapper::ColumnVector& estimate);

  /** get the filter posterior
   * \param time the time of the filter posterior
   * \param estimate the filter posterior as a tf transform
   */
  void getEstimate(tf::Time time, tf::Transform& estimate);

  /** get the filter posterior
   * \param time the time of the filter posterior
   * \param estimate the filter posterior as a stamped tf transform
   */
  void getEstimate(tf::Time time, tf::StampedTransform& estiamte);

  /** get the filter posterior
   * \param estimate the filter posterior as a pose with covariance
   */
  void getEstimate(carmen_ekf_odometry_odometry_message* estimate);

  /** Add a sensor measurement to the measurement buffer
   * \param meas the measurement to add
   */
  void addMeasurement(const tf::StampedTransform& meas);

  /** Add a sensor measurement to the measurement buffer
   * \param meas the measurement to add
   * \param covar the 6x6 covariance matrix of this measurement, as defined in the PoseWithCovariance message
   */
  void addMeasurement(const tf::StampedTransform& meas, const MatrixWrapper::SymmetricMatrix& covar);

  void addLandmarks(const tf::StampedTransform& observation, const tf::StampedTransform& measurement);

private:
  /// correct for angle overflow
  void angleOverflowCorrect(double& a, double ref);

  // decompose Transform into x,y,z,Rx,Ry,Rz
  void decomposeTransform(const tf::StampedTransform& trans,
			  double& x, double& y, double&z, double&Rx, double& Ry, double& Rz);
  void decomposeTransform(const tf::Transform& trans,
			  double& x, double& y, double&z, double&Rx, double& Ry, double& Rz);

  void decomposeObservation(const tf::Transform& trans,
  				double& r, double& theta);


  // pdf / model / filter
  BFL::AnalyticSystemModelGaussianUncertainty*            sys_model_;
  BFL::NonLinearAnalyticConditionalGaussianOdo*           sys_pdf_;
  BFL::AnalyticMeasurementModelGaussianUncertainty*       landmark_meas_model_;
  BFL::NonLinearAnalyticConditionalGaussianMeasurement*   landmark_meas_pdf_;
  BFL::LinearAnalyticConditionalGaussian*                 vo_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* vo_meas_model_;
  BFL::LinearAnalyticConditionalGaussian*                 neural_globalpos_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* neural_globalpos_meas_model_;
  BFL::Gaussian*                                          prior_;
  BFL::ExtendedKalmanFilter*                              filter_;
  MatrixWrapper::SymmetricMatrix                          landmark_covariance, vo_covariance_, neural_globalpos_covariance_, landmark_measurement_covariance_[15], landmark_observation_covariance_[15];

  // vars
  MatrixWrapper::ColumnVector vel_desi_, filter_estimate_old_vec_;
  tf::Transform filter_estimate_old_;
  tf::StampedTransform landmark_meas_, landmark_meas_old_, vo_meas_, vo_meas_old_, neural_globalpos_meas_, neural_globalpos_meas_old_, landmark_measurement_meas_[15], landmark_measurement_meas_old_[15], landmark_observation_meas_[15], landmark_observation_meas_old_[15];
  tf::Time filter_time_old_;
  bool filter_initialized_, landmark_initialized_, odom_initialized_, vo_initialized_, neural_globalpos_initialized_;
  int landmark_observation_number;

  // tf transformer
  tf::Transformer transformer_;

}; // class

}; // namespace

#endif
