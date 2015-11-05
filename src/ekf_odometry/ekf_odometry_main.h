#ifndef __EKF_ODOMETRY_MAIN__
#define __EKF_ODOMETRY_MAIN__

#include "ekf_odometry_core.h"

static carmen_pose_3D_t car_pose_g;
static carmen_pose_3D_t camera_pose_g;
static carmen_pose_3D_t board_pose_g;
tf::Transformer transformer;

// ekf filter
estimation::OdomEstimation my_filter_;
tf::StampedTransform output_;

// vectors
tf::Transform vo_meas_, neural_globalpos_meas_;
tf::Time vo_time_, neural_globalpos_time_;
tf::Time vo_stamp_, filter_stamp_, neural_globalpos_stamp_, landmark_measurement_stamp_, landmark_observation_stamp_;
tf::Time vo_init_stamp_, neural_globalpos_init_stamp_;
bool vo_active_ = false, neural_globalpos_active_ = false;
bool vo_initializing_ = false, neural_globalpos_initializing_ = false;
MatrixWrapper::SymmetricMatrix vo_covariance_(6), neural_globalpos_covariance_(6), landmark_covariance_(2);
bool debug_, self_diagnose_;
std::string output_frame_, base_footprint_frame_, tf_prefix_;

// log files for debugging
std::ofstream vo_file_, corr_file_, time_file_, extra_file_;

// counters
unsigned int vo_callback_counter_ = 0, neural_globapos_callback_counter_ = 0, ekf_sent_counter_ = 0;

#endif
