#include "ekf_odometry_core.h"

using namespace MatrixWrapper;
using namespace BFL;
using namespace tf;
using namespace std;
//s using namespace ros;

extern int ekf_odometry_visual_search_num_landmarks_g;

namespace estimation
{
// constructor
OdomEstimation::OdomEstimation():
    														prior_(NULL),
    														filter_(NULL),
    														filter_initialized_(false),
    														vo_initialized_(false),
    														neural_globalpos_initialized_(false)
{
	// create SYSTEM MODEL
	ColumnVector sysNoise_Mu(6);  sysNoise_Mu = 0;
	SymmetricMatrix sysNoise_Cov(6); sysNoise_Cov = 0;
	for (unsigned int i=1; i<=6; i++) sysNoise_Cov(i,i) = pow(1.0, 2);
	for (unsigned int i=1; i<=6; i++) sysNoise_Mu(i) = pow(0.1, 2);
	Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
	sys_pdf_   = new NonLinearAnalyticConditionalGaussianOdo(system_Uncertainty);
	sys_model_ = new AnalyticSystemModelGaussianUncertainty(sys_pdf_);

	// create LANDMARK MEASUREMENT MODEL
	ColumnVector measNoiseLandmark_Mu(2);  measNoiseLandmark_Mu = 0;
	SymmetricMatrix measNoiseLandmark_Cov(2); measNoiseLandmark_Cov = 0;
	for (unsigned int i=1; i<=2; i++) measNoiseLandmark_Cov(i,i) = pow(1.0, 2);
	for (unsigned int i=1; i<=2; i++) measNoiseLandmark_Mu(i) = pow(0.1, 2);
	Gaussian measurement_Uncertainty_Landmark(measNoiseLandmark_Mu, measNoiseLandmark_Cov);
	landmark_meas_pdf_   = new NonLinearAnalyticConditionalGaussianMeasurement(measurement_Uncertainty_Landmark);
	landmark_meas_model_ = new AnalyticMeasurementModelGaussianUncertainty(landmark_meas_pdf_);

	// create MEASUREMENT MODEL VO
	ColumnVector measNoiseVo_Mu(6);  measNoiseVo_Mu = 0;
	SymmetricMatrix measNoiseVo_Cov(6);  measNoiseVo_Cov = 0;
	for (unsigned int i=1; i<=6; i++) measNoiseVo_Cov(i,i) = pow(1.0, 2);
	Gaussian measurement_Uncertainty_Vo(measNoiseVo_Mu, measNoiseVo_Cov);
	Matrix Hvo(6,6);  Hvo = 0;
	Hvo(1,1) = 1;    Hvo(2,2) = 1;    Hvo(3,3) = 1;    Hvo(4,4) = 1;    Hvo(5,5) = 1;    Hvo(6,6) = 1;
	vo_meas_pdf_   = new LinearAnalyticConditionalGaussian(Hvo, measurement_Uncertainty_Vo);
	vo_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(vo_meas_pdf_);

	// create MEASUREMENT MODEL NEURAL GLOBAL
	ColumnVector measNoiseNeuralGlobalpos_Mu(6);  measNoiseNeuralGlobalpos_Mu = 0;
	SymmetricMatrix measNoiseNeuralGlobalpos_Cov(6);  measNoiseNeuralGlobalpos_Cov = 0;
	for (unsigned int i=1; i<=6; i++) measNoiseNeuralGlobalpos_Cov(i,i) = pow(2.5, 2);
	for (unsigned int i=1; i<=6; i++) measNoiseNeuralGlobalpos_Mu(i) = pow(0.25, 2);
	Gaussian measurement_Uncertainty_NeuralGlobalpos(measNoiseNeuralGlobalpos_Mu, measNoiseNeuralGlobalpos_Cov);
	Matrix HNeuralGlobalpos(6,6);  HNeuralGlobalpos = 0;
	HNeuralGlobalpos(1,1) = 1;    HNeuralGlobalpos(2,2) = 1;    HNeuralGlobalpos(3,3) = 1;
	HNeuralGlobalpos(4,4) = 1;    HNeuralGlobalpos(5,5) = 1;    HNeuralGlobalpos(6,6) = 1;
	neural_globalpos_meas_pdf_   = new LinearAnalyticConditionalGaussian(HNeuralGlobalpos, measurement_Uncertainty_NeuralGlobalpos);
	neural_globalpos_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(neural_globalpos_meas_pdf_);
};



// destructor
OdomEstimation::~OdomEstimation(){
	if (filter_) delete filter_;
	if (prior_)  delete prior_;
	delete vo_meas_model_;
	delete vo_meas_pdf_;
	delete neural_globalpos_meas_model_;
	delete neural_globalpos_meas_pdf_;
	delete landmark_meas_model_;
	delete landmark_meas_pdf_;
	delete sys_pdf_;
	delete sys_model_;
};


// initialize prior density of filter
void OdomEstimation::initialize(const Transform& prior, const Time& time)
{
	// set prior of filter
	ColumnVector prior_Mu(6);
	decomposeTransform(prior, prior_Mu(1), prior_Mu(2), prior_Mu(3), prior_Mu(4), prior_Mu(5), prior_Mu(6));

	printf("x:%f y:%f\n", prior_Mu(1), prior_Mu(2));

	SymmetricMatrix prior_Cov(6);
	for (unsigned int i=1; i<=6; i++) {
		for (unsigned int j=1; j<=6; j++){
			if (i==j)  prior_Cov(i,j) = pow(0.001,2);
			else prior_Cov(i,j) = 0;
		}
	}
	prior_  = new Gaussian(prior_Mu,prior_Cov);
	filter_ = new ExtendedKalmanFilter(prior_);

	// remember prior
	addMeasurement(StampedTransform(prior, time, "/odom_combined", "/base_footprint"));
	filter_estimate_old_vec_ = prior_Mu;
	filter_estimate_old_ = prior;
	filter_time_old_     = time;

	// filter initialized
	filter_initialized_ = true;
}


// update filter
bool OdomEstimation::update(bool neural_globalpos_active, bool vo_active, int use_landmark_correction, const Time&  filter_time)
{
	// only update filter when it is initialized
	if (!filter_initialized_){
		carmen_verbose("Cannot update filter when filter was not initialized first.\n");
		return false;
	}

	// only update filter for time later than current filter time
	double dt = (filter_time - filter_time_old_).toSec();
	if (dt == 0)
	{
		carmen_verbose("Time step equals zero!\n");
		return false;
	}
	if (dt <  0){
		carmen_verbose("Will not update robot pose with time %f sec in the past.\n", dt);
		return false;
	}
	carmen_verbose("Update filter at time %f with dt %f\n", filter_time.toSec(), dt);


	// prediction
	if (vo_active){
		if (!transformer_.canTransform("/base_footprint","/vo", filter_time)){
			carmen_verbose("filter time older than vo message buffer\n");
			return false;
		}
		transformer_.lookupTransform("/vo", "/base_footprint", filter_time, vo_meas_);
		if (vo_initialized_){
			// convert absolute vo measurements to relative vo measurements
			ColumnVector vo_rel(6), vo_old_rel(6);
			decomposeTransform(vo_meas_, vo_rel(1),  vo_rel(2), vo_rel(3), vo_rel(4), vo_rel(5), vo_rel(6));
			decomposeTransform(vo_meas_old_, vo_old_rel(1),  vo_old_rel(2), vo_old_rel(3), vo_old_rel(4), vo_old_rel(5), vo_old_rel(6));

			double v, phi, L, dx, dy, dtheta, cos_theta;

			L = 2.625;
			dx = vo_rel(1) - vo_old_rel(1);
			dy = vo_rel(2) - vo_old_rel(2);
			dtheta = vo_rel(6) - vo_old_rel(6);

			cos_theta = fabs(cos(vo_old_rel(6)));

			if(cos_theta < 0.01)
			{
				v = dy / sin(vo_old_rel(6));
			}
			else
			{
				v = dx / cos(vo_old_rel(6));
			}

			if(fabs(v) < 0.01)
			{
				v = 0.0;
				phi = 0.0;
			}
			else
			{
				//check limits for -180/180
				if(fabs(dtheta) >= M_PI)
					dtheta = vo_rel(6) - fabs(vo_old_rel(6));

				phi = atan((dtheta * L) / v);
			}

			//printf("v: %f, phi: %f\n", v, phi);

			ColumnVector vel_desi(2);

			vel_desi(1) = v;
			vel_desi(2) = phi;

			sys_pdf_->AdditiveNoiseSigmaSet(vo_covariance_ * pow(dt,2));
			filter_->Update(sys_model_, vel_desi);
		}
		else vo_initialized_ = true;
		vo_meas_old_ = vo_meas_;
	}
	// sensor not active
	else vo_initialized_ = false;


	//correction

	// process neural globalpos measurement
	carmen_verbose("process neural_globalpos measurements...\n");
	if (neural_globalpos_active){
		std::string error_msg;

		if (!transformer_.canTransform("/base_footprint","/neural_globalpos", filter_time, &error_msg)){
			printf("filter time older than neural_globalpos message buffer: %s\n", error_msg.c_str());
			return false;
		}

		transformer_.lookupTransform("/neural_globalpos", "/base_footprint", filter_time, neural_globalpos_meas_);

		if (neural_globalpos_initialized_){
			ColumnVector neural_globalpos_vec(6);
			decomposeTransform(neural_globalpos_meas_, neural_globalpos_vec(1),  neural_globalpos_vec(2), neural_globalpos_vec(3), neural_globalpos_vec(4), neural_globalpos_vec(5), neural_globalpos_vec(6));
			angleOverflowCorrect(neural_globalpos_vec(6), filter_estimate_old_vec_(6));

			neural_globalpos_meas_pdf_->AdditiveNoiseSigmaSet(neural_globalpos_covariance_ * pow(dt,2));

			filter_->Update(neural_globalpos_meas_model_,  neural_globalpos_vec);
		}
		else neural_globalpos_initialized_ = true;
		neural_globalpos_meas_old_ = neural_globalpos_meas_;
	}
	// sensor not active
	else neural_globalpos_initialized_ = false;

	// process landmarks measurements
	carmen_verbose("process landmarks measurements...\n");

	if(use_landmark_correction)
	{
		std::string error_msg;
		char measurement_frame[1024];
		char observation_frame[1024];

		for(int i = 0;  i < ekf_odometry_visual_search_num_landmarks_g; i++) //numero max de landmarks
		{
			sprintf(measurement_frame, "/landmark_measurement%d", i);
			sprintf(observation_frame, "/landmark_observation%d", i);

			if (!transformer_.canTransform("/base_footprint", measurement_frame, filter_time, &error_msg) ||
					!transformer_.canTransform("/base_footprint", observation_frame, filter_time, &error_msg)){
				continue;
			}

			transformer_.lookupTransform(measurement_frame, "/base_footprint", filter_time, landmark_measurement_meas_[i]);
			transformer_.lookupTransform(observation_frame, "/base_footprint", filter_time, landmark_observation_meas_[i]);

			if (neural_globalpos_initialized_){
				ColumnVector observation_data(2);
				ColumnVector measurement_data(2);

				//landmark_meas_pdf_->AdditiveNoiseSigmaSet(landmark_observation_covariance_[i] * pow(dt,2));
				landmark_meas_pdf_->AdditiveNoiseSigmaSet(landmark_observation_covariance_[i]);

				decomposeObservation(landmark_observation_meas_[i], observation_data(1),  observation_data(2));
				//observation_data += landmark_meas_pdf_->AdditiveNoiseMuGet();

				//printf("landmark_observation(%d) - r: %f, theta: %f\n", i, observation_data(1), observation_data(2));

				measurement_data(1) = landmark_measurement_meas_[i].getOrigin().x();
				measurement_data(2) = landmark_measurement_meas_[i].getOrigin().y();

				filter_->Update(landmark_meas_model_, observation_data, measurement_data);
			}
			else neural_globalpos_initialized_ = true;

			landmark_measurement_meas_old_[i] = landmark_measurement_meas_[i];
			landmark_observation_meas_old_[i] = landmark_observation_meas_[i];
		}
	}

	// remember last estimate
	filter_estimate_old_vec_ = filter_->PostGet()->ExpectedValueGet();
	tf::Quaternion q;
	q.setRPY(filter_estimate_old_vec_(4), filter_estimate_old_vec_(5), filter_estimate_old_vec_(6));
	filter_estimate_old_ = Transform(q,
			Vector3(filter_estimate_old_vec_(1), filter_estimate_old_vec_(2), filter_estimate_old_vec_(3)));
	filter_time_old_ = filter_time;
	addMeasurement(StampedTransform(filter_estimate_old_, filter_time, "/odom_combined", "/base_footprint"));

	return true;
};

void OdomEstimation::addMeasurement(const StampedTransform& meas)
{
	carmen_verbose("AddMeasurement from %s to %s:  (%f, %f, %f)  (%f, %f, %f, %f)\n",
			meas.frame_id_.c_str(), meas.child_frame_id_.c_str(),
			meas.getOrigin().x(), meas.getOrigin().y(), meas.getOrigin().z(),
			meas.getRotation().x(),  meas.getRotation().y(),
			meas.getRotation().z(), meas.getRotation().w());


	if(!transformer_.setTransform(meas))
	{
		printf("Cannot add transform from %s to %s.\n", meas.frame_id_.c_str(), meas.child_frame_id_.c_str());
	}
}

void OdomEstimation::addMeasurement(const StampedTransform& meas, const MatrixWrapper::SymmetricMatrix& covar)
{
	// check covariance
	for (unsigned int i=0; i<covar.rows(); i++){
		if (covar(i+1,i+1) == 0){
			carmen_verbose("Covariance specified for measurement on topic %s is zero\n", meas.child_frame_id_.c_str());
			return;
		}
	}
	// add measurements
	addMeasurement(meas);
	if (meas.child_frame_id_ == "/vo")   vo_covariance_   = covar;
	else if (meas.child_frame_id_ == "/neural_globalpos")  neural_globalpos_covariance_  = covar;
	else if (meas.child_frame_id_ == "/landmark_measurement0") landmark_measurement_covariance_[0] = covar;
	else if	(meas.child_frame_id_ == "/landmark_measurement1") landmark_measurement_covariance_[1] = covar;
	else if (meas.child_frame_id_ == "/landmark_measurement2") landmark_measurement_covariance_[2] = covar;
	else if (meas.child_frame_id_ == "/landmark_measurement3") landmark_measurement_covariance_[3] = covar;
	else if (meas.child_frame_id_ == "/landmark_measurement4") landmark_measurement_covariance_[4] = covar;
	else if (meas.child_frame_id_ == "/landmark_measurement5") landmark_measurement_covariance_[5] = covar;
	else if	(meas.child_frame_id_ == "/landmark_measurement6") landmark_measurement_covariance_[6] = covar;
	else if (meas.child_frame_id_ == "/landmark_measurement7") landmark_measurement_covariance_[7] = covar;
	else if (meas.child_frame_id_ == "/landmark_measurement8") landmark_measurement_covariance_[8] = covar;
	else if (meas.child_frame_id_ == "/landmark_measurement9") landmark_measurement_covariance_[9] = covar;
	else if (meas.child_frame_id_ == "/landmark_measurement10") landmark_measurement_covariance_[10] = covar;
	else if	(meas.child_frame_id_ == "/landmark_measurement11") landmark_measurement_covariance_[11] = covar;
	else if (meas.child_frame_id_ == "/landmark_measurement12") landmark_measurement_covariance_[12] = covar;
	else if (meas.child_frame_id_ == "/landmark_measurement13") landmark_measurement_covariance_[13] = covar;
	else if (meas.child_frame_id_ == "/landmark_measurement14") landmark_measurement_covariance_[14] = covar;
	else if (meas.child_frame_id_ == "/landmark_observation0") landmark_observation_covariance_[0] = covar;
	else if	(meas.child_frame_id_ == "/landmark_observation1") landmark_observation_covariance_[1] = covar;
	else if (meas.child_frame_id_ == "/landmark_observation2") landmark_observation_covariance_[2] = covar;
	else if (meas.child_frame_id_ == "/landmark_observation3") landmark_observation_covariance_[3] = covar;
	else if (meas.child_frame_id_ == "/landmark_observation4") landmark_observation_covariance_[4] = covar;
	else if (meas.child_frame_id_ == "/landmark_observation5") landmark_observation_covariance_[5] = covar;
	else if	(meas.child_frame_id_ == "/landmark_observation6") landmark_observation_covariance_[6] = covar;
	else if (meas.child_frame_id_ == "/landmark_observation7") landmark_observation_covariance_[7] = covar;
	else if (meas.child_frame_id_ == "/landmark_observation8") landmark_observation_covariance_[8] = covar;
	else if (meas.child_frame_id_ == "/landmark_observation9") landmark_observation_covariance_[9] = covar;
	else if (meas.child_frame_id_ == "/landmark_observation10") landmark_observation_covariance_[10] = covar;
	else if	(meas.child_frame_id_ == "/landmark_observation11") landmark_observation_covariance_[11] = covar;
	else if (meas.child_frame_id_ == "/landmark_observation12") landmark_observation_covariance_[12] = covar;
	else if (meas.child_frame_id_ == "/landmark_observation13") landmark_observation_covariance_[13] = covar;
	else if (meas.child_frame_id_ == "/landmark_observation14") landmark_observation_covariance_[14] = covar;
	else carmen_verbose("Adding a measurement for an unknown sensor %s\n", meas.child_frame_id_.c_str());

};


// get latest filter posterior as vector
void OdomEstimation::getEstimate(ColumnVector& estimate)
{
	estimate = filter_estimate_old_vec_;
};

// get filter posterior at time 'time' as Transform
void OdomEstimation::getEstimate(Time time, Transform& estimate)
{
	StampedTransform tmp;
	if (!transformer_.canTransform("/base_footprint","/odom_combined", time)){
		carmen_verbose("Cannot get transform at time %f\n", time.toSec());
		return;
	}
	transformer_.lookupTransform("/odom_combined", "/base_footprint", time, tmp);
	estimate = tmp;
};

// get filter posterior at time 'time' as Stamped Transform
void OdomEstimation::getEstimate(Time time, StampedTransform& estimate)
{
	if (!transformer_.canTransform("/odom_combined", "/base_footprint", time)){
		carmen_verbose("Cannot get transform at time %f\n", time.toSec());
		return;
	}
	transformer_.lookupTransform("/odom_combined", "/base_footprint", time, estimate);
};

//get most recent filter posterior as PoseWithCovarianceStamped
void OdomEstimation::getEstimate(carmen_ekf_odometry_odometry_message* estimate)
{
	// pose
	StampedTransform tmp;
	if (!transformer_.canTransform("/odom_combined", "/base_footprint", tf::Time(0))){
		carmen_verbose("Cannot get transform at time %f", 0.0);
		return;
	}
	transformer_.lookupTransform("/odom_combined", "/base_footprint", tf::Time(0), tmp);

	double y, p, r;
	tf::Matrix3x3(tmp.getRotation()).getRPY(r, p, y);

	estimate->estimated_pose.position.x = tmp.getOrigin().x();
	estimate->estimated_pose.position.y = tmp.getOrigin().y();
	estimate->estimated_pose.position.z = 0.0;

	estimate->estimated_pose.orientation.yaw = y;
	estimate->estimated_pose.orientation.pitch = 0.0;
	estimate->estimated_pose.orientation.roll = 0.0;

	estimate->timestamp = tmp.stamp_.toSec();
	estimate->host = carmen_get_host();

	// covariance
	SymmetricMatrix covar =  filter_->PostGet()->CovarianceGet();
	for (unsigned int i=0; i<6; i++)
		for (unsigned int j=0; j<6; j++)
			estimate->covariance[i][j] = covar(i+1,j+1);
};

// correct for angle overflow
void OdomEstimation::angleOverflowCorrect(double& a, double ref)
{
	while ((a-ref) >  M_PI) a -= 2*M_PI;
	while ((a-ref) < -M_PI) a += 2*M_PI;
};

// decompose Transform into x,y,z,Rx,Ry,Rz
void OdomEstimation::decomposeTransform(const StampedTransform& trans,
		double& x, double& y, double&z, double&Rx, double& Ry, double& Rz){
	x = trans.getOrigin().x();
	y = trans.getOrigin().y();
	z = trans.getOrigin().z();
	trans.getBasis().getEulerYPR(Rz, Ry, Rx);
};

// decompose Transform into x,y,z,Rx,Ry,Rz
void OdomEstimation::decomposeTransform(const Transform& trans,
		double& x, double& y, double&z, double&Rx, double& Ry, double& Rz){
	x = trans.getOrigin().x();
	y = trans.getOrigin().y();
	z = trans.getOrigin().z();
	trans.getBasis().getEulerYPR(Rz, Ry, Rx);
};

void OdomEstimation::decomposeObservation(const Transform& trans,
		double& r, double& theta){

	r = sqrt((trans.getOrigin().x() * trans.getOrigin().x()) + (trans.getOrigin().y() * trans.getOrigin().y()));
	theta = atan2(trans.getOrigin().y(), trans.getOrigin().x()) -  filter_estimate_old_vec_(6);

	if(fabs(theta) >= M_PI)
		theta = atan2(trans.getOrigin().y(), trans.getOrigin().x()) - fabs(filter_estimate_old_vec_(6));
};

}; // namespace
