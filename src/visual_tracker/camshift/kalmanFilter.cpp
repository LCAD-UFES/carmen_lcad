#include "kalmanFilter.h"

KalmanFilter::KalmanFilter(){

	kalman = cvCreateKalman(4, 2);
	kalmanState = cvCreateMat( 4, 1, CV_32FC1 );
	kalmanMeasurement = cvCreateMat( 2, 1, CV_32FC1 );
}

void KalmanFilter::startKalmanPosFilter(float px, float py){


				//x  y vx vy
	float A[] =  {1, 0, 1, 0, //xk = Ax(k-1)*x(k-1) + x(k-1)
				  0, 1, 0, 1, //yk = Ay(k-1)*y(k-1) + y(k-1)
				  0, 0, 1, 0, //Axk = Ax(k-1)
				  0, 0, 0, 1}; //Ayk = Ay(k-1)


	memcpy(kalman->transition_matrix->data.fl, A, sizeof(A));
	cvSetIdentity( kalman->measurement_matrix, cvRealScalar(1) );
	cvSetIdentity( kalman->process_noise_cov, cvRealScalar(1e-5) );
	cvSetIdentity( kalman->measurement_noise_cov, cvRealScalar(1e-3) );
	cvSetIdentity( kalman->error_cov_post, cvRealScalar(1));

	cvSet( kalmanState, cvRealScalar(0));
	kalmanState->data.fl[0] = px;
	kalmanState->data.fl[1] = py;
	cvCopy(kalmanState, kalman->state_post);
}

void KalmanFilter::estimaPos(){
		cvKalmanPredict(kalman, NULL);
}

void KalmanFilter::correctPos(float* px, float* py){
	kalmanMeasurement->data.fl[0] = *px;
	kalmanMeasurement->data.fl[1] = *py;
	cvKalmanCorrect(kalman, kalmanMeasurement);
	*px = 	 kalman->state_post->data.fl[0];
	*py = 	 kalman->state_post->data.fl[1];
}

KalmanFilter::~KalmanFilter(){
	
	cvReleaseKalman(&kalman);
	cvReleaseMat(&kalmanState);
	cvReleaseMat(&kalmanMeasurement);

}
