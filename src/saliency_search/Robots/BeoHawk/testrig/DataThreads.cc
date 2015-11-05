/*
 * DataThreads.cc
 *
 *  Created on: Feb 19, 2010
 *      Author: uscr
 */

#include <cstdio>

#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <pthread.h>
#include <sys/time.h>

#include "Control.h"
#include "DataThreads.h"
#include "Atomic6DOF.h"
#include "TestrigGUI.h"
#include "KalmanFilter.h"
#include "SimpleRectangleFinder.h"

extern struct currentIMUData curIMUData;
extern struct KalmanFilterData pitchFilter, rollFilter;

struct CollectedData myData;
bool isRunning;
TestrigGUI * myGui;
pthread_mutex_t camMutex, imuMutex;

CvCapture * camera;
IplImage * frame;

double outlierWeights[OUTLIER_SUM_RANGE];
double weightedSum[NUM_POINTS];


void initializeDataThreads() {

	isRunning = true;

	myData.oldestPoint = 0;

	for (int i = 0; i < NUM_POINTS; i++) {
		myData.camPitchAngle[i] = 0;
		myData.camRollAngle[i] = 0;
		myData.imuPitchAccelVal[i] = 0;
		myData.imuPitchAngle[i] = 0;
		myData.imuPitchGyroVal[i] = 0;
		myData.imuRollAccelVal[i] = 0;
		myData.imuRollAngle[i] = 0;
		myData.imuRollGyroVal[i] = 0;
		myData.imuDeltaT[i] = 0;
	}

	for (int i = 0; i < OUTLIER_SUM_RANGE; i++) {
		myData.imuOutlierSumPoints[i] = 0;
		outlierWeights[i] = 1.0;
	}

	for (int i = 0; i < STARTING_SUM_NUM; i++) {

	}

	myData.imuLastT = 0;

	//myData.currentCamImage = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
	myData.frameCounter = 0;

	pthread_mutex_init(&camMutex, NULL);
	pthread_mutex_init(&imuMutex, NULL);

}

void shutdownDataThreads() {
	isRunning = false;
}

void * runGuiThread(void * var) {

	while (isRunning) {
		myGui->emitReadyToUpdateGUI();
		usleep(100000);
	}

	pthread_mutex_destroy(&camMutex);
	pthread_mutex_destroy(&imuMutex);
	pthread_exit(NULL);
	return NULL;
}

void * runIMUThread(void * var) {

	if (!initializeAtomic6DOF()) {
		printf("Not getting IMU stuffs...That makes me a sad panda.\n");
		pthread_exit(NULL);
	}

//	if (!initializeController()) {
//		printf("Not getting Propeller stuffs...\n");
//		pthread_exit(NULL);
//	}

//	for (unsigned char i = 0; i < 30; i++) {
//		setMotors(i, i, i, i);
//		usleep(100000);
//	}

	struct timeval curTime;

	initializeKalmanFilter(&pitchFilter, 0.012, 0.001, 0.003);
	initializeKalmanFilter(&rollFilter, 0.012, 0.001, 0.003);

	while (isRunning) {

		while (!isIMUDataReady()) {}
		gettimeofday(&curTime, 0);

		//do kalman stuffs here
		double delta_t = (curTime.tv_usec - myData.imuLastT)/ 1000000.0;
		if (delta_t < 0)
			delta_t += 1.0;

		//update gui
		pthread_mutex_lock(&imuMutex);
		int oldestVal = myData.oldestPoint;
		updateKalmanFilter(&pitchFilter, curIMUData.adj_gyro_pitch, curIMUData.adj_accel_pitch, delta_t);
		updateKalmanFilter(&rollFilter, curIMUData.adj_gyro_roll, curIMUData.adj_accel_roll, delta_t);
		myData.imuPitchAccelVal[oldestVal] = curIMUData.accel_y;
		myData.imuPitchGyroVal[oldestVal] = curIMUData.gyro_pitch;
		myData.imuRollAccelVal[oldestVal] = curIMUData.accel_x;
		myData.imuRollGyroVal[oldestVal] = curIMUData.gyro_roll;
		myData.imuPitchAngle[oldestVal] = pitchFilter.x_0;
		myData.imuRollAngle[oldestVal] = rollFilter.x_0;
		myData.imuDeltaT[oldestVal] = delta_t*1000.0;
		myData.imuLastT = curTime.tv_usec;

		myData.oldestPoint++;
		if (myData.oldestPoint == NUM_POINTS)
			myData.oldestPoint = 0;
		pthread_mutex_unlock(&imuMutex);



		usleep(2000);

	}

	//printf ("leaving IMU\n");
	shutdownAtomic6DOF();
	//shutdownController();

	pthread_exit(NULL);

	return NULL;
}

void * runCamThread(void * var) {
	pthread_exit(NULL);
	//Initialize OpenCV:
	IplImage * gray = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
	IplImage * rChan = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);

	//Initialize Rectangle Finder:
	SimpleRectangleFinder rect (rChan);


	while (isRunning) {

		//Get the red channel and grayscale from the current frame
		frame = cvQueryFrame(camera);
		//cvShowImage("test", frame);
/*
		cvCvtColor(frame, gray, CV_RGB2GRAY);
		cvSplit(frame, NULL, NULL, rChan, NULL);

		//Perform subtract away the average channel intensity from
		// the red channel intensity - this gives how strongly an object
		// is exclusively red
		cvMax (rChan, gray, rChan);
		cvSub (rChan, gray, rChan);

		//Split the values - if they are above the threshold, make them 255
		// otherwise make them 0
		cvThreshold (rChan, rChan, RED_THRESH, 255, CV_THRESH_BINARY);

		int centerx, centery;
		rect.search(rChan, AR_PATT_WIDTH, centerx, centery);

		cvCircle(frame, cvPoint(centerx, centery), 10, CV_RGB(0, 255, 0), 5);

		//printf("\n--------------------------\n");
		//cvDrawContours(frame, accepted_contours, CV_RGB(255,0,0), CV_RGB(0,255,0), 10, 1,
		//		CV_AA, cvPoint(0, 0));
*/

		pthread_mutex_lock(&camMutex);
		cvCopy(frame, myData.currentCamImage);
		myData.frameCounter++;
		pthread_mutex_unlock(&camMutex);

		usleep(20000);
	}
	/*
	 printf("Got Frame (%dx%d)\n", frame->width, frame->height);

	 cvCvtColor (frame, abgr, CV_RGB2RGBA);

	 //convert to abgr
	 for (long i = 0; i < frame->imageSize; i += 4) {
	 temp1 = frame->imageData[i];
	 temp2 = frame->imageData[i + 1];
	 frame->imageData[i]   = frame->imageData[i + 3];
	 frame->imageData[i+1] = frame->imageData[i + 2];
	 frame->imageData[i+3] = temp1;
	 frame->imageData[i+2] = temp2;
	 }

	 if( arDetectMarker((ARUint8 *) frame->imageData, AR_PATT_THRESH, &marker_info, &marker_num) < 0 ) {
	 printf ("marker error, aborting!\n");
	 pthread_exit(NULL);
	 }

	 if (marker_num > 0) {
	 //arGetTransMatCont( marker_info, patt_trans_old, patt_center, AR_PATT_WIDTH, patt_trans_new);

	 memcpy(patt_trans_old, patt_trans_new, 12*sizeof(double));

	 cvCircle(frame, cvPoint(marker_info->pos[0], marker_info->pos[1]), 10, CV_RGB(255,0,0));
	 printf("Center: (%f, %f)\n", marker_info->pos[0], marker_info->pos[1]);
	 printf ("%d markers detected!\n", marker_num);

	 marker_info;
	 }*/

	cvReleaseImage(&gray);
	cvReleaseImage(&rChan);

	pthread_exit(NULL);
}
