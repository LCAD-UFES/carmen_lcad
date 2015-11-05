/*
 * DataThreads.cc
 *
 *  Created on: Feb 18, 2010
 *      Author: uscr
 */

#ifndef DATATHREADS_H_
#define DATATHREADS_H_

#include <opencv/cv.h>


#define NUM_POINTS				1000
#define OUTLIER_SUM_RANGE		7
//below is in degrees for a jump
#define OUTLIER_REJECT_ERROR	5

#define STARTING_SUM_NUM		1


struct CollectedData {
	double imuPitchAngle[NUM_POINTS];
	double imuRollAngle[NUM_POINTS];
	double imuPitchGyroVal[NUM_POINTS];
	double imuPitchAccelVal[NUM_POINTS];
	double imuRollGyroVal[NUM_POINTS];
	double imuRollAccelVal[NUM_POINTS];

	double camPitchAngle[NUM_POINTS];
	double camRollAngle[NUM_POINTS];

	double imuOutlierSumPoints[OUTLIER_SUM_RANGE];

	int oldestPoint;

	double imuDeltaT[NUM_POINTS]; //in milliseconds
	long imuLastT; //in microseconds

	bool guiReading;

	IplImage * currentCamImage;
	CvPoint curRectangle[4];
	int frameCounter;
};


//stuffs for artoolkit:
#define RED_THRESH				40

void initializeDataThreads();
void shutdownDataThreads();
void * runIMUThread(void * var);
void * runCamThread(void * var);
void * runGuiThread(void * var);


#endif
