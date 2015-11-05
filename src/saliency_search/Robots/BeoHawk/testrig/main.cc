/*
 * main.cc
 *
 *  Created on: Feb 18, 2010
 *      Author: uscr
 */

#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <iostream>
#include <pthread.h>
#include <cstdio>

#include "TestrigGUI.h"
#include "DataThreads.h"

extern int serialCon;
extern IplImage * frame;
extern CvCapture * camera;
extern TestrigGUI * myGui;

int main (int argc, char *argv[]) {

	//camera = cvCreateCameraCapture(0);
	IplImage * gray, * fakeGray;

	//Get the font stuffs:
	CvFont defaultFont;
	cvInitFont (&defaultFont, CV_FONT_HERSHEY_DUPLEX, 0.75, 0.75);
	//cvNamedWindow("BeoHawk Testrig - Calibrate Test-Rig", CV_WINDOW_AUTOSIZE );

	//frame = cvQueryFrame (camera);
	//gray = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
	//fakeGray = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
	//printf ("Input Camera Image is %dx%d\n", frame->width, frame->height);
/*
	//==============================================//
	//   STEP 1: Center the camera on the thingy    //
	//==============================================//

	while (cvGetWindowHandle("BeoHawk Testrig - Calibrate Test-Rig")) {
		frame = cvQueryFrame (camera);
		cvCvtColor (frame, gray, CV_RGB2GRAY);
		cvCvtColor (gray, fakeGray, CV_GRAY2RGB);

		if (!gray) return 1;

		cvLine (fakeGray, cvPoint(fakeGray->width/2,0),
				cvPoint(fakeGray->width/2, fakeGray->height), CV_RGB(255, 0, 0), 2);
		cvLine (fakeGray, cvPoint(0,fakeGray->height/2),
						cvPoint(fakeGray->width, fakeGray->height/2), CV_RGB(255, 0, 0), 2);

		cvPutText(fakeGray, "Center the test rig in this image,",
				cvPoint(50, 30), &defaultFont, CV_RGB(255, 255, 117));
		cvPutText(fakeGray, "and then close this window.",
				cvPoint(50, 60), &defaultFont, CV_RGB(255, 255, 117));

		cvShowImage("BeoHawk Testrig - Calibrate Test-Rig", fakeGray);
		char c = cvWaitKey(10);
		if( c == 27 ) break;
	}

	cvDestroyAllWindows();
	usleep(30000);
*/
	//=========================================//
	//   STEP 2: Run the GUI, IMU, AR stuffs   //
	//=========================================//
	QApplication gui(argc, argv);
	TestrigGUI trg;
	myGui = &trg;
	trg.show();
	gui.connect(&gui, SIGNAL(lastWindowClosed()), &gui, SLOT(quit()));

	//do run threads here
	pthread_t imuThread, camThread, guiThread;
	pthread_attr_t attr;
	void * status;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

	//start em up
	initializeDataThreads();
	pthread_create(&imuThread, &attr, runIMUThread, NULL);
	pthread_create(&camThread, &attr, runCamThread, NULL);
	pthread_create(&guiThread, &attr, runGuiThread, NULL);

	gui.exec();

	//do join here
	shutdownDataThreads();
	pthread_attr_destroy(&attr);
	pthread_join(imuThread, &status);
	pthread_join(camThread, &status);
	pthread_join(guiThread, &status);

	//cvReleaseImage(&gray);
	//cvReleaseImage(&fakeGray);
	//cvReleaseCapture(&camera);

	return 0;
}
