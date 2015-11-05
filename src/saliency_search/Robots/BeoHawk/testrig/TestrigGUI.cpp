/*
 * testrigGUI.cc
 *
 *  Created on: Feb 18, 2010
 *      Author: uscr
 */

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <pthread.h>
#include <cmath>
#include <cstdio>

#include <QtCore/QString>
#include <QtGui/QPen>

#include "TestrigGUI.h"

extern struct CollectedData myData;
extern struct KalmanFilterData pitchFilter, rollFilter;
extern pthread_mutex_t camMutex, imuMutex;
extern IplImage * frame;

TestrigGUI::TestrigGUI() {
	ui.setupUi(this);
	connect (this, SIGNAL(readyToUpdateGUI()), this, SLOT(updateGUIDisplay()));
	connect (ui.controlVelocity, SIGNAL(valueChanged(int)), this, SLOT(updateControls()));
	connect (ui.controlYaw, SIGNAL(valueChanged(int)), this, SLOT(updateControls()));
	connect (ui.reset_pitch, SIGNAL(clicked()), this, SLOT(updatePitchFilter()));
	connect (ui.reset_roll, SIGNAL(clicked()), this, SLOT(updateRollFilter()));

	//curIplImage = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
	frameCounter = -1;
	ui.roll_cmp->setTitle(QString("Roll Filtered Angle"));
	ui.pitch_cmp->setTitle(QString("Pitch Filtered Angle"));
	ui.roll_raw->setTitle(QwtText("Roll Raw Data (<font color=#006699>Gyro</font>, <font color=#999966>Accel</font>)"));
	ui.pitch_raw->setTitle(QwtText("Pitch Raw Data (<font color=#006699>Gyro</font>, <font color=#999966>Accel</font>)"));

	for (int i = 0; i < NUM_POINTS; i++) {
		myGuiPlotData.camPitchAngle[i] = 0;
		myGuiPlotData.camRollAngle[i] = 0;
		myGuiPlotData.imuPitchAccelVal[i] = 0;
		myGuiPlotData.imuPitchAngle[i] = 0;
		myGuiPlotData.imuPitchGyroVal[i] = 0;
		myGuiPlotData.imuRollAccelVal[i] = 0;
		myGuiPlotData.imuRollAngle[i] = 0;
		myGuiPlotData.imuRollGyroVal[i] = 0;
		myGuiPlotData.imuDeltaT[i] = 0;
	}
	myGuiPlotData.imuLastT = 0;

	rollGyroCurve = new QwtPlotCurve();
	rollAccelCurve = new QwtPlotCurve();
	rollFilterCurve = new QwtPlotCurve();
	pitchGyroCurve = new QwtPlotCurve();
	pitchAccelCurve = new QwtPlotCurve();
	pitchFilterCurve = new QwtPlotCurve();

	rollGyroCurve->setRawData(incrementalTime, myGuiPlotData.imuRollGyroVal, NUM_POINTS);
	rollGyroCurve->setPen(QPen(QColor(0, 102, 153)));
	rollAccelCurve->setRawData(incrementalTime, myGuiPlotData.imuRollAccelVal, NUM_POINTS);
	rollAccelCurve->setPen(QPen(QColor(153, 153, 102)));
	rollFilterCurve->setRawData(incrementalTime, myGuiPlotData.imuRollAngle, NUM_POINTS);
	pitchGyroCurve->setRawData(incrementalTime, myGuiPlotData.imuPitchGyroVal, NUM_POINTS);
	pitchGyroCurve->setPen(QPen(QColor(0, 102, 153)));
	pitchAccelCurve->setRawData(incrementalTime, myGuiPlotData.imuPitchAccelVal, NUM_POINTS);
	pitchAccelCurve->setPen(QPen(QColor(153, 153, 102)));
	pitchFilterCurve->setRawData(incrementalTime, myGuiPlotData.imuPitchAngle, NUM_POINTS);

	ui.roll_raw->setAutoReplot(false);
	ui.roll_cmp->setAutoReplot(false);
	ui.pitch_raw->setAutoReplot(false);
	ui.pitch_cmp->setAutoReplot(false);

	rollGyroCurve->attach(ui.roll_raw);
	rollAccelCurve->attach(ui.roll_raw);
	rollFilterCurve->attach(ui.roll_cmp);
	pitchGyroCurve->attach(ui.pitch_raw);
	pitchAccelCurve->attach(ui.pitch_raw);
	pitchFilterCurve->attach(ui.pitch_cmp);
	//ui.roll_raw->setAxisScale(0, 200, 800, 100);
	ui.roll_raw->setAxisScale(2, 0, 10500, 2000);
	ui.roll_cmp->setAxisScale(2, 0, 10500, 2000);
	ui.pitch_raw->setAxisScale(2, 0, 10500, 2000);
	ui.pitch_cmp->setAxisScale(2, 0, 10500, 2000);


	fft_in = ( fftw_complex* ) fftw_malloc( sizeof( fftw_complex ) * NUM_POINTS );
	fft_out = ( fftw_complex* ) fftw_malloc( sizeof( fftw_complex ) * NUM_POINTS );
	plan_forward  = fftw_plan_dft_1d( NUM_POINTS, fft_in, fft_out, FFTW_FORWARD, FFTW_ESTIMATE );

	fftCurve = new QwtPlotCurve();
	fftCurve->setRawData(incrementalTime, myGuiPlotData.imuPitchAngle, NUM_POINTS);
	fftCurve->setStyle(QwtPlotCurve::Dots);
	ui.fft_plot->setAutoReplot(false);
	fftCurve->attach(ui.fft_plot);
	ui.fft_plot->setAxisScale(0, 0, 10000000, 1000000);
	ui.fft_plot->setAxisScale(2, 500, 1000, 100);

	ui.fft_select->insertItem(0, QString("Pitch Filtered Data"));
	ui.fft_select->insertItem(1, QString("Pitch Raw Gyro Data"));
	ui.fft_select->insertItem(2, QString("Pitch Raw Accel Data"));
	ui.fft_select->insertItem(3, QString("Roll Filtered Data"));
	ui.fft_select->insertItem(4, QString("Roll Raw Gyro Data"));
	ui.fft_select->insertItem(5, QString("Roll Raw Accel Data"));

}

TestrigGUI::~TestrigGUI() {
	rollGyroCurve->detach();

    fftw_destroy_plan( plan_forward );
    fftw_free( fft_in );
    fftw_free( fft_out );

	//cvReleaseImage (&curIplImage);
}

void TestrigGUI::emitReadyToUpdateGUI() {
	emit readyToUpdateGUI();
}

double TestrigGUI::getMean(double * data) {
	double mean = 0;

	for (int i = 0; i < NUM_POINTS; i++) {
		mean += data[i];
	}

	return mean / (double) NUM_POINTS;
}

double TestrigGUI::getStandardDeviation(double * data) {

	int sdWidth = 30;
	double mean = 0, var = 0;
	int startVal = myGuiData.oldestPoint - sdWidth;
	if (startVal < 0)
		startVal += NUM_POINTS;

	for (int i = startVal; i != myGuiData.oldestPoint; i++) {
		mean += data[i];
		if (i + 1 == NUM_POINTS)
			i = -1;
	}

	mean /= (double) sdWidth;

	for (int i = startVal; i != myGuiData.oldestPoint; i++) {
		var += (data[i] - mean)*(data[i] - mean);
		if (i + 1 == NUM_POINTS)
			i = -1;
	}

	var /= (double) sdWidth;

	return var;
}


void TestrigGUI::updateGUIDisplay() {


	//Calculate Statistics:
	pthread_mutex_lock(&camMutex);
	pthread_mutex_lock(&imuMutex);
	memcpy(&myGuiData, &myData, sizeof(struct CollectedData));
	memcpy(&guiRollFilter, &rollFilter, sizeof(struct KalmanFilterData));
	memcpy(&guiPitchFilter, &pitchFilter, sizeof(struct KalmanFilterData));
	pthread_mutex_unlock(&imuMutex);
	pthread_mutex_unlock(&camMutex);

	//Throw in the stat stuff at the bottom of the page
	ui.pitch_accel_sd->setText(QString::number(getStandardDeviation(myGuiData.imuPitchAccelVal), 'g', 3));
	ui.pitch_gyro_sd->setText(QString::number(getStandardDeviation(myGuiData.imuPitchGyroVal), 'g', 3));
	ui.roll_accel_sd->setText(QString::number(getStandardDeviation(myGuiData.imuRollAccelVal), 'g', 3));
	ui.roll_gyro_sd->setText(QString::number(getStandardDeviation(myGuiData.imuRollGyroVal), 'g', 3));
	ui.imu_up_per_sec->setText(QString::number( (1000.0 / getMean(myGuiData.imuDeltaT)) , 'g', 3));
	//camupdates

	//update kalman stuff if the button is unchecked
	if (!ui.pauseKalmanUpdates->isChecked()) {

		ui.roll_accel_var->setValue(guiRollFilter.Sz*1000);
		ui.roll_gyro_var->setValue(guiRollFilter.Sw_00);
		ui.roll_bias_var->setValue(guiRollFilter.Sw_11);

		ui.pitch_accel_var->setValue(guiPitchFilter.Sz*1000);
		ui.pitch_gyro_var->setValue(guiPitchFilter.Sw_00);
		ui.pitch_bias_var->setValue(guiPitchFilter.Sw_11);
	}

	ui.roll_kalman_0->setText(QString::number(guiRollFilter.K_0));
	ui.roll_kalman_1->setText(QString::number(guiRollFilter.K_1));
	ui.roll_p_00->setText(QString::number(guiRollFilter.P_00));
	ui.roll_p_01->setText(QString::number(guiRollFilter.P_01));
	ui.roll_p_10->setText(QString::number(guiRollFilter.P_10));
	ui.roll_p_11->setText(QString::number(guiRollFilter.P_11));

	ui.pitch_kalman_0->setText(QString::number(guiPitchFilter.K_0));
	ui.pitch_kalman_1->setText(QString::number(guiPitchFilter.K_1));
	ui.pitch_p_00->setText(QString::number(guiPitchFilter.P_00));
	ui.pitch_p_01->setText(QString::number(guiPitchFilter.P_01));
	ui.pitch_p_10->setText(QString::number(guiPitchFilter.P_10));
	ui.pitch_p_11->setText(QString::number(guiPitchFilter.P_11));

	int startVal = myGuiData.oldestPoint + 1;
	if (startVal < 0)
		startVal += NUM_POINTS;

	switch (ui.tabWidget->currentIndex()) {
	//=========================================================================
	case 0: //graph stuffs

		//rotate all of the relevant data arrays to the correct value so that we don't
		// get the line from the last value to the first:
		//also get the incremental time (AKA ranges from 0ms-last ms)
		incrementalTime[0] = 0;
		myGuiPlotData.imuPitchAccelVal[0] = myGuiData.imuPitchAccelVal[myGuiData.oldestPoint];
		myGuiPlotData.imuPitchGyroVal[0] = myGuiData.imuPitchGyroVal[myGuiData.oldestPoint];
		myGuiPlotData.imuPitchAngle[0] = myGuiData.imuPitchAngle[myGuiData.oldestPoint];

		myGuiPlotData.imuRollAccelVal[0] = myGuiData.imuRollAccelVal[myGuiData.oldestPoint];
		myGuiPlotData.imuRollGyroVal[0] = myGuiData.imuRollGyroVal[myGuiData.oldestPoint];
		myGuiPlotData.imuRollAngle[0] = myGuiData.imuRollAngle[myGuiData.oldestPoint];

		for (int j = 1, i = startVal; j < NUM_POINTS; i++, j++) {

			myGuiPlotData.imuPitchAccelVal[j] = myGuiData.imuPitchAccelVal[i];
			myGuiPlotData.imuPitchGyroVal[j] = myGuiData.imuPitchGyroVal[i];
			myGuiPlotData.imuPitchAngle[j] = myGuiData.imuPitchAngle[i];
			myGuiPlotData.imuRollAccelVal[j] = myGuiData.imuRollAccelVal[i];
			myGuiPlotData.imuRollGyroVal[j] = myGuiData.imuRollGyroVal[i];
			myGuiPlotData.imuRollAngle[j] = myGuiData.imuRollAngle[i];

			incrementalTime[j] =  myGuiData.imuDeltaT[i] + incrementalTime[j - 1];

			if (i + 1 == NUM_POINTS)
				i = -1;
		}

		if (ui.constPitchAxis->isChecked()) {
			ui.pitch_cmp->setAxisScale(0, -30.0, 30.0, 5.0);
		} else {
			ui.pitch_cmp->setAxisAutoScale(0);
		}
		if (ui.constRollAxis->isChecked()) {
			ui.roll_cmp->setAxisScale(0, -30.0, 30.0, 5.0);
		} else {
			ui.roll_cmp->setAxisAutoScale(0);
		}

		ui.roll_raw->replot();
		ui.pitch_raw->replot();
		ui.roll_cmp->replot();
		ui.pitch_cmp->replot();
		break;
	//=========================================================================
	case 1: //fft stuffs

		double * fftDataArray;

		switch (ui.fft_select->currentIndex()) {
		case 0: //pitch filtered data
			fftDataArray = myGuiData.imuPitchAngle;
			break;
		case 1: //pitch gyro data
			fftDataArray = myGuiData.imuPitchGyroVal;
			break;
		case 2: //pitch accel data
			fftDataArray = myGuiData.imuPitchAccelVal;
			break;
		case 3: //roll filtered data
			fftDataArray = myGuiData.imuRollAngle;
			break;
		case 4: //roll gyro data
			fftDataArray = myGuiData.imuRollGyroVal;
			break;
		case 5: //roll accel data
			fftDataArray = myGuiData.imuRollAccelVal;
			break;
		default:
			fftDataArray = myGuiData.imuPitchAngle;
			break;
		}

		for (int j = 1, i = startVal; j < NUM_POINTS; i++, j++) {
			fft_in[j][0] = fftDataArray[i];
			fft_in[j][1] = 0;

			if (i + 1 == NUM_POINTS)
				i = -1;
		}

		fftw_execute( plan_forward );

		for (int i = 0; i < NUM_POINTS; i++) {
			incrementalTime[i] = i;
			myGuiPlotData.imuPitchAngle[i] = fft_out[i][0]*fft_out[i][0] + fft_out[i][1]*fft_out[i][1];
		}

		ui.fft_plot->replot();
		break;
	//=========================================================================
	case 2: //control stuffs


		break;
	//=========================================================================
	case 3: //cam stuffs
		if (frameCounter != myData.frameCounter) {

			pthread_mutex_lock(&camMutex);
			frameCounter = myData.frameCounter;
			//cvCvtColor(myData.currentCamImage, curIplImage, CV_BGR2RGB);
			pthread_mutex_unlock(&camMutex);

			//curQImage = QImage ((unsigned char *) curIplImage->imageData, curIplImage->width, curIplImage->height,
			//		curIplImage->widthStep, QImage::Format_RGB888);
			//ui.cameraLabel->setPixmap(QPixmap::fromImage(curQImage));
		}
		break;
	default:
		break;
	}
}




void TestrigGUI::updateControls() {

}

void TestrigGUI::updatePitchFilter() {
	//if paused only do the following
	if (ui.pauseKalmanUpdates->isChecked()) {

		pthread_mutex_lock(&imuMutex);
		initializeKalmanFilter(&pitchFilter, ui.pitch_accel_var->value()/1000.0,
				ui.pitch_gyro_var->value(), ui.pitch_bias_var->value());
		pthread_mutex_unlock(&imuMutex);
	}
}

void TestrigGUI::updateRollFilter() {
	//if paused only do the following
	if (ui.pauseKalmanUpdates->isChecked()) {

		pthread_mutex_lock(&imuMutex);
		initializeKalmanFilter(&rollFilter, ui.roll_accel_var->value()/1000.0,
				ui.roll_gyro_var->value(), ui.roll_bias_var->value());
		pthread_mutex_unlock(&imuMutex);
	}
}
