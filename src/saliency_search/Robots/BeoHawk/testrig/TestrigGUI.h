/*
 * TestrigGUI.h
 *
 *  Created on: Feb 18, 2010
 *      Author: uscr
 */

#ifndef TESTRIGGUI_H_
#define TESTRIGGUI_H_

#include <QtGui/QMainWindow>
#include <QtGui/QImage>
#include <opencv/cv.h>
#include <qwt_plot_curve.h>

#include <fftw3.h>

#include "ui_testrig.h"
#include "DataThreads.h"
#include "KalmanFilter.h"


class TestrigGUI : public QMainWindow {

	Q_OBJECT

public:
	TestrigGUI();
	virtual ~TestrigGUI();
	void emitReadyToUpdateGUI();
	double getStandardDeviation(double * vals);
	double getMean(double * vals);

signals:
	void readyToUpdateGUI();

public slots:
	void updateGUIDisplay();
	void updateControls();
	void updatePitchFilter();
	void updateRollFilter();

private:
	Ui::testrig ui;
	QImage curQImage;
	char * inputImageDataPtr; //READ ONLY and unstable
	IplImage * curIplImage;
	int frameCounter;
	struct CollectedData myGuiData, myGuiPlotData;
	struct KalmanFilterData guiRollFilter, guiPitchFilter;
	QwtPlotCurve * rollGyroCurve, * pitchGyroCurve, * rollAccelCurve, * pitchAccelCurve, * rollFilterCurve, * pitchFilterCurve;
	QwtPlotCurve * fftCurve;
	fftw_complex    *fft_in, *fft_out;
	fftw_plan       plan_forward;

	double incrementalTime[NUM_POINTS];

};

#endif /* TESTRIGGUI_H_ */
