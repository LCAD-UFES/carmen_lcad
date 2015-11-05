#ifndef VISUAL_CONTROL_H
#define VISUAL_CONTROL_H

#include <iostream>
#include <fstream>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <math.h>

#include "kalmanFilter.h"
#include "vertRetangulo.h"

#define TRESH_VAR_VERT 3

class VisualControl{

private:

	VertRetangulo vert1;	//As vari�veis vertX s�o os centros dos quadrados sobre os quais se realiza o tracking.

	KalmanFilter v1;		//As vari�veis vX s�o os filtros de kalman dos dos v�rtices acima.

	IplImage* imagePersHSV;
	IplImage* mask;

	CvPoint2D32f pt;
	CvPoint2D32f pt_nc;

	int vmin, vmax, smin; //valores para definir a mask do HSV.

	void update(IplImage *im);
public:

	~VisualControl();
	VisualControl(int width, int height);	//Construtor
	int startVisualControl(const IplImage *im, const CvRect *rect = NULL);
	int trackVisualControl(const IplImage *im); //Gera as velocidades do Pionner

	CvPoint2D32f getPredictedPoint() { return pt_nc;};
	CvPoint2D32f getCorrectedPoint() { return pt;};

	int getRectWidth() { return vert1.getRect().width; };
	int getRectHeight() { return vert1.getRect().height; };

	CvPoint2D32f getRectCoordLeftUp() {
		return cvPoint2D32f(
			pt_nc.x - ((float)vert1.getRect().width)/2.0f,
			pt_nc.y - ((float)vert1.getRect().height)/2.0f);
	};

	CvPoint2D32f getRectCoordRightDown() {
		return cvPoint2D32f(
			pt_nc.x + ((float)vert1.getRect().width)/2.0f,
			pt_nc.y + ((float)vert1.getRect().height)/2.0f);
	};
};


#endif
