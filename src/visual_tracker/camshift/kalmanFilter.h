//Esse arquivo tem por objetivo realizar o filtro de vari�veis que possam ser modeladas conforme segue abaixo:
//				  x  y vx vy
//	        	 |1  0  1  0| //xk = Ax(k-1)*x(k-1) + x(k-1)
//			A =	 |0  1  0  1| //yk = Ay(k-1)*y(k-1) + y(k-1)
//				 |0  0  1  0| //Axk = Ax(k-1)
//				 |0  0  0  1| //Ayk = Ay(k-1)


#ifndef KALMAN_FILTER
#define KALMAN_FILTER

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

class KalmanFilter{

private:

	CvKalman *kalman;
	CvMat* kalmanMeasurement; //Matriz de medidas
	CvMat* kalmanState; //Matriz de estados

public:

	KalmanFilter();
	void startKalmanPosFilter(float px, float py);
	void estimaPos();
	void correctPos(float *px, float *py);
	~KalmanFilter();

};



#endif
