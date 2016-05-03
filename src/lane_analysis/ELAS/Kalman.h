#ifndef __KALMAN_H
#define __KALMAN_H

#include <opencv2/opencv.hpp>
#include "AnaliseDasHoughs.h"

using namespace std;
using namespace cv;

#define THRES_MIN_EVIDENCIAS 20
#define THRES_FRAMES_COM_EVIDENCIAS 10
#define THRES_FRAMES_SEM_EVIDENCIAS 10

struct KalmanState {
	// estados
	bool estaDesativado;
	bool reiniciar;
	bool manter;
	bool limpar;
	// contadores
	int nEvidencias; // número de evidências
	int nFramesSemEvidencias; // frames consecutivos sem evidências
	int nFramesComEvidencias; // frames consecutivos com evidências e com Kalman desativado
	// houghs
	HoughDoMeio *hough;
	HoughDoMeio _hough;
};

namespace Kalman {

	void inicializa(KalmanFilter *KF, KalmanState *_estadoKalman, int m, int n);
	void estimar(KalmanFilter *KF, KalmanState *state, const HoughDoMeio &measurement, bool estimarLargura = true);
	void realizarEstimativa(KalmanFilter *KF, KalmanState *state, const Mat1d &kalmanMeasurement, bool estimarLargura = true);
	void resetaEstado(KalmanState *state, const HoughDoMeio *houghMeasurement = NULL);
	void resetaKalman(KalmanFilter *KF, int m, int n);
	void resetaKalman(KalmanFilter *KF, int m, int n, const Mat1d &kalmanMeasurement);
	void alteraMatrixTransicao(KalmanFilter *KF, bool estimarLargura);

}

#endif // __KALMAN_H