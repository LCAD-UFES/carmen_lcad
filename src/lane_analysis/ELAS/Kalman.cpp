#include "Kalman.h"

using namespace std;
using namespace cv;


void Kalman::inicializa(KalmanFilter *KF, KalmanState *state, int m, int n) {
	resetaKalman(KF, m, n);
	resetaEstado(state);
}

void Kalman::resetaKalman(KalmanFilter *KF, int m, int n) {
	// vetor de estado terá 6 params
	// vetor de medida terá 3 params
	// não haverá vetor de controle
	KF->init(m, n, 0, CV_64F);

	// inicialização do filtro
    double matrix []= {
		1, 0, 0, 1, 0, 0,		// base += delta_base
		0, 1, 0, 0, 1, 0,		// topo += delta_topo
		0, 0, 1, 0, 0, 1,		// largura += delta_largura
		0, 0, 0, 1, 0, 0,		// delta_base
		0, 0, 0, 0, 1, 0,		// delta_topo
		0, 0, 0, 0, 0, 1};      // delta_largura
	KF->transitionMatrix = (Mat(m, m, CV_64F,matrix));		
	setIdentity(KF->measurementMatrix);
	setIdentity(KF->processNoiseCov, Scalar::all(1e-5));
	setIdentity(KF->measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(KF->errorCovPost, Scalar::all(1));
}

void Kalman::resetaKalman(KalmanFilter *KF, int m, int n, const Mat1d &kalmanMeasurement) {
	// vetor de estado terá 6 params
	// vetor de medida terá 3 params
	// não haverá vetor de controle
	KF->init(m, n, 0, CV_64F);

	// inicialização do filtro
	double matrix []= {
		1, 0, 0, 1, 0, 0,		// base += delta_base
		0, 1, 0, 0, 1, 0,		// topo += delta_topo
		0, 0, 1, 0, 0, 1,		// largura += delta_largura
		0, 0, 0, 1, 0, 0,		// delta_base
		0, 0, 0, 0, 1, 0,		// delta_topo
		0, 0, 0, 0, 0, 1};      // delta_largura
	KF->transitionMatrix = (Mat(m, m, CV_64F,matrix));	
	setIdentity(KF->measurementMatrix);
	setIdentity(KF->processNoiseCov, Scalar::all(1e-5));
	setIdentity(KF->measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(KF->errorCovPost, Scalar::all(1));

	KF->statePre.at<double>(0) = kalmanMeasurement.at<double>(0);
	KF->statePre.at<double>(1) = kalmanMeasurement.at<double>(1);
	KF->statePre.at<double>(2) = kalmanMeasurement.at<double>(2);
	KF->statePre.at<double>(3) = 0;
	KF->statePre.at<double>(4) = 0;
	KF->statePre.at<double>(5) = 0;

	KF->statePost.at<double>(0) = kalmanMeasurement.at<double>(0);
	KF->statePost.at<double>(1) = kalmanMeasurement.at<double>(1);
	KF->statePost.at<double>(2) = kalmanMeasurement.at<double>(2);
	KF->statePost.at<double>(3) = 0;
	KF->statePost.at<double>(4) = 0;
	KF->statePost.at<double>(5) = 0;
}
void Kalman::resetaEstado(KalmanState *state, const HoughDoMeio *houghMeasurement) {
	// inicializa estado
	state->estaDesativado = true;
	state->limpar = false;
	state->manter = false;
	state->reiniciar = false;
	state->nEvidencias = 0;
	state->nFramesSemEvidencias = 0;
	state->nFramesComEvidencias = 0;
	if (houghMeasurement == NULL) {
		state->hough = NULL;
	} else {
		state->_hough = *houghMeasurement;
		state->hough = &(state->_hough);
	}
}

void Kalman::estimar(KalmanFilter *KF, KalmanState *state, const HoughDoMeio &measurement, bool estimarLargura) {

	Mat1d kalmanMeasurement = measurement.toKalman();

	// state-machine
	if (state->nEvidencias > THRES_MIN_EVIDENCIAS) {
		if (state->limpar) {
			state->nFramesComEvidencias++;
			state->nFramesSemEvidencias = 0;
			if (state->nFramesComEvidencias > THRES_FRAMES_COM_EVIDENCIAS) {
				state->reiniciar = true;
				state->estaDesativado = false;
				state->manter = false;
			} else {
				state->manter = true;
			}
		} else {
			state->reiniciar = false;
			state->manter = false;
			state->limpar = false;
			state->nFramesSemEvidencias = 0;
			state->nFramesComEvidencias = 0;
			realizarEstimativa(KF, state, kalmanMeasurement, estimarLargura);
			state->estaDesativado = false;
		}
	} else {
		state->nFramesSemEvidencias++;
		state->nFramesComEvidencias = 0;
		if (state->nFramesSemEvidencias > THRES_FRAMES_SEM_EVIDENCIAS) {
			state->limpar = true;
		} else {
			state->estaDesativado = true;
		}
	}

	// reiniciar
	if (state->reiniciar) {
		resetaKalman(KF, 6, 3, kalmanMeasurement);
		resetaEstado(state);
		realizarEstimativa(KF, state, kalmanMeasurement, estimarLargura);
		// cout << "opa, acho que agora voltou... essa ai deve ser boa!" << endl;
		state->reiniciar = false;
		state->estaDesativado = false;
		return;
	}

	// limpar
	if (state->limpar) {
		state->hough = NULL; // não há estimado
		resetaKalman(KF, 6, 3);
		int auxSem = state->nFramesSemEvidencias;
		int auxCom = state->nFramesComEvidencias;
		resetaEstado(state);
		state->nFramesSemEvidencias = auxSem;
		state->nFramesComEvidencias = auxCom;
		// cout << "muito incerto... melhor dizer que nao sei de nada!" << endl;
		state->limpar = true;
		return;
	}

	// desativado ou manter
	if (state->estaDesativado || state->manter) {
		// não atualiza estimado (hough do estado)
		state->manter = false;
		// cout << "sei nao... esta estranho, vou manter a ultima" << endl;
		return; 
	}
}

void Kalman::realizarEstimativa(KalmanFilter *KF, KalmanState *state, const Mat1d &kalmanMeasurement, bool estimarLargura) {
	
	// altera a matrix de transição para que a largura seja ou não estimada conforme necessário
	if (!estimarLargura) alteraMatrixTransicao(KF, false);
	
	// predição
	KF->predict();
	
	// estima
	Mat1d estimado = KF->correct(kalmanMeasurement);
	
	// atualiza o estimado
	state->_hough = HoughDoMeio(estimado);
	state->hough = &(state->_hough);

	if (!estimarLargura) alteraMatrixTransicao(KF, true);
}

void Kalman::alteraMatrixTransicao(KalmanFilter *KF, bool estimarLargura) {
	if (estimarLargura) {
        double matrix []= {
		1, 0, 0, 1, 0, 0,		// base += delta_base
		0, 1, 0, 0, 1, 0,		// topo += delta_topo
		0, 0, 1, 0, 0, 1,		// largura += delta_largura
		0, 0, 0, 1, 0, 0,		// delta_base
		0, 0, 0, 0, 1, 0,		// delta_topo
		0, 0, 0, 0, 0, 1};      // delta_largura
		KF->transitionMatrix = (Mat(KF->transitionMatrix.cols, KF->transitionMatrix.rows, CV_64F, matrix));
	} else {
        double matrix []= {
		1, 0, 0, 1, 0, 0,		// base += delta_base
		0, 1, 0, 0, 1, 0,		// topo += delta_topo
		0, 0, 1, 0, 0, 1,		// largura += delta_largura
		0, 0, 0, 1, 0, 0,		// delta_base
		0, 0, 0, 0, 1, 0,		// delta_topo
		0, 0, 0, 0, 0, 1};      // delta_largura
		KF->transitionMatrix = (Mat(KF->transitionMatrix.cols, KF->transitionMatrix.rows, CV_64F, matrix));		// delta_largura-- não estima/atualiza
	}
}
