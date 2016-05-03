#ifndef __HOUGHS_H
#define __HOUGHS_H

#include <opencv2/opencv.hpp>
#include "AnaliseDasHoughs.h"
#include "Kalman.h"

#define TAMANHO_BUFFER_HOUGHS 10

using namespace std;
using namespace cv;

namespace Houghs {
	vector<HoughLine> getHoughs(const Mat1b &mapa1, const Rect &roi, vector<HoughLine> &houghsAdjacent, const int maxNumLines = 200, int houghThresholdInicial = 20, int houghStep = 10, double houghMinLineLength = 5, double houghMaxLineGap = 5);
	int contaEvidencias(const vector<HoughLine> &houghs, const Mat1d &mapa, ConfigXML *config);
	bool validaHoughs(vector<HoughLine> &houghs, const KalmanState &state, ConfigXML *config);
	void validaEmptyHoughs(vector<HoughLine> &houghs, const KalmanState &state, ConfigXML *config);
	double diferencaAngulos(HoughLine &h1, HoughLine &h2);

	// buffer
	void getMediaDesvioBaseBuffer(deque<HoughDoMeio> &buffer, double &outMedia, double &outDesvio);
	bool validaHough(HoughLine &hough, deque<HoughLine> &buffer, ConfigXML *config);
	void posicaoBufferStatistics(deque<HoughLine> &buffer, double &outMedia, double &outDesvio, ConfigXML *config);
	void anguloBufferStatistics(deque<HoughLine> &buffer, double &outMedia, double &outDesvio, ConfigXML *config);
}

#endif // __HOUGHS_H
