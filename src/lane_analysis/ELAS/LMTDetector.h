#ifndef __LMT_DETECTOR_H
#define __LMT_DETECTOR_H

#include <opencv2/opencv.hpp>
#include "Kalman.h"
#include "FiltroDeParticulasHough.h"
#include "utils/Helper.h"
#include "utils/common.h"

using namespace std;
using namespace cv;

// Lane Marking Type (LMT) Detector
class LMTDetector {
	public:
		LMTDetector(ConfigXML * _config);
		vector<LMT> executar(const Mat1b &mapa, const Mat3b &colorFrame, const Mat1b &grayFrameRoiIPM, double alturaConfiavel, const KalmanState &kalman);
		vector<LMT> executar(const Mat1b &mapa, const Mat3b &colorFrame, const Mat1b &grayFrameRoiIPM, double alturaConfiavel, const HoughDoMeio * laneBase);
		void resetBuffer();
		static Mat3b LMTtoIMG(LMT _lmt, ConfigXML * _config);

	private:
		ConfigXML * config;
		deque<LMT> bufferLMT[2];

		// cor
		double getPercentualAmarelo(const Mat1b &histogramaEvidencias, const Mat1b &histogramaAmarelo);
		LMT_COLOR getCor(const Mat1b &histogramaEvidencias, const Mat3b &colorFrame, vector<Point2d> spline, int tamanhoBusca, int alturaConfiavel);

		// continuidade e tipo
		LMT getTipoFaixa(int continuidade, int cor, int laneSide, int continuidadeExterna);
		LMT aplicaBuffer(const LMT &_lmt, int laneSide);
		LMT getLMT(const Mat1b &mapa, const Mat1b &grayFrameRoiIPM, const vector<Point2d> &lanePoints, int laneSide, const LMT_COLOR cor, int tamanhoBusca = 6);

		// display
		Mat1b montarHistograma(const Mat1b &mapa, vector<Point2d> &spline, int tamanhoBusca, int alturaConfiavel);
		Mat1b desenharHistograma(const Mat1b &histograma);
};

#endif // __LMT_DETECTOR_H
