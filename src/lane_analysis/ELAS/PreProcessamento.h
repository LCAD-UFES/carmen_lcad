#ifndef __PRE_PROCESSAMENTO_H
#define __PRE_PROCESSAMENTO_H

#include <opencv2/opencv.hpp>
#include "utils/IPM.h"

using namespace std;
using namespace cv;


class PreProcessamento {

	public:
		PreProcessamento(const Rect &_regiaoDeInteresse, IPM *_ipm, bool _verbose = false, bool _display = false);
		Mat1b executar(const Mat3b &colorFrame, Mat1b &outGrayFrameRoi, IPM *_ipm = nullptr);
		Mat1b executar(const Mat3b &colorFrame, IPM *_ipm = nullptr);
		double getTime() { return tempoExecutando; };
		vector<Point> getVerticesIPM();
		Mat1b getMascaraIPM(int _dilateSize = 0);

	private:
		void view(const Mat3b &entrada, const Mat1b &saida);
		Rect regiaoDeInteresse;
		IPM *ipm;
		double tempoExecutando;
		bool display;
		bool verbose;
};

#endif // __PRE_PROCESSAMENTO_H
