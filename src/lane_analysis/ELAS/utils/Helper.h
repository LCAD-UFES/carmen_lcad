#ifndef __HELPER_H
#define __HELPER_H

#include <opencv2/opencv.hpp>
#include <numeric>
#include <chrono>
#include "IPM.h"

using namespace std;
using namespace cv;

namespace Helper
{
	// convers�es de cor e tipo
	Mat1b converteParaEscalaDeCinza(const Mat3b &imagem);
	Mat1d from8Uto64F(const Mat1b &from);

	// transforma��es relacionadas a IPM
	Mat1b toROI(const Mat1b &imgIPM, IPM *ipm, int interpolacao = INTER_LINEAR); // IPM -> ROI
	Mat1b toIPM(const Mat1b &imgROI, IPM *ipm, int interpolacao = INTER_LINEAR); // ROI -> IPM
	Mat1b toIPM(const Mat1b &img, const Rect &roi, IPM *ipm, int interpolacao = INTER_LINEAR); // Imagem -> IPM
	Mat1b getROI(const Mat1b &imagem, const Rect &roi); // dada uma imagem, retorna a ROI
	Mat1b fillPerspectiva(const Mat1b &imgROI, const Rect &roi, const Size &tamanho, const uchar _default = 0); // pega uma ROI e coloca numa iamgem de tamanho normal com fundo da cor escolhida

	// histogramas
	void desenharHistograma(const Mat1b &histograma, Mat1b &img, Scalar cor = Scalar(255));
	void desenharHistograma(const Mat1d &histograma, Mat1b &img, Scalar cor = Scalar(255));
	void visualizarHistograma(const Mat1d &histograma, const string &windowTitle, int height = 150, bool singleColumn = false);
	void desenharHistograma(const vector<double> &histograma, Mat1b &img, double fator = 1, Scalar cor = Scalar(255));
	void desenharHistograma(const deque<double> &histograma, Mat1b &img, double fator = 1, Scalar cor = Scalar(255));

	// opera��es morfol�gicas
	Mat1b morphDilate(const Mat1b &from, int size);
	Mat1b morphErode(const Mat1b &from, int size);
	Mat1b skeleton(const Mat1b &from, const int size = 3);
	void thinningIteration(cv::Mat& img, int iter);
	void thinning(const cv::Mat& src, cv::Mat& dst);

	// rota��o
	void rotate2D(const Mat & src, Mat & dst, const float degrees, int flag = INTER_NEAREST);

	// contagem de areas brancas ou pretas numa linha
	int contarSegmentosBranco(const Mat1b &img, int minTamanho = -1, int maxTamanho = -1);
	int contarSegmentosPreto(const Mat1b &img, int minTamanho = -1, int maxTamanho = -1);

	// custom
	void filter2Dsoma(const Mat1d &input, const Mat1d &kernel, Mat1d &dst);
	vector<Point> mat2vector(const Mat1b &img);
	Point2d mediaPontos(const vector<Point> &pontos);
	void mediaDesvioDouble(const vector<double> &v, double &mean, double &stddev);
	double getAngulo(const Point &p1, const Point &p2);
	Rect validaRectCols(Rect &&_rect, int cols);

	// buffer
	template <class T>
	void pushBuffer(deque<T> &buffer, const T &item, const int _maxsize) {
		if (buffer.size() >= _maxsize) buffer.pop_back();
		buffer.push_front(item);
	}

	// from: http://stackoverflow.com/a/22349223/4228275
	// arredondamento para m�ltiplos (auxilia na gera��o de histogramas)
	template<typename T> T roundMultiple(T value, T multiple) {
		if (multiple == 0) return value;
		return static_cast<T>(std::round(static_cast<double>(value) / static_cast<double>(multiple))*static_cast<double>(multiple));
	}
}

struct TimeMeasurement {
	int64 _start, _end;
	void start() { _start = cv::getTickCount(); }
	void end() { _end = cv::getTickCount(); }
	double duration() { return ((double)(_end - _start) * 1000. / cv::getTickFrequency()); }
};

void displayTimePerformance(map<int, vector<double> > &all_times);
double getTotalTime(map<int, TimeMeasurement> &timer);

namespace Task {
	enum {
		ALL = 0,
		FEATURE_MAPS = 1,
		CROSSWALK = 2,
		ROAD_SIGNS = 3,
		SIGNS_REMOVAL = 4,
		CANDIDATES_GENERATION = 5,
		KALMAN = 6,
		PARTICLE_FILTER = 7,
		LMT = 8,
		ADJACENT_LANES = 9,
		LANE_CENTER_DEVIATION = 10
	};
	string getName(int _taskID);
};

#endif // __HELPER_H
