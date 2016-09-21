#ifndef __FAIXAS_ADJACENTES_H
#define __FAIXAS_ADJACENTES_H

#include <opencv2/opencv.hpp>
#include "utils/Helper.h"
#include "utils/common.h"
#include "AnaliseDasHoughs.h"
#include "FiltroDeParticulasHough.h"

using namespace std;
using namespace cv;

vector<int> faixasAdjacentes(const Mat &mapa, const Mat1b &grayFrameRoiIPM, const Mat3b &colorFrameRoiIPM, const HoughDoMeio *hough, const ParticleHough &particle, const LMT &leftLMT, const LMT &rightLMT, const vector<int> &maxAdjacentes);
Point3d mediaDesvio3D(const Mat3b &img, double &media, double &desvio);
int contaEvidencias(const Mat1b &img);

vector<int> faixasAdjacentesHoughs(const Mat1b &mapa, const vector<HoughLine> &houghs, const vector<HoughLine> &houghs2, const HoughDoMeio *hough, const LMT &leftLMT, const LMT &rightLMT, ConfigXML *config);
int contarEvidencasEmbaixoDaHoughIPM(const HoughLine &hough, const Mat1b &mapaIPM, const int regiaoBusca);
Mat1d montarHistograma1D(const vector<HoughLine> &_houghs, const Mat1b &mapa, bool normalizar, ConfigXML *config, int pLeft, int pRight, int regiaoBusca, const double angulo);

#endif // __FAIXAS_ADJACENTES_H
