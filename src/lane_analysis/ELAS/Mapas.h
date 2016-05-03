#ifndef __MAPAS_H
#define __MAPAS_H

#include <opencv2/opencv.hpp>
#include "utils/Helper.h"
#include "utils/HelperXML.h"

using namespace std;
using namespace cv;

// Mapa1: Filtro do Nieto
// Mapa2: Diferenca de Gaussianas
// Mapa3: Lane Markings pelo Desvio Padrão
// Mapa4: Mapa de "Obstáculos"
// Mapa5: Mapa de Lane Markings por Desvio Padrão

namespace Mapas
{
	// mapas
	Mat1b getMapa1(const Mat1b &inGrayPerspectiva, int tauInicio = 10, int tauFim = 100, int thres = 30);
	Mat1b getMapa2(const Mat1b &inGrayRoiIPM, const Mat &inMascaraIPM);
	Mat1b getMapa3(const Mat1b &inGrayRoiIPM, const Mat1b &mapa1ipm, int dilateSize = 5);
	Mat1b getMapa4(const Mat1b &inGrayRoiIPM, const Mat &inMascaraIPM);
	Mat1b getMapa5(const Mat1b &inGrayRoi, const Mat1b &mapa1, const ConfigXML *config);
	Mat1b iteracaoM6(const Mat1b &inGrayRoi, const Mat1b &mapa1);
	Mat1b getMapa6(const Mat1b &inGrayRoi, const Mat1b &mapa1, const ConfigXML *config);
	Mat1b getMapaResultante(const Mat1b &mapa1ipm, const Mat1b &mapa2ipm, const Mat1b &mapa5ipm);

	// outros
	void filtroNieto(Mat1b &srcGRAY, Mat1b &dstGRAY, int tauInicio = 10, int tauFim = 100);
	Mat diferencaDeGaussianas(const Mat &imagem, bool horizontal = false, double menor = 2.0, double maior = 2.2, double thres = 0.3);
}

#endif // __MAPAS_H