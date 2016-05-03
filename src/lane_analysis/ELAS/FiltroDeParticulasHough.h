#ifndef __FILTRO_DE_PARTICULAS_HOUGH_H
#define __FILTRO_DE_PARTICULAS_HOUGH_H

#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <ctime> // time
#include <algorithm> // std::sort
#include <numeric> // std::accumulate
#include <iostream>

#include "AnaliseDasHoughs.h"
#include "SinalizacaoHorizontal.h"
#include "utils/common.h"
#include "utils/spline.h"


using namespace std;
using namespace cv;

// carmen implementation
double randomGaussian(double mean, double std);
double randomUniform(double min, double max);

class ParticleHough {
	public:
		vector<double> points; // 2 pontos superiores da spline
		double laneWidth; // largura superior
		double weight;

		inline bool operator<(const ParticleHough &p1) const;
		
		ParticleHough(const ParticleHough &p);
		ParticleHough(double posicaoCarroIPM = 0, double larguraInicial = 0);
		~ParticleHough();

		vector<Point2d> getSplinePoints(const HoughDoMeio &hough, int roiHeight, const int laneSide, const double factor = 1.0) const;
		static ParticleHough getRandom(double xBase, double xMean, double xStd, double larguraBase);
		tk::spline getSplineFunc(const HoughDoMeio &hough, int roiHeight, const int laneSide);
		Point2d getPoint(int _y, const HoughDoMeio &hough, ConfigXML * _config, int laneSide, bool onPerspective = false);
};

class ParticleFilterHough {
	private:
		vector<ParticleHough> particles;
		ParticleHough par[500];
		bool verbose, display;
		bool disabled = false;
		double desvioPadraoPoints = 5;			// => 15.0 / 3
		double desvioPadraoLargura = 1.6666;	// => 05.0 / 3

		vector<double> calculaErros(const HoughDoMeio &hough, const Mat &mapaProbabilistico, const double confidenceHeight);
		vector<double> calculaErrosConfianca(const HoughDoMeio &hough, const Mat &mapaProbabilistico, const double confidenceHeight);
		void setRandom(int n, double posicaoCarroIPM, double larguraInicial);
		void predict(const HoughDoMeio &hough);
		void correct(vector<double> erros);
		void normalizaPesos();
		void resample();
		
	public:
		int alturaConfiavel;
		ParticleFilterHough(int n, double posicaoCarroIPM, double larguraInicial, int alturaConfiavelInicial, bool _verbose, bool _display);
		ParticleHough getBestParticle();
		vector<ParticleHough> getBestParticles(int number);
		void getParticulasMaiorMenorPeso(ParticleHough &maiorPeso, ParticleHough &menorPeso);
		vector<ParticleHough> getParticles();
		int atualizaAlturaConfiavel(ParticleHough *ultimaVirtualBest, const HoughDoMeio &hough, const Mat1b &mapaBinario, const vector<SinalizacaoHorizontal::Blob> &roadSigns);
		int atualizaAlturaConfiavel(const HoughDoMeio &hough, const Mat &mapa);
		void reset(const HoughDoMeio &hough, bool parcial);

		ParticleHough executar(const HoughDoMeio &hough, const Mat &mapaProbabilistico);
		void view(const HoughDoMeio *hough, const Mat &imagemPerspectiva, Mat &displayFrameRoiIPM, ConfigXML *_config, bool onlyPoints = false, int alturaConfiavel = -1, const vector<Scalar> &cores = { Scalar(255, 0, 0), Scalar(0, 255, 0), Scalar(0, 0, 255) });
};

void mostrarParticulas(const vector<ParticleHough> &particulas, const HoughDoMeio &hough, int alturaConfiavel, Mat &imagem, const cv::Scalar cor = cv::Scalar(255, 0, 0), const bool intercalar = true);

#endif // __FILTRO_DE_PARTICULAS_HOUGH_H
