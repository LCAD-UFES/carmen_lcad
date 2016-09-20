#ifndef __ANALISE_DAS_HOUGHS_H
#define __ANALISE_DAS_HOUGHS_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "utils/common.h"
#include "utils/HelperXML.h"
#include "utils/Helper.h"

class HoughDoMeio;
class HoughLine;
class AnaliseDasHoughs;
struct lessThanPoint;

class HoughDoMeio {
	public:
		HoughDoMeio() { };
		HoughDoMeio(const cv::Mat1d &kalmanMeasurement);
		double xBase;
		double xTopo; // usado para calcular o angulo
		double largura;

		cv::Mat1d toKalman() const;
		std::vector<HoughLine> toVetorDeHoughs(const cv::Rect &roi, IPM *ipm = NULL) const;
		std::vector<cv::Point2d> getHoughPoints(int roiHeight, const int laneSide, bool ateMetade = false) const;

		// utilidades
		void print();
		void draw(cv::Mat3b &ipmImage, const cv::Scalar &cor, int thickness = 1) const;
		void draw(cv::Mat &image, const cv::Scalar &cor, ConfigXML *_config) const;
		
};

class HoughLine {
	public:
		HoughLine();

		Point2d _p1, _p2;	// iniciais
		Point2d p1, p2;		// extendidos

		double getAngulo() const;
		double getX(int y) const;
		double getY(int x) const;
		HoughLine toIPM(ConfigXML *config) const;
		HoughLine fromIPM(ConfigXML *config) const;
		void projetar(double largura, int direcao, ConfigXML *config, HoughLine &projetado);

		static HoughLine create(const cv::Point2d &p, double angulo);
		static HoughLine create(const cv::Point2d &p1, const Point2d &p2, double minY, double maxY);
		static HoughLine create(HoughDoMeio &hough, ConfigXML *_config);
		static std::vector<HoughLine> getVetorDeHoughs(std::vector<std::vector<std::vector<cv::Point> > > lineSegmentsClusters, const cv::Rect &roi);
		static std::vector<HoughLine> getVetorDeHoughs(std::vector<std::vector<std::vector<cv::Point> > > lineSegmentsClusters, int baseY, int topoY);

		inline bool operator<(const HoughLine &h1) { return (this->getAngulo() > h1.getAngulo()); };
		
		bool isEmpty() const;
		void print() const;
		void draw(cv::Mat &img, const cv::Scalar &cor, int thickness = 2);
		static HoughLine empty() { HoughLine empty; return empty; };
		static HoughDoMeio getKalmanMeasurement(const HoughLine &h1, const HoughLine &h2, ConfigXML *config, cv::Mat img = Mat());
		double calculaDistancia(const HoughLine &baseline);
};

class AnaliseDasHoughs {
	public:
		AnaliseDasHoughs(ConfigXML *_config);
		void setColorFrame(const cv::Mat3b &_colorFrame);
		void executar(const cv::Mat1b &mapa1ipm, const std::vector<HoughLine> &houghs, std::vector<HoughLine> &houghsAnteriores, const cv::Mat1b &mapaIPM);
		void executar2D(const cv::Mat1b &mapa1ipm, const std::vector<HoughLine> &houghs, HoughDoMeio *houghsAnteriores, const cv::Mat1b &mapaIPM, std::vector<HoughLine> &outHoughs);
		static double mostrarDesvioDoCentro(cv::Mat &displayColorFrame, const HoughDoMeio &measurement, double xCarro);
		cv::Mat3b colorFrame;
	private:
		double larguraPista;
		cv::Scalar esqCor = cv::Scalar(255, 0, 255);
		cv::Scalar dirCor = cv::Scalar(255, 255, 0);
		
		cv::Mat1d montarHistograma(const std::vector<HoughLine> &_houghs, int _tamanho, int _tamanho_bin, bool normalizar);
		cv::Mat1d montarHistograma(const std::vector<HoughLine> &_houghs, const cv::Mat1b &mapa, int _tamanho, int _tamanho_bin, bool normalizar); // esse leva em considera��o as evid�ncias em baixo da hough
		int contarEvidencasEmbaixoDaHoughIPM(const HoughLine &hough, const cv::Mat1b &mapaIPM, const int regiaoBusca = 0);
		void desenhaHistograma(const cv::Mat1d &histograma, cv::Mat3b &imagem, const cv::Scalar &cor = cv::Scalar(255, 255, 255));
		void encontraMaximoNosHistogramas(const cv::Mat1d &esqHistograma, const cv::Mat1d &dirHistograma, int *esqHistogramaMaxX, int *dirHistogramaMaxX);
		void selecionarHoughs(const std::vector<HoughLine> &houghs, int esqHistogramaMaxX, int dirHistogramaMaxX, int houghArea, std::vector<HoughLine> *esq, std::vector<HoughLine> *dir);
		void selecionarHoughs(const std::vector<HoughLine> &houghs, std::vector<HoughLine> *esq, std::vector<HoughLine> *dir);
		void processarHistogramaParcial(cv::Mat1d &histograma, int laneSide, const double epsilon = 1.e-6);
		void selecionarHoughsDouble(const std::vector<HoughLine> &houghs, double _x, int houghArea, std::vector<HoughLine> *out);
		int encontrarMaximoMaisProximo(const cv::Mat1d &histograma, int direcao, int referencia, bool estaNormalizado);
		void pegarMedianaNaRegiao(const std::vector<HoughLine> &houghs, int posicaoBase, int tamanhoRegiao, HoughLine &outHough);

		cv::Mat1d montarHistograma2D(const std::vector<HoughLine> &_houghs, const Mat1b &mapa, map<Point, std::vector<HoughLine>, lessThanPoint> &outHoughsHistograma2D, bool normalizar = true);
		cv::Mat1d montarHistograma1D(const cv::Mat1d &histograma2d, map<Point, std::vector<HoughLine>, lessThanPoint> &houghsHistograma2D, HoughDoMeio *houghsAnteriores, map<int, std::vector<HoughLine>> &outHoughsHistograma1D, int laneSide, bool normalizar = true);
		int getAnguloDominante(const cv::Mat1d &histograma2d, int gaussianBlurSize = 5);
		void mascaraHistogramas(int anguloDominante, int regiao, cv::Mat1d &esqHistograma2d, cv::Mat1d &dirHistograma2d);
		cv::Mat1d combinaHistogramas2D(const cv::Mat1d &esqHistograma, const cv::Mat1d &dirHistograma);
		void processarHistogramaParcialLocal(cv::Mat1d &histograma, int laneSide, const double epsilon = 1.e-6);

		cv::Point posicaoCarro;
		cv::Point posicaoCarroIPM;
		ConfigXML * config;

		
};

#endif // __ANALISE_DAS_HOUGHS_H
