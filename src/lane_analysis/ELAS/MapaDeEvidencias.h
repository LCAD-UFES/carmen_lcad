#ifndef __MAPA_DE_EVIDENCIAS_H
#define __MAPA_DE_EVIDENCIAS_H

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

enum {
	TEMPORALBLUR_NONE = 0,			// sem temporal blur
	TEMPORALBLUR_BINARY = 1,		// aplicado na imagem binária (saída)
	TEMPORALBLUR_GRAYSCALED = 2		// aplicado na imagem em escala de cinza (entrada)
};

class MapaDeEvidencias {

	public:
		MapaDeEvidencias(const int _temporalBlur, bool _verbose = false, bool _display = false);
		Mat executar(const Mat inGrayFrameRoiIPM, const Mat &inMascaraIPM, deque<Mat> &bufferTemporalBlur, deque<Mat> &bufferMascara);
		Mat executar(const Mat inGrayFrameRoiIPM, const Mat &inMascaraIPM);
		void getMapas(Mat &outMapaEsquerda, Mat &outMapaDireita);
		double getTime() { return tempoExecutando; };

	private:
		void view(const Mat &entrada, const Mat &mapaBinario, const Mat &mascara, const Mat &mapaProbabilistico);
		int metodoTemporalBlur;
		int windowTemporalBlur = 3;
		int windowMascara = 10;
		double tempoExecutando;
		bool display = false;
		bool verbose = false;
		Mat mapaBinarioComMascara;

		Mat aplicarTemporalBlur(const Mat &imagem, deque<Mat>& bufferTemporalBlur);
		Mat thresholdDifferenceOfGaussians(const Mat &imagem);

		Mat gerarMascara(const Mat &mapaBinario, deque<Mat> &bufferMascara, int bufferSize, int morphSize);
		vector<Point> getContornoMaisProximo(const Mat &imagem, const Point &p);
		Mat aplicarMascara(const Mat &mapaBinario, const Mat &mascara);
		Mat transformarEmMapaProbabilistico(const Mat &mapaDeEvidenciasBinario);

		Mat translateImg(Mat &img, double offsetx, double offsety);
};

#endif // __MAPA_DE_EVIDENCIAS_H