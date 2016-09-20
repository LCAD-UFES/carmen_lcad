#ifndef __SINALIZACAO_HORIZONTAL_H
#define __SINALIZACAO_HORIZONTAL_H

#include <opencv2/opencv.hpp>
#include "Kalman.h"
#include "utils/Helper.h"
#include "AnaliseDasHoughs.h"

using namespace std;
using namespace cv;

namespace SinalizacaoHorizontal {

	enum BlobTipo {
		NAO_SINALIZACAO = 0,
		SINALIZACAO = 1,
		FAIXA_DE_PEDESTRE = 2,
		OUTROS = 3
	};

	struct Blob {
		Mat1b binario;
		Mat1b gray;
		Rect regiao;
		BlobTipo tipo;
		vector<Point> pointsPerspectiva;
		vector<Point> pointsIPM;
		int templateId = -1;
	};
	
	struct TemplateSinalizacao {
		int id;
		Mat1b imagem;
		TemplateSinalizacao(int i, Mat1b img) : id(i), imagem(img) {}
	};

	static vector<TemplateSinalizacao> templates;
	Mat3b toIMG(int _id);
	string toText(int _id);

	void templateMatching(const Mat1b &inGrayRoiIPM, const Mat1b &mapaBinario, const KalmanState &kalman);
	void fastMatchTemplate(Mat &srca, Mat &srcb, Mat &dst, int maxlevel);
	vector<int> executar(const Mat1b &inGrayRoiIPM, const Mat1b &mapa2ipm, const Mat1b &mapa4ipm, const Mat3b &framePerspectiva, const HoughDoMeio * kalman_hough, ConfigXML *config, vector<Blob> &outBlobs, vector<viz_symbols> &symbols);
	vector<Blob> scan(const Mat1b &mapaBin, const Mat1b &mapaGray, const double thresholdAsfalto, const double tamanhoPercentual = 0.5);
	vector<Blob> trim(const vector<Blob> &blobs);
	int loadTemplates();
	TemplateSinalizacao matchTemplates(Mat1b &sample, const double threshold = 0.65, int flag = CV_TM_CCORR_NORMED);
	bool eFaixaDePedestre(const Blob &_blob);
	void setPoints(Blob &_blob, const vector<Point2d> &esqHough, const vector<Point2d> &dirHough, ConfigXML *config);
	void removerSinalizacao(Mat1b &mapa, vector<Blob> &blobs, int erodeSize = 5, int dilateSize = 20);

	// faixa de pedestre
	void detectarFaixaDePedestre(const Mat1b &mapa, const Rect &roi);
	Mat1d montarHistograma2D(const vector<HoughLine> &_houghs, const Mat1b &mapa, bool normalizar = false);
	int getAnguloDominante(const Mat1d &histograma2d, int gaussianBlurSize = 5);

	// segunda tentativa
	Mat1b detectarFaixaDePedestre2(const Mat1b &mapa, const Rect &roi, const Point &posicaoCarroIPM);
	vector<HoughLine> getHoughs(const Mat1b &mapa);
}

#endif // __SINALIZACAO_HORIZONTAL_H
