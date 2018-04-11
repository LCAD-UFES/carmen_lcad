#include "Mapas.h"

using namespace std;
using namespace cv;

Mat1b Mapas::getMapa1(const Mat1b &inGrayPerspectiva, int tauInicio, int tauFim, int thres) {

	Mat1b mapa1 = inGrayPerspectiva.clone();

	// aplica o filtro do Nieto
	filtroNieto(mapa1, mapa1, tauInicio, tauFim);

	// aplica o threshold na imagem filtrada
	threshold(mapa1, mapa1, thres, 255, CV_THRESH_BINARY);

	return mapa1;
}
Mat1b Mapas::getMapa2(const Mat1b &inGrayRoiIPM, const Mat &inMascaraIPM) {
	// faz a diferença de gaussianas na imagem
	Mat mapa2ipm = diferencaDeGaussianas(inGrayRoiIPM);

	// remove as bordas indesejadas da IPM
	bitwise_and(mapa2ipm, inMascaraIPM, mapa2ipm);

	return mapa2ipm;
}
Mat1b Mapas::getMapa3(const Mat1b &inGrayRoiIPM, const Mat1b &mapa1ipm, int dilateSize) {
	Mat1b mapa3, mapa1ipmDilatado;

	// imagens a serem constrúidas
	Size imgSize = inGrayRoiIPM.size();
	Mat1b binaryPavement = Mat1b(imgSize, uchar(0));
	Mat1b grayMarkings = Mat1b(imgSize, uchar(0));
	Mat1b binaryMarkings = Mat1b(imgSize, uchar(0));

	// valores iniciais das gaussianas
	Scalar meanPavement, stddevPavement;

	// somente o que é imagem, na IPM, ficará branco
	Mat1b binaryIpmInvMask = Mat1b(imgSize);
	cv::bitwise_not(inGrayRoiIPM == 0, binaryIpmInvMask);

	// aplica o dilate no mapa 1 IPM
	Mat1b dilateKernel = getStructuringElement(MORPH_ELLIPSE, Size(1 * dilateSize + 1, 1 * dilateSize + 1), Point(dilateSize, dilateSize));
	dilate(mapa1ipm, mapa1ipmDilatado, dilateKernel, Point(-1, -1), 3);

	// ************************************** PAVEMENT
	// calcula as máscaras
	cv::bitwise_not(mapa1ipm, binaryPavement);
	cv::bitwise_and(binaryIpmInvMask, binaryPavement, binaryPavement);
	// calcula a média e covariância do pavimento
	cv::meanStdDev(inGrayRoiIPM, meanPavement, stddevPavement, binaryPavement);

	// ************************************** LANE MARKINGS
	inGrayRoiIPM.copyTo(grayMarkings);
	double thresMarkings = meanPavement[0] + 3 * stddevPavement[0];
	mapa3 = grayMarkings > thresMarkings;

	return mapa3;
}
Mat1b Mapas::getMapa4(const Mat1b &inGrayRoiIPM, const Mat &inMascaraIPM) {
	
	Mat mapa4ipm, mapa4ipmNegativo, mapa4ipmPositivo;

	// negativo => escuro para claro
	Sobel(inGrayRoiIPM, mapa4ipmNegativo, CV_32F, 0, 1);
	mapa4ipmPositivo = mapa4ipmNegativo.clone();
	mapa4ipmNegativo = -1 * mapa4ipmNegativo;
	threshold(mapa4ipmNegativo, mapa4ipmNegativo, 30, 255, CV_8U);
	mapa4ipmNegativo.convertTo(mapa4ipmNegativo, CV_8U);

	// positivo => claro para escuro
	threshold(mapa4ipmPositivo, mapa4ipmPositivo, 30, 255, CV_8U);
	mapa4ipmPositivo.convertTo(mapa4ipmPositivo, CV_8U);

	// junta os dois mapas
	bitwise_or(mapa4ipmNegativo, mapa4ipmPositivo, mapa4ipm);

	// remove as bordas indesejadas da IPM
	bitwise_and(mapa4ipm, inMascaraIPM, mapa4ipm);

	return mapa4ipm;
}
Mat1b Mapas::getMapa5(const Mat1b &inGrayRoi, const Mat1b &mapa1, const ConfigXML *config) {
	// mapa binário do mapa 1
	Mat1b mapaROI8uc;
	(mapa1(config->roi)).convertTo(mapaROI8uc, CV_8UC1, 255.0);
	threshold(mapaROI8uc, mapaROI8uc, 10, 255, CV_8UC1); // passa para binário

	// inversa do resultante
	Mat mapaROIInversoResultante;
	bitwise_not(mapaROI8uc, mapaROIInversoResultante);

	// média e desvio padrão dos que estão embaixo da inversa do resultante ("asfalto")
	mapaROIInversoResultante = Helper::morphErode(mapaROIInversoResultante, 3);
	Scalar meanFiltro, stddevFiltro;
	cv::meanStdDev(inGrayRoi, meanFiltro, stddevFiltro, mapaROIInversoResultante);
	Mat markings_nao_asfalto = inGrayRoi > meanFiltro[0] + 2 * stddevFiltro[0]; // mascara com os pixels que estao abaixo da threshold do asfalto

	// subset dos pixels que devem ser usados para calcular
	Mat escolhidos;
	bitwise_and(mapaROI8uc, markings_nao_asfalto, escolhidos);
	escolhidos = Helper::morphErode(escolhidos, 3);

	// media e desvio padrao dos pixels escolhidos
	cv::meanStdDev(inGrayRoi, meanFiltro, stddevFiltro, escolhidos);

	Mat mapa5 = inGrayRoi > meanFiltro[0] - stddevFiltro[0];
	Mat mapa5ipm = Helper::toIPM(mapa5, config->ipm, INTER_NEAREST);

	return mapa5ipm;
}
Mat1b Mapas::iteracaoM6(const Mat1b &inGrayRoi, const Mat1b &mapa1roi) {
	// inversa do resultante
	Mat mapaROIInversoResultante;
	bitwise_not(mapa1roi, mapaROIInversoResultante);

	// média e desvio padrão dos que estão embaixo da inversa do resultante ("asfalto")
	// TODO: erode
	Scalar meanFiltro, stddevFiltro;
	cv::meanStdDev(inGrayRoi, meanFiltro, stddevFiltro, mapaROIInversoResultante);
	Mat markings_nao_asfalto = inGrayRoi > meanFiltro[0] + 2 * stddevFiltro[0]; // mascara com os pixels que estao abaixo da threshold do asfalto

	// subset dos pixels que devem ser usados para calcular
	Mat escolhidos;
	bitwise_and(mapa1roi, markings_nao_asfalto, escolhidos);
	// TODO: erode

	// media e desvio padrao dos pixels escolhidos
	cv::meanStdDev(inGrayRoi, meanFiltro, stddevFiltro, escolhidos);

	Mat output = inGrayRoi > meanFiltro[0] - stddevFiltro[0];
	return output;
}
Mat1b Mapas::getMapa6(const Mat1b &inGrayRoi, const Mat1b &mapa1, const ConfigXML *config) {
	// mapa binário do mapa 1
	Mat1b mapa1roi;
	(mapa1(config->roi)).convertTo(mapa1roi, CV_8UC1, 255.0);
	threshold(mapa1roi, mapa1roi, 10, 255, CV_8UC1); // passa para binário
	
	Mat mapa6;
	mapa6 = iteracaoM6(inGrayRoi, mapa1roi);
	bitwise_and(mapa6, mapa1roi, mapa6);
	mapa6 = iteracaoM6(inGrayRoi, mapa6);
	bitwise_and(mapa6, mapa1roi, mapa6);
	mapa6 = iteracaoM6(inGrayRoi, mapa6);
	bitwise_and(mapa6, mapa1roi, mapa6);

	Mat mapa6ipm = Helper::toIPM(mapa6, config->ipm, INTER_NEAREST);
	imshow("mapa6ipm", mapa6ipm);

	return mapa6ipm;
}
Mat1b Mapas::getMapaResultante(const Mat1b &mapa1, const Mat1b &mapa2, const Mat1b &mapa5) {

	Mat1d mapaResultante;
	Mat1d m1 = mapa1.clone();
	Mat1d m2 = mapa2.clone();
	Mat1d m5 = mapa5.clone();

	bitwise_and(m1, m2, mapaResultante);
	bitwise_or(m2, mapaResultante, mapaResultante);
	bitwise_and(m5, mapaResultante, mapaResultante);

	return mapaResultante;
}

void Mapas::filtroNieto(Mat1b &srcGRAY, Mat1b &dstGRAY, int tauInicio, int tauFim) {
	Mat1b tempDst = Mat1b(srcGRAY.size(), 0);

	int aux = 0;
	double alturaInicioVariacao = (double)srcGRAY.rows / 2;
	double tauTaxaVariacao = double(tauFim - tauInicio) / alturaInicioVariacao;
	int tau = tauInicio;
	for (int j = 0; j < srcGRAY.rows; ++j) {
		unsigned char *ptRowSrc = srcGRAY.ptr<uchar>(j);
		unsigned char *ptRowDst = tempDst.ptr<uchar>(j);
		if (j > alturaInicioVariacao) tau = int(tauInicio + tauTaxaVariacao * (j - alturaInicioVariacao));
		for (int i = tau; i < srcGRAY.cols - tau; ++i) {

			unsigned char aux2 = ptRowSrc[i];

			if (ptRowSrc[i] != 0) {
				aux = 2 * ptRowSrc[i];
				aux += -ptRowSrc[i - tau];
				aux += -ptRowSrc[i + tau];
				aux += -abs((int)(ptRowSrc[i - tau] - ptRowSrc[i + tau]));

				aux = (aux < 0) ? 0 : aux;
				aux = (aux > 255) ? 255 : aux;

				ptRowDst[i] = (unsigned char)aux;
			}
		}
	}
	dstGRAY = tempDst.clone();
}
Mat Mapas::diferencaDeGaussianas(const Mat &imagem, bool horizontal, double menor, double maior, double thres) {

	Mat imagemResultado = Mat(imagem.size(), CV_8UC1);
	Mat imagem32F(imagem.rows, imagem.cols, CV_8UC1);

	// converte para 32-float para aplicar as gaussianas
	imagem.convertTo(imagem32F, CV_32F);

	// aplica 2 blurs gaussianos
	Mat gaussianaMenor, gaussianaMaior, DoG;
	if (!horizontal) {
		GaussianBlur(imagem32F, gaussianaMenor, Size(0, 1), menor);
		GaussianBlur(imagem32F, gaussianaMaior, Size(0, 1), maior);
	} else {
		GaussianBlur(imagem32F, gaussianaMenor, Size(1, 0), menor);
		GaussianBlur(imagem32F, gaussianaMaior, Size(1, 0), maior);
	}

	// diferença de gaussianas
	DoG = gaussianaMenor - gaussianaMaior;

	// aplica um threshold
	threshold(DoG, DoG, thres, 255, THRESH_BINARY);

	DoG.convertTo(imagemResultado, CV_8UC1);

	return imagemResultado;
}