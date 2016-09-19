#include "MapaDeEvidencias.h"

using namespace std;
using namespace cv;


MapaDeEvidencias::MapaDeEvidencias(const int _temporalBlur, bool _verbose, bool _display) {
	metodoTemporalBlur = _temporalBlur;
	verbose = _verbose;
	display = _display;
}

Mat MapaDeEvidencias::executar(const Mat inGrayFrameRoiIPM, const Mat &inMascaraIPM) {
	// inicializa o contador de tempo
	double tempoInicio = static_cast<double>(getTickCount());

	Mat mapaBinario = thresholdDifferenceOfGaussians(inGrayFrameRoiIPM); // gera a DoG
	bitwise_and(mapaBinario, inMascaraIPM, mapaBinario); // remove o que está fora da IPM

	// calcula o tempo de execução
	double tempoFim = static_cast<double>(getTickCount());
	tempoExecutando = ((tempoFim - tempoInicio) / getTickFrequency()) * 1000;

	// exibe as saídas definidas (texto e/ou imagem)
	if (verbose) cout << "- mapa de evidencias DoG: " << tempoExecutando << " ms" << endl;

	return mapaBinario;
}

Mat MapaDeEvidencias::executar(const Mat inGrayFrameRoiIPM, const Mat &inMascaraIPM, deque<Mat> &bufferTemporalBlur, deque<Mat> &bufferMascara) {

	// inicializa o contador de tempo
	double tempoInicio = static_cast<double>(getTickCount());

	Mat mapaBinario;
	switch (metodoTemporalBlur) {
	case TEMPORALBLUR_BINARY:
		mapaBinario = thresholdDifferenceOfGaussians(inGrayFrameRoiIPM);
		medianBlur(mapaBinario, mapaBinario, 3); // pequenos ruídos
		mapaBinario = aplicarTemporalBlur(mapaBinario, bufferTemporalBlur);
		break;
	case TEMPORALBLUR_GRAYSCALED:
		mapaBinario = thresholdDifferenceOfGaussians(aplicarTemporalBlur(inGrayFrameRoiIPM, bufferTemporalBlur));
		break;
	case TEMPORALBLUR_NONE:
		mapaBinario = thresholdDifferenceOfGaussians(inGrayFrameRoiIPM);
		break;
	}

	// remove o que está fora da IPM
	bitwise_and(mapaBinario, inMascaraIPM, mapaBinario);

	Mat mascaraFinal = gerarMascara(mapaBinario, bufferMascara, 10, 1);
	this->mapaBinarioComMascara = aplicarMascara(mapaBinario, mascaraFinal);
	Mat mapaProbabilistico = transformarEmMapaProbabilistico(this->mapaBinarioComMascara);

	// calcula o tempo de execução
	double tempoFim = static_cast<double>(getTickCount());
	tempoExecutando = ((tempoFim - tempoInicio) / getTickFrequency()) * 1000;

	// exibe as saídas definidas (texto e/ou imagem)
	if (verbose) cout << "- mapa de evidencias: " << tempoExecutando << " ms" << endl;
	if (display) view(inGrayFrameRoiIPM, mapaBinario, mascaraFinal, mapaProbabilistico);

	return mapaProbabilistico;
}

Mat MapaDeEvidencias::aplicarTemporalBlur(const Mat &imagem, deque<Mat>& bufferTemporalBlur) {
	// adiciona a imagem no buffer
	if (bufferTemporalBlur.size() >= (unsigned int)windowTemporalBlur) {
		bufferTemporalBlur.pop_front();
	}
	bufferTemporalBlur.push_back(imagem);

	// cria um imagem para guardar a imagem acumulada
	Mat imagemAcumulada = Mat(bufferTemporalBlur.back().size(), uchar(0));
	for (Mat img : bufferTemporalBlur) {
		bitwise_or(imagemAcumulada, img, imagemAcumulada);
	}

	return imagemAcumulada;
}

Mat MapaDeEvidencias::thresholdDifferenceOfGaussians(const Mat &imagem) {

	Mat imagemResultado = Mat(imagem.size(), CV_8UC1);
	Mat imagem32F(imagem.rows, imagem.cols, CV_8UC1);

	// converte para 32-float para aplicar as gaussianas
	imagem.convertTo(imagem32F, CV_32F);

	// aplica 2 blurs gaussianos
	Mat gaussianaMenor, gaussianaMaior, DoG;
	double gaussiana_menor = 2;
	double gaussiana_maior = 2.2;
	GaussianBlur(imagem32F, gaussianaMenor, Size(0, 1), gaussiana_menor);
	GaussianBlur(imagem32F, gaussianaMaior, Size(0, 1), gaussiana_maior);

	// diferença de gaussianas
	DoG = gaussianaMenor - gaussianaMaior;

	// aplica um threshold
	double thres_value = 0.30;
	threshold(DoG, DoG, thres_value, 255, THRESH_BINARY);

	DoG.convertTo(imagemResultado, CV_8UC1);

	return imagemResultado;
}

Mat MapaDeEvidencias::gerarMascara(const Mat &mapaBinario, deque<Mat> &bufferMascara, int bufferSize, int morphSize) {

	// adiciona a imagem no buffer
	if (bufferMascara.size() >= bufferSize) bufferMascara.pop_front();
	bufferMascara.push_back(mapaBinario);

	// cria um imagem para guardar a imagem acumulada
	Mat imagemAcumulada = Mat(bufferMascara.back().size(), uchar(0));
	for (Mat img : bufferMascara) bitwise_or(imagemAcumulada, img, imagemAcumulada);

	// dilata a imagem
	Mat imagemDilatada;
	Mat elementoEstruturante = getStructuringElement(MORPH_ELLIPSE, Size(1 * morphSize + 1, 5 * morphSize + 1), Point(morphSize, morphSize));
	dilate(imagemAcumulada, imagemDilatada, elementoEstruturante, Point(-1, -1), 5);
	bitwise_not(imagemDilatada, imagemDilatada);

	// pega o contorno mais próximo do centro da imagem
	vector<Point> contorno = getContornoMaisProximo(imagemDilatada, Point(mapaBinario.cols / 2, mapaBinario.rows));

	// se nenhum contorno foi selecionado, retornar uma imagem em branco
	if (contorno[0] == Point(-1, -1)) return Mat::ones(imagemDilatada.size(), CV_8UC1);

	vector<vector<Point> > contornos;
	contornos.push_back(contorno);

	Mat mascara = Mat::zeros(mapaBinario.size(), CV_8UC1);
	drawContours(mascara, contornos, 0, 255, 60);

	return mascara;

}
vector<Point> MapaDeEvidencias::getContornoMaisProximo(const Mat &imagem, const Point &p) {

	Mat mascara = imagem.clone();
	vector<Point> contornoSelecionado;

	// encontra os contornos
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(mascara, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	// pega o contorno mais próximo
	int selectedContour = -1;
	vector<double> distances;
	for (unsigned int i = 0; i < contours.size(); i++) {
		distances.push_back(abs(pointPolygonTest(contours[i], p, true)));
	}
	double min_distance = numeric_limits<double>::max();
	int min_idx = -1;
	for (unsigned int i = 0; i < distances.size(); i++) {
		if (distances[i] < min_distance) {
			min_distance = distances[i];
			min_idx = i;
		}
	}
	if (min_idx == -1) contornoSelecionado = { Point(-1, -1) };
	else contornoSelecionado = contours[min_idx];

	return contornoSelecionado;
}

Mat MapaDeEvidencias::aplicarMascara(const Mat &mapaBinario, const Mat &mascara) {
	Mat mapaDeEvidencia;
	bitwise_and(mapaBinario, mascara, mapaDeEvidencia);
	if (countNonZero(mapaDeEvidencia) != 0) return mapaDeEvidencia;
	else return mapaBinario;
}

Mat MapaDeEvidencias::transformarEmMapaProbabilistico(const Mat &mapaDeEvidenciasBinario) {

	// converte para 32-float para aplicar as gaussianas
	Mat mapaDeEvidenciasBinario32F;
	mapaDeEvidenciasBinario.convertTo(mapaDeEvidenciasBinario32F, CV_32FC1);

	// aplicar os blurs
	Mat blur1 = Mat(mapaDeEvidenciasBinario.size(), CV_32FC1);
	Mat blur2 = Mat(mapaDeEvidenciasBinario.size(), CV_32FC1);
	GaussianBlur(mapaDeEvidenciasBinario32F, blur1, Size(101, 1), 0, 0);
	GaussianBlur(mapaDeEvidenciasBinario32F, blur2, Size(21, 1), 0, 0);
	mapaDeEvidenciasBinario32F = blur1 + blur2;

	// normaliza entre [0, 1]
	normalize(mapaDeEvidenciasBinario32F, mapaDeEvidenciasBinario32F, 0.0, 1.0, NORM_MINMAX, CV_32F);

	return mapaDeEvidenciasBinario32F;
}

void MapaDeEvidencias::getMapas(Mat &outMapaEsquerda, Mat &outMapaDireita) {

	// converte para 32-float para aplicar as gaussianas
	Mat mapaDeEvidenciasBinario32F;
	this->mapaBinarioComMascara.convertTo(mapaDeEvidenciasBinario32F, CV_32FC1);

	// DoG horizontal
	double sigmaGaussiana = 3; // TODO: calibrar
	double dist = 2 * sigmaGaussiana; // TODO: calibrar para faixa dupla
	Mat gaussianblur = Mat(this->mapaBinarioComMascara.size(), CV_32FC1);
	GaussianBlur(mapaDeEvidenciasBinario32F, gaussianblur, Size(0, 1), sigmaGaussiana);

	// Esquerda
	Mat shiftEsquerda(this->mapaBinarioComMascara.size(), CV_32FC1);
	shiftEsquerda = translateImg(gaussianblur, dist, 0);
	outMapaEsquerda = gaussianblur - shiftEsquerda;

	// Direita
	Mat shiftDireita(this->mapaBinarioComMascara.size(), CV_32FC1);
	shiftDireita = translateImg(gaussianblur, -dist, 0);
	outMapaDireita = gaussianblur - shiftDireita;

	normalize(outMapaEsquerda, outMapaEsquerda, 0.0, 1.0, NORM_MINMAX, CV_32F);
	normalize(outMapaDireita, outMapaDireita, 0.0, 1.0, NORM_MINMAX, CV_32F);

}

Mat MapaDeEvidencias::translateImg(Mat &img, double offsetx, double offsety) {
	Mat out;
	Mat trans_mat = (Mat_<double>(2, 3) << 1, 0, offsetx, 0, 1, offsety);
	warpAffine(img, out, trans_mat, img.size());
	return out;
}

void MapaDeEvidencias::view(const Mat &entrada, const Mat &mapaBinario, const Mat &mascara, const Mat &mapaProbabilistico) {

	// define as dimensões da imagem final e as áreas onde as imagens serão colocadas
	Mat imagemComposta = Mat(entrada.rows * 4, entrada.cols, CV_8UC3);
	Rect areaEntrada = Rect(0, 0, entrada.cols, entrada.rows);
	Rect areaMapaBinario = Rect(0, entrada.rows * 1, mapaBinario.cols, mapaBinario.rows);
	Rect areaMascara = Rect(0, entrada.rows * 2, mascara.cols, mascara.rows);
	Rect areaMapaProbabilistico = Rect(0, entrada.rows * 3, mapaProbabilistico.cols, mapaProbabilistico.rows);
	
	// copia as imagens para suas posições
	cvtColor(entrada, imagemComposta(areaEntrada), CV_GRAY2RGB);
	cvtColor(mapaBinario, imagemComposta(areaMapaBinario), CV_GRAY2RGB);
	cvtColor(mascara, imagemComposta(areaMascara), CV_GRAY2RGB);
	Mat mapaProbabilistico8UC3;
	normalize(mapaProbabilistico, mapaProbabilistico8UC3, 0, 255, NORM_MINMAX, CV_8UC3);
	applyColorMap(mapaProbabilistico8UC3, mapaProbabilistico8UC3, COLORMAP_JET);
	mapaProbabilistico8UC3.copyTo(imagemComposta(areaMapaProbabilistico));

	// mostra o resultado
	imshow("Mapa de Evidências", imagemComposta);

}