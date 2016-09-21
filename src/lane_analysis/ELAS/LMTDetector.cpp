#include "LMTDetector.h"

using namespace std;
using namespace cv;

LMTDetector::LMTDetector(ConfigXML * _config) {
	config = _config;
}

vector<LMT> LMTDetector::executar(const Mat1b &mapa, const Mat3b &colorFrame, const Mat1b &grayFrameRoiIPM, double alturaConfiavel, const KalmanState &kalman) {
	return executar(mapa, colorFrame, grayFrameRoiIPM, alturaConfiavel, kalman.hough);
}

vector<LMT> LMTDetector::executar(const Mat1b &mapa, const Mat3b &colorFrame, const Mat1b &grayFrameRoiIPM, double alturaConfiavel, const HoughDoMeio * laneBase) {
	Mat3b displayImg = colorFrame.clone();
	LMT esqLMT = LMT::NONE;
	LMT dirLMT = LMT::NONE;
	
	// 1. pegar uma regiao (perpendicular?) em volta de cada pixel do kalman (ate a metade da IPM == altura fixa do kalman)
	if (laneBase != NULL) {

		const int tamanhoBusca = 4;
		const int alturaRoi = config->roi.height;
		const Mat1b *mapaFaixas = &mapa;
		
		// 1. Usando a spline
		// vector<Point2d> esqBestVirtual = bestParticle.getSplinePoints(*(kalman.hough), alturaRoi, LANE_LEFT);
		// vector<Point2d> dirBestVirtual = bestParticle.getSplinePoints(*(kalman.hough), alturaRoi, LANE_RIGHT);
		// 2. Usando a hough
		vector<Point2d> esqBestVirtual = laneBase->getHoughPoints(alturaRoi, LANE_LEFT);
		vector<Point2d> dirBestVirtual = laneBase->getHoughPoints(alturaRoi, LANE_RIGHT);

		// monta os histogramas internos
		Mat1b esqHistFaixa = montarHistograma(mapa, esqBestVirtual, tamanhoBusca, (int)alturaConfiavel);
		Mat1b dirHistFaixa = montarHistograma(mapa, dirBestVirtual, tamanhoBusca, (int)alturaConfiavel);

		// faixa da esquerda
		LMT_COLOR esqCor = getCor(esqHistFaixa, colorFrame, esqBestVirtual, tamanhoBusca, (int)alturaConfiavel);
		esqLMT = getLMT(mapa, grayFrameRoiIPM, esqBestVirtual, LANE_LEFT, esqCor, tamanhoBusca * 2);
		esqLMT = aplicaBuffer(esqLMT, LANE_LEFT);

		// faixa da direita
		LMT_COLOR dirCor = getCor(dirHistFaixa, colorFrame, dirBestVirtual, tamanhoBusca, (int)alturaConfiavel);
		dirLMT = getLMT(mapa, grayFrameRoiIPM, dirBestVirtual, LANE_RIGHT, dirCor, tamanhoBusca * 2);
		dirLMT = aplicaBuffer(dirLMT, LANE_RIGHT);

		if (config->display) {
			Mat3b esqTipoFaixa = LMTtoIMG(esqLMT, config);
			Mat3b dirTipoFaixa = LMTtoIMG(dirLMT, config);

			const int padding = 15;
			Rect esqRoi = Rect(padding, padding, esqTipoFaixa.cols, esqTipoFaixa.rows);
			Rect dirRoi = Rect(colorFrame.cols - padding - dirTipoFaixa.cols, padding, dirTipoFaixa.cols, dirTipoFaixa.rows);

			esqTipoFaixa.copyTo(displayImg(esqRoi));
			dirTipoFaixa.copyTo(displayImg(dirRoi));
		}
	}
	if (config->display) imshow("Tipo de Faixa", displayImg);

	return { esqLMT, dirLMT };
}

void LMTDetector::resetBuffer() {
	bufferLMT[0].clear();
	bufferLMT[1].clear();
}

// cor
LMT_COLOR LMTDetector::getCor(const Mat1b &histogramaEvidencias, const Mat3b &colorFrame, vector<Point2d> spline, int tamanhoBusca, int alturaConfiavel) {
	Mat3b hsv;
	Mat1b amarelado;
	cvtColor(colorFrame, hsv, CV_BGR2HSV);
	//inRange(hsv, Scalar(15, 80, 80), Scalar(25, 200, 200), amarelado);
	inRange(hsv, Scalar(10, 0, 0), Scalar(25, 255, 255), amarelado);
	Mat1b mascaraAmareloIPM = Helper::toIPM(Helper::getROI(amarelado, config->roi), config->ipm, INTER_CUBIC);

	// histograma de amarelos
	Mat1b esqAmareloHist = montarHistograma(mascaraAmareloIPM, spline, tamanhoBusca, (int)alturaConfiavel);

	// percentual de amarelos
	double esqPercentualAmarelo = getPercentualAmarelo(histogramaEvidencias, esqAmareloHist);

	// decisao da cor
	const double thresCor = 0.3;
	return (esqPercentualAmarelo > thresCor) ? LMT_COLOR::YELLOW : LMT_COLOR::WHITE;
}
double LMTDetector::getPercentualAmarelo(const Mat1b &histogramaEvidencias, const Mat1b &histogramaAmarelo) {

	int nEvidencias = 0;
	int nAmareloEEvidencias = 0;

	for (int i = 0; i < histogramaEvidencias.cols; i++) {
		if (histogramaEvidencias.at<uchar>(i) == 255) {
			nEvidencias++;
			if (histogramaAmarelo.at<uchar>(i) == 255) {
				nAmareloEEvidencias++;
			}
		}
	}

	if (nEvidencias == 0) return 0;
	return (double)nAmareloEEvidencias / (double)nEvidencias;
}

// continuidade e tipo
LMT LMTDetector::aplicaBuffer(const LMT &_lmt, int laneSide) {
	
	// adiciona o elemento
	const int maxSize = 30; // tamanho m�ximo do buffer
	if (bufferLMT[laneSide].size() >= maxSize) bufferLMT[laneSide].pop_front(); // caso o buffer ja tenho o numero maximo de elementos, retire o primeiro
	bufferLMT[laneSide].push_back(_lmt); // adicione o elemento no final

	// pega o elemento mais constante
	int contador[8];
	contador[0] = 0; // NONE
	contador[1] = 0; // SCB
	contador[2] = 0; // SSB
	contador[3] = 0; // SCA
	contador[4] = 0; // SSA
	contador[5] = 0; // DCA
	contador[6] = 0; // DSC
	contador[7] = 0; // DCS
	for (unsigned int i = 0; i < bufferLMT[laneSide].size(); i++) {
		contador[bufferLMT[laneSide][i]]++;
	}
	int max = 0;
	int maxIdx = -1;
	for (int i = 0; i < 8; i++) {
		if (contador[i] > max) {
			max = contador[i];
			maxIdx = i;
		}
	}
	
	LMT saida;
	switch (maxIdx) {
		case 0: saida = LMT::NONE; break;
		case 1: saida = LMT::SCB; break;
		case 2: saida = LMT::SSB; break;
		case 3: saida = LMT::SCA; break;
		case 4: saida = LMT::SSA; break;
		case 5: saida = LMT::DCA; break;
		case 6: saida = LMT::DSC; break;
		case 7: saida = LMT::DCS; break;
	}

	return saida;
}
LMT LMTDetector::getTipoFaixa(int continuidade, int cor, int laneSide, int continuidadeExterna) {
	if (continuidadeExterna == -1) { // faixa simples
		if (continuidade == 0) { // continua
			if (cor == 0) { // branca
				return LMT::SCB;
			} else { // amarela
				return LMT::SCA;
			}
		} else { // seccionada
			if (cor == 0) { // branca
				return LMT::SSB;
			} else { // amarela
				return LMT::SSA;
			}
		}
	} else { // faixa dupla
		if (continuidade == 0 && continuidadeExterna == 0) {
			return LMT::DCA;
		} else {
			if (laneSide == LANE_LEFT) { // esquerda
				if (continuidade == 0 && continuidadeExterna == 1) return LMT::DSC;
				else return LMT::DCS;
			} else { // direita
				if (continuidade == 0 && continuidadeExterna == 1) return LMT::DCS;
				else return LMT::DSC;
			}
		}
	}
}
LMT LMTDetector::getLMT(const Mat1b &mapa, const Mat1b &grayFrameRoiIPM, const vector<Point2d> &lanePoints, int laneSide, const LMT_COLOR cor, int tamanhoBusca) {

	const int tamanhoBuscaAmbas = tamanhoBusca * 2;
	vector<Point2d> spline = lanePoints;
	const int factor = 1; // 3
	int inicioAmbas = -1 * factor * tamanhoBuscaAmbas;
	int fimAmbas = factor * tamanhoBuscaAmbas;
	Size imgSize = Size(2 * factor * tamanhoBuscaAmbas, (int)spline.size());

	Mat1b imgAmbas = Mat1b(imgSize, uchar(0));
	Mat1b evidencias = Mat1b(imgSize, uchar(0));

	// mapa do ou
	Mat1b histOu = Mat1b(Size((int)spline.size(), 1), uchar(0));
	for (int p = 0; p < spline.size(); p++) { // para cada ponto da spline
		for (int i = inicioAmbas; i < fimAmbas; i++) { // para cada pixel do espa�o de busca
			if (mapa.at<uchar>((int)spline[p].y, (int)spline[p].x + i) > 0) {
				histOu.at<uchar>(p) = 255;
				break;
			}
		}
	}
	medianBlur(histOu, histOu, 5); // remove os pequenos ruidos

	// desenha o histograma do OU
	// Mat1b imgHistOu = Mat1b(Size(histOu.cols, histOu.cols), uchar(0));
	// Helper::desenharHistograma(histOu, imgHistOu);
	// imshow("Histograma OU por linha", imgHistOu);

	// monta o histograma de evidencias dos pontos
	for (int p = 0; p < spline.size(); p++) { // para cada ponto da spline
		for (int i = inicioAmbas; i < fimAmbas; i++) { // para cada pixel do espa�o de busca
			imgAmbas.at<uchar>(Point(i + factor * tamanhoBuscaAmbas, p)) = grayFrameRoiIPM.at<uchar>((int)spline[p].y, (int)spline[p].x + i);
		}
		normalize(imgAmbas, imgAmbas, 0.0, 255.0, NORM_MINMAX);
		// calcula a media e o desvio padrao da linha e aplica um threshold
		Scalar media, desvio;
		Rect area = Rect(0, p, imgAmbas.cols, 1);
		meanStdDev(imgAmbas(area), media, desvio);
		Mat1b markings = (imgAmbas(area) > (media[0] + desvio[0]));

		if (histOu.at<uchar>(p) == 255) markings.copyTo(evidencias(area));
	}
	medianBlur(evidencias, evidencias, 3); // remove os pequenos ru�dos

	// 1. verificar % de branco-preto-branco nas 'evidencias'
	Mat1b histBPB = Mat1b(Size(evidencias.rows, 1), uchar(0));
	Mat1i pontosBrancoBPB = Mat1i(Size(2, evidencias.rows), int(-1));
	const int minTamanhoPreto = 2; // branco-preto(x pixels)-branco
	int qtdEsq = 0, qtdDir = 0;

	for (int j = 0; j < evidencias.rows; j++) {

		bool naSequencia = false;
		int nSequencias = 0;
		int tamanhoSequencia = 0;

		int primeiroBrancoIdx = -1; // assume que come�a com preto
		int ultimoBrancoIdx = -1;

		for (int i = 0; i < evidencias.cols; i++) {
			int pixel = evidencias.at<uchar>(j, i);
			// inicio e termino de sequencias
			if (pixel == 0 && primeiroBrancoIdx != -1) { // inicio e continuidade
				naSequencia = true;
				tamanhoSequencia++;
			} else if (pixel == 255 && naSequencia) { // termino
				naSequencia = false;
				if (tamanhoSequencia > minTamanhoPreto) {
					ultimoBrancoIdx = i;
					histBPB.at<uchar>(j) = 255;
					pontosBrancoBPB.at<int>(Point(0, j)) = primeiroBrancoIdx;
					pontosBrancoBPB.at<int>(Point(1, j)) = ultimoBrancoIdx;
					break; // encontrou uma sequ�ncia
				} else {
					primeiroBrancoIdx = i;
					tamanhoSequencia = 0;
				}
			} else {
				if (pixel == 255) primeiroBrancoIdx = i;
				naSequencia = false;
				tamanhoSequencia = 0;
			}
		}

		// se nao encontrou um BPB, verifica se esta mais a esquerda ou a direita do BPB anterior
		if ((pontosBrancoBPB.at<int>(Point(0, j)) == -1 && pontosBrancoBPB.at<int>(Point(1, j)) == -1) // se o atual e linha simples
			&& j != 0 && pontosBrancoBPB.at<int>(Point(0, j - 1)) != -1 && pontosBrancoBPB.at<int>(Point(1, j - 1)) != -1) { // e o anterior e linha dupla
			Rect areaLinha = Rect(0, j, evidencias.cols, 1);
			Mat1b linha = evidencias(areaLinha).clone();
			GaussianBlur(linha, linha, Size(3, 3), 0, 0);

			int maxIdx = -1;
			for (int i = 0; i < linha.cols; i++) {
				if (linha.at<uchar>(i) == 255) {
					maxIdx = i;
				}
			}

			// verifica qual lado tem mais evidencias proximas, quando nao ha BPB
			int distEsq = abs(maxIdx - pontosBrancoBPB.at<int>(Point(0, j - 1)));
			int distDir = abs(maxIdx - pontosBrancoBPB.at<int>(Point(1, j - 1)));
			if (distEsq < distDir) qtdEsq++;
			else qtdDir++;
		}
	}

	// percentual com evidencias, para diferencias as simples (dashed ou solid)
	double percentualComEvidencias = countNonZero(histOu);
	percentualComEvidencias /= (double)histOu.cols;
	const double thresPercentualEvidencias = 0.8;

	if (cor == LMT_COLOR::WHITE) { // LMT branco == simples
		if (percentualComEvidencias < thresPercentualEvidencias) return LMT::SSB;
		else return LMT::SCB;
	} else { // LMT amarelo

		double nonZeros = countNonZero(histBPB);
		double all = histBPB.cols;
		double percentualBPB = (nonZeros / all);
		const double thresPercentualBPB = 0.20;
		if (percentualBPB > 1 - thresPercentualBPB) {
			return LMT::DCA;
		} else if (percentualBPB < thresPercentualBPB) {
			if (percentualComEvidencias < thresPercentualEvidencias) return LMT::SSA;
			else return LMT::SCA;
		} else {
			if (qtdEsq > qtdDir) return LMT::DCS;
			else return LMT::DSC;
		}
	}
	return LMT::NONE;
}

// display
Mat3b LMTDetector::LMTtoIMG(LMT _lmt, ConfigXML * _config) {
	string tipo;

	switch (_lmt) {
	case LMT::SCB: tipo = "type_1"; break;
	case LMT::SCA: tipo = "type_3"; break;
	case LMT::SSB: tipo = "type_2"; break;
	case LMT::SSA: tipo = "type_4"; break;
	case LMT::DCA: tipo = "type_5"; break;
	case LMT::DSC: tipo = "type_6"; break;
	case LMT::DCS: tipo = "type_7"; break;
	}

	Mat3b img = imread(_config->DATA_DIR + "images/lmt/" + tipo + ".png");
	return img;
}
Mat1b LMTDetector::montarHistograma(const Mat1b &mapa, vector<Point2d> &spline, int tamanhoBusca, int alturaConfiavel) {

	Mat1b outHistograma = Mat1b(Size((int)spline.size(), 1), uchar(0));
	for (int p = 0; p < spline.size(); p++) { // para cada ponto da spline
		for (int i = -tamanhoBusca; i <= tamanhoBusca; i++) { // para cada pixel do espaco de busca
			if (mapa.at<uchar>((int)spline[p].y, (int)spline[p].x + i) == 255) { // se houver evidencia
				outHistograma.at<uchar>(p) = 255;
				break;
			}
		}
	}
	medianBlur(outHistograma, outHistograma, 5); // remove as pequenas interrupcoes (ruidos)
	return outHistograma;

}
Mat1b LMTDetector::desenharHistograma(const Mat1b &histograma) {
	Mat1b imgHistograma = Mat1b(Size(histograma.cols, histograma.cols), uchar(0));
	for (int i = 1; i < histograma.cols; i++) {
		line(imgHistograma,
			Point(i - 1, histograma.cols - (int)(histograma.cols * ((double)histograma.at<uchar>(i - 1) / 255.0))),
			Point(i, histograma.cols - (int)(histograma.cols * ((double)histograma.at<uchar>(i) / 255.0))),
			Scalar(255));
	}
	return imgHistograma;
}
