#include "Houghs.h"

using namespace std;
using namespace cv;



vector<HoughLine> Houghs::getHoughs(std::vector<bounding_box> bouding_boxes_list, const Mat1b &mapa1, const Rect &roi, vector<HoughLine> &houghsAdjacent, const int maxNumLines, int houghThresholdInicial, int houghStep, double houghMinLineLength, double houghMaxLineGap) {

	double tempoInicio = static_cast<double>(getTickCount());

	// pega somente a regi�o de interesse
	/*Mat1b mascaraRoi = Mat1b(mapa1.size(), uchar(0));
	mascaraRoi(Rect(roi.x, roi.y, roi.width, roi.height)).setTo(255);
	Mat1b binaryFrameFiltradoMasked;
	cv::bitwise_and(mapa1, mascaraRoi, binaryFrameFiltradoMasked);
    */
	// hough
	vector<vector<Point> > lineSegments;
	vector<vector<Point> > lineSegmentsForAdjacent;
	vector<Point> aux;
	vector<Vec4i> lines;

	/*int houghThreshold = houghThresholdInicial;
	cv::HoughLinesP(binaryFrameFiltradoMasked, lines, 1, CV_PI / 180, houghThreshold, houghMinLineLength, houghMaxLineGap);
	while (lines.size() > maxNumLines) {
		lines.clear();
		houghThreshold += houghStep;
		cv::HoughLinesP(binaryFrameFiltradoMasked, lines, 1, CV_PI / 180, houghThreshold, houghMinLineLength, houghMaxLineGap);
	}
	 */

	// guarda as houghs (line segments)
	for (size_t i = 0; i<bouding_boxes_list.size(); i++)	{
		Point pt1, pt2;
		pt1.x = bouding_boxes_list[i].pt1.x;
		pt1.y = bouding_boxes_list[i].pt1.y;
		pt2.x = bouding_boxes_list[i].pt2.x;
		pt2.y = bouding_boxes_list[i].pt2.y;

		aux.clear();
		aux.push_back(pt1);
		aux.push_back(pt2);

		Point2f h = pt2 - pt1;
		h *= 1/cv::norm(h);
		double angleDegrees = cv::fastAtan2(h.y, h.x);

		// remove houghs em determinada inclina��o
		// threshold: if (1.0*dx / 3.0 > dy) {
		// int dx = abs(pt2.x - pt1.x);
		// int dy = abs(pt2.y - pt1.y);
		if (angleDegrees > 330 || angleDegrees < 30) {
			lineSegmentsForAdjacent.push_back(aux);
			continue;
		}

		// Store into vector of pairs of Points for msac
		lineSegments.push_back(aux);
	}

	// seta as houghs que ser�o utilizadas para estimar a presen�a de faixas adjacentes
	if (lineSegmentsForAdjacent.size() > 0) houghsAdjacent = HoughLine::getVetorDeHoughs({ lineSegmentsForAdjacent }, roi);

	// transforma os line segments nas houghs
	vector<HoughLine> houghs = HoughLine::getVetorDeHoughs({ lineSegments }, roi);

	return houghs;
}

int Houghs::contaEvidencias(const vector<HoughLine> &houghs, const Mat1d &mapa, ConfigXML *config) {
	int nEvidencias = 0;
	const int larguraBusca = 2; // para cada lado

	if (houghs[0].isEmpty() && houghs[1].isEmpty()) return nEvidencias; // caso n�o tenha houghs

	// converte as houghs para IPM
	HoughLine ipmH1 = houghs[0].toIPM(config);
	HoughLine ipmH2 = houghs[1].toIPM(config);

	// conta embaixo das houghs
	for (int i = mapa.rows / 2; i < mapa.rows; i++) {
		
		// esquerda
		for (int j = -larguraBusca; j <= larguraBusca; j++) {
			int p1_x = (int)ipmH1.getX(i);
			if (mapa.at<double>(Point(p1_x + j, i)) != 0) {
				nEvidencias++;
				break;
			}
		}

		// direita
		for (int j = -larguraBusca; j <= larguraBusca; j++) {
			int p2_x = (int)ipmH2.getX(i);
			if (mapa.at<double>(Point(p2_x + j, i)) != 0) {
				nEvidencias++;
				break;
			}
		}
	}

	return nEvidencias;
}

bool Houghs::validaHoughs(vector<HoughLine> &houghs, const KalmanState &state, ConfigXML *config) {
	
	vector<HoughLine> bkpHoughs = houghs;
	validaEmptyHoughs(houghs, state, config);

	HoughDoMeio measurement = HoughLine::getKalmanMeasurement(houghs[0], houghs[1], config, Mat());
	// troca de faixa
	if (state.hough != NULL) {
		double distanciaBase = abs(state._hough.xBase - measurement.xBase);
		if (distanciaBase > state._hough.largura * 0.7) { 
			return true; // lane change = true
		}
	}

	validaEmptyHoughs(houghs, state, config);
	return false;
}

void Houghs::validaEmptyHoughs(vector<HoughLine> &houghs, const KalmanState &state, ConfigXML *config) {
	// esquerda(n) && direita(y)
	if (houghs[0].isEmpty() && !houghs[1].isEmpty()) {
		houghs[1].projetar(state._hough.largura, -1, config, houghs[0]);
		// cout << "Sem hough na esquerda. Valor sera estimado pela largura anterior!" << endl;
	}
	// esquerda(y) && direita(n)
	if (houghs[1].isEmpty() && !houghs[0].isEmpty()) {
		houghs[0].projetar(state._hough.largura, 1, config, houghs[1]);
		// cout << "Sem hough na direita. Valor sera estimado pela largura anterior!" << endl;
	}
	// esquerda(n) && direita(n)
	if (houghs[0].isEmpty() && houghs[1].isEmpty() && state.hough != NULL) {
		houghs = state._hough.toVetorDeHoughs(config->roi, config->ipm);
		// cout << "Nenhuma das duas houghs." << endl;
	}
}

double Houghs::diferencaAngulos(HoughLine &h1, HoughLine &h2) {
	Point2d direcao1 = h1.p2 - h1.p1;
	double ang1 = atan2(direcao1.y, direcao1.x);
	Point2d direcao2 = h2.p2 - h2.p1;
	double ang2 = atan2(direcao2.y, direcao2.x);
	return abs(ang2 - ang1);
}

void Houghs::getMediaDesvioBaseBuffer(deque<HoughDoMeio> &buffer, double &outMedia, double &outDesvio) {
	double sum = 0;
	for (HoughDoMeio hough : buffer) sum += hough.xBase;
	double m = sum / buffer.size();
	double accum = 0.0;
	std::for_each(std::begin(buffer), std::end(buffer), [&](const HoughDoMeio h) {
		accum += (h.xBase - m) * (h.xBase - m);
	});
	double stdev = sqrt(accum / (buffer.size() - 1));

	outMedia = m;
	outDesvio = stdev;
}

bool Houghs::validaHough(HoughLine &hough, deque<HoughLine> &buffer, ConfigXML *config) {

	double mediaPosicao, desvioPosicao;
	posicaoBufferStatistics(buffer, mediaPosicao, desvioPosicao, config);

	double mediaAngulo, desvioAngulo;
	anguloBufferStatistics(buffer, mediaAngulo, desvioAngulo, config);

	if (abs(hough.toIPM(config).p1.x - mediaPosicao) > 15) return false;
	if (abs(hough.toIPM(config).getAngulo() - mediaAngulo) > 15) return false;

	return true;
}

void Houghs::posicaoBufferStatistics(deque<HoughLine> &buffer, double &outMedia, double &outDesvio, ConfigXML *config) {
	// media
	double sum = 0;
	for (HoughLine hough : buffer) sum += hough.toIPM(config).p1.x;
	double m = sum / buffer.size();

	// desvio padrao
	double accum = 0.0;
	std::for_each(std::begin(buffer), std::end(buffer), [&](const HoughLine h) {
		accum += (h.toIPM(config).p1.x - m) * (h.toIPM(config).p1.x - m);
	});
	double stdev = sqrt(accum / (buffer.size() - 1));

	// retornos
	outMedia = m;
	outDesvio = stdev;
}

void Houghs::anguloBufferStatistics(deque<HoughLine> &buffer, double &outMedia, double &outDesvio, ConfigXML *config) {
	// media
	double sum = 0;
	for (HoughLine hough : buffer) sum += hough.toIPM(config).getAngulo();
	double m = sum / buffer.size();

	// desvio padrao
	double accum = 0.0;
	std::for_each(std::begin(buffer), std::end(buffer), [&](const HoughLine h) {
		accum += (h.toIPM(config).getAngulo() - m) * (h.toIPM(config).getAngulo() - m);
	});
	double stdev = sqrt(accum / (buffer.size() - 1));

	// retornos
	outMedia = m;
	outDesvio = stdev;
}
