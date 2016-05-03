#include "AnaliseDasHoughs.h"

using namespace std;
using namespace cv;

struct OrdenaHough {
	OrdenaHough(ConfigXML * __config) { this->_config = __config; }
	bool operator () (HoughLine i, HoughLine j) { return (i.toIPM(_config).getAngulo() > j.toIPM(_config).getAngulo()); };

	ConfigXML * _config;
};

struct OrdenaHoughPelaDistancia {
	OrdenaHoughPelaDistancia(const HoughLine &_baseline, ConfigXML * __config) { this->baseline = _baseline; this->_config = __config; }
	bool operator () (HoughLine i, HoughLine j) { return (i.toIPM(_config).calculaDistancia(baseline) > j.toIPM(_config).calculaDistancia(baseline)); }; // decrescente

	ConfigXML * _config;
	HoughLine baseline;

};

struct lessThanPoint {
	bool operator()(cv::Point const& a, cv::Point const& b) {
		return (a.x < b.x) || (a.x == b.x && a.y < b.y);
	}
};

// =================================
// HOUGH LINE DO MEIO
// =================================
HoughDoMeio::HoughDoMeio(const Mat1d &kalmanMeasurement) {
	xBase = kalmanMeasurement.at<double>(0);
	xTopo = kalmanMeasurement.at<double>(1);
	largura = kalmanMeasurement.at<double>(2);
}

Mat1d HoughDoMeio::toKalman() const {
	Mat1d out = Mat1d(Size(1, 3), double(0));
	out.at<double>(0) = xBase;
	out.at<double>(1) = xTopo;
	out.at<double>(2) = largura;
	return out;
}

vector<HoughLine> HoughDoMeio::toVetorDeHoughs(const Rect &roi, IPM *ipm) const {
	if (ipm == NULL) {
		HoughLine esq;
		esq.p1 = Point2d(xBase - largura / 2, roi.y + roi.height);
		esq.p2 = Point2d(xTopo - largura / 2, roi.y);
		esq._p1 = esq.p1;
		esq._p2 = esq.p2;

		HoughLine dir;
		dir.p1 = Point2d(xBase + largura / 2, roi.y + roi.height);
		dir.p2 = Point2d(xTopo + largura / 2, roi.y);
		dir._p1 = dir.p1;
		dir._p2 = dir.p2;

		return { esq, dir };
	} else {
		HoughLine esq;
		esq.p1 = ipm->applyHomographyInv(Point2d(xBase - largura / 2, roi.height));
		esq.p2 = ipm->applyHomographyInv(Point2d(xTopo - largura / 2, 0));
		esq.p1.y += roi.y;
		esq.p2.y += roi.y;
		esq._p1 = esq.p1;
		esq._p2 = esq.p2;

		HoughLine dir;
		dir.p1 = ipm->applyHomographyInv(Point2d(xBase + largura / 2, roi.height));
		dir.p2 = ipm->applyHomographyInv(Point2d(xTopo + largura / 2, 0));
		dir.p1.y += roi.y;
		dir.p2.y += roi.y;
		dir._p1 = dir.p1;
		dir._p2 = dir.p2;

		return { esq, dir };
	}
}

vector<Point2d> HoughDoMeio::getHoughPoints(int roiHeight, const int laneSide, bool ateMetade) const {
	
	vector<Point2d> pontos;

	int inicio = (ateMetade) ? roiHeight / 2 : 0;
	int fim = roiHeight;
	double deltaX = (xBase - xTopo) / (double)roiHeight;

	int fator = (laneSide == LANE_LEFT) ? -1 : 1;
	for (int i = inicio; i < fim; i++) {
		double shift = fator * (largura / 2.0);
		double deltaAcumulado = i*deltaX;
		pontos.push_back(Point2d(xTopo + shift + deltaAcumulado, i));
	}

	return pontos;
}

void HoughDoMeio::print() {
	cout << "HoughDoMeio:" << endl;
	cout << " - xBase: " << xBase << endl;
	cout << " - xTopo: " << xTopo << endl;
	cout << " - largura: " << largura<< endl;
}

void HoughDoMeio::draw(Mat3b &ipmImage, const Scalar &cor, int thickness) const  {
	line(ipmImage, Point((int)(xBase - largura / 2), ipmImage.rows), Point((int)(xTopo - largura / 2), 0), cor, thickness); // esq
	line(ipmImage, Point((int)(xBase + largura / 2), ipmImage.rows), Point((int)(xTopo + largura / 2), 0), cor, thickness); // dir
	line(ipmImage, Point((int)(xBase), ipmImage.rows), Point((int)(xTopo), 0), cor, thickness); // meio
}

void HoughDoMeio::draw(Mat &image, const Scalar &cor, ConfigXML *_config) const {
	
	Point2d ipmP1, ipmP2, p1, p2;

	// meio
	ipmP1 = Point2d(xBase, _config->roi.height);
	ipmP2 = Point2d(xTopo, 0);
	p1 = _config->ipm->applyHomographyInv(ipmP1);
	p2 = _config->ipm->applyHomographyInv(ipmP2);
	p1.y += _config->roi.y;
	p2.y += _config->roi.y;
	line(image, p1, p2, cor, 1); // meio

	// esq
	ipmP1 = Point2d(xBase - largura / 2, _config->roi.height);
	ipmP2 = Point2d(xTopo - largura / 2, 0);
	p1 = _config->ipm->applyHomographyInv(ipmP1);
	p2 = _config->ipm->applyHomographyInv(ipmP2);
	p1.y += _config->roi.y;
	p2.y += _config->roi.y;
	line(image, p1, p2, cor, 1); // esq

	// dir
	ipmP1 = Point2d(xBase + largura / 2, _config->roi.height);
	ipmP2 = Point2d(xTopo + largura / 2, 0);
	p1 = _config->ipm->applyHomographyInv(ipmP1);
	p2 = _config->ipm->applyHomographyInv(ipmP2);
	p1.y += _config->roi.y;
	p2.y += _config->roi.y;
	line(image, p1, p2, cor, 1); // dir
}

// =================================
// HOUGH LINE
// =================================
HoughLine::HoughLine() {
	p1 = Point(-1, -1);
	p2 = Point(-1, -1);
}

bool HoughLine::isEmpty() const {
	return (p1.x == -1 && p1.y == -1 && p2.x == -1 && p2.y == -1);
}

double HoughLine::getX(int y) const {
	// calcula as varia��es nos eixos
	double dx = p2.x - p1.x;
	double dy = p2.y - p1.y;

	// dy == 0 -> linha vertical
	return (dy == 0) ? p1.x : (p1.x + ((y - p1.y) * (dx / dy)));
}

double HoughLine::getAngulo() const {
	// calcula o angulo
	double rad2Degree = 180.0 / CV_PI;
	return (atan2(p1.y - p2.y, p1.x - p2.x) * rad2Degree);
}

HoughLine HoughLine::toIPM(ConfigXML *config) const {

	int diff = config->roi.y;

	HoughLine h;
	h.p1 = config->ipm->applyHomography(Point2d(p1.x, p1.y - diff));
	h.p2 = config->ipm->applyHomography(Point2d(p2.x, p2.y - diff));
	h._p1 = config->ipm->applyHomography(Point2d(_p1.x, _p1.y - diff));
	h._p2 = config->ipm->applyHomography(Point2d(_p2.x, _p2.y - diff));
	return h;
}

HoughLine HoughLine::fromIPM(ConfigXML *config) const {

	int diff = config->roi.y;

	HoughLine h;
	h.p1 = config->ipm->applyHomographyInv(p1);
	h.p2 = config->ipm->applyHomographyInv(p2);
	h.p1.y += diff;
	h.p2.y += diff;
	h._p1 = h.p1;
	h._p2 = h.p2;
	return h;
}

void HoughLine::projetar(double largura, int direcao, ConfigXML *config, HoughLine &projetado) {
	HoughLine ipmHough = this->toIPM(config);
	HoughLine volta = ipmHough.fromIPM(config);
	HoughLine ipmProjetado;
	ipmProjetado.p1.x = ipmHough.p1.x + direcao * largura;
	ipmProjetado.p1.y = ipmHough.p1.y;
	ipmProjetado.p2.x = ipmHough.p2.x + direcao * largura;
	ipmProjetado.p2.y = ipmHough.p2.y;
	ipmProjetado._p1 = ipmProjetado.p1;
	ipmProjetado._p2 = ipmProjetado.p2;
	projetado = ipmProjetado.fromIPM(config);
}

HoughDoMeio HoughLine::getKalmanMeasurement(const HoughLine &h1, const HoughLine &h2, ConfigXML *config, Mat img) {

	// converte as houghs para IPM
	HoughLine ipmH1 = h1.toIPM(config);
	HoughLine ipmH2 = h2.toIPM(config);

	// calcula os parametros
	HoughDoMeio kalmanMeasure;
	kalmanMeasure.xBase = (ipmH1.p1.x + ipmH2.p1.x) / 2.0;
	kalmanMeasure.xTopo = (ipmH1.p2.x + ipmH2.p2.x) / 2.0;
	kalmanMeasure.largura = abs(ipmH1.p1.x - ipmH2.p1.x);

	// mostra em uma imagem
	if (!img.empty()) {
		int altura = (int)(ipmH1.p1.y - ipmH1.p2.y);
		line(img, Point((int)ipmH1.p1.x, altura), Point((int)ipmH1.p2.x, 0), Scalar(255, 0, 255)); // esq
		line(img, Point((int)ipmH2.p1.x, altura), Point((int)ipmH2.p2.x, 0), Scalar(255, 255, 0)); // dir
		line(img, Point((int)kalmanMeasure.xBase, altura), Point((int)kalmanMeasure.xTopo, 0), Scalar(0, 0, 255)); // meio
		imshow("Kalman Measurement", img);
	}

	return kalmanMeasure;
}

HoughLine HoughLine::create(const Point2d &p1, const Point2d &p2, double minY, double maxY) {

	// calcula as varia��es nos eixos
	double dx = p2.x - p1.x;
	double dy = p2.y - p1.y;

	// a HoughLine ser� definida por dois pontos
	// p1 -> ponto na base da ROI (minY)
	// p2 -> ponto no topo da ROI (maxY)

	// falta descobrir o valor no eixo X nessas posi��es Y
	// dy == 0 -> linha vertical
	double minX = (dy == 0) ? p1.x : p1.x + ((minY - p1.y) * (dx / dy));
	double maxX = (dy == 0) ? p1.x : p1.x + ((maxY - p1.y) * (dx / dy));

	HoughLine out;
	out._p1 = p1;
	out._p2 = p2;
	out.p1 = Point2d(minX, minY);
	out.p2 = Point2d(maxX, maxY);
	return out;
}

HoughLine HoughLine::create(HoughDoMeio &hough, ConfigXML *_config) {
	Point2d ipmP1 = Point2d(hough.xBase, _config->roi.height);
	Point2d ipmP2 = Point2d(hough.xTopo, 0);
	Point2d p1 = _config->ipm->applyHomographyInv(ipmP1);
	Point2d p2 = _config->ipm->applyHomographyInv(ipmP2);

	p1.y += _config->roi.y;
	p2.y += _config->roi.y;

	HoughLine out;
	out.p1 = p1;
	out.p2 = p2;
	out._p1 = p1;
	out._p2 = p2;

	return out;
}

vector<HoughLine> HoughLine::getVetorDeHoughs(vector<vector<vector<Point> > > lineSegmentsClusters, const Rect &roi) {
	vector<HoughLine> linhas;
	// se houver houghs
	if (lineSegmentsClusters.size() > 0) {
		// para cada hough (s� o primeiro cluster importa)
		for (unsigned int i = 0; i < lineSegmentsClusters[0].size(); i++) {

			// pega os pontos
			Point *p1 = &lineSegmentsClusters[0][i][0];
			Point *p2 = &lineSegmentsClusters[0][i][1];

			double minY = roi.y + roi.height;
			double maxY = roi.y;
			HoughLine linha = create(*p1, *p2, minY, maxY);

			// adiciona a linha
			linhas.push_back(linha);
		}
	}
	return linhas;
}

vector<HoughLine> HoughLine::getVetorDeHoughs(vector<vector<vector<Point> > > lineSegmentsClusters, int baseY, int topoY) {
	vector<HoughLine> linhas;
	// se houver houghs
	if (lineSegmentsClusters.size() > 0) {
		// para cada hough (s� o primeiro cluster importa)
		for (unsigned int i = 0; i < lineSegmentsClusters[0].size(); i++) {

			// pega os pontos
			Point *p1 = &lineSegmentsClusters[0][i][0];
			Point *p2 = &lineSegmentsClusters[0][i][1];

			HoughLine linha = create(*p1, *p2, baseY, topoY);

			// adiciona a linha
			linhas.push_back(linha);
		}
	}
	return linhas;
}

void HoughLine::print() const  {
	cout << "HoughLine" << endl;
	cout << "- p1: " << p1 << endl;
	cout << "- p2: " << p2 << endl;
}

void HoughLine::draw(Mat &img, const Scalar &cor, int thickness) {
	if (isEmpty()) return;
	line(img, p1, p2, cor, thickness);
}

double HoughLine::calculaDistancia(const HoughLine &baseline) {
	// se um dos dois for vazio, n�o tem o que calcular
	if (baseline.isEmpty() || this->isEmpty()) return numeric_limits<double>::max();

	double sum = 0;
	for (int i = (int)this->p1.y; i < (int)this->p2.y; ++i) {
		double diff = abs(this->getX(i) - baseline.getX(i));
		sum += diff;
	}
	return sum;
}

// =================================
// AN�LISE DAS HOUGHS
// =================================
AnaliseDasHoughs::AnaliseDasHoughs(ConfigXML *_config) {
	config = _config;
	colorFrame = Mat3b(Size(config->dataset.FrameSize.width, config->dataset.FrameSize.height), Vec3b(0,0,0));

	posicaoCarro = Point2d(config->dataset.FrameSize.width / 2.0, config->dataset.FrameSize.height);
	posicaoCarroIPM = config->ipm->applyHomography(Point2d(config->dataset.FrameSize.width / 2.0, (double)config->roi.height));
	larguraPista = (config->dataset._IPM.tr - config->dataset._IPM.tl);
}

void AnaliseDasHoughs::setColorFrame(const Mat3b &_colorFrame) {
	colorFrame = _colorFrame;
}

void AnaliseDasHoughs::processarHistogramaParcial(Mat1d &histograma, int laneSide, const double epsilon) {
	// calcula de maneira 'eficiente' a soma acumulada
	// 1 1 2 1 1 => 6 5 4 2 1 para LANE_LEFT
	// 1 1 2 1 1 => 1 2 4 5 6 para LANE_RIGHT
	Mat1d histogramaAcumulado = Mat1d(histograma.size(), double(0));
	if (laneSide == LANE_LEFT) {
		histogramaAcumulado.at<double>(histograma.cols - 1) = histograma.at<double>(histograma.cols - 1); // inicializa
		for (int i = histograma.cols - 2; i >= 0; i--) {
			histogramaAcumulado.at<double>(i) = histogramaAcumulado.at<double>(i + 1) + histograma.at<double>(i) + epsilon;
		}
	} else {
		histogramaAcumulado.at<double>(0) = histograma.at<double>(0); // inicializa
		for (int i = 1; i < histograma.cols -1; i++) {
			histogramaAcumulado.at<double>(i) = histogramaAcumulado.at<double>(i - 1) + histograma.at<double>(i) + epsilon;
		}
	}
	normalize(histogramaAcumulado, histogramaAcumulado, 0.0, 1.0, NORM_MINMAX);
	histogramaAcumulado = 1 - histogramaAcumulado;
	histograma = histograma.mul(histogramaAcumulado);
	normalize(histograma, histograma, 0.0, 1.0, NORM_MINMAX);
}

void AnaliseDasHoughs::processarHistogramaParcialLocal(Mat1d &histograma, int laneSide, const double epsilon) {

	const int tamanhoRegiao = 200;
	const double maxPenalizacao = 1.4;

	// histograma total
	const int regiao = 15;
	Mat1d histogramaProcessado = histograma.clone();
	if (laneSide == LANE_LEFT) {
		for (int i = (histograma.cols / 2) - regiao; i >= 0; i--) {
			double penalizacao = maxPenalizacao * histograma.at<double>(i);
			if (penalizacao > 1) penalizacao = 1;
			int fim = (i - tamanhoRegiao < 0) ? 0 : i - tamanhoRegiao;
			for (int j = i - 1; j > fim; j--) {
				histogramaProcessado.at<double>(j) = histogramaProcessado.at<double>(j) * (1 - penalizacao);
			}
		}
	} else {
		for (int i = (histograma.cols / 2) + regiao; i < histograma.cols - 1; i++) {
			double penalizacao = maxPenalizacao * histograma.at<double>(i);
			if (penalizacao > 1) penalizacao = 1;
			int fim = (i + tamanhoRegiao > histogramaProcessado.cols - 1) ? histogramaProcessado.cols - 1 : i + tamanhoRegiao;
			for (int j = i + 1; j < fim; j++) {
				histogramaProcessado.at<double>(j) = histogramaProcessado.at<double>(j) * (1 - penalizacao);
			}
		}
	}

	histograma = histogramaProcessado.clone();
}

void AnaliseDasHoughs::executar2D(const Mat1b &mapa1ipm, const vector<HoughLine> &houghs, HoughDoMeio *houghsAnteriores, const Mat1b &mapaIPM, vector<HoughLine> &outHoughs) {
	double tempoInicio = static_cast<double>(getTickCount());

	const int larguraImagem = mapa1ipm.cols;

	// pega a metade de baixo da imagem
	Rect areaDeCima = Rect(0, 0, mapaIPM.cols, mapaIPM.rows / 2);
	Mat1b metadeDeBaixo = mapaIPM.clone();
	metadeDeBaixo(areaDeCima) = 0;

	vector<HoughLine> esqTemp, dirTemp;
	selecionarHoughs(houghs, &esqTemp, &dirTemp); // houghs na perspectiva

	// monta o histograma 2d: posicao x angulo
	map<Point, vector<HoughLine>, lessThanPoint> esqHoughsHistograma2D, dirHoughsHistograma2D;
	Mat1d esqHistograma2D = montarHistograma2D(esqTemp, metadeDeBaixo, esqHoughsHistograma2D, false);
	Mat1d dirHistograma2D = montarHistograma2D(dirTemp, metadeDeBaixo, dirHoughsHistograma2D, false);
	Mat1d histograma2D = combinaHistogramas2D(esqHistograma2D, dirHistograma2D);

	// verifica o angulo dominante
	int anguloDominante = getAnguloDominante(histograma2D);
	if (anguloDominante == -1) return;
	const int tamRegiaoAngulo = 15; // [-regiaoAngulo --- anguloDominante --- +regiaoAngulo]

	// aplica uma mascara para manter somente os angulos proximos ao dominante
	mascaraHistogramas(anguloDominante, tamRegiaoAngulo, esqHistograma2D, dirHistograma2D);

	normalize(esqHistograma2D, esqHistograma2D, 0.0, 1.0, NORM_MINMAX);
	normalize(dirHistograma2D, dirHistograma2D, 0.0, 1.0, NORM_MINMAX);
	
	// monta o histograma 1d: posicao
	map<int, vector<HoughLine>> esqHoughsHistograma1D, dirHoughsHistograma1D;
	Mat1d esqHistograma1D = montarHistograma1D(esqHistograma2D, esqHoughsHistograma2D, houghsAnteriores, esqHoughsHistograma1D, LANE_LEFT);
	Mat1d dirHistograma1D = montarHistograma1D(dirHistograma2D, dirHoughsHistograma2D, houghsAnteriores, dirHoughsHistograma1D, LANE_RIGHT);

	// modifica o histograma
	processarHistogramaParcialLocal(esqHistograma1D, LANE_LEFT);
	processarHistogramaParcialLocal(dirHistograma1D, LANE_RIGHT);

	// encontra o X que representa os m�ximos dos histogramas
	int esqHistogramaMaxX, dirHistogramaMaxX;
	encontraMaximoNosHistogramas(esqHistograma1D, dirHistograma1D, &esqHistogramaMaxX, &dirHistogramaMaxX);

	// pega as houghs dos picos
	vector<HoughLine> esqHoughs = esqHoughsHistograma1D[esqHistogramaMaxX];
	vector<HoughLine> dirHoughs = dirHoughsHistograma1D[dirHistogramaMaxX];

	// ordena as houghs
	// sort(esqHoughs.begin(), esqHoughs.end());
	// sort(dirHoughs.begin(), dirHoughs.end());

	// pega a mediana
	HoughLine esq = (esqHoughs.size() > 0) ? esqHoughs[esqHoughs.size() / 2] : HoughLine::empty();
	HoughLine dir = (dirHoughs.size() > 0) ? dirHoughs[dirHoughs.size() / 2] : HoughLine::empty();

	// retorna as houghs encontradas
	outHoughs[0] = esq;
	outHoughs[1] = dir;

	// calcula o tempo de execu��o
	double tempoFim = static_cast<double>(getTickCount());
	double tempoExecutando = ((tempoFim - tempoInicio) / getTickFrequency()) * 1000;

	// exibe as sa�das definidas (texto e/ou imagem)
	if (config->verbose) {
		cout << "- analise das houghs: " << tempoExecutando << " ms" << endl;
		if (esq.isEmpty()) cout << "  - esquerda: sem hough" << endl;
		if (dir.isEmpty()) cout << "  - direita: sem hough" << endl;
	}

	if (config->display) {
		// histograma da metade de baixo em 1d
		Mat1d histograma1D = Mat1d(Size(esqHistograma1D.cols, esqHistograma1D.rows), double(0));
		Rect esqArea1D = Rect(0, 0, posicaoCarroIPM.x, esqHistograma1D.rows);
		Rect dirArea1D = Rect(posicaoCarroIPM.x, 0, dirHistograma1D.cols - posicaoCarroIPM.x, dirHistograma1D.rows);
		esqHistograma1D(esqArea1D).copyTo(histograma1D(esqArea1D));
		dirHistograma1D(dirArea1D).copyTo(histograma1D(dirArea1D));
		Mat3b imgHistograma1D = Mat3b(Size(dirHistograma1D.cols, 150), Vec3b(0, 0, 0));
		desenhaHistograma(histograma1D, imgHistograma1D);
		circle(imgHistograma1D, Point(esqHistogramaMaxX, 10), 3, esqCor, CV_FILLED);
		circle(imgHistograma1D, Point(dirHistogramaMaxX, 10), 3, dirCor, CV_FILLED);

		// visualiza as houghs selecionadas
		for (HoughLine h : esqTemp) line(this->colorFrame, h._p1, h._p2, esqCor);
		for (HoughLine h : dirTemp) line(this->colorFrame, h._p1, h._p2, dirCor);

		// visualizar as houghs finais
		Scalar preto = Scalar(0, 0, 0);
		if (!esq.isEmpty()) esq.draw(this->colorFrame, esqCor);
		if (!dir.isEmpty()) dir.draw(this->colorFrame, dirCor);
		if (!esq.isEmpty()) esq.draw(this->colorFrame, preto, 1);
		if (!dir.isEmpty()) dir.draw(this->colorFrame, preto, 1);

		// monta a imagem final
		Mat3b imgFinal = Mat3b(Size(imgHistograma1D.cols, this->colorFrame.rows), Vec3b(0, 0, 0));
		this->colorFrame.copyTo(this->colorFrame(Rect(0, 0, this->colorFrame.cols, this->colorFrame.rows)));
		imgHistograma1D.copyTo(this->colorFrame(Rect(0, 0, imgHistograma1D.cols, imgHistograma1D.rows)));

		imshow("Analise das Houghs", this->colorFrame);
		imshow("Histograma2D", histograma2D);
	}
}

void AnaliseDasHoughs::executar(const Mat1b &mapa1ipm, const vector<HoughLine> &houghs, vector<HoughLine> &houghsAnteriores, const Mat1b &mapaIPM) {
	
	double tempoInicio = static_cast<double>(getTickCount());

	const int larguraImagem = mapa1ipm.cols;

	// pega a metade de baixo da imagem
	Rect areaDeCima = Rect(0, 0, mapa1ipm.cols, mapa1ipm.rows / 2);
	Mat1b metadeDeBaixo = mapaIPM.clone();
	metadeDeBaixo(areaDeCima) = 0;

	vector<HoughLine> esqTemp, dirTemp;
	selecionarHoughs(houghs, &esqTemp, &dirTemp); // houghs na perspectiva
	const int tamanho_bin_inicial = 2;
	Mat1d esqHistograma = montarHistograma(esqTemp, metadeDeBaixo, larguraImagem, tamanho_bin_inicial, true);
	Mat1d dirHistograma = montarHistograma(dirTemp, metadeDeBaixo, larguraImagem, tamanho_bin_inicial, true);

	// modifica o histograma
	processarHistogramaParcial(esqHistograma, LANE_LEFT);
	processarHistogramaParcial(dirHistograma, LANE_RIGHT);

	Mat1d histogramaMetadeDeBaixo = Mat1d(Size(larguraImagem, 1), double(0));
	Rect areaEsq = Rect(0, 0, posicaoCarroIPM.x, 1);
	Rect areaDir = Rect(posicaoCarroIPM.x, 0, mapa1ipm.cols - posicaoCarroIPM.x, 1);
	esqHistograma(areaEsq).copyTo(histogramaMetadeDeBaixo(areaEsq));
	dirHistograma(areaDir).copyTo(histogramaMetadeDeBaixo(areaDir));

	// encontra o X que representa os m�ximos dos histogramas
	int esqHistogramaMaxX, dirHistogramaMaxX;
	encontraMaximoNosHistogramas(esqHistograma, dirHistograma, &esqHistogramaMaxX, &dirHistogramaMaxX);
	// esqHistogramaMaxX += posicaoCarroIPM.x - (int)larguraPista;
	// dirHistogramaMaxX -= posicaoCarroIPM.x;

	// pega a faixa mais interna usando a troca de sinal da diferen�a
	double thresSegundoMax = 0.4;
	const int tamanhoFaixa = 6;
	const int tamanhoVerificacao = tamanhoFaixa * 3;
	double ultimaDiferenca;
	const int tamanho_bin = 10;

	// seleciona as houghs que est�o dentro da regi�o na IPM
	vector<HoughLine> esqHoughsSelecionadas, dirHoughsSelecionadas;
	selecionarHoughs(houghs, esqHistogramaMaxX, dirHistogramaMaxX, (2 * tamanho_bin_inicial) + 1, &esqHoughsSelecionadas, &dirHoughsSelecionadas);

	// escolhe a hough da esquerda
	HoughLine esq;
	if (esqHoughsSelecionadas.size() > 0) {
		// histograma das houghs selecionadas na perspectiva
		Mat1d esqHistHoughsSelecionadas = montarHistograma(esqHoughsSelecionadas, larguraImagem, tamanho_bin, true);
		// pega o pico mais interno
		int esqMaxX = encontrarMaximoMaisProximo(esqHistHoughsSelecionadas, -1, posicaoCarro.x, true);
		// escolhe a mediana
		pegarMedianaNaRegiao(esqHoughsSelecionadas, esqMaxX, tamanho_bin / 2, esq);
	}

	// escolhe a hough da direita
	HoughLine dir;
	if (dirHoughsSelecionadas.size() > 0) {
		// histograma das houghs selecionadas na perspectiva
		Mat1d dirHistHoughsSelecionadas = montarHistograma(dirHoughsSelecionadas, larguraImagem, tamanho_bin, true);
		// pega o pico mais interno
		int dirMaxX = encontrarMaximoMaisProximo(dirHistHoughsSelecionadas, 1, posicaoCarro.x, true);
		// escolhe a mediana
		pegarMedianaNaRegiao(dirHoughsSelecionadas, dirMaxX, tamanho_bin / 2, dir);
	}

	// verifica faixa interna em rela��o a da faixa esquerda
	ultimaDiferenca = 0;
	int esqCont = 0;
	bool esqTemInterna = false;
	if (esqHistogramaMaxX != -1) { // se houver pico
		for (int i = esqHistogramaMaxX; i < esqHistogramaMaxX + tamanhoVerificacao - 1; i++) {
			double diferencaAtual = histogramaMetadeDeBaixo.at<double>(i + 1) - histogramaMetadeDeBaixo.at<double>(i);
			// verifica se h� troca de negativo para positivo
			if (ultimaDiferenca > 0 && diferencaAtual <= 0) {
				int esqSegundoHistogramaMaxX = i;
				if (histogramaMetadeDeBaixo.at<double>(esqSegundoHistogramaMaxX) > thresSegundoMax) {
					esqHistogramaMaxX = esqSegundoHistogramaMaxX;
					esqTemInterna = true;

					// seleciona a hough da esquerda mais pr�xima externamente do ponto interno
					double esqMinDist = numeric_limits<double>::max();
					int esqMinIdx = -1;
					for (unsigned int h = 0; h < esqHoughsSelecionadas.size(); h++) {
						double dist = esqHistogramaMaxX - esqHoughsSelecionadas[h].toIPM(config).p1.x;
						if (abs(dist) >= 1 && dist < tamanhoVerificacao && dist < esqMinDist) {
							esqMinDist = dist;
							esqMinIdx = h;
						}
					}
					if (esqMinIdx != -1) esqHistogramaMaxX = (int)esqHoughsSelecionadas[esqMinIdx].toIPM(config).p1.x;
					break;
				}
			}
			ultimaDiferenca = diferencaAtual;
		}
	}

	// verifica faixa interna em rela��o a da faixa direita
	ultimaDiferenca = 0;
	int dirCont = 0;
	bool dirTemInterna = false;
	if (dirHistogramaMaxX != -1) { // se houver pico
		for (int i = dirHistogramaMaxX; i > dirHistogramaMaxX - tamanhoVerificacao; i--) {
			double diferencaAtual = histogramaMetadeDeBaixo.at<double>(i - 1) - histogramaMetadeDeBaixo.at<double>(i);
			// verifica se h� troca de negativo para positivo
			if (ultimaDiferenca > 0 && diferencaAtual <= 0) {
				int dirSegundoHistogramaMaxX = i;
				if (histogramaMetadeDeBaixo.at<double>(dirSegundoHistogramaMaxX) > thresSegundoMax) {
					dirHistogramaMaxX = dirSegundoHistogramaMaxX;
					dirTemInterna = true;

					// seleciona a hough da direita mais pr�xima externamente do ponto interno
					double dirMinDist = numeric_limits<double>::max();
					int dirMinIdx = -1;
					for (unsigned int h = 0; h < dirHoughsSelecionadas.size(); h++) {
						double dist = dirHoughsSelecionadas[h].toIPM(config).p1.x - dirHistogramaMaxX;
						if (abs(dist) >= 1 && dist < tamanhoVerificacao && dist < dirMinDist) {
							dirMinDist = dist;
							dirMinIdx = h;
						}
					}
					if (dirMinIdx != -1) dirHistogramaMaxX = (int)dirHoughsSelecionadas[dirMinIdx].toIPM(config).p1.x;
					break;
				}
			}
			ultimaDiferenca = diferencaAtual;
		}
	}

	if (esqTemInterna) {
		// pega somente o angulo da mediana que j� foi calculada
		if (!esq.isEmpty()) {
			HoughLine esqIPM = esq.toIPM(config);
			double diffX = esqIPM.p1.x - esqHistogramaMaxX;
			if (abs(diffX) >= 1) { // o histograma n�o tem precis�o, e um pixel representa muito na perspectiva
				esqIPM.p1.x = esqHistogramaMaxX;
				esqIPM._p1.x = esqIPM.p1.x;
				esqIPM.p2.x += diffX;
				esqIPM._p2.x = esqIPM.p2.x;
				esq = esqIPM.fromIPM(config);
			}
		}
	}

	if (dirTemInterna) {
		// pega somente o angulo da mediana
		if (!dir.isEmpty()) {
			HoughLine dirIPM = dir.toIPM(config);
			double diffX = dirIPM.p1.x - dirHistogramaMaxX;
			if (abs(diffX) >= 1) { // o histograma n�o tem precis�o, e um pixel representa muito na perspectiva
				dirIPM.p1.x = dirHistogramaMaxX;
				dirIPM._p1.x = dirIPM.p1.x;
				dirIPM.p2.x += diffX;
				dirIPM._p2.x = dirIPM.p2.x;
				dir = dirIPM.fromIPM(config);
			}
		}
	}

	// retorna as houghs encontradas
	houghsAnteriores[0] = esq;
	houghsAnteriores[1] = dir;

	// calcula o tempo de execu��o
	double tempoFim = static_cast<double>(getTickCount());
	double tempoExecutando = ((tempoFim - tempoInicio) / getTickFrequency()) * 1000;

	// exibe as sa�das definidas (texto e/ou imagem)
	if (config->verbose) {
		cout << "- analise das houghs: " << tempoExecutando << " ms" << endl;
		if (esq.isEmpty()) cout << "  - esquerda: sem hough" << endl;
		if (dir.isEmpty()) cout << "  - direita: sem hough" << endl;
	}

	if (config->display) {
		// desenha o histograma
		Mat3b imgHistograma = Mat3b(Size(larguraImagem, 150), Vec3b(0, 0, 0));
		desenhaHistograma(histogramaMetadeDeBaixo, imgHistograma);
		circle(imgHistograma, Point(esqHistogramaMaxX, 10), 3, esqCor, CV_FILLED);
		circle(imgHistograma, Point(dirHistogramaMaxX, 10), 3, dirCor, CV_FILLED);

		// visualiza as houghs selecionadas
		for (HoughLine h : esqHoughsSelecionadas) line(this->colorFrame, h._p1, h._p2, esqCor);
		for (HoughLine h : dirHoughsSelecionadas) line(this->colorFrame, h._p1, h._p2, dirCor);

		// visualizar as houghs finais
		Scalar preto = Scalar(0, 0, 0);
		if (!esq.isEmpty()) esq.draw(this->colorFrame, esqCor);
		if (!dir.isEmpty()) dir.draw(this->colorFrame, dirCor);
		if (!esq.isEmpty()) esq.draw(this->colorFrame, preto, 1);
		if (!dir.isEmpty()) dir.draw(this->colorFrame, preto, 1);

		// monta a imagem final
		Mat3b imgFinal = Mat3b(Size(larguraImagem, imgHistograma.rows + this->colorFrame.rows), Vec3b(0, 0, 0));
		imgHistograma.copyTo(imgFinal(Rect(0, 0, imgHistograma.cols, imgHistograma.rows)));
		this->colorFrame.copyTo(imgFinal(Rect(0, imgHistograma.rows, this->colorFrame.cols, this->colorFrame.rows)));

		imshow("Analise das Houghs", imgFinal);
	}
}

Mat1d AnaliseDasHoughs::combinaHistogramas2D(const Mat1d &esqHistograma, const Mat1d &dirHistograma) {

	if (esqHistograma.size() != dirHistograma.size())
		cerr << "combinaHistogramas(): histogramas possuem tamanhos diferentes e nao podem ser combinados!" << endl;

	// inicializa o histograma combinado
	Mat1d histogramaCombinado = Mat1d(Size(esqHistograma.cols, esqHistograma.rows), double(0));

	// inicializa as area de cada histograma
	Rect esqArea = Rect(0, 0, posicaoCarroIPM.x, esqHistograma.rows);
	Rect dirArea = Rect(posicaoCarroIPM.x, 0, dirHistograma.cols - posicaoCarroIPM.x, dirHistograma.rows);

	// copia a parte de cada histograma para o histograma combinado
	esqHistograma(esqArea).copyTo(histogramaCombinado(esqArea));
	dirHistograma(dirArea).copyTo(histogramaCombinado(dirArea));

	normalize(histogramaCombinado, histogramaCombinado, 0.0, 1.0, NORM_MINMAX);

	return histogramaCombinado;
}

int AnaliseDasHoughs::getAnguloDominante(const Mat1d &histograma2d, int gaussianBlurSize) {
	Mat1d _histograma2d, histogramaAngulos;

	// aplica um blur
	GaussianBlur(histograma2d, _histograma2d, Size(gaussianBlurSize, gaussianBlurSize), 0);
	
	// reduz o histograma a uma �nica coluna, acumulando (somando) as linhas
	cv::reduce(_histograma2d, histogramaAngulos, 1, CV_REDUCE_SUM);

	int anguloDominante = -1;
	double maxVal = 0;
	for (int i = 0; i < histogramaAngulos.rows; ++i) {
		if (histogramaAngulos.at<double>(i) > maxVal) {
			anguloDominante = i;
			maxVal = histogramaAngulos.at<double>(i);
		}
	}

	return anguloDominante;
}

void AnaliseDasHoughs::mascaraHistogramas(int anguloDominante, int regiao, Mat1d &esqHistograma2d, Mat1d &dirHistograma2d) {
	// regi�o a ser mantida
	Rect regiaoAngulos = Rect(0, anguloDominante - regiao, esqHistograma2d.cols, 2 * regiao + 1);
	
	// cria uma mascara para os angulos, para ter o mesmo efeito de rejeit�-los
	Mat1d marcaraAngulos = Mat1d::zeros(esqHistograma2d.size());
	marcaraAngulos(regiaoAngulos) = 1;
	//imshow("marcaraAngulos", marcaraAngulos);

	// aplica a mascara
	esqHistograma2d = esqHistograma2d.mul(marcaraAngulos);
	dirHistograma2d = dirHistograma2d.mul(marcaraAngulos);
}

Mat1d AnaliseDasHoughs::montarHistograma2D(const vector<HoughLine> &_houghs, const Mat1b &mapa, map<Point, vector<HoughLine>, lessThanPoint> &outHoughsHistograma2D, bool normalizar) {

	int _binsize_posicao = 3;
	int _binsize_angulo = 1;

	// inicia histograma com zeros
	Mat1d histograma2d = Mat1d(Size(mapa.cols, 360), double(0));
	Mat1d histogramaMax2d = Mat1d(Size(mapa.cols, 360), double(0));

	// calcula o histograma
	for (HoughLine h : _houghs) {
		HoughLine hIPM = h.toIPM(this->config);
		if (hIPM.p1.x < 0 || hIPM.p1.x >= mapa.cols) continue;

		// from: http://stackoverflow.com/a/4073700/4228275
		// retorna o n�mero arredondado pra cima com base num fator
		// ex.: f(86, 5) = 85
		//      f(84, 5) = 85
		auto f = [](double num, int factor) {
			if (factor == 0) return num;
			return static_cast<double>(round(static_cast<double>(num) / static_cast<double>(factor))*static_cast<double>(factor));
		};

		int y = (int)f(hIPM.getAngulo(), _binsize_angulo);
		int x = (int)hIPM.p1.x;
		x -= x % _binsize_posicao;

		double n = contarEvidencasEmbaixoDaHoughIPM(hIPM, mapa, 3);
		
		if (n>0 && n > histogramaMax2d.at<double>(Point(x, y))) {
			histograma2d.at<double>(Point(x, y)) = n;
			histogramaMax2d.at<double>(Point(x, y)) = n;

			outHoughsHistograma2D[Point(x, y)].clear();
			outHoughsHistograma2D[Point(x, y)].push_back(h);
		}		
	}

	if (normalizar) cv::normalize(histograma2d, histograma2d, 0.0, 1.0, NORM_MINMAX);

	// embora o bin n�o seja a posi��o correta, dado o deslocamento causado pelo tamanho deste,
	// nesse caso a posi��o do bin n�o interfere, dado que cada bin tem a hough vinculado a ele e
	// a hough possui sua posi��o independentemente da do bin

	return histograma2d;
}

Mat1d AnaliseDasHoughs::montarHistograma1D(const Mat1d &histograma2d, map<Point, vector<HoughLine>, lessThanPoint> &houghsHistograma2D, HoughDoMeio *houghsAnteriores, map<int, vector<HoughLine>> &outHoughsHistograma1D, int laneSide, bool normalizar) {

	vector<HoughLine> _houghsAnteriores = { HoughLine::empty(), HoughLine::empty() };
	if (houghsAnteriores != NULL) _houghsAnteriores = (*houghsAnteriores).toVetorDeHoughs(config->roi, config->ipm);
	HoughLine *houghAnterior = (laneSide == LANE_LEFT) ? &_houghsAnteriores[0] : &_houghsAnteriores[1];
	const double anguloHoughAnterior = (*houghAnterior).toIPM(config).getAngulo();

	// varre as colunas para pegar a melhor
	Mat1d histograma1d = Mat1d(Size(histograma2d.cols, 1), double(0));
	for (int i = 0; i < histograma2d.cols; i++) {
		// varre as linhas da coluna (houghs com angulos diferentes)
		for (int j = 0; j < histograma2d.rows; j++) {
			double n = histograma2d.at<double>(Point(i, j));
			if (n>0 && n >= histograma1d.at<double>(i)) {
				// se o angulo da hough � maior que a maior atual
				if (n > histograma1d.at<double>(i)) {
					histograma1d.at<double>(i) = n;
					outHoughsHistograma1D[i] = houghsHistograma2D[Point(i, j)];
				// sen�o, se � igual, verifica qual � mais pr�xima ao angulo da anterior, se dispon�vel
				} else {
					if (houghsAnteriores != NULL && houghsHistograma2D[Point(i, j)].size() > 0) {
						// calcula a diferen�a de angulo entre a m�xima e a hough do kalman
						double distMax = abs(outHoughsHistograma1D[i][0].toIPM(config).getAngulo() - anguloHoughAnterior);
						double distH = abs(houghsHistograma2D[Point(i, j)][0].toIPM(config).getAngulo() - anguloHoughAnterior);
						if (distH <= distMax) {
							histograma1d.at<double>(i) = n;
							outHoughsHistograma1D[i] = houghsHistograma2D[Point(i, j)];
						}
					}
				}
			}
		}
	}

	// normaliza, caso queira
	if (normalizar) cv::normalize(histograma1d, histograma1d, 0.0, 1.0, NORM_MINMAX);

	return histograma1d;
}

Mat1d AnaliseDasHoughs::montarHistograma(const vector<HoughLine> &_houghs, int _tamanho, int _tamanho_bin, bool normalizar) {
	
	// inicia histograma com zeros
	Mat1d out = Mat1d(Size(_tamanho, 1), double(0));

	// calcula o histograma
	for (HoughLine h : _houghs) {
		HoughLine hIPM = h.toIPM(this->config);
		if (hIPM.p1.x < 0 || hIPM.p1.x >= _tamanho) continue;
		int x = (int)hIPM.p1.x;
		x -= x % _tamanho_bin;
		out.at<double>(x) += 1;
	}
	
	// move o histograma para a direita em bin_size / 2 porque o bin est� acumulando no 
	int shift = _tamanho_bin / 2;
	for (int i = _tamanho - 1; i >= 0; i--) out.at<double>(i) = (i < shift) ? 0 : out.at<double>(i - shift);

	// normaliza, caso queira
	if (normalizar) cv::normalize(out, out, 0.0, 1.0, NORM_MINMAX);

	return out;
}

Mat1d AnaliseDasHoughs::montarHistograma(const vector<HoughLine> &_houghs, const Mat1b &mapa, int _tamanho, int _tamanho_bin, bool normalizar) {
	// inicia histograma com zeros
	Mat1d out = Mat1d(Size(_tamanho, 1), double(0));

	// caso n�o tenha houghs
	if (_houghs.size() == 0) return out;

	// calcula o histograma
	for (HoughLine h : _houghs) {
		HoughLine hIPM = h.toIPM(this->config);
		if (hIPM.p1.x < 0 || hIPM.p1.x >= _tamanho) continue;
		int x = (int)hIPM.p1.x;
		x -= x % _tamanho_bin;
		out.at<double>(x) += contarEvidencasEmbaixoDaHoughIPM(hIPM, mapa, 0);
	}

	// move o histograma para a direita em bin_size / 2 porque o bin est� acumulando no 
	int shift = _tamanho_bin / 2;
	for (int i = _tamanho - 1; i >= 0; i--) out.at<double>(i) = (i < shift) ? 0 : out.at<double>(i - shift);

	// normaliza, caso queira
	if (normalizar) cv::normalize(out, out, 0.0, 1.0, NORM_MINMAX);

	return out;
}

int AnaliseDasHoughs::contarEvidencasEmbaixoDaHoughIPM(const HoughLine &hough, const Mat1b &mapaIPM, const int regiaoBusca) {
	int nEvidencias = 0;

	// caso a hough seja vazia
	if (hough.isEmpty()) return nEvidencias;

	// conta embaixo das houghs
	// apenas na metade inferior da imagem
	for (int i = mapaIPM.rows / 2; i < mapaIPM.rows; i++) {
		int p1_x = (int)hough.getX(i);
		for (int j = 0; j <= regiaoBusca; j++) {
			Point p1esq = Point(p1_x - j, i);
			Point p1dir = Point(p1_x + j, i);

			bool p1esqDentro = (p1esq.x >= 0 && p1esq.x < mapaIPM.cols);
			bool p1dirDentro = (p1dir.x >= 0 && p1dir.x < mapaIPM.cols);

			if ((p1esqDentro && mapaIPM.at<uchar>(p1esq) != 0) || (p1dirDentro && mapaIPM.at<uchar>(p1dir) != 0)) {
				nEvidencias += regiaoBusca - j;
				break;
			}
		}
	}
	return nEvidencias;
}

void AnaliseDasHoughs::desenhaHistograma(const Mat1d &histograma, Mat3b &imagem, const Scalar &cor) {
	
	if (!config->display) return;

	for (int i = 1; i < imagem.cols; i++) {
		line(imagem, Point(i - 1, (int)(imagem.rows - (histograma.at<double>(i - 1) * imagem.rows))), Point(i, (int)(imagem.rows - (histograma.at<double>(i) * imagem.rows))), cor);
	}

}

void AnaliseDasHoughs::encontraMaximoNosHistogramas(const Mat1d &esqHistograma, const Mat1d &dirHistograma, int *esqHistogramaMaxX, int *dirHistogramaMaxX) {
	// acha o max dos histogramas de cada lado
	int esqMaxIdx = -1;
	double esqMaxValue = 0;
	for (int i = 0; i < esqHistograma.cols; i++) {
		if (esqHistograma.at<double>(i) > esqMaxValue) {
			esqMaxValue = esqHistograma.at<double>(i);
			esqMaxIdx = i;
		}
	}
	int dirMaxIdx = -1;
	double dirMaxValue = 0;
	for (int i = 0; i < dirHistograma.cols; i++) {
		if (dirHistograma.at<double>(i) > dirMaxValue) {
			dirMaxValue = dirHistograma.at<double>(i);
			dirMaxIdx = i;
		}
	}

	*esqHistogramaMaxX = esqMaxIdx;
	*dirHistogramaMaxX = dirMaxIdx;

}

void AnaliseDasHoughs::selecionarHoughs(const vector<HoughLine> &houghs, int esqHistogramaMaxX, int dirHistogramaMaxX, int houghArea, vector<HoughLine> *esq, vector<HoughLine> *dir) {
	// [houghArea -- x -- houghArea], x = maxHistX
	for (HoughLine h : houghs) {

		int hough_x = (int)h.p1.x;
		Point houghIPM = config->ipm->applyHomography(Point(hough_x, config->roi.height));

		int distEsq = esqHistogramaMaxX - houghIPM.x;
		int distDir = dirHistogramaMaxX - houghIPM.x;

		if (abs(distEsq) < houghArea) esq->push_back(h);
		if (abs(distDir) < houghArea) dir->push_back(h);
	}
}

void AnaliseDasHoughs::selecionarHoughs(const vector<HoughLine> &houghs, vector<HoughLine> *esq, vector<HoughLine> *dir) {
	// [houghArea -- x -- houghArea], x = maxHistX
	for (HoughLine h : houghs) {

		HoughLine hIPM = h.toIPM(config);

		// diferen�a da base da hough para a posi��o do carro
		int diff = static_cast<int>(hIPM.p1.x - posicaoCarroIPM.x);

		// se a diferen�a for positiva, a hough est� a direita, sen�o a esquerda
		if (diff < 0) esq->push_back(h);
		else dir->push_back(h);
	}
}

void AnaliseDasHoughs::selecionarHoughsDouble(const vector<HoughLine> &houghs, double _x, int houghArea, vector<HoughLine> *out) {
	// [houghArea -- x -- houghArea], x = maxHistX
	for (HoughLine h : houghs) {
		int hough_x = (int)h.p1.x;
		Point houghIPM = config->ipm->applyHomography(Point(hough_x, config->roi.height));
		int dist = (int)(_x - houghIPM.x);
		if (abs(dist) < houghArea) out->push_back(h);
	}
}

int AnaliseDasHoughs::encontrarMaximoMaisProximo(const Mat1d &histograma, int direcao, int posicaoCarroX, bool estaNormalizado) {
	
	const int tamanho = (direcao > 0) ? histograma.cols - posicaoCarroX : posicaoCarroX;
	double max = -1;
	int maxIdx = -1;

	for (int i = 0; i < tamanho; i++) {
		// pega a posicao e o valor que est�o sendo analisados
		int posicao = posicaoCarroX + (i * direcao);
		double valor = histograma.at<double>(posicao);
		// verifica qual � o maior
		if (estaNormalizado && valor == 1) return posicao;
		// s� > para pegar o primeiro valor encontrado, seguindo a dire��o desejada
		if (histograma.at<double>(posicao) > max) {
			max = histograma.at<double>(posicao);
			maxIdx = posicao;
		}
	}

	return maxIdx;
}

void AnaliseDasHoughs::pegarMedianaNaRegiao(const vector<HoughLine> &houghs, int posicaoBase, int tamanhoRegiao, HoughLine &outHoughMediana) {
	vector<HoughLine> houghsRegiao;

	if (houghsRegiao.size() == 1) {
		outHoughMediana = houghs[0];
		return;
	}

	// seleciona as houghs que est�o dentro da regi�o desejada
	for (HoughLine h : houghs) {
		HoughLine hIPM = h.toIPM(this->config);
		int x = (int)hIPM.p1.x;
		int dist = abs(x - posicaoBase);
		if (dist <= tamanhoRegiao) houghsRegiao.push_back(h);
	}

	if (houghsRegiao.size() > 0) {
		// ordena para pegar a mediana
		std::sort(houghsRegiao.begin(), houghsRegiao.end(), OrdenaHough(config));
		// pega a mediana
		int mediana = (int)floor(houghsRegiao.size() / 2);
		outHoughMediana = houghsRegiao[mediana];
	}
}

double AnaliseDasHoughs::mostrarDesvioDoCentro(Mat &displayColorFrame, const HoughDoMeio &measurement, double xCarro) {
	
	const double larguraImagem = displayColorFrame.cols;
	const double larguraIndicador = larguraImagem / 2.0;
	const int alturaIndicador = 20;

	double distancia = measurement.xBase - xCarro;
	double percentualDistancia = (abs(distancia) / (measurement.largura / 2.0));

	const double tamanhoMaximo = larguraIndicador / 2.0;
	if (distancia < 0) {
		Rect indicador = Rect((int)(larguraImagem / 2.0), 15, (int)(tamanhoMaximo * percentualDistancia), alturaIndicador);
		rectangle(displayColorFrame, indicador, Scalar(0,255,0), -1);
	} else {
		int tam = (int)(tamanhoMaximo * percentualDistancia);
		Rect indicador = Rect((int)((larguraImagem / 2.0) - tam), 15, tam, alturaIndicador);
		rectangle(displayColorFrame, indicador, Scalar(0, 255, 0), -1);
	}

	Rect bordaIndicador = Rect((int)((larguraImagem / 2.0) - tamanhoMaximo), 15, (int)larguraIndicador, alturaIndicador);
	rectangle(displayColorFrame, bordaIndicador, Scalar(127, 127, 127));
	
	return measurement.xBase;
}
