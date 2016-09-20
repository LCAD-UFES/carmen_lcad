#include "PreProcessamento.h"

using namespace std;
using namespace cv;

PreProcessamento::PreProcessamento(const Rect &_regiaoDeInteresse, IPM *_ipm, bool _verbose, bool _display) {
	regiaoDeInteresse = _regiaoDeInteresse;
	ipm = _ipm;
	verbose = _verbose;
	display = _display;
}

Mat1b PreProcessamento::executar(const Mat3b &colorFrame, Mat1b &outGrayFrameRoi, IPM *_ipm) {

	// inicializa variáveis
	Mat1b outGrayFrameRoiIPM;

	// converte a imagem para escala de cinza
	cvtColor(colorFrame, outGrayFrameRoi, CV_BGR2GRAY);

	// pega somente a região de interesse
	outGrayFrameRoi = outGrayFrameRoi(regiaoDeInteresse);

	// aplica IPM na imagem
	IPM * __ipm = (_ipm == nullptr) ? ipm : _ipm;
	__ipm->applyHomography(outGrayFrameRoi, outGrayFrameRoiIPM);

	// exibe as saídas definidas (texto e/ou imagem)
	if (display) view(colorFrame, outGrayFrameRoiIPM);

	return outGrayFrameRoiIPM;
}

Mat1b PreProcessamento::executar(const Mat3b &colorFrame, IPM *_ipm) {
	
	// aplica IPM na região de interesse da imagem
	Mat3b outGrayFrameRoiIPM;
	IPM * __ipm = (_ipm == nullptr) ? ipm : _ipm;
	__ipm->applyHomography(colorFrame(regiaoDeInteresse), outGrayFrameRoiIPM);

	// exibe as saídas definidas (texto e/ou imagem)
	if (display) view(colorFrame, outGrayFrameRoiIPM);

	return outGrayFrameRoiIPM;
}

void PreProcessamento::view(const Mat3b &entrada, const Mat1b &saida) {

	// define as dimensões da imagem final e as áreas onde as imagens serão colocadas
	Mat imagemComposta = Mat(entrada.rows, entrada.cols, CV_8UC3);
	Rect areaImagemEntrada = Rect(0, 0, entrada.cols, entrada.rows);
	Rect areaImagemSaida = Rect(0, 0, saida.cols, saida.rows);

	// copia as imagens para suas posições
	entrada.copyTo(imagemComposta(areaImagemEntrada));
	cvtColor(saida, imagemComposta(areaImagemSaida), CV_GRAY2RGB);

	// desenha a região de interesse
	rectangle(imagemComposta, regiaoDeInteresse, Scalar(0, 0, 255), 1);

	// mostra o resultado
	imshow("Pré-processamento", imagemComposta);

}

vector<Point> PreProcessamento::getVerticesIPM() {

	Point p1 = Point(0, 0);
	Point p2 = Point(regiaoDeInteresse.width, 0);
	Point p3 = Point(regiaoDeInteresse.width, regiaoDeInteresse.height);
	Point p4 = Point(0, regiaoDeInteresse.height);

	vector<Point> vertices;
	vertices.push_back(ipm->applyHomography(p1));
	vertices.push_back(ipm->applyHomography(p2));
	vertices.push_back(ipm->applyHomography(p3));
	vertices.push_back(ipm->applyHomography(p4));

	return vertices;
}

Mat1b PreProcessamento::getMascaraIPM(int _dilateSize) {

	Mat1b mascara = Mat1b(this->regiaoDeInteresse.size(), uchar(0));
	vector<Point> verticesIPM = this->getVerticesIPM();

	Point vertices[1][4];
	vertices[0][0] = verticesIPM[0];
	vertices[0][1] = verticesIPM[1];
	vertices[0][2] = verticesIPM[2];
	vertices[0][3] = verticesIPM[3];

	const Point* ppt[1] = { vertices[0] };
	int npt[] = { static_cast<int>(verticesIPM.size()) };

	fillPoly(mascara, ppt, npt, 1, Scalar(255));
	
	if (_dilateSize != 0) {
		int absDilateSize = abs(_dilateSize);
		Mat1b kernel = getStructuringElement(MORPH_ELLIPSE, Size(2 * absDilateSize + 1, 2 * absDilateSize + 1), Point(absDilateSize, absDilateSize));
		
		if (_dilateSize > 0) dilate(mascara, mascara, kernel);
		else erode(mascara, mascara, kernel);
	}

	return mascara;

}