#include "nieto.h"
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;


Nieto::Nieto(bool _verbose, bool _display) {
	verbose = _verbose;
	display = _display;
}

void Nieto::nietoLaneMarkingsDetector(Mat1b &srcGRAY, Mat1b &dstGRAY, int tauInicio, int tauFim) {

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
	
Mat1b Nieto::filtro(const Mat3b &inColorFrame, int tauInicio, int tauFim, int thres) {
	double tempoInicio = static_cast<double>(getTickCount());

	Mat1b outBinaryFrameFiltrado = Mat1b(inColorFrame.size());
	// converte para cinza
	cvtColor(inColorFrame, outBinaryFrameFiltrado, CV_BGR2GRAY);
	// aplica o filtro do Nieto
	nietoLaneMarkingsDetector(outBinaryFrameFiltrado, outBinaryFrameFiltrado, tauInicio, tauFim);
	// aplica o threshold na imagem filtrada
	threshold(outBinaryFrameFiltrado, outBinaryFrameFiltrado, thres, 255, CV_THRESH_BINARY);

	// calcula o tempo de execu��o
	double tempoFim = static_cast<double>(getTickCount());
	double tempoExecutando = ((tempoFim - tempoInicio) / getTickFrequency()) * 1000;

	// exibe as sa�das definidas (texto e/ou imagem)
	if (verbose) cout << "- nieto.filtro: " << tempoExecutando << " ms" << endl;

	return outBinaryFrameFiltrado;
}

Mat1b Nieto::ipm(const Mat1b &inBinaryFrameFiltrado, const Rect &roi, IPM * ipm) {
	Mat1b outBinaryFrameFiltradoRoiIPM;

	// aplica IPM
	ipm->applyHomography(inBinaryFrameFiltrado(roi), outBinaryFrameFiltradoRoiIPM);
	// aplica um threshold para manter a imagem bin�ria, pois a IPM causa uma 'interpola��o'
	threshold(outBinaryFrameFiltradoRoiIPM, outBinaryFrameFiltradoRoiIPM, 1, 255, CV_THRESH_BINARY);

	return outBinaryFrameFiltradoRoiIPM;
}

vector<vector<vector<Point> > > Nieto::vp(const Mat1b &inBinaryFrameFiltrado, Mat3b &inVanishingPointImage, const Rect &roi, const int maxNumLines, int houghThresholdInicial, int houghStep, double houghMinLineLength, double houghMaxLineGap) {

	double tempoInicio = static_cast<double>(getTickCount());

	// pega somente a regi�o de interesse
	Mat1b mascaraRoi = Mat1b(inBinaryFrameFiltrado.size(), uchar(0));
	mascaraRoi(Rect(roi.x, roi.y, roi.width, roi.height)).setTo(255);
	Mat1b binaryFrameFiltradoMasked;
	cv::bitwise_and(inBinaryFrameFiltrado, mascaraRoi, binaryFrameFiltradoMasked);
	if (display) cvtColor(inBinaryFrameFiltrado, inVanishingPointImage, CV_GRAY2BGR);

	// Hough
	vector<vector<Point> > lineSegments;
	vector<Point> aux;
	vector<Vec4i> lines;
	int houghThreshold = houghThresholdInicial;

	cv::HoughLinesP(binaryFrameFiltradoMasked, lines, 1, CV_PI / 180, houghThreshold, houghMinLineLength, houghMaxLineGap);
	while (lines.size() > maxNumLines) {
		lines.clear();
		houghThreshold += houghStep;
		cv::HoughLinesP(binaryFrameFiltradoMasked, lines, 1, CV_PI / 180, houghThreshold, houghMinLineLength, houghMaxLineGap);
	}
	for (size_t i = 0; i<lines.size(); i++)	{
		Point pt1, pt2;
		pt1.x = lines[i][0];
		pt1.y = lines[i][1];
		pt2.x = lines[i][2];
		pt2.y = lines[i][3];

		// modificado
		int dx = abs(pt2.x - pt1.x);
		int dy = abs(pt2.y - pt1.y);
		if (1.0*dx / 3.0 > dy) {
			if (display) line(inVanishingPointImage, pt1, pt2, CV_RGB(0, 0, 255), 1);
			// continue;
		} else{
			if (display) line(inVanishingPointImage, pt1, pt2, CV_RGB(0, 255, 0), 1);
		}

		// Store into vector of pairs of Points for msac
		aux.clear();
		aux.push_back(pt1);
		aux.push_back(pt2);
		lineSegments.push_back(aux);
	}

	// Multiple vanishing points
	vector<Mat> vps;			// vector of vps: vps[vpNum], with vpNum=0...numDetectedVps
	vector<vector<int> > CS;	// index of Consensus Set for all vps: CS[vpNum] is a vector containing indexes of lineSegments belonging to Consensus Set of vp numVp
	vector<int> numInliers;
	vector<vector<vector<Point> > > lineSegmentsClusters;
	MSAC msac;
	bool msac_verbose = false;
	int numVps = 1;

	msac.init(MODE_NIETO, inBinaryFrameFiltrado.size(), msac_verbose);
	msac.multipleVPEstimation(lineSegments, lineSegmentsClusters, numInliers, vps, numVps); // Call msac function for multiple vanishing point estimation
	msac.drawCS(inVanishingPointImage, lineSegmentsClusters, vps); // Draw line segments according to their cluster

	// sa�das
	this->houghLines = lineSegmentsClusters;
	this->vanishingPoint = vps;

	// calcula o tempo de execu��o
	double tempoFim = static_cast<double>(getTickCount());
	double tempoExecutando = ((tempoFim - tempoInicio) / getTickFrequency()) * 1000;

	// exibe as sa�das definidas (texto e/ou imagem)
	if (verbose) cout << "- nieto.vp: " << tempoExecutando << " ms" << endl;
	if (display) imshow("Vanishing Point", inVanishingPointImage);

	return { lineSegments };
}

void Nieto::featureI(const Mat1b &inBinaryFrameFiltradoRoiIPM, const Mat1b &grayFrameRoiIPM, map<string, double> &outMeans0, map<string, double> &outCovs0, map<string, double> &outWeights0, bool aplicarSobel) {
	
	double tempoInicio = static_cast<double>(getTickCount());
	
	Size imgSize = inBinaryFrameFiltradoRoiIPM.size();
	Mat1b binaryFrameFiltradoRoiIPMDilated;
	if (aplicarSobel) {
		// aplica o Sobel
		Mat1b grayFrameRoiIPMSobel, grayFrameRoiIPMSobelX, grayFrameRoiIPMSobelY;
		Sobel(grayFrameRoiIPM, grayFrameRoiIPMSobelX, CV_8U, 1, 0);
		Sobel(grayFrameRoiIPM, grayFrameRoiIPMSobelY, CV_8U, 0, 1);
		addWeighted(grayFrameRoiIPMSobelX, 0.5, grayFrameRoiIPMSobelY, 0.5, 0, grayFrameRoiIPMSobel);
		Mat1b binaryFrameRoiIPMSobel;
		threshold(grayFrameRoiIPMSobel, binaryFrameRoiIPMSobel, 20, 255, THRESH_BINARY);
		imshow("grayFrameRoiIPMSobel", binaryFrameRoiIPMSobel);

		// aplica um close para remover as sujeiras
		erode(binaryFrameRoiIPMSobel, binaryFrameRoiIPMSobel, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)), Point(-1, -1));
		dilate(binaryFrameRoiIPMSobel, binaryFrameRoiIPMSobel, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)), Point(-1, -1));

		// dilata para formar a m�scara
		int morph_size = 5;
		Mat1b dilate_kernel = getStructuringElement(MORPH_ELLIPSE, Size(1 * morph_size + 1, 1 * morph_size + 1), Point(morph_size, morph_size));
		dilate(binaryFrameRoiIPMSobel, binaryFrameFiltradoRoiIPMDilated, dilate_kernel, Point(-1, -1), 3);
		imshow("binaryFrameFiltradoRoiIPMDilated", binaryFrameFiltradoRoiIPMDilated);
	} else {
		// aplica o dilate no filtro do nieto na IPM
		int morph_size = 5;
		Mat1b dilate_kernel = getStructuringElement(MORPH_ELLIPSE, Size(1 * morph_size + 1, 1 * morph_size + 1), Point(morph_size, morph_size));
		dilate(inBinaryFrameFiltradoRoiIPM, binaryFrameFiltradoRoiIPMDilated, dilate_kernel, Point(-1, -1), 3);
	}

	// Imagens que ser�o constru�das
	// - gray{i} = grayMasked
	// - binary{i} = bin�rio da gray{i}
	Mat1b grayPavement = Mat1b(imgSize, uchar(0));
	Mat1b binaryPavement = Mat1b(imgSize, uchar(0));
	Mat1b grayMarkings = Mat1b(imgSize, uchar(0));
	Mat1b binaryMarkings = Mat1b(imgSize, uchar(0));
	Mat1b grayObjects = Mat1b(imgSize, uchar(0));
	Mat1b binaryObjects = Mat1b(imgSize, uchar(0));
	Mat1b grayUnknown = Mat1b(imgSize, uchar(0));
	Mat1b binaryUnknown = Mat1b(imgSize, uchar(0));

	// valores iniciais das gaussianas
	map<string, Scalar> meansScalar, stddevScalar, weightsScalar;

	// somente o que � imagem, na IPM, ficar� branco
	Mat1b binaryIpmInvMask = Mat1b(imgSize);
	cv::bitwise_not(grayFrameRoiIPM == 0, binaryIpmInvMask);

	// ************************************** PAVEMENT
	// calcula as m�scaras
	cv::bitwise_not(binaryFrameFiltradoRoiIPMDilated, binaryPavement);
	cv::bitwise_and(binaryIpmInvMask, binaryPavement, binaryPavement);
	cv::bitwise_and(grayFrameRoiIPM, binaryPavement, grayPavement);
	// calcula a m�dia e covari�ncia do pavimento
	cv::meanStdDev(grayFrameRoiIPM, meansScalar["pavement"], stddevScalar["pavement"], binaryPavement);
	outMeans0["pavement"] = meansScalar["pavement"][0];
	outCovs0["pavement"] = stddevScalar["pavement"][0] * stddevScalar["pavement"][0];
	
	// ************************************** LANE MARKINGS
	// calcula as m�scaras
	grayFrameRoiIPM.copyTo(grayMarkings);
	double thresMarkings = outMeans0["pavement"] + 3 * stddevScalar["pavement"][0];
	binaryMarkings = grayMarkings > thresMarkings;
	cv::bitwise_and(grayFrameRoiIPM, binaryMarkings, grayMarkings);
	// calcula a m�dia e covari�ncia dos lane markings
	cv::meanStdDev(grayFrameRoiIPM, meansScalar["markings"], stddevScalar["markings"], binaryMarkings);
	outMeans0["markings"] = meansScalar["markings"][0];
	outCovs0["markings"] = stddevScalar["markings"][0] * stddevScalar["markings"][0];
		
	// ************************************** OBJECTS
	// calcula as m�scaras
	grayFrameRoiIPM.copyTo(grayObjects);
	double thresObjects = outMeans0["pavement"] - 3 * stddevScalar["pavement"][0];
	binaryObjects = grayObjects < thresObjects;
	cv::bitwise_and(binaryIpmInvMask, binaryObjects, binaryObjects);
	cv::bitwise_and(grayFrameRoiIPM, binaryObjects, grayObjects);
	// calcula a m�dia e covari�ncia dos objetos
	cv::meanStdDev(grayFrameRoiIPM, meansScalar["objects"], stddevScalar["objects"], binaryObjects);
	outMeans0["objects"] = meansScalar["objects"][0];
	outCovs0["objects"] = stddevScalar["objects"][0] * stddevScalar["objects"][0];
		
	// ************************************** UNKNOWN
	// calcula as m�scaras
	cv::bitwise_or(binaryUnknown, binaryPavement, binaryUnknown);
	cv::bitwise_or(binaryUnknown, binaryObjects, binaryUnknown);
	cv::bitwise_or(binaryUnknown, binaryMarkings, binaryUnknown);
	cv::bitwise_not(binaryUnknown, binaryUnknown, binaryIpmInvMask);
	cv::bitwise_and(grayFrameRoiIPM, binaryUnknown, grayUnknown);
	// calcula a m�dia e covari�ncia dos desconhecidos
	cv::meanStdDev(grayFrameRoiIPM, meansScalar["unknown"], stddevScalar["unknown"], binaryUnknown);
	outMeans0["unknown"] = meansScalar["unknown"][0];
	outCovs0["unknown"] = stddevScalar["unknown"][0] * stddevScalar["unknown"][0];

	// calcula os pesos iniciais
	double nPavements = countNonZero(binaryPavement);
	double nMarkings = countNonZero(binaryMarkings);
	double nObjects = countNonZero(binaryObjects);
	double nUnknown = countNonZero(binaryUnknown);
	double nTotal = (nPavements + nMarkings + nObjects + nUnknown);
	outWeights0["pavement"] = nPavements / nTotal;
	outWeights0["markings"] = nMarkings / nTotal;
	outWeights0["objects"] = nObjects / nTotal;
	outWeights0["unknown"] = nUnknown / nTotal;

	// calcula o tempo de execu��o
	double tempoFim = static_cast<double>(getTickCount());
	double tempoExecutando = ((tempoFim - tempoInicio) / getTickFrequency()) * 1000;

	// exibe as sa�das definidas (texto e/ou imagem)
	if (verbose) cout << "- nieto.featureI: " << tempoExecutando << " ms" << endl;
	if (display) {
		Mat1b imgResultNietoMasks = Mat1b(Size(grayFrameRoiIPM.cols, grayFrameRoiIPM.rows * 4), uchar(0));

		grayPavement.copyTo(imgResultNietoMasks(Rect(0, 0, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)));
		grayMarkings.copyTo(imgResultNietoMasks(Rect(0, grayFrameRoiIPM.rows, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)));
		grayObjects.copyTo(imgResultNietoMasks(Rect(0, 2 * grayFrameRoiIPM.rows, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)));

		imgResultNietoMasks(Rect(0, 3 * grayFrameRoiIPM.rows, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)).setTo(85, binaryPavement);
		imgResultNietoMasks(Rect(0, 3 * grayFrameRoiIPM.rows, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)).setTo(170, binaryObjects);
		imgResultNietoMasks(Rect(0, 3 * grayFrameRoiIPM.rows, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)).setTo(255, binaryMarkings);
		imgResultNietoMasks(Rect(0, 3 * grayFrameRoiIPM.rows, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)).setTo(0, binaryUnknown);

		imshow("Nieto - Mascaras", imgResultNietoMasks);

		// imshow("binaryIpmInvMask", binaryIpmInvMask);

		// descomente para visualizar o que foi calculado para o Pavement
		// imshow("grayPavement", grayPavement);
		// imshow("binaryPavement", binaryPavement);
		// cout << "p.mean: " << means0["pavement"] << ", p.covs: " << covs0["pavement"] << endl;

		// descomente para visualizar o que foi calculado para os Lane Markings
		// imshow("grayMarkings", grayMarkings);
		// imshow("binaryMarkings", binaryMarkings);
		// cout << "lm.mean: " << means0["markings"] << ", lm.covs: " << covs0["markings"] << endl;

		// descomente para visualizar o que foi calculado para os Objects
		// imshow("grayObjects", grayObjects);
		// imshow("binaryObjects", binaryObjects);
		// cout << "obj.mean: " << means0["objects"] << ", obj.covs: " << covs0["objects"] << endl;

		// descomente para visualizar o que foi calculado para o Unknown
		// imshow("grayUnknown", grayUnknown);
		// imshow("binaryUnknown", binaryUnknown);
		// cout << "unk.mean: " << means0["unknown"] << ", unk.covs: " << covs0["unknown"] << endl;

		// descomente para imprimir o peso inicial
		// cout << "w[pav]: " << outWeights0["pavement"] << endl;
		// cout << "w[lnm]: " << outWeights0["markings"] << endl;
		// cout << "w[obj]: " << outWeights0["objects"] << endl;
		// cout << "w[unk]: " << outWeights0["unknown"] << endl;
	}
}

double Nieto::getMean(const Mat1b &_image, const Mat1b &_mask) {
	double mean = 0;
	for (int j = 0; j < _image.rows; ++j) {
		for (int i = 0; i < _image.cols; ++i) {
			if (_mask.at<uchar>(Point(i, j)) > 0) mean += _image.at<uchar>(Point(i, j));
		}
	}
	mean /= countNonZero(_mask);
	return mean;
}

double Nieto::getVariance(const Mat1b &_image, double mean, const Mat1b &_mask) {
	double standard_deviation = 0;
	for (int j = 0; j < _image.rows; ++j) {
		for (int i = 0; i < _image.cols; ++i) {
			if (_mask.at<uchar>(Point(i, j)) > 0) standard_deviation += ((_image.at<uchar>(Point(i, j)) - mean) * (_image.at<uchar>(Point(i, j)) - mean));
		}
	}
	standard_deviation /= countNonZero(_mask);
	return (standard_deviation * standard_deviation);
}

void Nieto::featureL(const Mat1b &inGrayFrameFiltradoRoi, map<string, double> &outMeans0, map<string, double> &outCovs0, map<string, double> &outWeights0) {

	double tempoInicio = static_cast<double>(getTickCount());

	Size imgSize = inGrayFrameFiltradoRoi.size();

	// Imagens que ser�o constru�das
	Mat1b binaryPavement = Mat1b(imgSize, uchar(0));
	Mat1b binaryMarkings = Mat1b(imgSize, uchar(0));

	// pega o threshold inicial que separa as duas gaussianas (Pavement < thres < LaneMarkings)
	// Nieto: A reasonable threshold that separates these two components is the standard deviation
	Scalar meanFiltro, stddevFiltro;
	cv::meanStdDev(inGrayFrameFiltradoRoi, meanFiltro, stddevFiltro);

	// seta as m�scaras dos elementos que pertecem a ambas as classes
	binaryMarkings = inGrayFrameFiltradoRoi >  stddevFiltro[0];
	binaryPavement = inGrayFrameFiltradoRoi <= stddevFiltro[0];

	map<string, Scalar> meansScalar, stddevScalar;
	// ************************************** PAVEMENT
	cv::meanStdDev(inGrayFrameFiltradoRoi, meansScalar["pavement"], stddevScalar["pavement"], binaryPavement);
	outMeans0["pavement"] = getMean(inGrayFrameFiltradoRoi, binaryPavement);
	outCovs0["pavement"] = getVariance(inGrayFrameFiltradoRoi, outMeans0["pavement"], binaryPavement);

	// ************************************** LANE MARKINGS
	cv::meanStdDev(inGrayFrameFiltradoRoi, meansScalar["markings"], stddevScalar["markings"], binaryMarkings);
	outMeans0["markings"] = getMean(inGrayFrameFiltradoRoi.clone(), binaryMarkings);
	outCovs0["markings"] = getVariance(inGrayFrameFiltradoRoi.clone(), outMeans0["markings"], binaryMarkings);

	// ************************************** OBJECTS
	outMeans0["objects"] = outMeans0["pavement"];
	outCovs0["objects"] = outCovs0["pavement"];

	// ************************************** UNKNOWN
	outMeans0["unknown"] = 255.0 / 2.0;
	outCovs0["unknown"] = ((255.0 / 2.0) / sqrt(3)) * ((255.0 / 2.0) / sqrt(3));

	// calcula os pesos => propor��o de cada classe
	double nPavements = countNonZero(binaryPavement);
	double nMarkings = countNonZero(binaryMarkings);
	double nTotal = nPavements + nMarkings;
	double nUnknown = nTotal * 0.05;
	nTotal += nUnknown;
	outWeights0["pavement"] = (nPavements / 2) / nTotal;
	outWeights0["objects"] = outWeights0["pavement"];
	outWeights0["markings"] = nMarkings / nTotal;
	outWeights0["unknown"] = nUnknown / nTotal;

	// calcula o tempo de execu��o
	double tempoFim = static_cast<double>(getTickCount());
	double tempoExecutando = ((tempoFim - tempoInicio) / getTickFrequency()) * 1000;

	// exibe as sa�das definidas (texto e/ou imagem)
	if (verbose) cout << "- nieto.featureL: " << tempoExecutando << " ms" << endl;

	if (display)
	imshow("L - binaryMarkings", binaryMarkings);
	imshow("L - binaryPavement", binaryPavement);


	/*
	if (display) {
		Mat1b imgResultNietoMasks = Mat1b(Size(grayFrameRoiIPM.cols, grayFrameRoiIPM.rows * 4), uchar(0));

		grayPavement.copyTo(imgResultNietoMasks(Rect(0, 0, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)));
		grayMarkings.copyTo(imgResultNietoMasks(Rect(0, grayFrameRoiIPM.rows, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)));
		grayObjects.copyTo(imgResultNietoMasks(Rect(0, 2 * grayFrameRoiIPM.rows, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)));

		imgResultNietoMasks(Rect(0, 3 * grayFrameRoiIPM.rows, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)).setTo(85, binaryPavement);
		imgResultNietoMasks(Rect(0, 3 * grayFrameRoiIPM.rows, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)).setTo(170, binaryObjects);
		imgResultNietoMasks(Rect(0, 3 * grayFrameRoiIPM.rows, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)).setTo(255, binaryMarkings);
		imgResultNietoMasks(Rect(0, 3 * grayFrameRoiIPM.rows, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)).setTo(0, binaryUnknown);

		imshow("Nieto - Mascaras", imgResultNietoMasks);

		// imshow("binaryIpmInvMask", binaryIpmInvMask);

		// descomente para visualizar o que foi calculado para o Pavement
		// imshow("grayPavement", grayPavement);
		// imshow("binaryPavement", binaryPavement);
		// cout << "p.mean: " << means0["pavement"] << ", p.covs: " << covs0["pavement"] << endl;

		// descomente para visualizar o que foi calculado para os Lane Markings
		// imshow("grayMarkings", grayMarkings);
		// imshow("binaryMarkings", binaryMarkings);
		// cout << "lm.mean: " << means0["markings"] << ", lm.covs: " << covs0["markings"] << endl;

		// descomente para visualizar o que foi calculado para os Objects
		// imshow("grayObjects", grayObjects);
		// imshow("binaryObjects", binaryObjects);
		// cout << "obj.mean: " << means0["objects"] << ", obj.covs: " << covs0["objects"] << endl;

		// descomente para visualizar o que foi calculado para o Unknown
		// imshow("grayUnknown", grayUnknown);
		// imshow("binaryUnknown", binaryUnknown);
		// cout << "unk.mean: " << means0["unknown"] << ", unk.covs: " << covs0["unknown"] << endl;

		// descomente para imprimir o peso inicial
		// cout << "w[pav]: " << outWeights0["pavement"] << endl;
		// cout << "w[lnm]: " << outWeights0["markings"] << endl;
		// cout << "w[obj]: " << outWeights0["objects"] << endl;
		// cout << "w[unk]: " << outWeights0["unknown"] << endl;
	}
	*/
}

void Nieto::featureL_IPM(const Mat1b &inGrayFrameFiltradoRoi, IPM * _ipm, map<string, double> &outMeans0, map<string, double> &outCovs0, map<string, double> &outWeights0) {

	double tempoInicio = static_cast<double>(getTickCount());

	Size imgSize = inGrayFrameFiltradoRoi.size();

	// Imagens que ser�o constru�das
	Mat1b _binaryMarkings = Mat1b(imgSize, uchar(0));
	Mat1b _binaryPavement = Mat1b(imgSize, uchar(0));
	Mat1b binaryMarkings = Mat1b(imgSize, uchar(0)); // NA IPM
	Mat1b binaryPavement = Mat1b(imgSize, uchar(0)); // NA IPM

	// pega o threshold inicial que separa as duas gaussianas (Pavement < thres < LaneMarkings)
	// Nieto: A reasonable threshold that separates these two components is the standard deviation
	Scalar meanFiltro, stddevFiltro;
	cv::meanStdDev(inGrayFrameFiltradoRoi, meanFiltro, stddevFiltro);

	// seta as m�scaras dos elementos que pertecem a mabas as classes
	_binaryMarkings = inGrayFrameFiltradoRoi >  stddevFiltro[0];
	_binaryPavement = inGrayFrameFiltradoRoi <= stddevFiltro[0];

	_ipm->applyHomography(_binaryMarkings, binaryMarkings, INTER_NEAREST);
	_ipm->applyHomography(_binaryPavement, binaryPavement, INTER_NEAREST);


	map<string, Scalar> meansScalar, stddevScalar;
	// ************************************** PAVEMENT
	cv::meanStdDev(inGrayFrameFiltradoRoi, meansScalar["pavement"], stddevScalar["pavement"], _binaryPavement);
	outMeans0["pavement"] = getMean(inGrayFrameFiltradoRoi, _binaryPavement);
	outCovs0["pavement"] = getVariance(inGrayFrameFiltradoRoi.clone(), outMeans0["pavement"], _binaryPavement);

	// ************************************** LANE MARKINGS
	cv::meanStdDev(inGrayFrameFiltradoRoi, meansScalar["markings"], stddevScalar["markings"], _binaryMarkings);
	outMeans0["markings"] = getMean(inGrayFrameFiltradoRoi, _binaryMarkings);
	outCovs0["markings"] = getVariance(inGrayFrameFiltradoRoi.clone(), outMeans0["markings"], _binaryMarkings);

	// ************************************** OBJECTS
	outMeans0["objects"] = outMeans0["pavement"];
	outCovs0["objects"] = outCovs0["pavement"];

	// ************************************** UNKNOWN
	outMeans0["unknown"] = 255.0 / 2.0;
	outCovs0["unknown"] = ((255.0 / 2.0) / sqrt(3)) * ((255.0 / 2.0) / sqrt(3));

	// calcula os pesos => propor��o de cada classe
	double nPavements = countNonZero(binaryPavement);
	double nMarkings = countNonZero(binaryMarkings);
	double nTotal = nPavements + nMarkings;
	double nUnknown = nTotal * 0.05;
	nTotal += nUnknown;
	outWeights0["pavement"] = (nPavements / 2) / nTotal;
	outWeights0["objects"] = outWeights0["pavement"];
	outWeights0["markings"] = nMarkings / nTotal;
	outWeights0["unknown"] = nUnknown / nTotal;

	// calcula o tempo de execu��o
	double tempoFim = static_cast<double>(getTickCount());
	double tempoExecutando = ((tempoFim - tempoInicio) / getTickFrequency()) * 1000;

	// exibe as sa�das definidas (texto e/ou imagem)
	if (verbose) cout << "- nieto.featureL: " << tempoExecutando << " ms" << endl;

	// if (config.display)
	// imshow("L - binaryMarkings", binaryMarkings);
	// imshow("L - binaryPavement", binaryPavement);


	/*
	if (display) {
	Mat1b imgResultNietoMasks = Mat1b(Size(grayFrameRoiIPM.cols, grayFrameRoiIPM.rows * 4), uchar(0));

	grayPavement.copyTo(imgResultNietoMasks(Rect(0, 0, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)));
	grayMarkings.copyTo(imgResultNietoMasks(Rect(0, grayFrameRoiIPM.rows, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)));
	grayObjects.copyTo(imgResultNietoMasks(Rect(0, 2 * grayFrameRoiIPM.rows, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)));

	imgResultNietoMasks(Rect(0, 3 * grayFrameRoiIPM.rows, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)).setTo(85, binaryPavement);
	imgResultNietoMasks(Rect(0, 3 * grayFrameRoiIPM.rows, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)).setTo(170, binaryObjects);
	imgResultNietoMasks(Rect(0, 3 * grayFrameRoiIPM.rows, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)).setTo(255, binaryMarkings);
	imgResultNietoMasks(Rect(0, 3 * grayFrameRoiIPM.rows, grayFrameRoiIPM.cols, grayFrameRoiIPM.rows)).setTo(0, binaryUnknown);

	imshow("Nieto - Mascaras", imgResultNietoMasks);

	// imshow("binaryIpmInvMask", binaryIpmInvMask);

	// descomente para visualizar o que foi calculado para o Pavement
	// imshow("grayPavement", grayPavement);
	// imshow("binaryPavement", binaryPavement);
	// cout << "p.mean: " << means0["pavement"] << ", p.covs: " << covs0["pavement"] << endl;

	// descomente para visualizar o que foi calculado para os Lane Markings
	// imshow("grayMarkings", grayMarkings);
	// imshow("binaryMarkings", binaryMarkings);
	// cout << "lm.mean: " << means0["markings"] << ", lm.covs: " << covs0["markings"] << endl;

	// descomente para visualizar o que foi calculado para os Objects
	// imshow("grayObjects", grayObjects);
	// imshow("binaryObjects", binaryObjects);
	// cout << "obj.mean: " << means0["objects"] << ", obj.covs: " << covs0["objects"] << endl;

	// descomente para visualizar o que foi calculado para o Unknown
	// imshow("grayUnknown", grayUnknown);
	// imshow("binaryUnknown", binaryUnknown);
	// cout << "unk.mean: " << means0["unknown"] << ", unk.covs: " << covs0["unknown"] << endl;

	// descomente para imprimir o peso inicial
	// cout << "w[pav]: " << outWeights0["pavement"] << endl;
	// cout << "w[lnm]: " << outWeights0["markings"] << endl;
	// cout << "w[obj]: " << outWeights0["objects"] << endl;
	// cout << "w[unk]: " << outWeights0["unknown"] << endl;
	}
	*/
}

/*void Nieto::ExpectationMaximizationOpenCV(const Mat1b &inGrayFrameRoi, int maxIters, map<string, double> &_means0, map<string, double> &_covs0, map<string, double> &_weights0) {

	double tempoInicio = static_cast<double>(getTickCount());
	const int nClusters = 4; // 4 classes => {pavement, markings, objects, unknown}
	const bool aplicaResize = true;

	EM em = EM(nClusters, EM::COV_MAT_DIAGONAL);

	Mat1b grayFrameRoiClone = inGrayFrameRoi.clone();
	Mat1b trainGrayFrameRoiClone = inGrayFrameRoi.clone();
	if (aplicaResize) resize(grayFrameRoiClone, trainGrayFrameRoiClone, Size(160, 35), 0, 0, INTER_NEAREST);
	Mat1d samples = trainGrayFrameRoiClone.reshape(1, trainGrayFrameRoiClone.rows * trainGrayFrameRoiClone.cols);

	// formata o _means0
	Mat1d means0 = Mat1d(nClusters, 1, CV_64FC1);
	means0.at<double>(0) = _means0["pavement"];
	means0.at<double>(1) = _means0["markings"];
	means0.at<double>(2) = _means0["objects"];
	means0.at<double>(3) = 255.0 / 2.0;

	// formata o _covs0
	vector<Mat> covs0 = {
		Mat1d(1, 1, _covs0["pavement"]),
		Mat1d(1, 1, _covs0["markings"]),
		Mat1d(1, 1, _covs0["objects"]),
		Mat1d(1, 1, ((255.0 / 2.0) / sqrt(3)) * ((255.0 / 2.0) / sqrt(3)))
	};

	// formata o _weights0
	// Mat1d weights0 = *(Mat1f(nClusters, 1, CV_64FC1) << 0.75, 0.10, 0.10, 0.05);
	Mat1d weights0 = *(Mat1f(nClusters, 1, CV_64FC1) <<
		_weights0["pavement"],
		_weights0["markings"],
		_weights0["objects"],
		_weights0["unknown"]
		);

	// cout << means0 << endl;

	em.set("maxIters", maxIters);
	em.trainE(samples, means0, covs0, weights0);

	// calcula o tempo de execu��o
	double tempoFim = static_cast<double>(getTickCount());
	double tempoExecutando = ((tempoFim - tempoInicio) / getTickFrequency()) * 1000;

	// exibe as sa�das definidas (texto e/ou imagem)
	if (verbose) cout << "- em opencv (1 feature): " << tempoExecutando << " ms" << endl;
	if (display) {
		// predict
		Mat1b predictedImage = Mat1b(grayFrameRoiClone.size(), uchar(0));
		for (int j = 0; j < predictedImage.rows; ++j) {
			unsigned char *ptRowSrc = grayFrameRoiClone.ptr<uchar>(j);
			unsigned char *ptRowDst = predictedImage.ptr<uchar>(j);
			for (int i = 0; i < predictedImage.cols; ++i) {
				Vec2d emPredicted = em.predict(ptRowSrc[i]);
				switch ((int)emPredicted[1]) {
				case 0: ptRowDst[i] = 160; break;
				case 1: ptRowDst[i] = 255; break;
				case 2: ptRowDst[i] = 80; break;
				case 3: ptRowDst[i] = 0; break;
				}
			}
		}
		imshow("EM OpenCV - 1 Feature", predictedImage);
	}
}
*/
/*void Nieto::ExpectationMaximizationOpenCV2Features(const Mat1b &imageI, const Mat1b &imageL,
	map<string, double> &i_means0, map<string, double> &i_covs0, map<string, double> &i_weights0,
	map<string, double> &l_means0, map<string, double> &l_covs0, map<string, double> &l_weights0, int maxIters) {

	double tempoInicio = static_cast<double>(getTickCount());
	const int nFeatures = 2; // 2 features => {I, L}
	const int nClusters = 4; // 4 classes => {pavement, markings, objects, unknown}
	const bool aplicaResize = true;

	// samples feature I
	Mat1b I_imageClone = imageI.clone();
	Mat1b I_trainImageClone = imageI.clone();
	if (aplicaResize) resize(I_imageClone, I_trainImageClone, Size(160, 35), 0, 0, INTER_NEAREST);
	Mat1d I_samples = I_trainImageClone.reshape(1, I_trainImageClone.rows * I_trainImageClone.cols);

	// samples feature L
	Mat1b L_imageClone = imageL.clone();
	Mat1b L_trainImageClone = imageL.clone();
	if (aplicaResize) resize(L_imageClone, L_trainImageClone, Size(160, 35), 0, 0, INTER_NEAREST);
	Mat1d L_samples = L_trainImageClone.reshape(1, L_trainImageClone.rows * L_trainImageClone.cols);

	// junta as amostras (uma em cada linha)
	Mat1d samplesArray[] = { I_samples, L_samples };
	Mat1d samples;
	cv::hconcat(samplesArray, 2, samples);
	
	// formata o _means0
	Mat1d means0 = Mat1d(nClusters, nFeatures, CV_64FC1);
	means0.at<double>(0, 0) = i_means0["pavement"];
	means0.at<double>(1, 0) = i_means0["markings"];
	means0.at<double>(2, 0) = i_means0["objects"];
	means0.at<double>(3, 0) = 255.0 / 2.0;
	means0.at<double>(0, 1) = l_means0["pavement"];
	means0.at<double>(1, 1) = l_means0["markings"];
	means0.at<double>(2, 1) = l_means0["objects"];
	means0.at<double>(3, 1) = 255.0 / 2.0;

	// formata o _covs0
	Mat1d covs0_pavement = Mat1d(Size(nFeatures, nFeatures), double(0));
	covs0_pavement.at<double>(0, 0) = i_covs0["pavement"];
	covs0_pavement.at<double>(1, 1) = l_covs0["pavement"];
	Mat1d covs0_markings = Mat1d(Size(nFeatures, nFeatures), double(0));;
	covs0_markings.at<double>(0, 0) = i_covs0["markings"];
	covs0_markings.at<double>(1, 1) = l_covs0["markings"];
	Mat1d covs0_objects = Mat1d(Size(nFeatures, nFeatures), double(0));;
	covs0_objects.at<double>(0, 0) = i_covs0["objects"];
	covs0_objects.at<double>(1, 1) = l_covs0["objects"];
	Mat1d covs0_unknown = Mat1d(Size(nFeatures, nFeatures), double(0));;
	covs0_unknown.at<double>(0, 0) = ((255.0 / 2.0) / sqrt(3)) * ((255.0 / 2.0) / sqrt(3));
	covs0_unknown.at<double>(1, 1) = ((255.0 / 2.0) / sqrt(3)) * ((255.0 / 2.0) / sqrt(3));
	vector<Mat> covs0 = {
		covs0_pavement,
		covs0_markings,
		covs0_objects,
		covs0_unknown
	};

	// formata o _weights0
	Mat1d weights0 = Mat1d(nClusters, 1, CV_64FC1);
	double total_i = i_weights0["pavement"] + i_weights0["markings"] + i_weights0["objects"] + i_weights0["unknown"];
	double total_l = l_weights0["pavement"] + l_weights0["markings"] + l_weights0["objects"] + l_weights0["unknown"];
	double total_weights = total_i + total_l;
	weights0.at<double>(0, 0) = (i_weights0["pavement"] + l_weights0["pavement"]) / total_weights;
	weights0.at<double>(1, 0) = (i_weights0["markings"] + l_weights0["markings"]) / total_weights;
	weights0.at<double>(2, 0) = (i_weights0["objects"] + l_weights0["objects"]) / total_weights;
	weights0.at<double>(3, 0) = (i_weights0["unknown"] + l_weights0["unknown"]) / total_weights;

	// cout << means0 << endl;

	// condi��es do EM
	// dims => samples.cols
	// if (!(&means0) || (!means0.empty() && means0.rows == nClusters && means0.cols == samples.cols && means0.channels() == 1)) cout << "means - ok!" << endl;

	EM em = EM(nClusters, EM::COV_MAT_DIAGONAL);
	em.set("maxIters", maxIters);
	em.trainE(samples, means0, covs0, weights0);
	
	// calcula o tempo de execu��o
	double tempoFim = static_cast<double>(getTickCount());
	double tempoExecutando = ((tempoFim - tempoInicio) / getTickFrequency()) * 1000;

	// exibe as sa�das definidas (texto e/ou imagem)
	if (verbose) cout << "- em opencv (2 features): " << tempoExecutando << " ms" << endl;
	if (display) {
		// predict
		Mat1b predictedImage = Mat1b(I_imageClone.size(), uchar(0));
		for (int j = 0; j < predictedImage.rows; ++j) {
			unsigned char *ptRowI = I_imageClone.ptr<uchar>(j);
			unsigned char *ptRowL = L_imageClone.ptr<uchar>(j);
			unsigned char *ptRowDst = predictedImage.ptr<uchar>(j);
			for (int i = 0; i < predictedImage.cols; ++i) {
				
				Mat1d elementPredict = Mat1d(Size(2, 1), CV_64FC1);
				elementPredict.at<double>(0) = ptRowL[i];
				elementPredict.at<double>(1) = ptRowI[i];
				
				Vec2d emPredicted = em.predict(elementPredict);
				switch ((int)emPredicted[1]) {
				case 0: ptRowDst[i] = 160; break;
				case 1: ptRowDst[i] = 255; break;
				case 2: ptRowDst[i] = 80; break;
				case 3: ptRowDst[i] = 0; break;
				}
			}
		}
		imshow("EM OpenCV - 2 Features", predictedImage);
	}
}
*/
#ifdef USING_ARMADILLO

void Nieto::ExpectationMaximizationArmadillo(const Mat1b &inGrayFrameRoi, int maxIters, map<string, double> &_means0, map<string, double> &_covs0, map<string, double> &_weights0) {

	double tempoInicio = static_cast<double>(getTickCount());
	const int nClusters = 4; // 4 classes => {pavement, markings, objects, unknown}

	Mat1b grayFrameRoiClone = inGrayFrameRoi.clone();
	Mat1b trainGrayFrameRoiClone = inGrayFrameRoi.clone();
	resize(grayFrameRoiClone, trainGrayFrameRoiClone, Size(160, 35), 0, 0, INTER_NEAREST);
	Mat1d samples = trainGrayFrameRoiClone.reshape(1, trainGrayFrameRoiClone.rows * trainGrayFrameRoiClone.cols);
	arma::mat armaSamples(reinterpret_cast<double*>(samples.data), samples.rows, samples.cols);

	// cout << "size armaSamples: " << arma::size(armaSamples) << endl;

	// formata o _means0
	arma::mat means0(1, nClusters);
	means0.at(0, 0) = _means0["pavement"];
	means0.at(0, 1) = _means0["markings"];
	means0.at(0, 2) = _means0["objects"];
	means0.at(0, 3) = 255.0 / 2.0;

	// cout << "size means0: " << arma::size(means0) << endl;

	// formata o _covs0
	arma::mat covs0(1, nClusters);
	covs0.at(0, 0) = _covs0["pavement"];
	covs0.at(0, 1) = _covs0["markings"];
	covs0.at(0, 2) = _covs0["objects"];
	covs0.at(0, 3) = ((255.0 / 2.0) / sqrt(3)) * ((255.0 / 2.0) / sqrt(3));

	// cout << "size covs0: " << arma::size(covs0) << endl;

	// formata o _weights0
	arma::mat weights0(1, nClusters);
	weights0.at(0, 0) = _weights0["pavement"];
	weights0.at(0, 1) = _weights0["markings"];
	weights0.at(0, 2) = _weights0["objects"];
	weights0.at(0, 3) = _weights0["unknown"];

	// cout << "size weights0: " << arma::size(weights0) << endl;

	// if (!(size(means0) != size(covs0))) cout << "1 - ok!" << endl;
	// if (!(weights0.n_cols != means0.n_cols)) cout << "2 - ok!" << endl;
	// if (!(weights0.n_rows != 1)) cout << "3 - ok!" << endl;

	arma::gmm_diag em;
	em.set_params(means0, covs0, weights0);
	em.learn(armaSamples.t(), nClusters, arma::eucl_dist, arma::keep_existing, 0, maxIters, 1e-10, false);

	// calcula o tempo de execu��o
	double tempoFim = static_cast<double>(getTickCount());
	double tempoExecutando = ((tempoFim - tempoInicio) / getTickFrequency()) * 1000;

	// exibe as sa�das definidas (texto e/ou imagem)
	if (verbose) cout << "- em armadillo (1 feature): " << tempoExecutando << " ms" << endl;
	if (display) {
		// predict
		Mat1b predictedImage = Mat1b(grayFrameRoiClone.size(), uchar(0));
		for (int j = 0; j < predictedImage.rows; ++j) {
			unsigned char *ptRowSrc = grayFrameRoiClone.ptr<uchar>(j);
			unsigned char *ptRowDst = predictedImage.ptr<uchar>(j);
			for (int i = 0; i < predictedImage.cols; ++i) {
				arma::vec v;
				v << ptRowSrc[i];
				int emPredicted = em.assign(v, arma::eucl_dist);
				switch (emPredicted) {
				case 0: ptRowDst[i] = 160; break;
				case 1: ptRowDst[i] = 255; break;
				case 2: ptRowDst[i] = 80; break;
				case 3: ptRowDst[i] = 0; break;
				}
			}
		}
		imshow("EM Armadillo - 1 Feature", predictedImage);
	}
}

void Nieto::ExpectationMaximizationArmadillo2Features(const Mat1b &imageI, const Mat1b &imageL, 
	map<string, double> &i_means0, map<string, double> &i_covs0, map<string, double> &i_weights0, 
	map<string, double> &l_means0, map<string, double> &l_covs0, map<string, double> &l_weights0, int maxIters) {

	double tempoInicio = static_cast<double>(getTickCount());
	const int nClusters = 4; // 4 classes => {pavement, markings, objects, unknown}
	bool aplicaResize = true;
	
	// samples feature I
	Mat1b I_imageClone = imageI.clone();
	Mat1b I_trainImageClone = imageI.clone();
	if (aplicaResize) resize(I_imageClone, I_trainImageClone, Size(160, 35), 0, 0, INTER_NEAREST);
	Mat1d I_samples = I_trainImageClone.reshape(1, I_trainImageClone.rows * I_trainImageClone.cols);
	arma::mat I_armaSamples(reinterpret_cast<double*>(I_samples.data), I_samples.rows, I_samples.cols);
	
	// samples feature L
	Mat1b L_imageClone = imageL.clone();
	Mat1b L_trainImageClone = imageL.clone();
	if (aplicaResize) resize(L_imageClone, L_trainImageClone, Size(160, 35), 0, 0, INTER_NEAREST);
	Mat1d L_samples = L_trainImageClone.reshape(1, L_trainImageClone.rows * L_trainImageClone.cols);
	arma::mat L_armaSamples(reinterpret_cast<double*>(L_samples.data), L_samples.rows, L_samples.cols);
	
	// junta as amostras (uma em cada linha)
	arma::mat armaSamples = arma::join_rows(I_armaSamples, L_armaSamples);
	// cout << "size armaSamples: " << arma::size(armaSamples.t()) << endl;

	// formata o _means0
	arma::mat means0(2, nClusters);
	means0.at(0, 0) = i_means0["pavement"];
	means0.at(0, 1) = i_means0["markings"];
	means0.at(0, 2) = i_means0["objects"];
	means0.at(0, 3) = 255.0 / 2.0;
	means0.at(1, 0) = l_means0["pavement"];
	means0.at(1, 1) = l_means0["markings"];
	means0.at(1, 2) = l_means0["objects"];
	means0.at(1, 3) = 255.0 / 2.0;
	// cout << "size means0: " << arma::size(means0) << endl;

	// formata o _covs0
	arma::mat covs0(2, nClusters);
	covs0.at(0, 0) = i_covs0["pavement"];
	covs0.at(0, 1) = i_covs0["markings"];
	covs0.at(0, 2) = i_covs0["objects"];
	covs0.at(0, 3) = ((255.0 / 2.0) / sqrt(3)) * ((255.0 / 2.0) / sqrt(3));
	covs0.at(1, 0) = l_covs0["pavement"];
	covs0.at(1, 1) = l_covs0["markings"];
	covs0.at(1, 2) = l_covs0["objects"];
	covs0.at(1, 3) = ((255.0 / 2.0) / sqrt(3)) * ((255.0 / 2.0) / sqrt(3));
	// cout << "size covs0: " << arma::size(covs0) << endl;

	// formata o _weights0
	arma::mat weights0(1, nClusters);/*
	double total_i = i_weights0["pavement"] + i_weights0["markings"] + i_weights0["objects"] + i_weights0["unknown"];
	double total_l = l_weights0["pavement"] + l_weights0["markings"] + l_weights0["objects"] + l_weights0["unknown"];
	double total_weights = total_i + total_l;
	weights0.at(0, 0) = (i_weights0["pavement"] + l_weights0["pavement"]) / total_weights;
	weights0.at(0, 1) = (i_weights0["markings"] + l_weights0["markings"]) / total_weights;
	weights0.at(0, 2) = (i_weights0["objects"] + l_weights0["objects"]) / total_weights;
	weights0.at(0, 3) = (i_weights0["unknown"] + l_weights0["unknown"]) / total_weights;
	*/

	double total_i = i_weights0["pavement"] + i_weights0["objects"] + i_weights0["unknown"] + l_weights0["markings"];
	weights0.at(0, 0) = i_weights0["pavement"] / total_i;
	weights0.at(0, 1) = l_weights0["markings"] / total_i;
	weights0.at(0, 2) = i_weights0["objects"] / total_i;
	weights0.at(0, 3) = i_weights0["unknown"] / total_i;
	// cout << "size weights0: " << arma::size(weights0) << endl;
	weights0.at(0, 0) = trunc(weights0.at(0, 0) * 1000) / 1000;
	weights0.at(0, 1) = trunc(weights0.at(0, 1) * 1000) / 1000;
	weights0.at(0, 2) = trunc(weights0.at(0, 2) * 1000) / 1000;
	weights0.at(0, 3) = trunc(weights0.at(0, 3) * 1000) / 1000;

	double diff = 1 - (weights0.at(0, 0) + weights0.at(0, 1) + weights0.at(0, 2) + weights0.at(0, 3));
	weights0.at(0, 3) += diff;

	// if (!(size(means0) != size(covs0))) cout << "1 - ok!" << endl;
	// if (!(weights0.n_cols != means0.n_cols)) cout << "2 - ok!" << endl;
	// if (!(weights0.n_rows != 1)) cout << "3 - ok!" << endl;

	arma::gmm_diag em;
	em.set_params(means0, covs0, weights0);
	em.means.print("means a: ");
	em.dcovs.print("dcovs a: ");
	em.hefts.print("hefts a: ");
	em.learn(armaSamples.t(), nClusters, arma::eucl_dist, arma::keep_existing, 0, 1000, 1e-10, false);
	em.means.print("means b: ");
	em.dcovs.print("dcovs b: ");
	em.hefts.print("hefts b: ");
	
	// calcula o tempo de execu��o
	double tempoFim = static_cast<double>(getTickCount());
	double tempoExecutando = ((tempoFim - tempoInicio) / getTickFrequency()) * 1000;

	// exibe as sa�das definidas (texto e/ou imagem)
	if (verbose) cout << "- em armadillo (2 features): " << tempoExecutando << " ms" << endl;
	if (display) {
		// predict
		Mat1b predictedImage = Mat1b(imageI.size(), uchar(0));
		for (int j = 0; j < predictedImage.rows; ++j) {
			unsigned char *ptRowI = I_imageClone.ptr<uchar>(j);
			unsigned char *ptRowL = L_imageClone.ptr<uchar>(j);
			unsigned char *ptRowDst = predictedImage.ptr<uchar>(j);
			for (int i = 0; i < predictedImage.cols; ++i) {
				arma::vec v(2, 1);
				v << ptRowI[i] << ptRowL[i];
				int emPredicted = em.assign(v, arma::eucl_dist);
				switch (emPredicted) {
				case 0: ptRowDst[i] = 160; break;
				case 1: ptRowDst[i] = 255; break;
				case 2: ptRowDst[i] = 0; break;
				case 3: ptRowDst[i] = 0; break;
				}
			}
		}
		imshow("EM Armadillo - 2 Features", predictedImage);
	}
}

#endif
