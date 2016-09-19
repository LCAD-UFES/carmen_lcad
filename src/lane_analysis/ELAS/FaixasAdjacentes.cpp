#include "FaixasAdjacentes.h"

using namespace std;
using namespace cv;

int contarEvidencasEmbaixoDaHoughIPM(const HoughLine &houghIPM, const Mat1b &mapaIPM, const int regiaoBusca) {
	int nEvidencias = 0;

	// caso a hough seja vazia
	if (houghIPM.isEmpty()) return nEvidencias;

	// conta embaixo das houghs
	// apenas na metade inferior da imagem
	for (int i = mapaIPM.rows / 2; i < mapaIPM.rows; i++) {
		int p1_x = (int)houghIPM.getX(i);
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

Mat1d montarHistograma1D(const vector<HoughLine> &_houghs, const Mat1b &mapa, bool normalizar, ConfigXML *config, int pLeft, int pRight, int regiaoBusca, const double angulo) {
	// inicia histograma com zeros
	Mat1d out = Mat1d(Size(mapa.cols, 1), double(0));

	// caso n�o tenha houghs
	if (_houghs.size() == 0) return out;

	// calcula o histograma
	for (HoughLine h : _houghs) {
		HoughLine hIPM = h.toIPM(config);
		if (hIPM.p1.x < 0 || hIPM.p1.x >= mapa.cols) continue;
		if (abs(hIPM.getAngulo() - angulo) > 15) continue;
		int x = (int)hIPM.p1.x;
		out.at<double>(x) += contarEvidencasEmbaixoDaHoughIPM(hIPM, mapa, 3);
	}

	// normaliza, caso queira
	if (normalizar) cv::normalize(out, out, 0.0, 1.0, NORM_MINMAX);

	return out;
}

vector<int> faixasAdjacentesHoughs(const Mat1b &mapaIPM, const vector<HoughLine> &houghs, const vector<HoughLine> &houghs2, const HoughDoMeio *hough, const LMT &leftLMT, const LMT &rightLMT, ConfigXML *config) {
	// default: no lanes
	vector<int> multipleLanes = { 0, 0 };
	if (hough == NULL) return multipleLanes;

	// decis�o baseada somente no tipo da faixa
	bool evalLeft = false;
	if (leftLMT != LMT::SCB && leftLMT != LMT::SCA && leftLMT != LMT::NONE) {
		multipleLanes[0] = (leftLMT != LMT::SSB) ? -1 : 1;
	} else evalLeft = true;

	bool evalRight = false;
	if (rightLMT != LMT::SCB && rightLMT != LMT::SCA && rightLMT != LMT::NONE) {
		multipleLanes[1] = (rightLMT != LMT::SSB) ? -1 : 1;
	} else evalRight = true;

	if (evalRight || evalLeft) {

		// pega o angulo do Kalman
		const double angulo = Helper::getAngulo(Point((int)hough->xBase, mapaIPM.rows), Point((int)hough->xTopo, 0));

		// junta os vetores de houghs
		vector<HoughLine> _houghs = houghs;
		_houghs.insert(_houghs.end(), houghs2.begin(), houghs2.end());

		// seta as regi�es de busca
		const int pLeft = static_cast<int>(hough->xBase - (hough->largura * 1.5));
		const int pRight = static_cast<int>(hough->xBase + (int)(hough->largura * 1.5));
		const int regiaoBusca = static_cast<int>(hough->largura * 0.4);
		const int metadeRegiaoBusca = regiaoBusca / 2;
		const int metadeLargura = static_cast<int>(hough->largura * 0.5);
		const int descentralizacao = regiaoBusca * 1; // [|||x|||] -> [||||||x|||] para o caso da esquerda
		const int shift = 5;

		// monta o histograma conforme a regi�o de busca e o angulo dominante
		Mat1d hist = montarHistograma1D(_houghs, mapaIPM, true, config, pLeft, pRight, regiaoBusca, angulo);
		Mat1d esqHist = hist(Helper::validaRectCols(Rect(pLeft - metadeRegiaoBusca - descentralizacao, 0, regiaoBusca + descentralizacao, 1), mapaIPM.cols));
		Mat1d dirHist = hist(Helper::validaRectCols(Rect(pRight - metadeRegiaoBusca, 0, regiaoBusca + descentralizacao, 1), mapaIPM.cols));

		const int esqTamEspaco = ((int)hough->xBase - metadeLargura - metadeRegiaoBusca) - (pLeft + metadeRegiaoBusca);
		Mat1d esqEspaco = hist(Helper::validaRectCols(Rect(pLeft + metadeRegiaoBusca + shift, 0, esqTamEspaco - shift, 1), mapaIPM.cols));
		const int dirTamEspaco = (pRight - metadeRegiaoBusca) - ((int)hough->xBase + metadeLargura + metadeRegiaoBusca);
		Mat1d dirEspaco = hist(Helper::validaRectCols(Rect((int)hough->xBase + metadeLargura + regiaoBusca, 0, dirTamEspaco - shift, 1), mapaIPM.cols));

		// atualiza o valor de sa�da caso necess�rio
		if (evalLeft && countNonZero(esqHist) > 0 && sum(esqEspaco)[0] < 0.3) multipleLanes[0] = 1;
		if (evalRight && countNonZero(dirHist) > 0 && sum(dirEspaco)[0] < 0.3) multipleLanes[1] = 1;

		// sa�da de texto
		if (false) {
			cout << "n houghs: " << _houghs.size() << endl;
			cout << "left(" << pLeft << "), right(" << pRight << "), regiao(" << regiaoBusca << "), angulo()" << endl;
			cout << "angulo: " << angulo << endl;
			cout << "non-zero left: " << countNonZero(esqHist) << endl;
			cout << "non-zero right: " << countNonZero(dirHist) << endl;
			cout << "non-zero left espaco: " << sum(esqEspaco)[0] << endl;
			cout << "non-zero right espaco: " << sum(dirEspaco)[0] << endl;

			Mat1b histCompleto = Mat1b(Size(hist.cols, 150), uchar(0));
			Helper::desenharHistograma(hist, histCompleto);

			rectangle(histCompleto, Rect(pLeft - metadeRegiaoBusca - descentralizacao, 5, regiaoBusca + descentralizacao, 5), Scalar(255), CV_FILLED);
			rectangle(histCompleto, Rect(pRight - metadeRegiaoBusca, 5, regiaoBusca + descentralizacao, 5), Scalar(255), CV_FILLED);

			rectangle(histCompleto, Helper::validaRectCols(Rect(pLeft + metadeRegiaoBusca + shift, 15, esqTamEspaco - shift, 5), mapaIPM.cols), Scalar(127), CV_FILLED);
			rectangle(histCompleto, Helper::validaRectCols(Rect((int)hough->xBase + metadeLargura + metadeRegiaoBusca, 15, dirTamEspaco - shift, 5), mapaIPM.cols), Scalar(127), CV_FILLED);

			imshow("histograma completo", histCompleto);
		}
	}

	return multipleLanes;
}

vector<int> faixasAdjacentes(const Mat &mapa, const Mat1b &grayFrameRoiIPM, const Mat3b &colorFrameRoiIPM, const HoughDoMeio *hough, const ParticleHough &particle, const LMT &leftLMT, const LMT &rightLMT, const vector<int> &maxAdjacentes) {
	
	// default: no lanes
	vector<int> multipleLanes = {0, 0};
	if (hough == NULL) return multipleLanes;

	const int larguraEval = static_cast<int>(hough->largura * 0.4);
	const int alturaEval = 10;
	const int tamanhoMinimo = 5;

	// decis�o baseada somente no tipo da faixa
	bool evalLeft = false;
	if (leftLMT != LMT::SCB && leftLMT != LMT::SCA && leftLMT != LMT::NONE) {
		multipleLanes[0] = (leftLMT != LMT::SSB) ? -1 : 1;
	} else evalLeft = true;

	bool evalRight = false;
	if (rightLMT != LMT::SCB && rightLMT != LMT::SCA && rightLMT != LMT::NONE) {
		multipleLanes[1] = (rightLMT != LMT::SSB) ? -1 : 1;
	} else evalRight = true;
	
	Mat3b colorTeste = colorFrameRoiIPM.clone();

	// se um dos lados tem que ser avaliado
	if (evalLeft || evalRight) {

		vector<Point2d> centerSplinePoints = particle.getSplinePoints(*hough, colorFrameRoiIPM.rows, LANE_CENTER);
		vector<Point3d> pontosCentro;
		double desvioCentro;
		for (int i = alturaEval / 2; i < colorFrameRoiIPM.rows - alturaEval / 2 - 1; i += alturaEval / 2) {
			// regiao do centro
			double mediaCentro;
			Rect areaEvalCentro((int)(centerSplinePoints[i].x - larguraEval / 2), (int)(centerSplinePoints[i].y - alturaEval / 2), larguraEval, alturaEval);
			Point3d pCentro = mediaDesvio3D(colorFrameRoiIPM(areaEvalCentro), mediaCentro, desvioCentro);
			// guarda os valores
			pontosCentro.push_back(pCentro);
			circle(colorTeste, centerSplinePoints[i], 0, BGR_BLUE, -1); // centro
		}

		//double desvioCentro = 20;

		// avalia o lado esquerdo
		if (evalLeft) {
			vector<Point2d> esqSplinePoints = particle.getSplinePoints(*hough, grayFrameRoiIPM.rows, LANE_LEFT, 2);
			double tamanhoSegmento = 0;
			int jj = 0;
			for (int i = alturaEval / 2; i < grayFrameRoiIPM.rows - alturaEval / 2 - 1; i += alturaEval / 2) {
				Scalar esqCor = BGR_RED;
				// regiao da esquerda
				double mediaEsq, desvioEsq;
				Rect areaEvalEsq((int)(esqSplinePoints[i].x - larguraEval / 2), (int)(esqSplinePoints[i].y - alturaEval / 2), larguraEval, alturaEval);
				Point3d pEsq = mediaDesvio3D(colorFrameRoiIPM(areaEvalEsq), mediaEsq, desvioEsq);
				double distEsq = cv::norm(pontosCentro[jj] - pEsq);
				if (desvioEsq < 1.5 * desvioCentro && distEsq < 1.5 * desvioCentro) {
					++tamanhoSegmento;
					esqCor = BGR_GREEN;
				} else {
					if (tamanhoSegmento > tamanhoMinimo) {
						multipleLanes[0] = 1;
						break;
					}
					tamanhoSegmento = 0;
				}
				jj++;
				circle(colorTeste, esqSplinePoints[i], 0, esqCor, -1); // DISPLAY: esquerda
			}
		}

		// avalia o lado direito
		if (evalRight) {
			vector<Point2d> dirSplinePoints = particle.getSplinePoints(*hough, grayFrameRoiIPM.rows, LANE_RIGHT, 2);
			double tamanhoSegmento = 0;
			int jj = 0;
			for (int i = alturaEval / 2; i < grayFrameRoiIPM.rows - alturaEval / 2 - 1; i += alturaEval / 2) {
				Scalar dirCor = BGR_RED;
				// regiao da direita
				double mediaDir, desvioDir;
				Rect areaEvalDir((int)(dirSplinePoints[i].x - larguraEval / 2), (int)(dirSplinePoints[i].y - alturaEval / 2), larguraEval, alturaEval);
				Point3d pDir = mediaDesvio3D(colorFrameRoiIPM(areaEvalDir), mediaDir, desvioDir);
				double distDir = cv::norm(pontosCentro[jj] - pDir);
				if (desvioDir < 1.5 * desvioCentro && distDir < 1.5 * desvioCentro) {
					++tamanhoSegmento;
					dirCor = BGR_GREEN;
				} else {
					if (tamanhoSegmento > tamanhoMinimo) {
						multipleLanes[1] = 1;
						break;
					}
					tamanhoSegmento = 0;
				}
				jj++;
				circle(colorTeste, dirSplinePoints[i], 0, dirCor, -1); // DISPLAY: direita
			}
		}
	}

	//imshow("colorTeste", colorTeste);

	return multipleLanes;
}

Point3d mediaDesvio3D(const Mat3b &img, double &media, double &desvio) {

	vector<double> norms;
	Point3d pMedia = Point3d(0,0,0);

	for (int i = 0; i < img.cols; ++i) {
		for (int j = 0; j < img.rows; ++j) {

			// guarda as normas dos pontos
			Vec3b ptAtual = img.at<Vec3b>(Point(i, j));
			norms.push_back(cv::norm(Point3d(ptAtual[0], ptAtual[1], ptAtual[2])));

			// calcula media iterativamente
			pMedia.x = ((pMedia.x * norms.size()) + ptAtual[0]) / (norms.size() + 1);
			pMedia.y = ((pMedia.y * norms.size()) + ptAtual[1]) / (norms.size() + 1);
			pMedia.z = ((pMedia.z * norms.size()) + ptAtual[2]) / (norms.size() + 1);
		}
	}

	Helper::mediaDesvioDouble(norms, media, desvio);

	return pMedia;
}

int contaEvidencias(const Mat1b &img) {
	return countNonZero(img);
}
