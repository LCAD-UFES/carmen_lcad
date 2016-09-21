#include "Helper.h"

using namespace std;
using namespace cv;

Mat1b Helper::converteParaEscalaDeCinza(const Mat3b &imagem) {
	Mat1b escalaDeCinza;
	cvtColor(imagem, escalaDeCinza, CV_BGR2GRAY);
	return escalaDeCinza;
}

Mat1d Helper::from8Uto64F(const Mat1b &from) {
	Mat1d out;
	from.convertTo(out, CV_64F);
	return out;
}

Mat1b Helper::toROI(const Mat1b &imgIPM, IPM *ipm, int interpolacao) {
	Mat1b imgROI;
	ipm->applyHomographyInv(imgIPM, imgROI, INTER_NEAREST);
	return imgROI;
}
Mat1b Helper::toIPM(const Mat1b &imgROI, IPM *ipm, int interpolacao) {
	Mat1b imgIPM;
	ipm->applyHomography(imgROI, imgIPM, INTER_NEAREST);
	return imgIPM;
}
Mat1b Helper::toIPM(const Mat1b &img, const Rect &roi, IPM *ipm, int interpolacao) {
	Mat1b imgIPM;
	ipm->applyHomography(img(roi), imgIPM, INTER_NEAREST);
	return imgIPM;
}
Mat1b Helper::getROI(const Mat1b &imagem, const Rect &roi) {
	return imagem(roi).clone();
}

Mat1b Helper::fillPerspectiva(const Mat1b &imgROI, const Rect &roi, const Size &tamanho, const uchar _default) {
	Mat1b perspectiva = Mat1b(tamanho, uchar(_default));
	imgROI.copyTo(perspectiva(roi));
	return perspectiva;
}

void Helper::desenharHistograma(const Mat1b &histograma, Mat1b &img, Scalar cor) {
	int altura = img.rows;
	for (int i = 1; i < histograma.cols; i++) {
		int yAnterior = (int)(((double)histograma.at<uchar>(i - 1) / 255.0) * altura);
		int yAtual = (int)(((double)histograma.at<uchar>(i) / 255.0) * altura);
		line(img, Point(i - 1, altura - yAnterior), Point(i, altura - yAtual), cor);
	}
}
void Helper::desenharHistograma(const Mat1d &histograma, Mat1b &img, Scalar cor) {
	int altura = img.rows;
	for (int i = 1; i < histograma.cols; i++) {
		int yAnterior = (int)(histograma.at<double>(i - 1) * altura);
		int yAtual = (int)(histograma.at<double>(i) * altura);
		line(img, Point(i - 1, altura - yAnterior), Point(i, altura - yAtual), cor);
	}
}
void Helper::desenharHistograma(const vector<double> &histograma, Mat1b &img, double fator, Scalar cor) {
	int altura = img.rows;
	for (unsigned int i = 1; i < histograma.size(); i++) {
		int yAnterior = (int)(histograma[i - 1] * fator);
		int yAtual = (int)(histograma[i] * fator);
		// yAnterior = (yAnterior < 255) ? (altura * int(yAnterior / 255.0)) : altura;
		// yAtual = (yAtual < 255) ? (altura * int(yAtual / 255.0)) : altura;
		line(img, Point(i - 1, altura - yAnterior), Point(i, altura - yAtual), cor);
	}
}
void Helper::desenharHistograma(const deque<double> &histograma, Mat1b &img, double fator, Scalar cor) {
	int altura = img.rows;
	for (unsigned int i = 1; i < histograma.size(); i++) {
		int yAnterior = (int)(histograma[i - 1] * fator);
		int yAtual = (int)(histograma[i] * fator);
		yAnterior = (yAnterior < 255) ? (altura * int(yAnterior / 255.0)) : altura;
		yAtual = (yAtual < 255) ? (altura * int(yAtual / 255.0)) : altura;
		line(img, Point(i - 1, altura - yAnterior), Point(i, altura - yAtual), cor);
	}
}
void Helper::visualizarHistograma(const Mat1d &histograma, const string &windowTitle, int height, bool singleColumn) {

	Mat1d histNormalizado = histograma.clone();
	if (singleColumn) transpose(histNormalizado, histNormalizado);
	normalize(histNormalizado, histNormalizado, 0.0, 1.0, NORM_MINMAX);

	Mat1b imgHistograma = Mat1b::zeros(Size(histNormalizado.cols, height));
	for (int i = 1; i < histNormalizado.cols; i++) {
		int yAnterior = (int)(histNormalizado.at<double>(i - 1) * height);
		int yAtual = (int)(histNormalizado.at<double>(i) * height);
		line(imgHistograma, Point(i - 1, height - yAnterior), Point(i, height - yAtual), Scalar(255));
	}

	imshow(windowTitle, imgHistograma);
}

void Helper::rotate2D(const Mat & src, Mat & dst, const float degrees, int flag) {
	Point2f center(src.cols / (float)2.0, src.rows / (float)2.0);
	Mat rot = getRotationMatrix2D(center, degrees, 1.0);
	Rect bbox = RotatedRect(center, src.size(), degrees).boundingRect();

	rot.at<double>(0, 2) += bbox.width / 2.0 - center.x;
	rot.at<double>(1, 2) += bbox.height / 2.0 - center.y;

	warpAffine(src, dst, rot, bbox.size(), flag);
}

int Helper::contarSegmentosBranco(const Mat1b &img, int minTamanho, int maxTamanho) {

	// n�o tem como ter segmento maior que isso
	if (maxTamanho == -1) maxTamanho = img.cols + 1;
	if (minTamanho == -1) minTamanho = 0;

	// vari�vel de s�ida
	int nSegmentos = 0;

	// vari�veis de controle
	bool naSequencia = false;
	int nSequencias = 0;
	int tamanhoSequencia = 0;

	for (int i = 0; i < img.cols; i++) {
		int pixel = img.at<uchar>(i);

		// inicio e termino de sequencias
		if (pixel == 255) { // inicio e continuidade
			naSequencia = true;
			++tamanhoSequencia;
		} else if (pixel == 0 && naSequencia) { // termino
			if ((tamanhoSequencia >= minTamanho) && (tamanhoSequencia <= maxTamanho)) ++nSegmentos;
			naSequencia = false;
			tamanhoSequencia = 0;
		} else {
			naSequencia = false;
		}
	}

	// se ele terminou em uma sequencia, conte +1
	if (naSequencia) ++nSegmentos;

	return nSegmentos;
}

int Helper::contarSegmentosPreto(const Mat1b &img, int minTamanho, int maxTamanho) {

	// n�o tem como ter segmento maior que isso
	if (maxTamanho == -1) maxTamanho = img.cols + 1;
	if (minTamanho == -1) minTamanho = 0;

	// vari�vel de s�ida
	int nSegmentos = 0;

	// vari�veis de controle
	bool naSequencia = false;
	int nSequencias = 0;
	int tamanhoSequencia = 0;

	for (int i = 0; i < img.cols; i++) {
		int pixel = img.at<uchar>(i);

		// inicio e termino de sequencias
		if (pixel == 0) { // inicio e continuidade
			naSequencia = true;
			++tamanhoSequencia;
		} else if (pixel == 255 && naSequencia) { // termino
			if ((tamanhoSequencia >= minTamanho) && (tamanhoSequencia <= maxTamanho)) ++nSegmentos;
			naSequencia = false;
			tamanhoSequencia = 0;
		} else {
			naSequencia = false;
		}
	}

	// se ele terminou em uma sequencia, conte +1
	if (naSequencia) ++nSegmentos;

	return nSegmentos;
}

Mat1b Helper::morphDilate(const Mat1b &from, int size) {
	Mat1b saida = Mat1b(from.size(), uchar(0));
	Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(size, size));
	dilate(from, saida, kernel);
	return saida;
}
Mat1b Helper::morphErode(const Mat1b &from, int size) {
	Mat1b saida = Mat1b(from.size(), uchar(0));
	Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(size, size));
	erode(from, saida, kernel);
	return saida;
}
Mat1b Helper::skeleton(const Mat1b &binaryImage, const int size) {

	Mat1b img = binaryImage.clone();
	Mat skel(img.size(), CV_8UC1, Scalar(0));
	Mat temp;
	Mat eroded;

	Mat element = getStructuringElement(MORPH_CROSS, cv::Size(size, size));

	bool done;
	do {
		erode(img, eroded, element);
		dilate(eroded, temp, element); // temp = open(img)
		subtract(img, temp, temp);
		bitwise_or(skel, temp, skel);
		eroded.copyTo(img);

		done = (countNonZero(img) == 0);
	} while (!done);

	return skel;
}
/**
* Perform one thinning iteration.
* Normally you wouldn't call this function directly from your code.
* from: https://github.com/bsdnoobz/zhang-suen-thinning
*
* Parameters:
* 		im    Binary image with range = [0,1]
* 		iter  0=even, 1=odd
*/
void Helper::thinningIteration(cv::Mat& img, int iter)
{
	// CV_Assert(img.channels() == 1);
	// CV_Assert(img.depth() != sizeof(uchar));
	// CV_Assert(img.rows > 3 && img.cols > 3);

	cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);

	int nRows = img.rows;
	int nCols = img.cols;

	if (img.isContinuous()) {
		nCols *= nRows;
		nRows = 1;
	}

	int x, y;
	uchar *pAbove;
	uchar *pCurr;
	uchar *pBelow;
	uchar *nw, *no, *ne;    // north (pAbove)
	uchar *we, *me, *ea;
	uchar *sw, *so, *se;    // south (pBelow)

	uchar *pDst;

	// initialize row pointers
	pAbove = NULL;
	pCurr = img.ptr<uchar>(0);
	pBelow = img.ptr<uchar>(1);

	for (y = 1; y < img.rows - 1; ++y) {
		// shift the rows up by one
		pAbove = pCurr;
		pCurr = pBelow;
		pBelow = img.ptr<uchar>(y + 1);

		pDst = marker.ptr<uchar>(y);

		// initialize col pointers
		no = &(pAbove[0]);
		ne = &(pAbove[1]);
		me = &(pCurr[0]);
		ea = &(pCurr[1]);
		so = &(pBelow[0]);
		se = &(pBelow[1]);

		for (x = 1; x < img.cols - 1; ++x) {
			// shift col pointers left by one (scan left to right)
			nw = no;
			no = ne;
			ne = &(pAbove[x + 1]);
			we = me;
			me = ea;
			ea = &(pCurr[x + 1]);
			sw = so;
			so = se;
			se = &(pBelow[x + 1]);

			int A = (*no == 0 && *ne == 1) + (*ne == 0 && *ea == 1) +
				(*ea == 0 && *se == 1) + (*se == 0 && *so == 1) +
				(*so == 0 && *sw == 1) + (*sw == 0 && *we == 1) +
				(*we == 0 && *nw == 1) + (*nw == 0 && *no == 1);
			int B = *no + *ne + *ea + *se + *so + *sw + *we + *nw;
			int m1 = iter == 0 ? (*no * *ea * *so) : (*no * *ea * *we);
			int m2 = iter == 0 ? (*ea * *so * *we) : (*no * *so * *we);

			if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
				pDst[x] = 1;
		}
	}

	img &= ~marker;
}
/**
* Function for thinning the given binary image
* from: https://github.com/bsdnoobz/zhang-suen-thinning
* obs.: too slow (~30ms per frame)
*
* Parameters:
* 		src  The source image, binary with range = [0,255]
* 		dst  The destination image
*/
void Helper::thinning(const cv::Mat& src, cv::Mat& dst)
{
	dst = src.clone();
	dst /= 255;         // convert to binary image

	cv::Mat prev = cv::Mat::zeros(dst.size(), CV_8UC1);
	cv::Mat diff;

	do {
		thinningIteration(dst, 0);
		thinningIteration(dst, 1);
		cv::absdiff(dst, prev, diff);
		dst.copyTo(prev);
	} while (cv::countNonZero(diff) > 0);

	dst *= 255;
}

void Helper::filter2Dsoma(const Mat1d &input, const Mat1d &kernel, Mat1d &dst) {
	Mat1b img = input.clone();

	// somar o kernel a imagem para obter um resultado, ao inves de convoluir
	const int shiftKernel = kernel.cols / 2;
	copyMakeBorder(img, img, 0, 0, shiftKernel, shiftKernel, BORDER_CONSTANT, 0.0);


}

vector<Point> Helper::mat2vector(const Mat1b &img) {
	vector<Point> points;
	for (int i = 0; i < img.cols; ++i) {
		for (int j = 0; j < img.rows; ++j) {
			Point p = Point(i, j);
			if (255 == (int)img.at<uchar>(p))
				points.push_back(p);
		}
	}
	return points;
}

Point2d Helper::mediaPontos(const vector<Point> &pontos) {
	Point sum;
	for (auto p : pontos) sum += p;
	Point2d media = sum * (1 / (double)pontos.size());
	return media;
}

// calculate the mean and standard deviation: http://stackoverflow.com/a/12405793/4228275
void Helper::mediaDesvioDouble(const vector<double> &v, double &mean, double &stddev) {

	double sum = std::accumulate(std::begin(v), std::end(v), 0.0);
	double m = sum / v.size();
	double accum = 0.0;
	std::for_each(std::begin(v), std::end(v), [&](const double d) {
		accum += (d - m) * (d - m);
	});
	double stdev = sqrt(accum / (v.size() - 1));

	mean = m;
	stddev = stdev;
}

double Helper::getAngulo(const Point &p1, const Point &p2) {
	// calcula o angulo
	double rad2Degree = 180.0 / CV_PI;
	return (atan2(p1.y - p2.y, p1.x - p2.x) * rad2Degree);
}

Rect Helper::validaRectCols(Rect &&_rect, int cols) {
	if (_rect.x < 0) {
		_rect.width += _rect.x;
		_rect.x = 0;
	} else if (_rect.x + _rect.width >= cols) {
		int diff = abs((_rect.x + _rect.width) - cols);
		_rect.width -= diff;
	}
	return _rect;
}

void displayTimePerformance(map<int, vector<double> > &all_times) {
	cout << endl << "Time measurements:" << endl;
	// pega a media e desvio de todos os modulos
	double sum_m = 0, sum_d = 0;
	for (auto _t : all_times) {
		if (_t.first == Task::ALL) continue;
		double m, d;
		Helper::mediaDesvioDouble(_t.second, m, d);
		cout << " -> " << Task::getName(_t.first) << ": " << m << " (" << d << ")" << endl;
		sum_m += m; sum_d += d;
	}
	cout << " -> total: " << sum_m << " (" << sum_d << ")" << endl;
}

double getTotalTime(map<int, TimeMeasurement> &timer) {
	double total_time = 0;
	for (auto _t : timer) if (_t.first!= Task::ALL) total_time += _t.second.duration();
	return total_time;
}

string Task::getName(int _taskID) {
	string out = "";
	
	switch (_taskID) {
		case 0:		out = "all";					break;
		case 1:		out = "feature maps";			break;
		case 2:		out = "crosswalk";				break;
		case 3:		out = "road sigs";				break;
		case 4:		out = "signs removal";			break;
		case 5:		out = "candidates generation";	break;
		case 6:		out = "kalman";					break;
		case 7:		out = "particle filter";		break;
		case 8:		out = "lmt";					break;
		case 9:		out = "adjacent lanes";			break;
		case 10:	out = "lane center deviation";	break;
	}

	return out;
}
