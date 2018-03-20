#include "KalmanHoughs.h"

using namespace std;
using namespace cv;

KalmanHoughs::KalmanHoughs(ConfigXML *_config, bool _verbose, bool _display) {
	config = _config;
	display = _display;
	verbose = _verbose;
	measurement = Mat1f(Size(1, 3), double(0));

	// vetor de estado terá 6 params
	// vetor de medida terá 3 params
	// não haverá vetor de controle
	_KF = new KalmanFilter(6, 3, 0, CV_64F);

	// inicialização do filtro
    double matrix []= {
		1, 0, 0, 1, 0, 0,		// base += delta_base
		0, 1, 0, 0, 1, 0,		// topo += delta_topo
		0, 0, 1, 0, 0, 1,		// largura += delta_largura
		0, 0, 0, 1, 0, 0,		// delta_base
		0, 0, 0, 0, 1, 0,		// delta_topo
		0, 0, 0, 0, 0, 1};      // delta_largura
	_KF->transitionMatrix = (Mat(6, 6, CV_64F, matrix));
	setIdentity(_KF->measurementMatrix);
	setIdentity(_KF->processNoiseCov, Scalar::all(1e-5));
	setIdentity(_KF->measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(_KF->errorCovPost, Scalar::all(1));
}

HoughDoMeio KalmanHoughs::executar(const HoughDoMeio &hough, const Mat &colorFramePerspectiva, const Mat3b &colorFrameRoiIPM) {
	
	double tempoInicio = static_cast<double>(getTickCount());

	// predição
	predict();
	// atualiza a medida
	setMeasurement(hough);
	// estima
	Mat1d estimado = correct();
	HoughDoMeio out = HoughDoMeio(estimado);

	// calcula o tempo de execução
	double tempoFim = static_cast<double>(getTickCount());
	double tempoExecutando = ((tempoFim - tempoInicio) / getTickFrequency()) * 1000;

	// exibe as saídas definidas (texto e/ou imagem)
	if (CONFIG_VERBOSE) cout << "- kalman houghs: " << tempoExecutando << " ms" << endl;
	if (CONFIG_DISPLAY) view(&out, colorFramePerspectiva, colorFrameRoiIPM, Scalar(0,0,255)); 

	return out;
}

void KalmanHoughs::view(HoughDoMeio *houghDoMeio, const Mat &colorFramePerspectiva, const Mat3b &colorFrameRoiIPM, const Scalar &cor) {
	// display perspectiva
	Mat imgPerspectiva = colorFramePerspectiva.clone();
	if (houghDoMeio != NULL) houghDoMeio->draw(imgPerspectiva, cor, config);

	// ipm
	Mat3b imgIPM = colorFrameRoiIPM.clone();
	Rect ipm = Rect(0, 0, colorFrameRoiIPM.cols, colorFrameRoiIPM.rows);
	if (houghDoMeio != NULL) {
		HoughLine _hough = HoughLine::create(*houghDoMeio, config);
		houghDoMeio->draw(imgIPM, cor);
	}
	imgIPM.copyTo(imgPerspectiva(ipm));
	imshow("KalmanHoughs", imgPerspectiva);
}

Mat1d KalmanHoughs::predict() { return _KF->predict(); }
Mat1d KalmanHoughs::correct() { return _KF->correct(measurement); }

// só seta a nova medida se houver vanishing point
void KalmanHoughs::setMeasurement(const HoughDoMeio &_hough) {
	measurement = _hough.toKalman();
}
