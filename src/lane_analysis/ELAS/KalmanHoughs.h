#ifndef __KALMAN_HOUGHS_H
#define __KALMAN_HOUGHS_H

#include <opencv2/opencv.hpp>
#include "AnaliseDasHoughs.h"

using namespace std;
using namespace cv;


class KalmanHoughs {

	public:
		KalmanHoughs(ConfigXML *_config, bool _verbose = false, bool _display = false);
		HoughDoMeio executar(const HoughDoMeio &hough, const Mat &colorFramePerspectiva, const Mat3b &colorFrameRoiIPM);
		void view(HoughDoMeio *houghDoMeio, const Mat &colorFramePerspectiva, const Mat3b &colorFrameRoiIPM, const Scalar &cor);
		
		void setMeasurement(const HoughDoMeio &_hough);
		KalmanFilter* getKalman() { return this->_KF; }

		Mat1d measurement;

	private:
		void view();
		bool display;
		bool verbose;
		KalmanFilter *_KF;
		ConfigXML *config;

		Mat1d predict();
		Mat1d correct();
};

#endif // __KALMAN_HOUGHS_H