#ifndef __NIETO
#define __NIETO
// #define USING_ARMADILLO


#include <opencv2/opencv.hpp>

#include "utils/IPM.h"
#include "libs/vanishingPoint/MSAC.h"

using namespace cv;
using namespace std;

class FeatureEM {
	public:
		FeatureEM(const Mat1b &input) { image = input.clone(); }
		Mat1b image;
		map<string, double> means0;
		map<string, double> covs0;
		map<string, double> weights0;
};

class Nieto {

	public:
		Nieto(bool _verbose = false, bool _display = false);
		vector<Mat> vanishingPoint;
		vector<vector<vector<Point> > > houghLines;
		
		Mat1b filtro(const Mat3b &inColorFrame, int tauInicio = 10, int tauFim = 60, int thres = 40);
		Mat1b ipm(const Mat1b &inBinaryFrameFiltrado, const Rect &roi, IPM * ipm);
		vector<vector<vector<Point> > > vp(const Mat1b &inBinaryFrameFiltrado, Mat3b &inVanishingPointImage, const Rect &roi, const int maxNumLines = 200, int houghThresholdInicial = 20, int houghStep = 10, double houghMinLineLength = 5, double houghMaxLineGap = 5);
		void featureI(const Mat1b &inBinaryFrameFiltradoRoiIPM, const Mat1b &grayFrameRoiIPM, map<string, double> &outMeans0, map<string, double> &outCovs0, map<string, double> &outWeights0, bool aplicarSobel = false);
		void featureL(const Mat1b &inGrayFrameFiltradoRoi, map<string, double> &outMeans0, map<string, double> &outCovs0, map<string, double> &outWeights0);
		void featureL_IPM(const Mat1b &inGrayFrameFiltradoRoi, IPM * _ipm, map<string, double> &outMeans0, map<string, double> &outCovs0, map<string, double> &outWeights0);
		void ExpectationMaximizationOpenCV(const Mat1b &inGrayFrameRoi, int maxIters, map<string, double> &_means0, map<string, double> &_covs0, map<string, double> &_weights0);
		void ExpectationMaximizationOpenCV2Features(const Mat1b &imageI, const Mat1b &imageL,
			map<string, double> &i_means0, map<string, double> &i_covs0, map<string, double> &i_weights0,
			map<string, double> &l_means0, map<string, double> &l_covs0, map<string, double> &l_weights0, int maxIters);

#ifdef USING_ARMADILLO
		void ExpectationMaximizationArmadillo(const Mat1b &inGrayFrameRoi, int maxIters, map<string, double> &_means0, map<string, double> &_covs0, map<string, double> &_weights0);
		void ExpectationMaximizationArmadillo2Features(const Mat1b &imageI, const Mat1b &imageL,
			map<string, double> &i_means0, map<string, double> &i_covs0, map<string, double> &i_weights0,
			map<string, double> &l_means0, map<string, double> &l_covs0, map<string, double> &l_weights0, int maxIters = 10);
#endif

		// https://marcosnietoblog.wordpress.com/2011/12/27/lane-markings-detection-and-vanishing-point-detection-with-opencv/
		void nietoLaneMarkingsDetector(Mat1b &srcGRAY, Mat1b &dstGRAY, int tauInicio, int tauFim);

	private:
		bool display;
		bool verbose;
		double getMean(const Mat1b &_image, const Mat1b &_mask);
		double getVariance(const Mat1b &_image, double mean, const Mat1b &_mask);

		
};

#endif // __NIETO
