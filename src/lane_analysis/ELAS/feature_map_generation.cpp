#include "feature_map_generation.h"

using namespace std;
using namespace cv;

void ELAS::feature_map_generation(const pre_processed * _pre_processed, const ConfigXML * _cfg, feature_maps * out) {
	printf("feature_map_generation()\n");

	// step row filter map: SRF
	out->map_srf = get_map_srf(_pre_processed->grayFrame, TAU_BEGIN, TAU_END, MAP_SRF_THRES);

	// step row filter map: SRF IPM
	out->map_srf_ipm = Helper::toIPM(out->map_srf, _cfg->roi, _cfg->ipm, INTER_NEAREST);

	// horizontal difference of gaussians map: HDOG
	out->map_hdog_ipm = get_map_hdog_ipm(_pre_processed->grayFrameRoiIPM, _pre_processed->maskIPM);

	// vertical absolute derivative map: VAD
	out->map_vad_ipm = get_map_vad_ipm(_pre_processed->grayFrameRoiIPM, _pre_processed->maskIPM);

	// intensity based map: INB
	out->map_inb_ipm = get_map_inb_ipm(_pre_processed->grayFrameRoi, out->map_srf, _cfg);

	// TODO: resulting map
	// Mat1d mapaIPM = Mapas::getMapaResultante(mapa1ipm, mapa2ipm, mapa5ipm);
}

Mat1b ELAS::get_map_srf(const Mat1b &grayFrame, int tau_begin, int tau_end, int _threshold) {
	Mat1b out_map_srf = grayFrame.clone();

	// apply the step row filter proposed by Nieto
	out_map_srf = filtroNieto(out_map_srf, tau_begin, tau_end);

	// applies a threshold on the filteresd image
	threshold(out_map_srf, out_map_srf, _threshold, 255, CV_THRESH_BINARY);

	return out_map_srf;
}

Mat1b ELAS::get_map_hdog_ipm(const Mat1b &grayFrameRoiIPM, const Mat &maskIPM) {
	Mat1b out_map_hdog = difference_of_gaussians(grayFrameRoiIPM);
	return out_map_hdog & maskIPM; // remove the edges caused by the IPM and return
}

Mat1b ELAS::get_map_vad_ipm(const Mat1b &grayFrameRoiIPM, const Mat &maskIPM) {
	Mat sobel_negative, sobel_positive;

	// negative => dark to bright
	Sobel(grayFrameRoiIPM, sobel_negative, CV_32F, 0, 1);
	sobel_positive = sobel_negative.clone();
	sobel_negative = -1 * sobel_negative;
	threshold(sobel_negative, sobel_negative, SOBEL_THRES, 255, CV_8U);
	sobel_negative.convertTo(sobel_negative, CV_8U);

	// positive => bright to dark
	threshold(sobel_positive, sobel_positive, SOBEL_THRES, 255, CV_8U);
	sobel_positive.convertTo(sobel_positive, CV_8U);

	// combine negative and positive, remove the edges caused by the IPM and return
	return (sobel_negative | sobel_positive) & maskIPM;
}

Mat1b ELAS::get_map_inb_ipm(const Mat1b &grayFrameRoi, const Mat1b &map_srf, const ConfigXML *_cfg) {
	// binary version of map_srf
	// TODO: srf should be a binary map, shouldn't it?
	Mat1b map_srf_binary;
	(map_srf(_cfg->roi)).convertTo(map_srf_binary, CV_8UC1, 255.0);
	threshold(map_srf_binary, map_srf_binary, INTENSITY_THRES, 255, CV_8UC1); // convert to binary

	// invert the srf binary
	Mat map_srf_inverse = ~map_srf_binary;

	// get the mean and standard deviation of the asphalt
	map_srf_inverse = Helper::morphErode(map_srf_inverse, 3);
	Scalar mean_asphalt, stddev_asphalt;
	cv::meanStdDev(grayFrameRoi, mean_asphalt, stddev_asphalt, map_srf_inverse);
	Mat markings = grayFrameRoi > mean_asphalt[0] + 2 * stddev_asphalt[0]; // mascara com os pixels que estao abaixo da threshold do asfalto

	// get only the intersection of expected markings and map_srf
	Mat subset_markings = Helper::morphErode(map_srf_binary & markings, 3);

	// get the mean and the standard deviation of the pavement markings
	Scalar mean_markings, stddev_markings;
	cv::meanStdDev(grayFrameRoi, mean_markings, stddev_markings, subset_markings);

	Mat map_inb = grayFrameRoi > mean_markings[0] - stddev_markings[0];
	Mat map_inb_ipm = Helper::toIPM(map_inb, _cfg->ipm, INTER_NEAREST);

	return map_inb_ipm;
}

Mat1b ELAS::filtroNieto(Mat1b &srcGRAY, int tauInicio, int tauFim) {
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
	return tempDst;
}

Mat ELAS::difference_of_gaussians(const Mat &frame, double gaussian_size_in, double gaussian_size_out, double thres, bool y_direction) {

	Mat imageOut = Mat(frame.size(), CV_8UC1);
	Mat image32F(frame.rows, frame.cols, CV_8UC1);

	// convert to 32-float to apply the gaussians
	frame.convertTo(image32F, CV_32F);

	// applies two gaussian blurs
	Mat gaussianIn, gaussianOut, DoG;
	if (!y_direction) {
		GaussianBlur(image32F, gaussianIn, Size(0, 1), gaussian_size_in);
		GaussianBlur(image32F, gaussianOut, Size(0, 1), gaussian_size_out);
	} else {
		GaussianBlur(image32F, gaussianIn, Size(1, 0), gaussian_size_in);
		GaussianBlur(image32F, gaussianOut, Size(1, 0), gaussian_size_out);
	}

	// difference of gaussians
	DoG = gaussianIn - gaussianOut;

	// applies a threshold
	threshold(DoG, DoG, thres, 255, THRESH_BINARY);

	DoG.convertTo(imageOut, CV_8UC1);

	return imageOut;
}
