#include "pre_processing.h"

using namespace std;
using namespace cv;

void ELAS::pre_processing(const Mat3b & original_frame, const ConfigXML * _cfg, pre_processed * out) {
	printf("pre_processing()\n");

	// frame full size
	out->colorFrame = original_frame;
	out->grayFrame = toGrayscale(original_frame);
	out->grayFrameRoi = out->grayFrame(_cfg->roi);
	out->grayFrameRoiIPM = toIPM(out->grayFrameRoi, _cfg->ipm);
	out->colorFrameRoiIPM = toIPM(original_frame(_cfg->roi), _cfg->ipm);
	out->maskIPM = get_mask_ipm(MASK_IPM_ERODE_SIZE, _cfg);
}

Mat1b ELAS::toGrayscale(const Mat3b & frame) {
	Mat1b grayFrame;
	cvtColor(frame, grayFrame, CV_BGR2GRAY);
	return grayFrame;
}

Mat1b ELAS::toIPM(const Mat1b & frameRoi, IPM * ipm) {
	Mat1b grayFrameRoiIPM;
	ipm->applyHomography(frameRoi, grayFrameRoiIPM);
	return grayFrameRoiIPM;
}

Mat3b ELAS::toIPM(const Mat3b & frameRoi, IPM * ipm) {
	Mat3b colorFrameRoiIPM;
	ipm->applyHomography(frameRoi, colorFrameRoiIPM);
	return colorFrameRoiIPM;
}

vector<Point> ELAS::get_vertices_ipm(const ConfigXML * _cfg) {
	// get the corners of the region of interest
	Point p1 = Point(0, 0);
	Point p2 = Point(_cfg->roi.width, 0);
	Point p3 = Point(_cfg->roi.width, _cfg->roi.height);
	Point p4 = Point(0, _cfg->roi.height);

	// apply the IPM
	vector<Point> vertices;
	vertices.push_back(_cfg->ipm->applyHomography(p1));
	vertices.push_back(_cfg->ipm->applyHomography(p2));
	vertices.push_back(_cfg->ipm->applyHomography(p3));
	vertices.push_back(_cfg->ipm->applyHomography(p4));

	return vertices;
}

// when the IPM is applied, the result has some black areas
// the goal of this method is to get a mask that ignores the edges of these areas
Mat1b ELAS::get_mask_ipm(int erode_size, const ConfigXML * _cfg) {
	Mat1b maskIPM = Mat1b(_cfg->roi.size(), uchar(0));
	vector<Point> verticesIPM = get_vertices_ipm(_cfg);

	// fill the usefull area of white
	fillConvexPoly(maskIPM, verticesIPM, Scalar(255));

	// apply erode to ignore the edges
	if (erode_size != 0) {
		int abs_erode_size = abs(erode_size);
		Mat1b kernel = getStructuringElement(MORPH_ELLIPSE, Size(2 * abs_erode_size + 1, 2 * abs_erode_size + 1), Point(abs_erode_size, abs_erode_size));

		if (erode_size > 0) dilate(maskIPM, maskIPM, kernel);
		else erode(maskIPM, maskIPM, kernel);
	}

	return maskIPM;
}
