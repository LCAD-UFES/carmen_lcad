/*
 * SimpleRectangleFinder.h
 *
 *  Created on: Feb 20, 2010
 *      Author: uscr
 */

#ifndef SIMPLERECTANGLEFINDER_H_
#define SIMPLERECTANGLEFINDER_H_

#include <opencv/cv.h>
#include <vector>

#define CANNY_THRESH			100.0
#define POLY_APPROX_ACCURACY	0.035
#define AREA_THRESH				5000
#define CV_CALIB_DFILE			"cv_calib/Distortion.xml"
#define CV_CALIB_IFILE			"cv_calib/Intrinsics.xml"
#define AR_PATT_THRESH		    80
#define AR_PATT_WIDTH			170

class SimpleRectangleFinder {
public:
	SimpleRectangleFinder(IplImage * imgRef);
	virtual ~SimpleRectangleFinder();
	void search(IplImage * subj, int width, int & centerx, int & centery);


private:
	IplImage * edges, * ideal;
	CvMemStorage *storage, *approxStorage;
	CvSeq *contours, *polyPoints;
	CvMat * distortMat, * cameraMat;
	IplImage * mapx, * mapy;

	std::vector<CvPoint2D32f> rectDeformCorn;
	std::vector<CvPoint2D32f> rectFlatCorn;
	double center[2];
};

#endif /* SIMPLERECTANGLEFINDER_H_ */
