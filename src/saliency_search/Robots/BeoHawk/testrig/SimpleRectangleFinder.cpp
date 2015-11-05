/*
 * SimpleRectangleFinder.cpp
 *
 *  Created on: Feb 20, 2010
 *      Author: uscr
 */

#include "SimpleRectangleFinder.h"
#include <cstdio>
#include <cmath>


SimpleRectangleFinder::SimpleRectangleFinder(IplImage * imgRef) {
	edges = cvCloneImage(imgRef);
	storage = cvCreateMemStorage(0);
	approxStorage = cvCreateMemStorage(0);

	contours = 0;
	polyPoints = 0;

	//Grab the Camera Calibration Data:
	cameraMat = (CvMat*)cvLoad(CV_CALIB_IFILE);
	distortMat = (CvMat*)cvLoad(CV_CALIB_DFILE);
	mapx = cvCreateImage( cvGetSize( imgRef ), IPL_DEPTH_32F, 1 );
	mapy = cvCreateImage( cvGetSize( imgRef ), IPL_DEPTH_32F, 1 );
	ideal = cvCreateImage( cvGetSize( imgRef ), IPL_DEPTH_8U, 1 );
	cvInitUndistortMap( cameraMat, distortMat, mapx, mapy );


	for (int i = 0; i < 4; i++) {
		rectDeformCorn.push_back(cvPoint2D32f(0,0));
		rectFlatCorn.push_back(cvPoint2D32f(0,0));
	}

}

SimpleRectangleFinder::~SimpleRectangleFinder() {
	cvReleaseImage (&edges);
	cvReleaseImage (&mapx);
	cvReleaseImage (&mapy);
	cvReleaseImage (&ideal);
	cvReleaseMat (&distortMat);
	cvReleaseMat (&cameraMat);

}

void SimpleRectangleFinder::search(IplImage * subj, int width, int & centerx, int & centery) {

	contours = 0;
	polyPoints = 0;
	storage = cvCreateMemStorage(0);
	approxStorage = cvCreateMemStorage(0);
	int area;

	//rather than f'ing around with the stupid cvUndistortPoints method which
	// does NOT want to work, let's just do a complete image undistort first
	// and ignore have distortion taken care of apriori
	//cvRemap(subj, ideal, mapx, mapy);

	//Get lines from the image, clean up the lines, and then find the
	// countours from the lines
	cvCanny(subj, edges, CANNY_THRESH, CANNY_THRESH, 3);
	cvDilate(edges, edges, 0, 1);
	cvFindContours(edges, storage, &contours, sizeof(CvContour),
			CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));

	for (CvSeq* curCont = contours; curCont != NULL; curCont = curCont->h_next) {
		polyPoints = cvApproxPoly(curCont, sizeof(CvContour), approxStorage,
				CV_POLY_APPROX_DP, cvContourPerimeter(curCont) * POLY_APPROX_ACCURACY, 0);

		if (polyPoints->total == 4) {
			area = (int) abs(cvContourArea(polyPoints));

			if (area > AREA_THRESH) {

				//Okay, so we think this is the one and we are going to go for it.
				// Next we need to convert the observed screen coordinates to
				// ideal screen coordinates.
				for (int i = 0; i < 4; i++) {
					rectDeformCorn[i].x = ((CvPoint *) cvGetSeqElem(polyPoints, i))->x;
					rectDeformCorn[i].y = ((CvPoint *) cvGetSeqElem(polyPoints, i))->y;
				}

				//Compute the center by finding the intersection of the lines
				// created by the vertices idealized above
				center[0] = ((rectDeformCorn[0].x*rectDeformCorn[2].y - rectDeformCorn[2].x*rectDeformCorn[0].y)*
							(rectDeformCorn[1].x - rectDeformCorn[3].x) -
							(rectDeformCorn[1].x*rectDeformCorn[3].y - rectDeformCorn[3].x*rectDeformCorn[1].y)*
							(rectDeformCorn[0].x - rectDeformCorn[2].x));
				center[0] /= 	((rectDeformCorn[0].x - rectDeformCorn[2].x)*
								(rectDeformCorn[1].y - rectDeformCorn[3].y) -
								(rectDeformCorn[1].x - rectDeformCorn[3].x)*
								(rectDeformCorn[0].y - rectDeformCorn[2].y));
				center[1] = ((rectDeformCorn[0].x*rectDeformCorn[2].y - rectDeformCorn[2].x*rectDeformCorn[0].y)*
							(rectDeformCorn[1].y - rectDeformCorn[3].y) -
							(rectDeformCorn[1].x*rectDeformCorn[3].y - rectDeformCorn[3].x*rectDeformCorn[1].y)*
							(rectDeformCorn[0].y - rectDeformCorn[2].y));
				center[1] /= 	((rectDeformCorn[0].x - rectDeformCorn[2].x)*
								(rectDeformCorn[1].y - rectDeformCorn[3].y) -
								(rectDeformCorn[1].x - rectDeformCorn[3].x)*
								(rectDeformCorn[0].y - rectDeformCorn[2].y));

				centerx = center[0];
				centery = center[1];

				//Ok so this is the square placed at the center of where we are
				rectFlatCorn[1].x = rectFlatCorn[2].x = center[0] + width/2.0;
				rectFlatCorn[1].y = rectFlatCorn[0].y = center[1] + width/2.0;
			    rectFlatCorn[3].x = rectFlatCorn[0].x = center[0] - width/2.0;
			    rectFlatCorn[2].y = rectFlatCorn[3].y = center[1] - width/2.0;

			    //find the perspective transformation:
			    CvMat * mapMatrix = cvCreateMat(3, 3, CV_32FC1);
			    cvGetPerspectiveTransform (&rectFlatCorn[0], &rectDeformCorn[0], mapMatrix);

			    double angle1 = asin(cvmGet(mapMatrix, 1, 0));
			    double angle2 = acos(cvmGet(mapMatrix, 0, 0) / cos(angle1));
			    double angle3 = acos(cvmGet(mapMatrix, 1, 1) / cos(angle1));

			    printf("-----------------------------\n");
			    printf ("[\t%f\t%f\t%f\t]\n", cvmGet(mapMatrix, 0, 0), cvmGet(mapMatrix, 0, 1), cvmGet(mapMatrix, 0, 2));
			    printf ("[\t%f\t%f\t%f\t]\n", cvmGet(mapMatrix, 1, 0), cvmGet(mapMatrix, 1, 1), cvmGet(mapMatrix, 1, 2));
			    printf ("[\t%f\t%f\t%f\t]\n", cvmGet(mapMatrix, 2, 0), cvmGet(mapMatrix, 2, 1), cvmGet(mapMatrix, 2, 2));
			    //printf ("angle1 = %f\n", angle1);
			    //printf ("angle2 = %f\n", angle2);
			    //printf ("angle3 = %f\n", angle3);

			    cvReleaseMat(&mapMatrix);
			}
		}
	}

	cvReleaseMemStorage(&approxStorage);
	cvReleaseMemStorage(&storage);
}
